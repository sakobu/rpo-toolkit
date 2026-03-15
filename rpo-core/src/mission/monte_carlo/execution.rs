//! Per-sample execution and ensemble statistics collection.

use nalgebra::Vector3;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;

use crate::constants::{MC_DEFAULT_COLLISION_THRESHOLD_KM, MIN_SPACECRAFT_MASS_KG};
use crate::elements::keplerian_conversions::state_to_keplerian;
use crate::elements::eci_ric_dcm::{eci_to_ric_dcm, eci_to_ric_relative};
use crate::elements::roe::compute_roe;
use crate::mission::config::{MissionConfig, SafetyConfig};
use crate::mission::errors::MissionError;
use crate::mission::safety::analyze_trajectory_safety;
use crate::mission::types::{Waypoint, WaypointMission};
use crate::mission::waypoints::plan_waypoint_mission;
use crate::propagation::nyx_bridge::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
};
use crate::propagation::propagator::{PropagatedState, PropagationModel};
use crate::types::{DepartureState, SpacecraftConfig, StateVector};

use super::sampling::{disperse_maneuver, sample_distribution};
use super::statistics::{compute_dispersion_envelope, compute_percentile_stats};
use super::types::{
    EnsembleStatistics, MonteCarloConfig, MonteCarloInput, MonteCarloMode, SampleResult,
};
use super::MonteCarloError;

/// Result of a single MC sample execution (internal).
pub(crate) struct SampleOutput {
    /// Lightweight result for the report.
    pub(crate) result: SampleResult,
    /// Full trajectory for dispersion envelope computation.
    pub(crate) trajectory: Vec<PropagatedState>,
}

/// Execute a single Monte Carlo sample.
///
/// Propagates chief + deputy through nyx with dispersed initial state and
/// maneuver execution errors. In closed-loop mode, re-targets the mission from
/// the dispersed state before propagation.
///
/// # Invariants
/// - `input.initial_chief` and `input.initial_deputy` must represent bound orbits.
/// - `input.nominal_mission.legs` must be non-empty (caller responsibility).
/// - Dispersed state must remain a bound orbit (`a > 0`, `0 <= e < 1`) or
///   an error is returned for this sample.
///
/// # Errors
/// - [`MonteCarloError::NegativeSma`] / [`MonteCarloError::InvalidEccentricity`]
///   if state dispersion produces an unbound orbit.
/// - [`MonteCarloError::NegativeSigma`] / [`MonteCarloError::NegativeHalfWidth`]
///   if dispersion parameters are invalid.
/// - [`MonteCarloError::EmptyEnsemble`] if a nyx trajectory segment is empty.
/// - Propagation or nyx bridge errors from bridge functions.
#[allow(clippy::similar_names, clippy::too_many_lines)]
pub(crate) fn run_single_sample(
    input: &MonteCarloInput<'_>,
    index: u32,
    master_seed: u64,
) -> Result<SampleOutput, MonteCarloError> {
    let config = input.config;
    let initial_chief = input.initial_chief;
    let initial_deputy = input.initial_deputy;
    let nominal_mission = input.nominal_mission;
    let deputy_config = input.deputy_config;
    let chief_config = input.chief_config;
    let almanac = input.almanac;

    let mut rng = ChaCha20Rng::seed_from_u64(master_seed.wrapping_add(u64::from(index)));

    // Disperse deputy initial state in RIC, convert to ECI
    let dispersed_deputy = if let Some(ref state_disp) = config.dispersions.state {
        let pos_ric_delta = Vector3::new(
            sample_distribution(&state_disp.position_radial_km, &mut rng)?,
            sample_distribution(&state_disp.position_intrack_km, &mut rng)?,
            sample_distribution(&state_disp.position_crosstrack_km, &mut rng)?,
        );
        let vel_ric_delta = Vector3::new(
            sample_distribution(&state_disp.velocity_radial_km_s, &mut rng)?,
            sample_distribution(&state_disp.velocity_intrack_km_s, &mut rng)?,
            sample_distribution(&state_disp.velocity_crosstrack_km_s, &mut rng)?,
        );

        // RIC → ECI via DCM transpose
        let dcm = eci_to_ric_dcm(initial_chief);
        let dcm_transpose = dcm.transpose();
        let pos_eci_delta = dcm_transpose * pos_ric_delta;
        let vel_eci_delta = dcm_transpose * vel_ric_delta;

        StateVector {
            epoch: initial_deputy.epoch,
            position_eci_km: initial_deputy.position_eci_km + pos_eci_delta,
            velocity_eci_km_s: initial_deputy.velocity_eci_km_s + vel_eci_delta,
        }
    } else {
        initial_deputy.clone()
    };

    // Validity check: ensure dispersed state is a bound orbit
    let ke = state_to_keplerian(&dispersed_deputy)
        .map_err(|_| MonteCarloError::NegativeSma {
            sample_index: index,
            a_km: 0.0,
        })?;
    if ke.a_km <= 0.0 {
        return Err(MonteCarloError::NegativeSma {
            sample_index: index,
            a_km: ke.a_km,
        });
    }
    if ke.e >= 1.0 {
        return Err(MonteCarloError::InvalidEccentricity {
            sample_index: index,
            e: ke.e,
        });
    }

    // Optionally disperse spacecraft properties
    let sample_deputy_config = if let Some(ref sc_disp) = config.dispersions.spacecraft {
        SpacecraftConfig {
            coeff_drag: deputy_config.coeff_drag
                + sample_distribution(&sc_disp.coeff_drag, &mut rng)?,
            drag_area_m2: (deputy_config.drag_area_m2
                + sample_distribution(&sc_disp.drag_area_m2, &mut rng)?)
            .max(0.0),
            dry_mass_kg: (deputy_config.dry_mass_kg
                + sample_distribution(&sc_disp.dry_mass_kg, &mut rng)?)
            .max(MIN_SPACECRAFT_MASS_KG),
            ..*deputy_config
        }
    } else {
        *deputy_config
    };

    // Determine which mission plan to use for propagation
    let (mission_to_use, converged) = match config.mode {
        MonteCarloMode::OpenLoop => (None, true),
        MonteCarloMode::ClosedLoop => {
            match retarget_from_dispersed(
                initial_chief,
                &dispersed_deputy,
                nominal_mission,
                input.mission_config,
                input.propagator,
            ) {
                Ok(retargeted) => (Some(retargeted), true),
                Err(_) => (None, false), // fall back to nominal plan
            }
        }
    };
    let active_mission = mission_to_use.as_ref().unwrap_or(nominal_mission);

    // Propagate through legs with maneuver execution errors
    let mut chief_state = initial_chief.clone();
    let mut deputy_state = dispersed_deputy;
    let mut total_dv = 0.0_f64;
    let mut waypoint_miss_km = Vec::with_capacity(active_mission.legs.len());
    let mut elapsed_total = 0.0_f64;

    let maneuver_disp = config.dispersions.maneuver.as_ref();
    let traj_steps = config.trajectory_steps.max(1);
    // u32 → usize: always widening on 32-bit and 64-bit platforms.
    let mut all_safety_pairs: Vec<(f64, StateVector, StateVector)> =
        Vec::with_capacity(traj_steps as usize * active_mission.legs.len());

    // Build dynamics once before the loop; clone per propagation call.
    // SpacecraftDynamics derives Clone (lightweight Arc ref-count bumps)
    // vs full construction (frame lookups, harmonics, drag/SRP setup).
    let dynamics_template = build_full_physics_dynamics(almanac)?;

    for leg in &active_mission.legs {
        // Apply dispersed departure Δv
        let dep_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.departure_maneuver.dv_ric_km_s, disp, &mut rng)?
        } else {
            leg.departure_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &dep_dv);
        total_dv += dep_dv.norm();

        // Propagate chief + deputy through this leg
        let chief_traj = nyx_propagate_segment(
            &chief_state,
            leg.tof_s,
            traj_steps,
            chief_config,
            dynamics_template.clone(),
            almanac,
        )?;

        let deputy_traj = nyx_propagate_segment(
            &deputy_state,
            leg.tof_s,
            traj_steps,
            &sample_deputy_config,
            dynamics_template.clone(),
            almanac,
        )?;

        // Collect chief/deputy pairs for safety analysis
        for (c_entry, d_entry) in chief_traj.iter().zip(deputy_traj.iter()) {
            all_safety_pairs.push((
                elapsed_total + c_entry.0,
                c_entry.1.clone(),
                d_entry.1.clone(),
            ));
        }

        // Update states to end of leg
        chief_state = chief_traj
            .last()
            .map(|(_, s)| s.clone())
            .ok_or(MonteCarloError::EmptyEnsemble)?;
        deputy_state = deputy_traj
            .last()
            .map(|(_, s)| s.clone())
            .ok_or(MonteCarloError::EmptyEnsemble)?;

        // Apply dispersed arrival Δv
        let arr_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.arrival_maneuver.dv_ric_km_s, disp, &mut rng)?
        } else {
            leg.arrival_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &arr_dv);
        total_dv += arr_dv.norm();

        // Compute miss distance at waypoint arrival
        let ric_rel = eci_to_ric_relative(&chief_state, &deputy_state);
        let miss = (ric_rel.position_ric_km - leg.to_position_ric_km).norm();
        waypoint_miss_km.push(miss);

        elapsed_total += leg.tof_s;
    }

    // Build safety states and compute safety metrics.
    // Note: nyx provides osculating ECI states, so ROE (and e/i separation)
    // are computed from osculating elements. The passive safety metric
    // (D'Amico Eq. 2.22) is orbit-averaged and designed for mean elements;
    // osculating input adds short-period noise but does not affect operational
    // safety metrics (3D distance, R/C distance).
    let safety_states = build_nyx_safety_states(&all_safety_pairs)?;
    let safety = analyze_trajectory_safety(&safety_states).ok();

    Ok(SampleOutput {
        result: SampleResult {
            index,
            total_dv_km_s: total_dv,
            safety,
            waypoint_miss_km,
            converged,
        },
        trajectory: safety_states,
    })
}

/// Re-target the mission from a dispersed deputy state (closed-loop mode).
///
/// Converts the dispersed ECI states to a `DepartureState`, extracts waypoints
/// from the nominal mission legs, and runs `plan_waypoint_mission` to produce
/// a new mission plan with updated Δvs.
///
/// # Invariants
/// - `chief` and `dispersed_deputy` must be convertible to Keplerian elements
///   (bound orbits, non-degenerate position vectors).
/// - `nominal_mission.legs` must be non-empty.
///
/// # Errors
/// - [`MissionError::Conversion`] if ECI → Keplerian or ROE computation fails.
/// - Any [`MissionError`] variant from `plan_waypoint_mission` (e.g., targeting
///   non-convergence).
fn retarget_from_dispersed(
    chief: &StateVector,
    dispersed_deputy: &StateVector,
    nominal_mission: &WaypointMission,
    mission_config: &MissionConfig,
    propagator: &PropagationModel,
) -> Result<WaypointMission, MissionError> {
    let chief_ke = state_to_keplerian(chief).map_err(MissionError::Conversion)?;
    let deputy_ke = state_to_keplerian(dispersed_deputy).map_err(MissionError::Conversion)?;
    let roe = compute_roe(&chief_ke, &deputy_ke).map_err(MissionError::Conversion)?;

    let departure = DepartureState {
        roe,
        chief: chief_ke,
        epoch: chief.epoch,
    };

    // Extract waypoints from nominal mission legs (same targets, same TOFs)
    let waypoints: Vec<Waypoint> = nominal_mission
        .legs
        .iter()
        .map(|leg| Waypoint {
            position_ric_km: leg.to_position_ric_km,
            velocity_ric_km_s: leg.target_velocity_ric_km_s,
            tof_s: Some(leg.tof_s),
        })
        .collect();

    plan_waypoint_mission(&departure, &waypoints, mission_config, propagator)
}

/// Collect aggregate ensemble statistics from successful samples.
///
/// Computes percentile distributions for Δv, safety metrics, waypoint miss
/// distances, collision probability, convergence rate, and trajectory
/// dispersion envelope.
///
/// # Invariants
/// - `samples` must be non-empty (caller ensures this after filtering).
/// - `trajectories.len() == samples.len()`.
/// - `total_num_samples` is the total number of MC samples attempted (including failures).
///
/// # Errors
/// Returns [`MonteCarloError::EmptyEnsemble`] if `compute_percentile_stats`
/// fails on an empty Δv vector (should not occur if `samples` is non-empty).
pub(crate) fn collect_ensemble_statistics(
    samples: &[SampleResult],
    trajectories: &[Vec<PropagatedState>],
    config: &MonteCarloConfig,
    total_num_samples: u32,
    safety_config: Option<&SafetyConfig>,
) -> Result<EnsembleStatistics, MonteCarloError> {
    let total_dvs: Vec<f64> = samples.iter().map(|s| s.total_dv_km_s).collect();

    // Extract safety metrics from samples that have them
    let mut min_rc_values = Vec::new();
    let mut min_3d_values = Vec::new();
    let mut min_ei_values = Vec::new();

    for s in samples {
        if let Some(ref safety) = s.safety {
            min_rc_values.push(safety.operational.min_rc_separation_km);
            min_3d_values.push(safety.operational.min_distance_3d_km);
            min_ei_values.push(safety.passive.min_ei_separation_km);
        }
    }

    let total_dv_stats = compute_percentile_stats(&total_dvs)?;

    let min_rc_stats = if min_rc_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&min_rc_values)?)
    };
    let min_3d_stats = if min_3d_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&min_3d_values)?)
    };
    let min_ei_stats = if min_ei_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&min_ei_values)?)
    };

    // Per-waypoint miss distance statistics
    let num_waypoints = samples
        .first()
        .map_or(0, |s| s.waypoint_miss_km.len());
    let mut waypoint_miss_stats = Vec::with_capacity(num_waypoints);
    for wp_idx in 0..num_waypoints {
        let misses: Vec<f64> = samples
            .iter()
            .filter_map(|s| s.waypoint_miss_km.get(wp_idx).copied())
            .collect();
        if misses.is_empty() {
            waypoint_miss_stats.push(None);
        } else {
            // compute_percentile_stats filters NaN internally; if all values
            // are non-finite, fall back to None
            match compute_percentile_stats(&misses) {
                Ok(stats) => waypoint_miss_stats.push(Some(stats)),
                Err(MonteCarloError::EmptyEnsemble) => {
                    waypoint_miss_stats.push(None);
                }
                Err(e) => return Err(e),
            }
        }
    }

    // Collision probability and convergence rate use total_num_samples as
    // denominator (not just successful samples) to avoid misleading rates.
    let n_total_f = f64::from(total_num_samples);

    let mut collision_count = 0_u32;
    let mut converged_count = 0_u32;
    let mut ei_violation_count = 0_u32;
    let mut keepout_violation_count = 0_u32;
    for s in samples {
        if s.safety.as_ref().is_some_and(|safety| {
            safety.operational.min_distance_3d_km < MC_DEFAULT_COLLISION_THRESHOLD_KM
        }) {
            collision_count += 1;
        }
        if let Some(sc) = safety_config
            && let Some(ref safety) = s.safety
        {
            if safety.passive.min_ei_separation_km < sc.min_ei_separation_km {
                ei_violation_count += 1;
            }
            if safety.operational.min_distance_3d_km < sc.min_distance_3d_km {
                keepout_violation_count += 1;
            }
        }
        if s.converged {
            converged_count += 1;
        }
    }
    let collision_probability = f64::from(collision_count) / n_total_f;
    let convergence_rate = f64::from(converged_count) / n_total_f;
    let ei_violation_rate = f64::from(ei_violation_count) / n_total_f;
    let keepout_violation_rate = f64::from(keepout_violation_count) / n_total_f;

    // Dispersion envelope
    let dispersion_envelope = compute_dispersion_envelope(trajectories, config.trajectory_steps);

    Ok(EnsembleStatistics {
        total_dv_km_s: total_dv_stats,
        min_rc_distance_km: min_rc_stats,
        min_3d_distance_km: min_3d_stats,
        min_ei_separation_km: min_ei_stats,
        waypoint_miss_km: waypoint_miss_stats,
        collision_probability,
        convergence_rate,
        ei_violation_rate,
        keepout_violation_rate,
        dispersion_envelope,
    })
}

//! Per-sample execution and ensemble statistics collection.

use nalgebra::Vector3;
use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;

use rpo_core::constants::{MC_DEFAULT_COLLISION_THRESHOLD_KM, MIN_SPACECRAFT_MASS_KG};
use rpo_core::elements::eci_ric_dcm::{eci_to_ric_dcm, eci_to_ric_relative};
use rpo_core::elements::keplerian_conversions::state_to_keplerian;
use rpo_core::elements::roe::compute_roe;
use rpo_core::mission::config::{MissionConfig, SafetyConfig};
use rpo_core::mission::errors::MissionError;
use rpo_core::mission::monte_carlo::{
    EnsembleStatistics, MonteCarloConfig, MonteCarloMode, SampleResult, SpacecraftDispersion,
    StateDispersion,
};
use rpo_core::mission::monte_carlo::MonteCarloError as CoreMonteCarloError;
use rpo_core::mission::safety::analyze_trajectory_safety;
use rpo_core::mission::types::{Waypoint, WaypointMission};
use rpo_core::mission::waypoints::plan_waypoint_mission;
use rpo_core::propagation::propagator::{PropagatedState, PropagationModel};
use rpo_core::types::{DepartureState, SpacecraftConfig, StateVector};

use crate::nyx_bridge::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
    ChiefDeputySnapshot,
};

use super::sampling::{disperse_maneuver, sample_distribution};
use super::statistics::{compute_dispersion_envelope, compute_percentile_stats};
use super::types::MonteCarloInput;
use super::MonteCarloError;

/// Result of a single MC sample execution (internal).
pub(crate) struct SampleOutput {
    /// Lightweight result for the report.
    pub(crate) result: SampleResult,
    /// Full trajectory for dispersion envelope computation.
    pub(crate) trajectory: Vec<PropagatedState>,
}

/// Accumulated results from propagating all mission legs with dispersed maneuvers.
struct LegPropagationResult {
    /// Total Δv magnitude across all maneuvers (km/s).
    total_dv_km_s: f64,
    /// Per-waypoint RIC miss distance (km).
    waypoint_miss_km: Vec<f64>,
    /// Chief/deputy snapshot pairs for safety analysis.
    safety_pairs: Vec<ChiefDeputySnapshot>,
}

/// Disperse the deputy initial state in RIC and validate the result.
///
/// Samples position and velocity deltas from the configured state dispersion,
/// converts RIC deltas to ECI via DCM transpose, and validates that the
/// resulting orbit is bound (`a > 0`, `0 <= e < 1`).
///
/// # Invariants
/// - `initial_chief` must have a non-degenerate position vector (for DCM).
///
/// # Errors
/// - [`CoreMonteCarloError::NegativeSma`] / [`CoreMonteCarloError::InvalidEccentricity`]
///   if the dispersed state is not a bound orbit.
/// - Sampling errors from [`sample_distribution`].
/// - [`DcmError`] via [`MonteCarloError`] if DCM computation fails.
fn disperse_deputy_state<R: Rng>(
    initial_chief: &StateVector,
    initial_deputy: &StateVector,
    state_disp: Option<&StateDispersion>,
    sample_index: u32,
    rng: &mut R,
) -> Result<StateVector, MonteCarloError> {
    let dispersed = if let Some(disp) = state_disp {
        let pos_ric_delta = Vector3::new(
            sample_distribution(&disp.position_radial_km, rng)?,
            sample_distribution(&disp.position_intrack_km, rng)?,
            sample_distribution(&disp.position_crosstrack_km, rng)?,
        );
        let vel_ric_delta = Vector3::new(
            sample_distribution(&disp.velocity_radial_km_s, rng)?,
            sample_distribution(&disp.velocity_intrack_km_s, rng)?,
            sample_distribution(&disp.velocity_crosstrack_km_s, rng)?,
        );

        // RIC → ECI via DCM transpose
        let dcm = eci_to_ric_dcm(initial_chief)?;
        let dcm_transpose = dcm.transpose();

        StateVector {
            epoch: initial_deputy.epoch,
            position_eci_km: initial_deputy.position_eci_km + dcm_transpose * pos_ric_delta,
            velocity_eci_km_s: initial_deputy.velocity_eci_km_s + dcm_transpose * vel_ric_delta,
        }
    } else {
        initial_deputy.clone()
    };

    // Validity check: ensure dispersed state is a bound orbit
    let ke = state_to_keplerian(&dispersed).map_err(|_| {
        CoreMonteCarloError::NegativeSma {
            sample_index,
            a_km: 0.0,
        }
    })?;
    if ke.a_km <= 0.0 {
        return Err(CoreMonteCarloError::NegativeSma {
            sample_index,
            a_km: ke.a_km,
        }
        .into());
    }
    if ke.e >= 1.0 {
        return Err(CoreMonteCarloError::InvalidEccentricity {
            sample_index,
            e: ke.e,
        }
        .into());
    }

    Ok(dispersed)
}

/// Optionally disperse spacecraft configuration properties.
///
/// Samples additive deltas for drag coefficient, drag area, and dry mass
/// from the configured spacecraft dispersion. Clamps area to non-negative
/// and mass to [`MIN_SPACECRAFT_MASS_KG`].
///
/// Returns the nominal config unchanged if no spacecraft dispersion is configured.
///
/// # Errors
/// Sampling errors from [`sample_distribution`].
fn disperse_spacecraft<R: Rng>(
    nominal: &SpacecraftConfig,
    sc_disp: Option<&SpacecraftDispersion>,
    rng: &mut R,
) -> Result<SpacecraftConfig, MonteCarloError> {
    if let Some(disp) = sc_disp {
        Ok(SpacecraftConfig {
            coeff_drag: nominal.coeff_drag + sample_distribution(&disp.coeff_drag, rng)?,
            drag_area_m2: (nominal.drag_area_m2 + sample_distribution(&disp.drag_area_m2, rng)?)
                .max(0.0),
            dry_mass_kg: (nominal.dry_mass_kg + sample_distribution(&disp.dry_mass_kg, rng)?)
                .max(MIN_SPACECRAFT_MASS_KG),
            ..*nominal
        })
    } else {
        Ok(*nominal)
    }
}

/// Propagate chief and deputy through all mission legs with dispersed maneuvers.
///
/// For each leg: applies dispersed departure Δv, propagates both vehicles through
/// nyx full-physics dynamics, collects chief/deputy snapshots for safety analysis
/// (skipping t=0), applies dispersed arrival Δv, and computes waypoint miss distance.
///
/// # Invariants
/// - `active_mission.legs` must be non-empty (caller responsibility).
/// - `input.almanac` must contain required frames and force models.
///
/// # Errors
/// - Maneuver dispersion errors from [`disperse_maneuver`].
/// - Nyx propagation or impulse application errors.
/// - [`CoreMonteCarloError::EmptyEnsemble`] if a trajectory segment is empty.
fn propagate_dispersed_legs<R: Rng>(
    input: &MonteCarloInput<'_>,
    dispersed_deputy: StateVector,
    active_mission: &WaypointMission,
    sample_deputy_config: &SpacecraftConfig,
    rng: &mut R,
) -> Result<LegPropagationResult, MonteCarloError> {
    let maneuver_disp = input.config.dispersions.maneuver.as_ref();
    let traj_steps = input.config.trajectory_steps;

    let mut chief_state = input.initial_chief.clone();
    let mut deputy_state = dispersed_deputy;
    let mut total_dv = 0.0_f64;
    let mut waypoint_miss_km = Vec::with_capacity(active_mission.legs.len());
    let mut elapsed_total_s = 0.0_f64;
    // u32 → usize: always widening on 32-bit and 64-bit platforms.
    let mut safety_pairs: Vec<ChiefDeputySnapshot> =
        Vec::with_capacity(traj_steps as usize * active_mission.legs.len());

    // Build dynamics once; clone per propagation call.
    // SpacecraftDynamics derives Clone (lightweight Arc ref-count bumps)
    // vs full construction (frame lookups, harmonics, drag/SRP setup).
    let dynamics_template = build_full_physics_dynamics(input.almanac)?;

    for leg in &active_mission.legs {
        // Apply dispersed departure Δv
        let dep_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.departure_maneuver.dv_ric_km_s, disp, rng)?
        } else {
            leg.departure_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &dep_dv)?;
        total_dv += dep_dv.norm();

        // Propagate chief + deputy through this leg
        let chief_traj = nyx_propagate_segment(
            &chief_state,
            leg.tof_s,
            traj_steps,
            input.chief_config,
            dynamics_template.clone(),
            input.almanac,
        )?;
        let deputy_traj = nyx_propagate_segment(
            &deputy_state,
            leg.tof_s,
            traj_steps,
            sample_deputy_config,
            dynamics_template.clone(),
            input.almanac,
        )?;

        // Extract final states before consuming trajectories into safety pairs.
        chief_state = chief_traj
            .last()
            .map(|ts| ts.state.clone())
            .ok_or(CoreMonteCarloError::EmptyEnsemble)?;
        deputy_state = deputy_traj
            .last()
            .map(|ts| ts.state.clone())
            .ok_or(CoreMonteCarloError::EmptyEnsemble)?;

        // Skip t=0 sample from each leg's safety analysis: at the maneuver
        // instant, positions haven't separated yet — distance is physically
        // meaningless (consistent with validation.rs build_leg_comparison_points).
        for (idx, (c_entry, d_entry)) in chief_traj.into_iter().zip(deputy_traj).enumerate() {
            if idx > 0 {
                safety_pairs.push(ChiefDeputySnapshot {
                    elapsed_s: elapsed_total_s + c_entry.elapsed_s,
                    chief: c_entry.state,
                    deputy: d_entry.state,
                });
            }
        }

        // Apply dispersed arrival Δv
        let arr_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.arrival_maneuver.dv_ric_km_s, disp, rng)?
        } else {
            leg.arrival_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &arr_dv)?;
        total_dv += arr_dv.norm();

        // Compute miss distance at waypoint arrival
        let ric_rel = eci_to_ric_relative(&chief_state, &deputy_state)?;
        let miss = (ric_rel.position_ric_km - leg.to_position_ric_km).norm();
        waypoint_miss_km.push(miss);

        elapsed_total_s += leg.tof_s;
    }

    Ok(LegPropagationResult {
        total_dv_km_s: total_dv,
        waypoint_miss_km,
        safety_pairs,
    })
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
/// - [`CoreMonteCarloError::NegativeSma`] / [`CoreMonteCarloError::InvalidEccentricity`]
///   if state dispersion produces an unbound orbit.
/// - [`CoreMonteCarloError::NegativeSigma`] / [`CoreMonteCarloError::NegativeHalfWidth`]
///   if dispersion parameters are invalid.
/// - [`CoreMonteCarloError::EmptyEnsemble`] if a nyx trajectory segment is empty.
/// - Propagation or nyx bridge errors from bridge functions.
#[allow(clippy::similar_names)]
pub(crate) fn run_single_sample(
    input: &MonteCarloInput<'_>,
    index: u32,
    master_seed: u64,
) -> Result<SampleOutput, MonteCarloError> {
    let config = input.config;
    let mut rng = ChaCha20Rng::seed_from_u64(master_seed.wrapping_add(u64::from(index)));

    // Phase 1: Disperse deputy initial state + validate bound orbit
    let dispersed_deputy = disperse_deputy_state(
        input.initial_chief,
        input.initial_deputy,
        config.dispersions.state.as_ref(),
        index,
        &mut rng,
    )?;

    // Phase 2: Optionally disperse spacecraft properties
    let sample_deputy_config = disperse_spacecraft(
        input.deputy_config,
        config.dispersions.spacecraft.as_ref(),
        &mut rng,
    )?;

    // Phase 3: Determine mission plan (open-loop nominal vs closed-loop retargeted)
    let (mission_to_use, converged) = match config.mode {
        MonteCarloMode::OpenLoop => (None, true),
        MonteCarloMode::ClosedLoop => {
            match retarget_from_dispersed(
                input.initial_chief,
                &dispersed_deputy,
                input.nominal_mission,
                input.mission_config,
                input.propagator,
            ) {
                Ok(retargeted) => (Some(retargeted), true),
                Err(_) => (None, false),
            }
        }
    };
    let active_mission = mission_to_use.as_ref().unwrap_or(input.nominal_mission);

    // Phase 4: Propagate through legs with dispersed maneuvers
    let prop_result = propagate_dispersed_legs(
        input,
        dispersed_deputy,
        active_mission,
        &sample_deputy_config,
        &mut rng,
    )?;

    // Phase 5: Safety analysis + result packaging
    // Note: nyx provides osculating ECI states, so ROE (and e/i separation)
    // are computed from osculating elements. The passive safety metric
    // (D'Amico Eq. 2.22) is orbit-averaged and designed for mean elements;
    // osculating input adds short-period noise but does not affect operational
    // safety metrics (3D distance, R/C distance).
    let safety_states = build_nyx_safety_states(&prop_result.safety_pairs)?;
    let safety = analyze_trajectory_safety(&safety_states).ok();

    Ok(SampleOutput {
        result: SampleResult {
            index,
            total_dv_km_s: prop_result.total_dv_km_s,
            safety,
            waypoint_miss_km: prop_result.waypoint_miss_km,
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
            velocity_ric_km_s: Some(leg.target_velocity_ric_km_s),
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
/// Returns [`CoreMonteCarloError::EmptyEnsemble`] if `compute_percentile_stats`
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
    let num_waypoints = samples.first().map_or(0, |s| s.waypoint_miss_km.len());
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
                Err(MonteCarloError::Core(CoreMonteCarloError::EmptyEnsemble)) => {
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

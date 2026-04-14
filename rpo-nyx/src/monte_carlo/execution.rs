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
use rpo_core::propagation::propagator::PropagationModel;
use rpo_core::types::{DepartureState, SpacecraftConfig, StateVector};

use nyx_space::md::prelude::SpacecraftDynamics;

use crate::nyx_bridge::{
    apply_impulse, build_nyx_safety_states, nyx_propagate_segment,
    ChiefDeputySnapshot,
};

use super::sampling::{disperse_maneuver, sample_distribution};
use super::statistics::{compute_dispersion_envelope, compute_percentile_stats};
use super::types::{MonteCarloInput, SampleTrajectorySummary};
use super::MonteCarloError;

/// Result of a single MC sample execution (internal).
pub(crate) struct SampleOutput {
    /// Lightweight result for the report.
    pub(crate) result: SampleResult,
    /// Projected trajectory summary for dispersion envelope computation.
    pub(crate) trajectory: SampleTrajectorySummary,
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
    dynamics: &SpacecraftDynamics,
) -> Result<LegPropagationResult, MonteCarloError> {
    let maneuver_disp = input.config.dispersions.maneuver.as_ref();
    let traj_steps = input.config.trajectory_steps;

    let mut chief_state = input.initial_chief.clone();
    let mut deputy_state = dispersed_deputy;
    let mut total_dv = 0.0_f64;
    let mut waypoint_miss_km = Vec::with_capacity(active_mission.legs.len());
    let mut elapsed_total_s = 0.0_f64;
    let mut safety_pairs: Vec<ChiefDeputySnapshot> = Vec::with_capacity(
        traj_steps as usize * active_mission.legs.len(), // u32 → usize: always safe (usize ≥ 32 bits)
    );

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
            dynamics.clone(),
            input.almanac,
        )?;
        let deputy_traj = nyx_propagate_segment(
            &deputy_state,
            leg.tof_s,
            traj_steps,
            sample_deputy_config,
            dynamics.clone(),
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
    dynamics: &SpacecraftDynamics,
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
        dynamics,
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
        trajectory: SampleTrajectorySummary::from_trajectory(&safety_states),
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
/// - `summaries.len() == samples.len()`.
/// - `total_num_samples` is the total number of MC samples attempted (including failures).
///
/// # Errors
/// Returns [`CoreMonteCarloError::EmptyEnsemble`] if `compute_percentile_stats`
/// fails on an empty Δv vector (should not occur if `samples` is non-empty).
pub(crate) fn collect_ensemble_statistics(
    samples: &[SampleResult],
    summaries: &[SampleTrajectorySummary],
    config: &MonteCarloConfig,
    total_num_samples: u32,
    safety_config: Option<&SafetyConfig>,
) -> Result<EnsembleStatistics, MonteCarloError> {
    let mut total_dvs: Vec<f64> = samples.iter().map(|s| s.total_dv_km_s).collect();

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

    let total_dv_stats = compute_percentile_stats(&mut total_dvs)?;

    let min_rc_stats = if min_rc_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&mut min_rc_values)?)
    };
    let min_3d_stats = if min_3d_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&mut min_3d_values)?)
    };
    let min_ei_stats = if min_ei_values.is_empty() {
        None
    } else {
        Some(compute_percentile_stats(&mut min_ei_values)?)
    };

    // Per-waypoint miss distance statistics
    let num_waypoints = samples.first().map_or(0, |s| s.waypoint_miss_km.len());
    let mut waypoint_miss_stats = Vec::with_capacity(num_waypoints);
    for wp_idx in 0..num_waypoints {
        let mut misses: Vec<f64> = samples
            .iter()
            .filter_map(|s| s.waypoint_miss_km.get(wp_idx).copied())
            .collect();
        if misses.is_empty() {
            waypoint_miss_stats.push(None);
        } else {
            // compute_percentile_stats filters NaN internally; if all values
            // are non-finite, fall back to None
            match compute_percentile_stats(&mut misses) {
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
    let dispersion_envelope = compute_dispersion_envelope(summaries, config.trajectory_steps);

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

#[cfg(test)]
mod tests {
    use super::{disperse_deputy_state, disperse_spacecraft};
    use hifitime::Epoch;
    use nalgebra::Vector3;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;
    use rpo_core::constants::MIN_SPACECRAFT_MASS_KG;
    use rpo_core::mission::monte_carlo::{
        Distribution, SpacecraftDispersion, StateDispersion,
    };
    use rpo_core::types::{SpacecraftConfig, StateVector};

    /// Pass-through results must match the nominal input exactly (no arithmetic).
    const PASSTHROUGH_TOL: f64 = 0.0;

    /// Position sigma (km) for a small Gaussian perturbation kept well within
    /// the HCW linearization regime for a 400-km chief (~10 m).
    const POS_SIGMA_KM: f64 = 0.010;

    /// Velocity sigma (km/s) paired with `POS_SIGMA_KM` (~0.1 mm/s). Small
    /// enough that the dispersed deputy remains a bound orbit for any seed.
    const VEL_SIGMA_KM_S: f64 = 1.0e-7;

    /// Loose 10-sigma upper bound on a single Gaussian draw. Used to confirm
    /// the output moved *some* non-trivial amount without asserting a specific
    /// realization (tests must be seed-independent in their semantics).
    const TEN_SIGMA: f64 = 10.0;

    /// Build a simple circular ~400 km chief state (ISS-like) in ECI.
    fn chief_state() -> StateVector {
        StateVector {
            epoch: Epoch::from_gregorian_utc_hms(2025, 1, 1, 0, 0, 0),
            position_eci_km: Vector3::new(6778.0, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, 7.668, 0.0),
        }
    }

    /// Deputy nominally 100 m in-track behind the chief (well within the HCW
    /// linearization regime).
    fn deputy_state() -> StateVector {
        let chief = chief_state();
        StateVector {
            epoch: chief.epoch,
            position_eci_km: chief.position_eci_km + Vector3::new(0.0, 0.1, 0.0),
            velocity_eci_km_s: chief.velocity_eci_km_s,
        }
    }

    /// Representative LEO spacecraft: 500 kg, 2.5 m² drag/SRP area.
    fn nominal_spacecraft() -> SpacecraftConfig {
        SpacecraftConfig {
            dry_mass_kg: 500.0,
            drag_area_m2: 2.5,
            coeff_drag: 2.2,
            srp_area_m2: 2.5,
            coeff_reflectivity: 1.3,
        }
    }

    fn gaussian(sigma: f64) -> Distribution {
        Distribution::Gaussian { sigma }
    }

    fn state_dispersion(pos_sigma_km: f64, vel_sigma_km_s: f64) -> StateDispersion {
        StateDispersion {
            position_radial_km: gaussian(pos_sigma_km),
            position_intrack_km: gaussian(pos_sigma_km),
            position_crosstrack_km: gaussian(pos_sigma_km),
            velocity_radial_km_s: gaussian(vel_sigma_km_s),
            velocity_intrack_km_s: gaussian(vel_sigma_km_s),
            velocity_crosstrack_km_s: gaussian(vel_sigma_km_s),
        }
    }

    /// `disperse_deputy_state` with `state_disp = None` returns the nominal
    /// deputy bitwise-unchanged (pass-through is a zero-arithmetic path).
    #[test]
    fn disperse_deputy_state_no_dispersion_is_passthrough() {
        let chief = chief_state();
        let deputy = deputy_state();
        let mut rng = ChaCha20Rng::seed_from_u64(1);

        let out = disperse_deputy_state(&chief, &deputy, None, 0, &mut rng)
            .expect("pass-through must not fail for a valid nominal deputy");

        assert_eq!(out.epoch, deputy.epoch);
        for i in 0..3 {
            assert!(
                (out.position_eci_km[i] - deputy.position_eci_km[i]).abs() <= PASSTHROUGH_TOL,
                "position component {i} drifted on pass-through",
            );
            assert!(
                (out.velocity_eci_km_s[i] - deputy.velocity_eci_km_s[i]).abs() <= PASSTHROUGH_TOL,
                "velocity component {i} drifted on pass-through",
            );
        }
    }

    /// With a small non-zero Gaussian sigma the dispersed deputy differs from
    /// the nominal, and the perturbation magnitude lies within a generous
    /// 10-sigma band on any single draw.
    #[test]
    fn disperse_deputy_state_applies_gaussian_dispersion() {
        let chief = chief_state();
        let deputy = deputy_state();
        let disp = state_dispersion(POS_SIGMA_KM, VEL_SIGMA_KM_S);
        let mut rng = ChaCha20Rng::seed_from_u64(42);

        let out = disperse_deputy_state(&chief, &deputy, Some(&disp), 0, &mut rng)
            .expect("small Gaussian dispersion must yield a bound orbit");

        let dpos = (out.position_eci_km - deputy.position_eci_km).norm();
        let dvel = (out.velocity_eci_km_s - deputy.velocity_eci_km_s).norm();

        // A ChaCha20(42) draw is non-zero for a non-degenerate Gaussian, so
        // the dispersed state must actually differ from the nominal.
        assert!(dpos > 0.0, "expected a non-zero position perturbation");
        assert!(dvel > 0.0, "expected a non-zero velocity perturbation");

        // Three independent 1-sigma draws combine into ||delta|| <= 3*sigma
        // in expectation; the 10*sigma ceiling is a very loose upper bound
        // that any reasonable single draw must satisfy.
        let pos_ceiling_km = TEN_SIGMA * (3.0_f64).sqrt() * POS_SIGMA_KM;
        let vel_ceiling_km_s = TEN_SIGMA * (3.0_f64).sqrt() * VEL_SIGMA_KM_S;
        assert!(
            dpos < pos_ceiling_km,
            "|dpos| = {dpos} km exceeded 10-sigma ceiling {pos_ceiling_km} km",
        );
        assert!(
            dvel < vel_ceiling_km_s,
            "|dvel| = {dvel} km/s exceeded 10-sigma ceiling {vel_ceiling_km_s} km/s",
        );
    }

    /// Identical seeds must yield identical dispersed states — same sample
    /// index, same chief/deputy, same dispersion spec → bitwise match.
    #[test]
    fn disperse_deputy_state_is_deterministic_for_fixed_seed() {
        let chief = chief_state();
        let deputy = deputy_state();
        let disp = state_dispersion(POS_SIGMA_KM, VEL_SIGMA_KM_S);

        let mut rng_a = ChaCha20Rng::seed_from_u64(12345);
        let mut rng_b = ChaCha20Rng::seed_from_u64(12345);

        let out_a = disperse_deputy_state(&chief, &deputy, Some(&disp), 7, &mut rng_a)
            .expect("run A must succeed");
        let out_b = disperse_deputy_state(&chief, &deputy, Some(&disp), 7, &mut rng_b)
            .expect("run B must succeed");

        assert_eq!(out_a.epoch, out_b.epoch);
        assert_eq!(out_a.position_eci_km, out_b.position_eci_km);
        assert_eq!(out_a.velocity_eci_km_s, out_b.velocity_eci_km_s);
    }

    /// `disperse_spacecraft` with `sc_disp = None` returns the nominal config
    /// field-for-field unchanged.
    #[test]
    fn disperse_spacecraft_no_dispersion_is_passthrough() {
        let nominal = nominal_spacecraft();
        let mut rng = ChaCha20Rng::seed_from_u64(2);

        let out = disperse_spacecraft(&nominal, None, &mut rng)
            .expect("pass-through must not fail");

        assert!((out.dry_mass_kg - nominal.dry_mass_kg).abs() <= PASSTHROUGH_TOL);
        assert!((out.drag_area_m2 - nominal.drag_area_m2).abs() <= PASSTHROUGH_TOL);
        assert!((out.coeff_drag - nominal.coeff_drag).abs() <= PASSTHROUGH_TOL);
        assert!((out.srp_area_m2 - nominal.srp_area_m2).abs() <= PASSTHROUGH_TOL);
        assert!((out.coeff_reflectivity - nominal.coeff_reflectivity).abs() <= PASSTHROUGH_TOL);
    }

    /// Every sample must respect the physical floors regardless of the draw:
    /// `drag_area_m2 >= 0` and `dry_mass_kg >= MIN_SPACECRAFT_MASS_KG`. With
    /// sigmas much larger than the nominals, a significant fraction of draws
    /// hits the clamp, so the property is genuinely exercised.
    #[test]
    fn disperse_spacecraft_clamps_to_physical_bounds() {
        let near_floor = SpacecraftConfig {
            dry_mass_kg: 0.2,
            drag_area_m2: 0.5,
            coeff_drag: 2.2,
            srp_area_m2: 2.5,
            coeff_reflectivity: 1.3,
        };
        let disp = SpacecraftDispersion {
            coeff_drag: gaussian(0.5),
            drag_area_m2: gaussian(100.0),
            dry_mass_kg: gaussian(10.0),
        };

        for seed in 0..200_u64 {
            let mut rng = ChaCha20Rng::seed_from_u64(seed);
            let out = disperse_spacecraft(&near_floor, Some(&disp), &mut rng)
                .expect("dispersion must succeed for valid sigma");
            assert!(
                out.drag_area_m2 >= 0.0,
                "drag_area_m2 = {} < 0 for seed {seed}",
                out.drag_area_m2,
            );
            assert!(
                out.dry_mass_kg >= MIN_SPACECRAFT_MASS_KG,
                "dry_mass_kg = {} < {MIN_SPACECRAFT_MASS_KG} for seed {seed}",
                out.dry_mass_kg,
            );
            // Untouched fields must come through via `..*nominal`.
            assert!((out.srp_area_m2 - near_floor.srp_area_m2).abs() <= PASSTHROUGH_TOL);
            assert!(
                (out.coeff_reflectivity - near_floor.coeff_reflectivity).abs() <= PASSTHROUGH_TOL,
            );
        }
    }

    /// Identical seeds must yield identical dispersed spacecraft configs on
    /// all three perturbed fields.
    #[test]
    fn disperse_spacecraft_is_deterministic_for_fixed_seed() {
        let nominal = nominal_spacecraft();
        let disp = SpacecraftDispersion {
            coeff_drag: gaussian(0.1),
            drag_area_m2: gaussian(0.2),
            dry_mass_kg: gaussian(5.0),
        };

        let mut rng_a = ChaCha20Rng::seed_from_u64(9001);
        let mut rng_b = ChaCha20Rng::seed_from_u64(9001);

        let out_a = disperse_spacecraft(&nominal, Some(&disp), &mut rng_a)
            .expect("run A must succeed");
        let out_b = disperse_spacecraft(&nominal, Some(&disp), &mut rng_b)
            .expect("run B must succeed");

        assert_eq!(out_a.coeff_drag.to_bits(), out_b.coeff_drag.to_bits());
        assert_eq!(out_a.drag_area_m2.to_bits(), out_b.drag_area_m2.to_bits());
        assert_eq!(out_a.dry_mass_kg.to_bits(), out_b.dry_mass_kg.to_bits());
    }
}

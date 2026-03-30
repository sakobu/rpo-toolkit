//! Core nyx validation orchestrator: per-leg and mission-level trajectory comparison.

use std::sync::Arc;

use anise::constants::frames::EARTH_J2000 as ANISE_EARTH_J2000;
use anise::prelude::Almanac;
use nalgebra::Vector3;
use serde::Serialize;

use crate::elements::eci_ric_dcm::eci_to_ric_relative;
use crate::mission::safety::analyze_trajectory_safety;
use crate::propagation::nyx_bridge::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
    query_anise_eclipse, state_to_orbit, ChiefDeputySnapshot, NyxBridgeError, TimedState,
};
use crate::propagation::propagator::PropagatedState;
use crate::types::{SpacecraftConfig, StateVector};

use crate::mission::types::{
    ManeuverLeg, ValidationPoint, ValidationReport, WaypointMission,
};
use super::eclipse::{EclipseSample, build_eclipse_validation};
use super::errors::ValidationError;
use super::statistics::{find_closest_analytical_ric, compute_report_statistics};

/// Configuration for nyx full-physics validation.
///
/// Groups sampling density and spacecraft properties that are shared
/// across [`validate_leg_nyx`] and [`validate_mission_nyx`].
#[derive(Debug, Clone, Copy)]
pub struct ValidationConfig {
    /// Number of intermediate comparison samples per leg (0 = final only).
    pub samples_per_leg: u32,
    /// Chief spacecraft properties (mass, drag area, SRP area, Cd, Cr).
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft properties.
    pub deputy_config: SpacecraftConfig,
}

/// Output from validating a single mission leg against nyx full-physics.
///
/// Contains per-sample analytical vs numerical comparison points and updated
/// chief/deputy ECI states for threading into the next leg.
///
/// All `StateVector` fields use ECI J2000 frame (`position_eci_km`,
/// `velocity_eci_km_s`). When serialized for the API wire format,
/// coordinates remain in ECI.
///
/// Used by the API server for per-leg progress streaming during validation.
#[derive(Debug, Clone, Serialize)]
pub struct LegValidationOutput {
    /// Per-sample analytical vs numerical RIC comparison points.
    pub points: Vec<ValidationPoint>,
    /// Chief/deputy ECI snapshot pairs for safety analysis.
    pub safety_pairs: Vec<ChiefDeputySnapshot>,
    /// Updated chief ECI state at end of this leg.
    pub chief_state_after: StateVector,
    /// Updated deputy ECI state at end of this leg (arrival delta-v applied).
    pub deputy_state_after: StateVector,
}

/// Output from comparing nyx-propagated states against analytical trajectory for one leg.
struct LegComparisonOutput {
    /// Per-sample validation comparison points.
    points: Vec<ValidationPoint>,
    /// Chief/deputy ECI snapshot pairs for safety analysis.
    safety_pairs: Vec<ChiefDeputySnapshot>,
    /// Eclipse samples for validation (empty if eclipse validation disabled).
    eclipse_samples: Vec<EclipseSample>,
}

/// Extract final chief/deputy ECI states from propagation results and apply arrival impulse.
///
/// Used at leg boundaries to thread state into the next leg or return updated
/// states from per-leg validation.
fn advance_leg_states(
    chief_results: &[TimedState],
    deputy_results: &[TimedState],
    arrival_dv_ric_km_s: &Vector3<f64>,
) -> Result<(StateVector, StateVector), ValidationError> {
    let chief = chief_results
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();
    let deputy_coast_end = deputy_results
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();
    let deputy = apply_impulse(&deputy_coast_end, &chief, arrival_dv_ric_km_s)?;
    Ok((chief, deputy))
}

/// Validate a single mission leg against nyx full-physics propagation.
///
/// Propagates chief and deputy through the leg using nyx full-physics dynamics,
/// compares against the analytical trajectory, and returns comparison points plus
/// updated ECI states for threading to the next leg.
///
/// This function does **not** collect eclipse samples; eclipse validation is only
/// available through the mission-level [`validate_mission_nyx`] orchestrator.
///
/// # Invariants
/// - `chief_state` and `deputy_state` must be valid bound ECI states
/// - `almanac` must contain Earth frame data and planetary ephemerides
///
/// # Errors
/// Returns [`ValidationError`] if propagation, impulse application, or frame conversion fails.
pub fn validate_leg_nyx(
    leg: &ManeuverLeg,
    chief_state: &StateVector,
    deputy_state: &StateVector,
    config: &ValidationConfig,
    almanac: &Arc<Almanac>,
    cumulative_time_s: f64,
) -> Result<LegValidationOutput, ValidationError> {
    let (chief_results, deputy_results) = propagate_leg_parallel(
        chief_state,
        deputy_state,
        leg,
        config.samples_per_leg,
        &config.chief_config,
        &config.deputy_config,
        almanac,
    )?;

    // Build comparison points (no eclipse — that stays in validate_mission_nyx)
    let leg_output = build_leg_comparison_points(
        &chief_results,
        &deputy_results,
        &leg.trajectory,
        cumulative_time_s,
        None, // no eclipse frame for per-leg API
        almanac,
    )?;

    let (chief_after, deputy_after) =
        advance_leg_states(&chief_results, &deputy_results, &leg.arrival_maneuver.dv_ric_km_s)?;

    Ok(LegValidationOutput {
        points: leg_output.points,
        safety_pairs: leg_output.safety_pairs,
        chief_state_after: chief_after,
        deputy_state_after: deputy_after,
    })
}

/// Build comparison points for a single leg of the mission.
///
/// Compares nyx-propagated chief/deputy states against analytical trajectory,
/// collects safety pairs and eclipse samples for downstream analysis.
fn build_leg_comparison_points(
    chief_results: &[TimedState],
    deputy_results: &[TimedState],
    trajectory: &[PropagatedState],
    cumulative_time_s: f64,
    earth_frame: Option<anise::prelude::Frame>,
    almanac: &Arc<Almanac>,
) -> Result<LegComparisonOutput, ValidationError> {
    let mut points = Vec::with_capacity(chief_results.len());
    let mut safety_pairs = Vec::with_capacity(chief_results.len());
    let mut eclipse_samples = Vec::new();

    for (idx, (chief_sample, deputy_sample)) in
        chief_results.iter().zip(deputy_results.iter()).enumerate()
    {
        let numerical_ric = eci_to_ric_relative(&chief_sample.state, &deputy_sample.state)?;
        let elapsed = cumulative_time_s + chief_sample.elapsed_s;
        let analytical_ric = find_closest_analytical_ric(trajectory, chief_sample.elapsed_s);

        let pos_err = (numerical_ric.position_ric_km - analytical_ric.position_ric_km).norm();
        let vel_err =
            (numerical_ric.velocity_ric_km_s - analytical_ric.velocity_ric_km_s).norm();

        // Skip t=0 sample from safety: at the maneuver instant, positions
        // haven't separated yet — distance is physically meaningless
        // (consistent with monte_carlo/execution.rs).
        if idx > 0 {
            safety_pairs.push(ChiefDeputySnapshot {
                elapsed_s: elapsed,
                chief: chief_sample.state.clone(),
                deputy: deputy_sample.state.clone(),
            });
        }
        points.push(ValidationPoint {
            elapsed_s: elapsed,
            analytical_ric,
            numerical_ric,
            position_error_km: pos_err,
            velocity_error_km_s: vel_err,
        });

        // Collect eclipse samples for validation
        if let Some(ef) = earth_frame {
            let chief_orbit = state_to_orbit(&chief_sample.state);
            if let Ok((eclipse_pct, sun_eci)) = query_anise_eclipse(chief_orbit, ef, almanac) {
                eclipse_samples.push(EclipseSample {
                    elapsed_s: elapsed,
                    epoch: chief_sample.state.epoch,
                    numerical_pct: eclipse_pct,
                    anise_sun_eci_km: sun_eci,
                    chief_eci_km: chief_sample.state.position_eci_km,
                });
            }
        }
    }
    Ok(LegComparisonOutput {
        points,
        safety_pairs,
        eclipse_samples,
    })
}

/// Propagate chief and deputy through a single leg in parallel via rayon.
///
/// Applies the leg's departure impulse to the deputy, then propagates both
/// vehicles through nyx full-physics dynamics concurrently using [`rayon::join`].
///
/// # Invariants
/// - `chief_state` and `deputy_state` must represent valid orbits.
/// - `almanac` must contain required frames and force models.
///
/// # Errors
/// - [`ValidationError`] if impulse application, dynamics setup, or propagation fails.
fn propagate_leg_parallel(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    leg: &ManeuverLeg,
    samples_per_leg: u32,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<(Vec<TimedState>, Vec<TimedState>), ValidationError> {
    // Build dynamics once; clone is cheap (Arc ref-count bumps).
    let dynamics = build_full_physics_dynamics(almanac)?;

    let (chief_result, deputy_result) = rayon::join(
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                chief_state,
                leg.tof_s,
                samples_per_leg,
                chief_config,
                dynamics.clone(),
                almanac,
            )?)
        },
        || -> Result<Vec<TimedState>, ValidationError> {
            let deputy_post_burn =
                apply_impulse(deputy_state, chief_state, &leg.departure_maneuver.dv_ric_km_s)?;
            Ok(nyx_propagate_segment(
                &deputy_post_burn,
                leg.tof_s,
                samples_per_leg,
                deputy_config,
                dynamics.clone(),
                almanac,
            )?)
        },
    );
    Ok((chief_result?, deputy_result?))
}

/// Validate a waypoint mission against nyx full-physics propagation.
///
/// Propagates chief and deputy through each mission leg using nyx with full
/// force models (J2 harmonics, drag, SRP, Sun/Moon third-body), applies
/// impulsive Δv maneuvers at burn epochs, and compares the resulting RIC
/// trajectory against the analytical trajectory from the mission planner.
///
/// # Algorithm
/// 1. For each leg: propagate chief through nyx, apply departure Δv to deputy,
///    propagate deputy through nyx, sample and compare RIC states.
/// 2. At leg boundaries: apply arrival Δv, advance to next leg.
/// 3. Build safety states from accumulated chief/deputy pairs.
/// 4. Compute aggregate statistics and return report.
///
/// # Invariants
/// - `mission.legs` must be non-empty (at least one maneuver leg)
/// - `chief_initial` and `deputy_initial` must be valid bound ECI states at mission start epoch
/// - `almanac` must contain Earth frame data (`IAU_EARTH`) and planetary ephemerides
///
/// # Arguments
/// * `mission` — Analytical waypoint mission (from `plan_waypoint_mission`)
/// * `chief_initial` — Chief ECI state at mission start
/// * `deputy_initial` — Deputy ECI state at mission start
/// * `config` — Validation settings (sampling density, spacecraft properties)
/// * `almanac` — Full-physics ANISE almanac (from `load_full_almanac`)
///
/// # Errors
/// Returns [`ValidationError`] if the mission has no legs, almanac frame lookup
/// fails, dynamics setup fails, propagation fails, or safety analysis fails.
pub fn validate_mission_nyx(
    mission: &WaypointMission,
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    config: &ValidationConfig,
    almanac: &Arc<Almanac>,
) -> Result<ValidationReport, ValidationError> {
    if mission.legs.is_empty() {
        return Err(ValidationError::EmptyTrajectory);
    }

    let mut chief_state = chief_initial.clone();
    let mut deputy_state = deputy_initial.clone();
    let mut cumulative_time = 0.0_f64;
    let mut leg_points = Vec::with_capacity(mission.legs.len());
    let estimated_total_samples = usize::try_from(config.samples_per_leg).unwrap_or(50)
        * mission.legs.len();
    let mut safety_pairs: Vec<ChiefDeputySnapshot> =
        Vec::with_capacity(estimated_total_samples);
    let eclipse_enabled = mission.eclipse.is_some();
    let mut eclipse_samples: Vec<EclipseSample> = if eclipse_enabled {
        Vec::with_capacity(estimated_total_samples)
    } else {
        Vec::new()
    };

    // Get Earth frame with radius info for ANISE eclipse queries
    let earth_frame = if eclipse_enabled {
        Some(
            almanac
                .frame_info(ANISE_EARTH_J2000)
                .map_err(|e| {
                    ValidationError::NyxBridge(Box::new(NyxBridgeError::FrameLookup {
                        source: e,
                    }))
                })?,
        )
    } else {
        None
    };

    for leg in &mission.legs {
        let (chief_results, deputy_results) = propagate_leg_parallel(
            &chief_state,
            &deputy_state,
            leg,
            config.samples_per_leg,
            &config.chief_config,
            &config.deputy_config,
            almanac,
        )?;

        // Build comparison points for this leg
        let leg_output = build_leg_comparison_points(
            &chief_results,
            &deputy_results,
            &leg.trajectory,
            cumulative_time,
            earth_frame,
            almanac,
        )?;
        safety_pairs.extend(leg_output.safety_pairs);
        eclipse_samples.extend(leg_output.eclipse_samples);
        leg_points.push(leg_output.points);

        (chief_state, deputy_state) =
            advance_leg_states(&chief_results, &deputy_results, &leg.arrival_maneuver.dv_ric_km_s)?;
        cumulative_time += leg.tof_s;
    }

    // Compute safety from nyx trajectory.
    // analyze_trajectory_safety processes a flat trajectory and always leaves
    // leg indices at 0. Derive the correct leg indices from elapsed time.
    let nyx_safety_states = build_nyx_safety_states(&safety_pairs)?;
    let numerical_safety = {
        let mut s = analyze_trajectory_safety(&nyx_safety_states)?;
        let mut found_3d = false;
        let mut found_rc = false;
        let mut cumulative = 0.0_f64;
        for (i, leg) in mission.legs.iter().enumerate() {
            cumulative += leg.tof_s;
            if !found_3d && s.operational.min_3d_elapsed_s <= cumulative {
                s.operational.min_3d_leg_index = i;
                found_3d = true;
            }
            if !found_rc && s.operational.min_rc_elapsed_s <= cumulative {
                s.operational.min_rc_leg_index = i;
                found_rc = true;
            }
            if found_3d && found_rc {
                break;
            }
        }
        s
    };

    // Compute eclipse validation if eclipse data is present
    let eclipse_validation = mission.eclipse.as_ref().and_then(|eclipse_data| {
        build_eclipse_validation(eclipse_data, &eclipse_samples)
    });

    // Compute statistics
    let stats = compute_report_statistics(&leg_points);

    Ok(ValidationReport {
        leg_points,
        max_position_error_km: stats.max_position_error_km,
        mean_position_error_km: stats.mean_position_error_km,
        rms_position_error_km: stats.rms_position_error_km,
        max_velocity_error_km_s: stats.max_velocity_error_km_s,
        analytical_safety: mission.safety,
        numerical_safety,
        chief_config: config.chief_config,
        deputy_config: config.deputy_config,
        eclipse_validation,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::propagation::nyx_bridge;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{
        iss_like_elements, test_epoch, DMF_RATE_NONZERO_LOWER_BOUND, DMF_RATE_UPPER_BOUND,
    };
    use crate::types::SpacecraftConfig;

    // =========================================================================
    // Full-Physics Integration Tests
    // =========================================================================

    /// Position tolerance for a single-leg transfer (~1 orbit).
    /// Unmodeled perturbations (drag, SRP, 3rd-body) contribute ~50m total;
    /// 10× margin gives 0.5 km.
    const FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM: f64 = 0.5;

    /// Position tolerance for a multi-leg mission (~3 legs × 0.75 period each).
    /// ~3× single-leg tolerance plus maneuver state mismatch across legs.
    const FULL_PHYSICS_MULTI_LEG_POS_TOL_KM: f64 = 3.0;

    /// Position tolerance for drag STM vs nyx comparison.
    /// DMF linear fit error + unmodeled SRP/3rd-body over 1 orbit.
    const DRAG_STM_VS_NYX_POS_TOL_KM: f64 = 1.0;

    /// Relative tolerance for R/C separation comparison (analytical vs numerical).
    /// Different sampling density + mean/osculating offset justify 50%.
    const SAFETY_RC_RELATIVE_TOL: f64 = 0.50;

    /// Absolute tolerance for 3D distance comparison (km).
    /// Same effects as R/C; absolute because 3D distance can be small.
    const SAFETY_3D_ABSOLUTE_TOL_KM: f64 = 0.5;

    /// Relative tolerance for e/i vector separation comparison.
    /// ROE-level drift O(1e-5) over 1–3 orbits.
    const SAFETY_EI_RELATIVE_TOL: f64 = 0.50;

    /// Below this threshold (km), R/C separations are operationally "at V-bar"
    /// and relative-error comparison is not meaningful.
    const SAFETY_RC_NEAR_ZERO_KM: f64 = 0.01;

    /// Guard threshold for improvement ratio computation (km).
    /// When the J2-only error is below this threshold, the improvement ratio
    /// is numerically meaningless (division by near-zero). Skip the diagnostic.
    const IMPROVEMENT_RATIO_GUARD_KM: f64 = 1e-10;

    /// Below this threshold (km), e/i separation values are too small
    /// for a meaningful relative-error comparison.
    const SAFETY_EI_NEAR_ZERO_KM: f64 = 1e-6;

    /// Single-leg transfer with nonzero initial ROE and mixed-axis waypoint,
    /// validated against nyx full-physics propagation.
    ///
    /// Chief: ISS-like orbit. Deputy: ~300m-scale formation (nonzero dex, dey, dix).
    /// Waypoint: [0.5, 3.0, 1.0] RIC km, TOF = 0.8 orbital periods.
    /// Non-integer period avoids CW singularity at nt = 2π (rank-1 Φ_rv).
    /// 50 samples for detailed error characterization.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_full_physics_single_leg() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        // ~300m-scale formation: nonzero dex, dey, dix
        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.3 / a,
            dey: -0.2 / a,
            dix: 0.2 / a,
            diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &[waypoint], &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let val_config = super::ValidationConfig {
            samples_per_leg: 50,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let report = crate::mission::validation::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Per-point error logging (every 10th sample)
        for (leg_idx, points) in report.leg_points.iter().enumerate() {
            for (i, p) in points.iter().enumerate() {
                if i % 10 == 0 || i == points.len() - 1 {
                    eprintln!(
                        "  leg {leg_idx} sample {i:>3}: t={:.0}s  pos_err={:.4} km",
                        p.elapsed_s, p.position_error_km,
                    );
                }
            }
        }

        eprintln!(
            "Full-physics single-leg: max={:.4} km, mean={:.4} km, rms={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert_eq!(report.leg_points.len(), 1, "should have 1 leg");
        assert!(
            report.max_position_error_km < FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM,
            "max position error = {:.4} km (expected < {FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM})",
            report.max_position_error_km,
        );
        assert!(
            report.rms_position_error_km <= report.max_position_error_km,
            "RMS ({:.4}) should be <= max ({:.4})",
            report.rms_position_error_km,
            report.max_position_error_km,
        );
    }

    /// Multi-waypoint mission with nonzero initial ROE and mixed-axis waypoints,
    /// validated against nyx full-physics propagation.
    ///
    /// Chief: ISS-like orbit. Deputy: ~300m-scale formation.
    /// 3 waypoints with 0.75-period TOF each, spanning R/I/C axes.
    /// Per-leg error characterization and error-growth logging.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_full_physics_multi_waypoint() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.3 / a,
            dey: -0.2 / a,
            dix: 0.2 / a,
            diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.5, 3.0, 0.5),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(1.0, -2.0, 1.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let val_config = super::ValidationConfig {
            samples_per_leg: 50,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let report = crate::mission::validation::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Per-leg error characterization
        for (leg_idx, points) in report.leg_points.iter().enumerate() {
            let leg_max = points
                .iter()
                .map(|p| p.position_error_km)
                .fold(0.0_f64, f64::max);
            eprintln!("  leg {leg_idx}: max_pos_err = {leg_max:.4} km ({} samples)", points.len());
        }

        eprintln!(
            "Full-physics multi-waypoint: max={:.4} km, mean={:.4} km, rms={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert_eq!(report.leg_points.len(), 3, "should have 3 legs");
        assert!(
            report.max_position_error_km < FULL_PHYSICS_MULTI_LEG_POS_TOL_KM,
            "max position error = {:.4} km (expected < {FULL_PHYSICS_MULTI_LEG_POS_TOL_KM})",
            report.max_position_error_km,
        );
    }

    /// End-to-end drag STM validation: extract DMF rates from nyx, plan with
    /// J2+drag STM, validate against nyx full-physics propagation.
    ///
    /// 1. ISS-like orbit, deputy colocated (zero ROE)
    /// 2. Chief: SERVICER_500KG (B*=0.0044), Deputy: 200kg/2m² (B*=0.022, ~5× higher)
    /// 3. Extract DMF rates → DragConfig
    /// 4. Plan with J2DragStm: single V-bar waypoint [0,5,0], 0.8 periods
    ///    (non-integer period avoids CW singularity at nt = 2π)
    /// 5. Plan same with J2Stm for comparison
    /// 6. Validate both against nyx
    /// 7. Assert drag-aware error < tolerance; log improvement ratio
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_drag_stm_vs_nyx_drag() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone(); // colocated start

        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig {
            dry_mass_kg: 200.0,
            drag_area_m2: 2.0,
            ..SpacecraftConfig::SERVICER_500KG
        };

        // Step 1: Extract DMF rates
        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let drag = nyx_bridge::extract_dmf_rates(
            &chief_sv, &deputy_sv, &chief_config, &deputy_config, &almanac,
        )
        .expect("DMF extraction should succeed");

        eprintln!(
            "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot,
        );

        // Sanity: da_dot should be nonzero, negative, and physically reasonable
        assert!(
            drag.da_dot.abs() > DMF_RATE_NONZERO_LOWER_BOUND,
            "da_dot should be nonzero, got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot < 0.0,
            "da_dot should be negative (deputy decays faster), got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot.abs() < DMF_RATE_UPPER_BOUND,
            "da_dot = {:.2e} seems unreasonably large",
            drag.da_dot,
        );

        // Step 2: Plan missions with both propagators
        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();

        // J2+drag mission
        let drag_propagator = PropagationModel::J2DragStm { drag };
        let drag_mission =
            plan_waypoint_mission(&departure, &[waypoint.clone()], &config, &drag_propagator)
                .expect("drag mission planning should succeed");

        // J2-only mission (comparison baseline)
        let j2_propagator = PropagationModel::J2Stm;
        let j2_mission =
            plan_waypoint_mission(&departure, &[waypoint], &config, &j2_propagator)
                .expect("J2 mission planning should succeed");

        // Step 3: Validate both against nyx
        let val_config = super::ValidationConfig {
            samples_per_leg: 50,
            chief_config,
            deputy_config,
        };
        let drag_report = crate::mission::validation::validate_mission_nyx(
            &drag_mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &almanac,
        )
        .expect("drag validation should succeed");

        let j2_report = crate::mission::validation::validate_mission_nyx(
            &j2_mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &almanac,
        )
        .expect("J2 validation should succeed");

        eprintln!(
            "Drag STM vs nyx: max={:.4} km, mean={:.4} km, rms={:.4} km",
            drag_report.max_position_error_km,
            drag_report.mean_position_error_km,
            drag_report.rms_position_error_km,
        );
        eprintln!(
            "J2-only vs nyx:  max={:.4} km, mean={:.4} km, rms={:.4} km",
            j2_report.max_position_error_km,
            j2_report.mean_position_error_km,
            j2_report.rms_position_error_km,
        );

        // Diagnostic: improvement ratio (not hard-asserted — SRP/3rd-body may dominate)
        if j2_report.max_position_error_km > IMPROVEMENT_RATIO_GUARD_KM {
            let improvement = j2_report.max_position_error_km / drag_report.max_position_error_km;
            eprintln!("Drag/J2 improvement ratio: {improvement:.2}×");
            if improvement < 1.0 {
                eprintln!(
                    "  Note: drag STM did not improve over J2-only for this scenario. \
                     For short single-orbit transfers with colocated start, differential \
                     drag effect may be negligible vs unmodeled perturbations (SRP, 3rd-body)."
                );
            }
        }

        // Core assertion: drag-aware propagation should match nyx within tolerance
        assert!(
            drag_report.max_position_error_km < DRAG_STM_VS_NYX_POS_TOL_KM,
            "drag STM max error = {:.4} km (expected < {DRAG_STM_VS_NYX_POS_TOL_KM})",
            drag_report.max_position_error_km,
        );
    }

    /// Safety metrics comparison: analytical (ROE-based) vs numerical (nyx) safety.
    ///
    /// Chief: ISS-like. Deputy: formation with perpendicular e/i vectors for
    /// meaningful e/i separation (dex=0.5/a, diy=0.5/a).
    /// 3 waypoints with 0.75-period TOF, safety analysis enabled.
    /// Compares R/C separation, 3D distance, and e/i separation between
    /// analytical and numerical trajectories.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_safety_full_physics() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::mission::config::{MissionConfig, SafetyConfig};
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        // Perpendicular e/i vectors: dex along ex, diy along iy → meaningful separation
        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.5 / a,
            dey: 0.0,
            dix: 0.0,
            diy: 0.5 / a,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.5, 5.0, 0.5),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 2.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig {
            safety: Some(SafetyConfig::default()),
            ..MissionConfig::default()
        };
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let val_config = super::ValidationConfig {
            samples_per_leg: 50,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let report = crate::mission::validation::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Analytical safety must be present (we enabled it)
        let analytical = report
            .analytical_safety
            .expect("analytical safety should be present (SafetyConfig enabled)");
        let numerical = &report.numerical_safety;

        eprintln!("Safety comparison (analytical vs numerical):");
        eprintln!(
            "  R/C separation:  {:.4} km vs {:.4} km",
            analytical.operational.min_rc_separation_km, numerical.operational.min_rc_separation_km,
        );
        eprintln!(
            "  3D distance:     {:.4} km vs {:.4} km",
            analytical.operational.min_distance_3d_km, numerical.operational.min_distance_3d_km,
        );
        eprintln!(
            "  e/i separation:  {:.4} km vs {:.4} km",
            analytical.passive.min_ei_separation_km, numerical.passive.min_ei_separation_km,
        );
        eprintln!(
            "  |δe|:            {:.6} vs {:.6}",
            analytical.passive.de_magnitude, numerical.passive.de_magnitude,
        );
        eprintln!(
            "  |δi|:            {:.6} vs {:.6}",
            analytical.passive.di_magnitude, numerical.passive.di_magnitude,
        );
        eprintln!(
            "  e/i phase angle: {:.4} rad vs {:.4} rad",
            analytical.passive.ei_phase_angle_rad, numerical.passive.ei_phase_angle_rad,
        );

        // R/C separation: relative agreement within 50%
        // When both values are near zero (V-bar configuration), relative comparison
        // is meaningless — skip it.
        let rc_ref = analytical.operational.min_rc_separation_km.max(numerical.operational.min_rc_separation_km);
        if rc_ref > SAFETY_RC_NEAR_ZERO_KM {
            let rc_rel_err =
                (analytical.operational.min_rc_separation_km - numerical.operational.min_rc_separation_km).abs() / rc_ref;
            eprintln!("  R/C relative error: {rc_rel_err:.2}");
            assert!(
                rc_rel_err < SAFETY_RC_RELATIVE_TOL,
                "R/C separation relative error = {rc_rel_err:.2} (expected < {SAFETY_RC_RELATIVE_TOL})",
            );
        } else {
            eprintln!(
                "  R/C near-zero ({rc_ref:.4} km < {SAFETY_RC_NEAR_ZERO_KM}): \
                 skipping relative comparison"
            );
        }

        // 3D distance: absolute agreement within 0.5 km
        let dist_3d_err =
            (analytical.operational.min_distance_3d_km - numerical.operational.min_distance_3d_km).abs();
        eprintln!("  3D distance absolute error: {dist_3d_err:.4} km");
        assert!(
            dist_3d_err < SAFETY_3D_ABSOLUTE_TOL_KM,
            "3D distance error = {dist_3d_err:.4} km (expected < {SAFETY_3D_ABSOLUTE_TOL_KM})",
        );

        // e/i separation: relative agreement within 50%
        let ei_ref = analytical.passive.min_ei_separation_km.max(numerical.passive.min_ei_separation_km);
        if ei_ref > SAFETY_EI_NEAR_ZERO_KM {
            let ei_rel_err =
                (analytical.passive.min_ei_separation_km - numerical.passive.min_ei_separation_km).abs() / ei_ref;
            eprintln!("  e/i relative error: {ei_rel_err:.2}");
            assert!(
                ei_rel_err < SAFETY_EI_RELATIVE_TOL,
                "e/i separation relative error = {ei_rel_err:.2} (expected < {SAFETY_EI_RELATIVE_TOL})",
            );
        }
    }
}

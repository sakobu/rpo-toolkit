//! Mission-level validation orchestrator: assembles per-leg propagation,
//! pre-COLA baseline safety, and final [`ValidationReport`] construction.

use std::sync::Arc;

use anise::constants::frames::EARTH_J2000 as ANISE_EARTH_J2000;
use anise::prelude::Almanac;
use nyx_space::md::prelude::SpacecraftDynamics;

use rpo_core::mission::safety::analyze_trajectory_safety;
use rpo_core::mission::types::{
    ColaEffectivenessEntry, ManeuverLeg, ValidationReport, WaypointMission,
};
use rpo_core::types::{SpacecraftConfig, StateVector};

use super::super::eclipse::{build_eclipse_validation, EclipseSample};
use super::super::errors::ValidationError;
use super::super::statistics::{compute_leg_summaries, compute_report_statistics};
use super::cola::{ColaBurn, ColaValidationInput};
use super::leg::{
    advance_leg_states, build_leg_comparison_points, collect_cola_effectiveness,
    propagate_leg, propagate_leg_parallel, propagate_leg_with_cola, LegPropagationCtx,
    MidLegBurn,
};
use crate::nyx_bridge::{build_nyx_safety_states, ChiefDeputySnapshot, NyxBridgeError};

/// Configuration for nyx full-physics validation.
///
/// Groups sampling density and spacecraft properties that are shared
/// across [`validate_mission_nyx`] and its internal per-leg helpers.
#[derive(Debug, Clone, Copy)]
pub struct ValidationConfig {
    /// Number of intermediate comparison samples per leg (0 = final only).
    pub samples_per_leg: u32,
    /// Chief spacecraft properties (mass, drag area, SRP area, Cd, Cr).
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft properties.
    pub deputy_config: SpacecraftConfig,
}

/// Bundled context for [`validate_mission_nyx`].
///
/// Groups the 7 parameters that every validation call requires, following
/// the Codebase "struct-based public APIs" rule. Borrows everything;
/// caller is responsible for building `dynamics` via
/// [`build_full_physics_dynamics`](crate::nyx_bridge::build_full_physics_dynamics).
///
/// # Invariants
/// - `mission.legs` must be non-empty.
/// - `chief_initial` and `deputy_initial` must be valid bound ECI states.
/// - `almanac` must contain Earth frame data and planetary ephemerides.
/// - `dynamics` must be built from the same `almanac`.
pub struct ValidationPipelineCtx<'a> {
    /// Analytical waypoint mission to validate.
    pub mission: &'a WaypointMission,
    /// Chief ECI state at mission start.
    pub chief_initial: &'a StateVector,
    /// Deputy ECI state at mission start.
    pub deputy_initial: &'a StateVector,
    /// Validation sampling and spacecraft configuration.
    pub config: &'a ValidationConfig,
    /// COLA burns and analytical maneuvers for effectiveness comparison.
    pub cola: &'a ColaValidationInput,
    /// ANISE almanac with ephemeris and frame data.
    pub almanac: &'a Arc<Almanac>,
    /// Pre-built full-physics dynamics.
    pub dynamics: &'a SpacecraftDynamics,
}

/// Look up the ANISE Earth frame for eclipse queries, if eclipse data is present.
fn lookup_earth_frame(
    almanac: &Arc<Almanac>,
    eclipse_enabled: bool,
) -> Result<Option<anise::prelude::Frame>, ValidationError> {
    if eclipse_enabled {
        almanac
            .frame_info(ANISE_EARTH_J2000)
            .map(Some)
            .map_err(|e| {
                ValidationError::NyxBridge(Box::new(NyxBridgeError::FrameLookup { source: e }))
            })
    } else {
        Ok(None)
    }
}

/// Propagate the full mission without COLA impulses to compute baseline safety.
///
/// Runs a complete mission loop using the unified [`propagate_leg`] for every
/// leg, collecting only safety pairs (no comparison points, eclipse, or COLA
/// data). On legs that carry a COLA burn in the post-COLA path, a sentinel
/// [`MidLegBurn`] with `impulse_dv_ric_km_s = None` is passed so the sampling
/// grid is split at the same elapsed time as the post-COLA propagation; the
/// physical trajectory is unchanged because no impulse is applied. This keeps
/// the pre-COLA baseline and post-COLA trajectory sample-aligned bit-for-bit
/// on the shared segment, which the Safety Comparison table relies on.
///
/// This must be a full loop rather than per-leg branching because downstream
/// legs' initial states depend on whether COLA was applied in earlier legs.
///
/// # Invariants
/// - `mission.legs` must be non-empty
/// - `chief_initial` and `deputy_initial` must have valid epochs
/// - `ctx` must reference a valid almanac and spacecraft configurations
fn compute_pre_cola_safety(
    mission: &WaypointMission,
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    cola: &ColaValidationInput,
    ctx: &LegPropagationCtx<'_>,
) -> Result<rpo_core::mission::types::SafetyMetrics, ValidationError> {
    let estimated_samples = ctx.samples_per_leg as usize // u32 → usize: always safe (usize ≥ 32 bits)
        * mission.legs.len();
    let mut chief = *chief_initial;
    let mut deputy = *deputy_initial;
    let mut cumulative = 0.0_f64;
    let mut safety_pairs: Vec<ChiefDeputySnapshot> =
        Vec::with_capacity(estimated_samples);

    for (leg_idx, leg) in mission.legs.iter().enumerate() {
        // Match the sampling split of the post-COLA path when a burn exists
        // on this leg. impulse_dv_ric_km_s = None means "split grid but stay
        // on the same physical trajectory" -- this is the whole point of the
        // unification: segment 1 samples match bit-for-bit between the
        // pre-COLA baseline and the post-COLA path.
        let burn = cola.burns.iter().find(|c| c.leg_index == leg_idx).map(|c| MidLegBurn {
            elapsed_s: c.elapsed_s,
            impulse_dv_ric_km_s: None,
        });

        let out = propagate_leg(&chief, &deputy, leg, burn.as_ref(), ctx)?;

        // Collect safety pairs inline (skip t=0, matching
        // build_leg_comparison_points).
        for (idx, (c, d)) in
            out.chief_results.iter().zip(out.deputy_results.iter()).enumerate()
        {
            if idx > 0 {
                safety_pairs.push(ChiefDeputySnapshot {
                    elapsed_s: cumulative + c.elapsed_s,
                    chief: c.state,
                    deputy: d.state,
                });
            }
        }

        (chief, deputy) = advance_leg_states(
            &out.chief_results,
            &out.deputy_results,
            &leg.arrival_maneuver.dv_ric_km_s,
        )?;
        cumulative += leg.tof_s;
    }

    let states = build_nyx_safety_states(&safety_pairs)?;
    let mut safety = analyze_trajectory_safety(&states)?;
    assign_safety_leg_indices(&mut safety, &mission.legs);
    Ok(safety)
}

/// Assign correct leg indices to the safety minimum-distance fields.
///
/// [`analyze_trajectory_safety`] processes a flat trajectory and always leaves
/// leg indices at 0. This function derives the correct leg index from elapsed
/// time and the per-leg durations.
fn assign_safety_leg_indices(
    safety: &mut rpo_core::mission::types::SafetyMetrics,
    legs: &[ManeuverLeg],
) {
    let mut found_3d = false;
    let mut found_rc = false;
    let mut cumulative = 0.0_f64;
    for (i, leg) in legs.iter().enumerate() {
        cumulative += leg.tof_s;
        if !found_3d && safety.operational.min_3d_elapsed_s <= cumulative {
            safety.operational.min_3d_leg_index = i;
            found_3d = true;
        }
        if !found_rc && safety.operational.min_rc_elapsed_s <= cumulative {
            safety.operational.min_rc_leg_index = i;
            found_rc = true;
        }
        if found_3d && found_rc {
            break;
        }
    }
}

/// Validate a waypoint mission against nyx full-physics propagation.
///
/// Propagates chief and deputy through each mission leg using nyx with full
/// force models (J2 harmonics, drag, SRP, Sun/Moon third-body), applies
/// impulsive delta-v maneuvers at burn epochs, and compares the resulting RIC
/// trajectory against the analytical trajectory from the mission planner.
///
/// # Algorithm
/// 1. For each leg: propagate chief through nyx, apply departure delta-v to deputy,
///    propagate deputy through nyx, sample and compare RIC states.
/// 2. At leg boundaries: apply arrival delta-v, advance to next leg.
/// 3. Build safety states from accumulated chief/deputy pairs.
/// 4. Compute aggregate statistics and return report.
///
/// # Invariants
/// - `mission.legs` must be non-empty (at least one maneuver leg)
/// - `chief_initial` and `deputy_initial` must be valid bound ECI states at mission start epoch
/// - `almanac` must contain Earth frame data (`IAU_EARTH`) and planetary ephemerides
///
/// # Arguments
/// * `mission` -- Analytical waypoint mission (from `plan_waypoint_mission`)
/// * `chief_initial` -- Chief ECI state at mission start
/// * `deputy_initial` -- Deputy ECI state at mission start
/// * `config` -- Validation settings (sampling density, spacecraft properties)
/// * `almanac` -- Full-physics ANISE almanac (from `load_full_almanac`)
///
/// # Errors
/// Returns [`ValidationError`] if the mission has no legs, almanac frame lookup
/// fails, dynamics setup fails, propagation fails, or safety analysis fails.
pub fn validate_mission_nyx(
    pipeline: &ValidationPipelineCtx<'_>,
) -> Result<ValidationReport, ValidationError> {
    let ValidationPipelineCtx {
        mission,
        chief_initial,
        deputy_initial,
        config,
        cola,
        almanac,
        dynamics,
    } = pipeline;

    if mission.legs.is_empty() {
        return Err(ValidationError::EmptyTrajectory);
    }

    let mut chief_state = **chief_initial;
    let mut deputy_state = **deputy_initial;
    let mut cumulative_time = 0.0_f64;
    let mut leg_points = Vec::with_capacity(mission.legs.len());
    let mut cola_effectiveness: Vec<ColaEffectivenessEntry> =
        Vec::with_capacity(cola.burns.len());
    let estimated_total_samples = config.samples_per_leg as usize // u32 → usize: always safe (usize ≥ 32 bits)
        * mission.legs.len();
    let mut safety_pairs: Vec<ChiefDeputySnapshot> =
        Vec::with_capacity(estimated_total_samples);
    let eclipse_enabled = mission.eclipse.is_some();
    let mut eclipse_samples: Vec<EclipseSample> = if eclipse_enabled {
        Vec::with_capacity(estimated_total_samples)
    } else {
        Vec::new()
    };

    let earth_frame = lookup_earth_frame(almanac, eclipse_enabled)?;

    let ctx = LegPropagationCtx {
        samples_per_leg: config.samples_per_leg,
        chief_config: &config.chief_config,
        deputy_config: &config.deputy_config,
        almanac,
        dynamics,
    };

    // Pre-COLA pass: when COLA burns exist, propagate without COLA to
    // establish a baseline numerical safety for apples-to-apples comparison.
    // The pre-COLA path threads `cola` through so it can split sampling at
    // the same elapsed_s as the post-COLA path (impulse = None), giving
    // bit-identical segment-1 sampling grids between the two paths.
    let pre_cola_numerical_safety = if cola.burns.is_empty() {
        None
    } else {
        Some(compute_pre_cola_safety(
            mission, chief_initial, deputy_initial, cola, &ctx,
        )?)
    };

    for (leg_idx, leg) in mission.legs.iter().enumerate() {
        let primary_cola: Option<&ColaBurn> = cola.burns.iter().find(|c| c.leg_index == leg_idx);

        let (chief_results, deputy_results, cola_split) = if let Some(burn) = primary_cola {
            let out = propagate_leg_with_cola(
                &chief_state, &deputy_state, leg, burn, &ctx,
            )?;
            collect_cola_effectiveness(
                &out, leg_idx, cola, &mut cola_effectiveness,
            );
            (out.chief_results, out.deputy_results, Some(out.cola_split_index))
        } else {
            let (c, d) = propagate_leg_parallel(
                &chief_state, &deputy_state, leg, &ctx,
            )?;
            (c, d, None)
        };

        // Build comparison points for this leg
        let leg_output = build_leg_comparison_points(
            &chief_results,
            &deputy_results,
            &leg.trajectory,
            cumulative_time,
            earth_frame,
            almanac,
            cola_split,
        )?;
        safety_pairs.extend(leg_output.safety_pairs);
        eclipse_samples.extend(leg_output.eclipse_samples);
        leg_points.push(leg_output.points);

        (chief_state, deputy_state) =
            advance_leg_states(&chief_results, &deputy_results, &leg.arrival_maneuver.dv_ric_km_s)?;
        cumulative_time += leg.tof_s;
    }

    // Compute safety from nyx trajectory and assign correct leg indices.
    let nyx_safety_states = build_nyx_safety_states(&safety_pairs)?;
    let mut numerical_safety = analyze_trajectory_safety(&nyx_safety_states)?;
    assign_safety_leg_indices(&mut numerical_safety, &mission.legs);

    // Compute eclipse validation if eclipse data is present
    let eclipse_validation = mission.eclipse.as_ref().and_then(|eclipse_data| {
        build_eclipse_validation(eclipse_data, &eclipse_samples)
    });

    // Compute per-leg summaries (single pass), then derive aggregate stats
    let leg_summaries = compute_leg_summaries(&leg_points);
    let stats = compute_report_statistics(&leg_summaries);

    Ok(ValidationReport {
        leg_points,
        max_position_error_km: stats.max_position_error_km,
        mean_position_error_km: stats.mean_position_error_km,
        rms_position_error_km: stats.rms_position_error_km,
        max_velocity_error_km_s: stats.max_velocity_error_km_s,
        analytical_safety: mission.safety,
        numerical_safety,
        pre_cola_numerical_safety,
        chief_config: config.chief_config,
        deputy_config: config.deputy_config,
        eclipse_validation,
        cola_effectiveness,
        leg_summaries,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::nyx_bridge;
    use rpo_core::mission::config::MissionConfig;
    use rpo_core::mission::types::Waypoint;
    use rpo_core::propagation::propagator::PropagationModel;
    use rpo_core::test_helpers::{DMF_RATE_NONZERO_LOWER_BOUND, DMF_RATE_UPPER_BOUND};
    use rpo_core::types::SpacecraftConfig;

    use super::super::cola::{ColaBurn, ColaValidationInput};
    use crate::validation::test_scenario;

    // =========================================================================
    // Full-Physics Integration Tests
    // =========================================================================

    /// Position tolerance for a single-leg transfer (~1 orbit).
    /// Unmodeled perturbations (drag, SRP, 3rd-body) contribute ~50m total;
    /// 10x margin gives 0.5 km.
    const FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM: f64 = 0.5;

    /// Position tolerance for a multi-leg mission (~3 legs x 0.75 period each).
    /// ~3x single-leg tolerance plus maneuver state mismatch across legs.
    const FULL_PHYSICS_MULTI_LEG_POS_TOL_KM: f64 = 3.0;

    /// Position tolerance for drag STM vs nyx comparison.
    /// DMF linear fit error + unmodeled SRP/3rd-body over 1 orbit.
    const DRAG_STM_VS_NYX_POS_TOL_KM: f64 = 1.0;

    /// Guard threshold for improvement ratio computation (km).
    /// When the J2-only error is below this threshold, the improvement ratio
    /// is numerically meaningless (division by near-zero). Skip the diagnostic.
    const IMPROVEMENT_RATIO_GUARD_KM: f64 = 1e-10;

    /// Single-leg transfer with nonzero initial ROE and mixed-axis waypoint,
    /// validated against nyx full-physics propagation.
    ///
    /// Chief: ISS-like orbit. Deputy: ~300m-scale formation (nonzero dex, dey, dix).
    /// Waypoint: [0.5, 3.0, 1.0] RIC km, TOF = 0.8 orbital periods.
    /// Non-integer period avoids CW singularity at nt = 2pi (rank-1 `phi_rv`).
    /// 50 samples for detailed error characterization.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn validate_full_physics_single_leg() {
        use test_scenario::{
            iss_formation_roe, plan_and_validate, DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            PlanAndValidateInput, ValidationContext,
        };

        let formation_roe = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);
        let period = ctx.chief_elements.period().unwrap();
        let waypoints = [Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        }];

        let report = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                waypoints: &waypoints,
                config: &MissionConfig::default(),
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &ColaValidationInput::default(),
            },
        );

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
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn validate_full_physics_multi_waypoint() {
        use test_scenario::{
            iss_formation_roe, plan_and_validate, DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            PlanAndValidateInput, ValidationContext,
        };

        let formation_roe = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);
        let period = ctx.chief_elements.period().unwrap();
        let tof = 0.75 * period;
        let waypoints = [
            Waypoint {
                position_ric_km: Vector3::new(0.5, 3.0, 0.5),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(1.0, -2.0, 1.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
        ];

        let report = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                waypoints: &waypoints,
                config: &MissionConfig::default(),
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &ColaValidationInput::default(),
            },
        );

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
    /// 2. Chief: `SERVICER_500KG` (B*=0.0044), Deputy: 200kg/2m2 (B*=0.022, ~5x higher)
    /// 3. Extract DMF rates -> `DragConfig`
    /// 4. Plan with `J2DragStm`: single V-bar waypoint [0,5,0], 0.8 periods
    ///    (non-integer period avoids CW singularity at nt = 2pi)
    /// 5. Plan same with `J2Stm` for comparison
    /// 6. Validate both against nyx
    /// 7. Assert drag-aware error < tolerance; log improvement ratio
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn validate_drag_stm_vs_nyx_drag() {
        use test_scenario::{
            plan_and_validate, DEFAULT_VALIDATION_SAMPLES_PER_LEG, PlanAndValidateInput,
            ValidationContext,
        };

        let ctx = ValidationContext::iss_colocated();
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig {
            dry_mass_kg: 200.0,
            drag_area_m2: 2.0,
            ..SpacecraftConfig::SERVICER_500KG
        };

        // Step 1: Extract DMF rates (uses the shared almanac from `ctx`).
        let drag = nyx_bridge::extract_dmf_rates(
            &ctx.chief_state,
            &ctx.deputy_state,
            &chief_config,
            &deputy_config,
            &ctx.almanac,
        )
        .expect("DMF extraction should succeed");

        eprintln!(
            "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot,
        );

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

        // Step 2: plan & validate with both propagators, sharing scaffolding.
        let period = ctx.chief_elements.period().unwrap();
        let waypoints = [Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        }];
        let config = MissionConfig::default();
        let default_cola = ColaValidationInput::default();

        let drag_propagator = PropagationModel::J2DragStm { drag };
        let drag_report = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                waypoints: &waypoints,
                config: &config,
                propagator: &drag_propagator,
                chief_config,
                deputy_config,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &default_cola,
            },
        );

        let j2_propagator = PropagationModel::J2Stm;
        let j2_report = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                waypoints: &waypoints,
                config: &config,
                propagator: &j2_propagator,
                chief_config,
                deputy_config,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &default_cola,
            },
        );

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

        // Diagnostic: improvement ratio (not hard-asserted -- SRP/3rd-body may dominate)
        if j2_report.max_position_error_km > IMPROVEMENT_RATIO_GUARD_KM {
            let improvement = j2_report.max_position_error_km / drag_report.max_position_error_km;
            eprintln!("Drag/J2 improvement ratio: {improvement:.2}x");
            if improvement < 1.0 {
                eprintln!(
                    "  Note: drag STM did not improve over J2-only for this scenario. \
                     For short single-orbit transfers with colocated start, differential \
                     drag effect may be negligible vs unmodeled perturbations (SRP, 3rd-body)."
                );
            }
        }

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
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn validate_safety_full_physics() {
        use rpo_core::mission::config::SafetyConfig;

        use test_scenario::{
            assert_safety_agreement, iss_formation_roe, plan_and_validate,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG, PlanAndValidateInput, ValidationContext,
        };

        // Perpendicular e/i vectors: dex along ex, diy along iy -> meaningful separation.
        let formation_roe = iss_formation_roe(0.5, 0.0, 0.0, 0.5);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);

        let period = ctx.chief_elements.period().unwrap();
        let tof = 0.75 * period;
        let waypoints = [
            Waypoint {
                position_ric_km: Vector3::new(0.0, 3.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.5, 5.0, 0.5),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 2.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
        ];
        let config = MissionConfig {
            safety: Some(SafetyConfig::default()),
            ..MissionConfig::default()
        };

        let report = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                waypoints: &waypoints,
                config: &config,
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &ColaValidationInput::default(),
            },
        );

        // Analytical safety must be present (we enabled it via SafetyConfig).
        let analytical = report
            .analytical_safety
            .as_ref()
            .expect("analytical safety should be present (SafetyConfig enabled)");
        assert_safety_agreement(analytical, &report.numerical_safety);
    }

    /// End-to-end COLA effectiveness validation.
    ///
    /// Plans a mission with a close POCA, computes avoidance, validates with nyx,
    /// and asserts that the nyx post-COLA minimum distance exceeds the target
    /// threshold fraction.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn validate_cola_effectiveness() {
        use rpo_core::mission::closest_approach::find_closest_approaches;
        use rpo_core::mission::config::SafetyConfig;
        use rpo_core::mission::{assess_cola, ClosestApproach, ColaAssessment, ColaConfig};

        use test_scenario::{
            assert_cola_effectiveness, iss_formation_roe, plan_mission, validate_planned,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG, PlanAndValidateInput, ValidationContext,
        };

        // Deputy: formation sized so POCA ~ 0.1 km (below 0.2 km threshold).
        // Minimum distance ~ a * min(de, di). For de=di=0.1/a: min distance ~ 0.1 km.
        // This gives scale = target/poca ~ 2, keeping COLA delta-v affordable.
        let formation_roe = iss_formation_roe(0.1, 0.0, 0.0, 0.1);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);

        // Waypoint: along-track displacement, ~0.8 period.
        // Along-track (T) motion is cheapest for near-circular chief orbits.
        let period = ctx.chief_elements.period().unwrap();
        let waypoints = [Waypoint {
            position_ric_km: Vector3::new(0.0, 0.5, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        }];
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig {
            safety: Some(SafetyConfig {
                min_distance_3d_km: 0.2,
                min_ei_separation_km: 0.0,
            }),
            ..MissionConfig::default()
        };

        // Plan the mission once (no nyx validation yet) so we can compute
        // POCA + assess COLA off the planned trajectory.
        let default_cola = ColaValidationInput::default();
        let plan_input = PlanAndValidateInput {
            waypoints: &waypoints,
            config: &config,
            propagator: &propagator,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
            samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            cola_input: &default_cola,
        };
        let mission = plan_mission(&ctx, &plan_input);

        // COLA assessment: target distance intentionally modest. Budget
        // generous (1 km/s) because tight formations require large ROE
        // corrections to reach the target minimum distance.
        let cola_config = ColaConfig {
            target_distance_km: 0.2,
            max_dv_km_s: 1.0,
        };
        let poca: Vec<Vec<ClosestApproach>> = mission
            .legs
            .iter()
            .enumerate()
            .map(|(i, leg)| {
                find_closest_approaches(
                    &leg.trajectory,
                    &leg.departure_chief_mean,
                    leg.departure_maneuver.epoch,
                    &propagator,
                    &leg.post_departure_roe,
                    i,
                )
                .expect("POCA computation should succeed for test formation")
            })
            .collect();
        let assessment = assess_cola(&mission, &poca, &propagator, &cola_config);
        let maneuvers = match &assessment {
            ColaAssessment::Avoidance { maneuvers, .. }
            | ColaAssessment::SecondaryConjunction { maneuvers, .. } => maneuvers.clone(),
            ColaAssessment::Nominal => {
                panic!(
                    "Test formation must produce a POCA violation triggering avoidance. \
                     Adjust deputy Keplerian offsets if this fails."
                );
            }
        };
        let cola_burns = super::super::cola::convert_cola_to_burns(Some(&maneuvers), &mission)
            .expect("COLA burn conversion should succeed");
        let cola_input = ColaValidationInput {
            burns: cola_burns,
            analytical_maneuvers: maneuvers.clone(),
            target_distance_km: Some(cola_config.target_distance_km),
        };

        // Now validate the (same) planned mission against nyx, passing the
        // populated COLA input so the report carries effectiveness data.
        let validate_input = PlanAndValidateInput {
            cola_input: &cola_input,
            ..plan_input
        };
        let report = validate_planned(&ctx, &mission, &validate_input);

        // Assert effectiveness data is populated, then check each entry.
        assert!(
            !report.cola_effectiveness.is_empty(),
            "COLA effectiveness should be populated when COLA burns are present",
        );
        for eff in &report.cola_effectiveness {
            assert_cola_effectiveness(eff);
        }
    }

    /// End-to-end regression: pre-COLA baseline safety must match post-COLA
    /// safety in the pre-burn window, since COLA cannot affect the pre-burn
    /// portion of the trajectory. This is the test that would have caught
    /// the sampling artifact where post-COLA looked worse than pre-COLA in
    /// the validate.md Safety Comparison table.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn pre_cola_and_post_cola_pre_burn_window_match_within_sampling_tolerance() {
        use rpo_core::constants::{TEST_F64_POSITION_NOISE_KM, TEST_SAMPLING_REGRESSION_TOL_KM};
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;

        use test_scenario::{
            iss_formation_roe, plan_mission, DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            PlanAndValidateInput, ValidationContext,
        };

        // Mid-coast COLA burn time, prime-ish to avoid coinciding with sampling grid points.
        const MID_COAST_BURN_ELAPSED_S: f64 = 709.0;

        // Build a single-leg mission with a nonzero formation so the ROE
        // trajectory is interesting. COLA burn will be injected at mid-leg.
        let formation = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation);

        let period = ctx.chief_elements.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };

        let default_cola = ColaValidationInput::default();
        let propagator = PropagationModel::J2Stm;
        let input = PlanAndValidateInput {
            waypoints: &[waypoint],
            config: &MissionConfig::default(),
            propagator: &propagator,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
            samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            cola_input: &default_cola,
        };
        let mission = plan_mission(&ctx, &input);

        // COLA burn at MID_COAST_BURN_ELAPSED_S. The dv is small enough not to
        // blow up the trajectory but large enough that the post-burn segment
        // diverges from the baseline.
        let leg_tof_s = mission.legs[0].tof_s;
        assert!(MID_COAST_BURN_ELAPSED_S < leg_tof_s, "burn time must be inside leg tof");
        let cola = ColaValidationInput {
            burns: vec![ColaBurn {
                leg_index: 0,
                elapsed_s: MID_COAST_BURN_ELAPSED_S,
                dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
            }],
            analytical_maneuvers: vec![],
            target_distance_km: None,
        };

        let val_config = super::ValidationConfig {
            samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let dynamics = crate::nyx_bridge::build_full_physics_dynamics(&ctx.almanac)
            .expect("dynamics should build from test almanac");
        let pipeline = crate::validation::ValidationPipelineCtx {
            mission: &mission,
            chief_initial: &ctx.chief_state,
            deputy_initial: &ctx.deputy_state,
            config: &val_config,
            cola: &cola,
            almanac: &ctx.almanac,
            dynamics: &dynamics,
        };
        let report = crate::validation::validate_mission_nyx(&pipeline)
            .expect("validation should succeed");

        let pre_cola = report
            .pre_cola_numerical_safety
            .as_ref()
            .expect("pre_cola_numerical_safety must be populated when COLA is present");
        let post_cola_min_3d_km = report.numerical_safety.operational.min_distance_3d_km;
        let pre_cola_min_3d_km = pre_cola.operational.min_distance_3d_km;

        // COLA can only reduce the post-burn minimum; it cannot affect
        // pre-burn. So pre_cola_min_3d must be <= post_cola_min_3d +
        // sampling_tolerance.
        let delta_km = (pre_cola_min_3d_km - post_cola_min_3d_km).abs();
        assert!(
            delta_km < TEST_SAMPLING_REGRESSION_TOL_KM
                || pre_cola_min_3d_km <= post_cola_min_3d_km + TEST_SAMPLING_REGRESSION_TOL_KM,
            "pre-COLA min ({pre_cola_min_3d_km:.9} km) must be <= post-COLA min ({post_cola_min_3d_km:.9} km) \
             within sampling tolerance; delta = {delta_km:.9} km"
        );

        assert!(
            (pre_cola_min_3d_km - post_cola_min_3d_km).abs() < TEST_F64_POSITION_NOISE_KM
                || pre_cola_min_3d_km <= post_cola_min_3d_km,
            "pre-COLA min must equal post-COLA min or be strictly less (COLA only acts post-split); \
             pre = {pre_cola_min_3d_km:.12} km, post = {post_cola_min_3d_km:.12} km"
        );
    }
}

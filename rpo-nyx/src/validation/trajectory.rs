//! Core nyx validation orchestrator: per-leg and mission-level trajectory comparison.

use std::sync::Arc;

use anise::constants::frames::EARTH_J2000 as ANISE_EARTH_J2000;
use anise::prelude::Almanac;
use nalgebra::Vector3;
use serde::Serialize;

use rpo_core::elements::eci_ric_dcm::eci_to_ric_relative;
use rpo_core::mission::safety::analyze_trajectory_safety;
use crate::nyx_bridge::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
    query_anise_eclipse, state_to_orbit, ChiefDeputySnapshot, NyxBridgeError, TimedState,
};
use rpo_core::propagation::propagator::PropagatedState;
use rpo_core::types::{SpacecraftConfig, StateVector};

use rpo_core::mission::avoidance::AvoidanceManeuver;
use rpo_core::mission::types::{
    ColaEffectivenessEntry, ManeuverLeg, ValidationPoint, ValidationReport, WaypointMission,
};
use super::eclipse::{EclipseSample, build_eclipse_validation};
use super::errors::ValidationError;
use super::statistics::{compute_leg_summaries, find_closest_analytical_ric, compute_report_statistics};

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

/// A mid-coast COLA impulse for nyx validation.
///
/// # Invariants
/// - `elapsed_s` must be in `(0, leg.tof_s)` -- burn cannot be at leg boundaries
/// - `dv_ric_km_s` must be finite
#[derive(Debug, Clone, Copy)]
pub struct ColaBurn {
    /// Index of the mission leg this burn applies to.
    pub leg_index: usize,
    /// Time from leg departure to COLA burn (seconds).
    pub elapsed_s: f64,
    /// Delta-v in RIC frame (km/s).
    pub dv_ric_km_s: Vector3<f64>,
}

/// All COLA-related inputs for nyx validation: burns to inject plus analytical
/// context for effectiveness comparison.
///
/// Bundles the nyx-format impulse burns with the analytical avoidance maneuvers
/// and target threshold so callers pass a single COLA struct instead of
/// separate `&[ColaBurn]` + context parameters.
///
/// Unit and frame assumptions are carried by field-level types
/// (`ColaBurn::dv_ric_km_s`, `AvoidanceManeuver::post_avoidance_poca_km`, etc.)
/// per crate boundary convention.
#[derive(Debug, Clone, Default)]
pub struct ColaValidationInput {
    /// Nyx-format impulse burns to inject during propagation.
    pub burns: Vec<ColaBurn>,
    /// Analytical avoidance maneuvers from `assess_cola()` for effectiveness comparison.
    pub analytical_maneuvers: Vec<AvoidanceManeuver>,
    /// Target COLA separation threshold (km) from `ColaConfig::target_distance_km`.
    pub target_distance_km: Option<f64>,
}

/// Convert COLA avoidance maneuvers to validation burn commands.
///
/// Uses each maneuver's epoch minus the leg departure epoch to compute `elapsed_s`.
///
/// Uses strict rejection (`elapsed_s <= 0 || >= tof_s`) because this function
/// consumes externally-specified burn times that must be exactly within the leg.
/// This differs from the avoidance module's lenient clamping
/// (`BURN_TIME_CLAMP_FRACTION`), which generates burns and can safely nudge
/// near-boundary solutions inward.
///
/// # Errors
/// Returns [`ValidationError::ColaEpochOutOfBounds`] if a maneuver's epoch
/// falls outside `(0, leg.tof_s)` for the corresponding leg.
pub fn convert_cola_to_burns(
    maneuvers: Option<&[AvoidanceManeuver]>,
    mission: &WaypointMission,
) -> Result<Vec<ColaBurn>, ValidationError> {
    let Some(maneuvers) = maneuvers else {
        return Ok(Vec::new());
    };

    let mut burns = Vec::with_capacity(maneuvers.len());
    for m in maneuvers {
        let leg = &mission.legs[m.leg_index];
        let leg_departure_epoch = leg.departure_maneuver.epoch;
        let elapsed_s = (m.epoch - leg_departure_epoch).to_seconds();

        if elapsed_s <= 0.0 || elapsed_s >= leg.tof_s {
            return Err(ValidationError::ColaEpochOutOfBounds {
                elapsed_s,
                tof_s: leg.tof_s,
                leg_index: m.leg_index,
            });
        }

        burns.push(ColaBurn {
            leg_index: m.leg_index,
            elapsed_s,
            dv_ric_km_s: m.dv_ric_km_s,
        });
    }
    Ok(burns)
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
    let ctx = LegPropagationCtx {
        samples_per_leg: config.samples_per_leg,
        chief_config: &config.chief_config,
        deputy_config: &config.deputy_config,
        almanac,
    };
    let (chief_results, deputy_results) = propagate_leg_parallel(
        chief_state, deputy_state, leg, &ctx,
    )?;

    // Build comparison points (no eclipse or COLA -- those stay in validate_mission_nyx)
    let leg_output = build_leg_comparison_points(
        &chief_results,
        &deputy_results,
        &leg.trajectory,
        cumulative_time_s,
        None, // no eclipse frame for per-leg API
        almanac,
        None, // no COLA for per-leg API
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
///
/// When `cola_split_index` is `Some(n)`, points at index `>= n` are tagged
/// `post_cola = true` -- they follow a COLA burn and should be excluded from
/// fidelity statistics.
fn build_leg_comparison_points(
    chief_results: &[TimedState],
    deputy_results: &[TimedState],
    trajectory: &[PropagatedState],
    cumulative_time_s: f64,
    earth_frame: Option<anise::prelude::Frame>,
    almanac: &Arc<Almanac>,
    cola_split_index: Option<usize>,
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
        // haven't separated yet -- distance is physically meaningless
        // (consistent with monte_carlo/execution.rs).
        if idx > 0 {
            safety_pairs.push(ChiefDeputySnapshot {
                elapsed_s: elapsed,
                chief: chief_sample.state.clone(),
                deputy: deputy_sample.state.clone(),
            });
        }
        let post_cola = cola_split_index.is_some_and(|split| idx >= split);
        points.push(ValidationPoint {
            elapsed_s: elapsed,
            analytical_ric,
            numerical_ric,
            position_error_km: pos_err,
            velocity_error_km_s: vel_err,
            post_cola,
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
/// Thin wrapper over [`propagate_leg`] with `burn = None`. Preserved as a
/// helper for call sites that do not need the COLA split. Applies the leg's
/// departure impulse to the deputy and returns both trajectories as tuples.
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
    ctx: &LegPropagationCtx<'_>,
) -> Result<(Vec<TimedState>, Vec<TimedState>), ValidationError> {
    let out = propagate_leg(chief_state, deputy_state, leg, None, ctx)?;
    Ok((out.chief_results, out.deputy_results))
}

/// Optional mid-leg burn insertion point for [`propagate_leg`].
///
/// Present on every leg that has a COLA burn solved for it. The impulse field
/// distinguishes the two callers:
///
/// - `impulse_dv_ric_km_s = Some(_)` -- apply this RIC delta-v to the deputy
///   at `elapsed_s` (post-COLA path, main validate loop).
/// - `impulse_dv_ric_km_s = None` -- split sampling at `elapsed_s` but do not
///   apply any impulse (pre-COLA baseline path; keeps sampling density
///   identical to the post-COLA path for apples-to-apples comparison).
struct MidLegBurn {
    /// Time from leg departure to the burn split point (seconds).
    elapsed_s: f64,
    /// Delta-v in RIC frame (km/s), or `None` for a sampling-only split.
    impulse_dv_ric_km_s: Option<Vector3<f64>>,
}

/// Output from [`propagate_leg`]: per-sample chief/deputy states and an
/// optional split index marking where segment 2 begins when a split was used.
struct LegPropOutput {
    /// Chief ECI states sampled across the leg (single or two segments).
    chief_results: Vec<TimedState>,
    /// Deputy ECI states sampled across the leg (single or two segments).
    deputy_results: Vec<TimedState>,
    /// Index of the first post-split sample when `burn` was `Some`; `None`
    /// otherwise.
    split_index: Option<usize>,
}

/// Propagate chief and deputy through a single segment in parallel.
///
/// Internal helper used by [`propagate_leg`] for each of the 1 or 2 nyx
/// segments in a leg. Takes pre-built dynamics so the caller can reuse them
/// across segments.
fn propagate_segment_parallel(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    duration_s: f64,
    n_samples: u32,
    dynamics: &nyx_space::md::prelude::SpacecraftDynamics,
    ctx: &LegPropagationCtx<'_>,
) -> Result<(Vec<TimedState>, Vec<TimedState>), ValidationError> {
    let (chief_result, deputy_result) = rayon::join(
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                chief_state,
                duration_s,
                n_samples,
                ctx.chief_config,
                dynamics.clone(),
                ctx.almanac,
            )?)
        },
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                deputy_state,
                duration_s,
                n_samples,
                ctx.deputy_config,
                dynamics.clone(),
                ctx.almanac,
            )?)
        },
    );
    Ok((chief_result?, deputy_result?))
}

/// Propagate a leg with optional mid-leg sampling split and optional impulse.
///
/// When `burn` is `None`: single uniform segment of `samples_per_leg` samples
/// over `leg.tof_s`, step = `leg.tof_s / samples_per_leg`. Equivalent to the
/// pre-unification [`propagate_leg_parallel`] behavior.
///
/// When `burn` is `Some`: sampling is split at `burn.elapsed_s`. `n1` is the
/// smallest integer such that `n1 * step_s >= burn.elapsed_s`;
/// `n2 = samples_per_leg - n1` (min 1). If `burn.impulse_dv_ric_km_s` is
/// `Some`, that RIC delta-v is applied to the deputy at the split point;
/// otherwise the split is sampling-only and the deputy continues on the same
/// physical trajectory.
///
/// Unifying the split logic between the pre-COLA baseline path
/// (impulse = `None`) and the post-COLA path (impulse = `Some`) guarantees the
/// two paths see the same sampling grid in the pre-burn window by
/// construction, eliminating the sampling artifact that made post-COLA look
/// worse than pre-COLA in the validate.md Safety Comparison table.
///
/// # Invariants
/// - `chief_state` and `deputy_state` must represent valid orbits.
/// - When `burn` is `Some`, `burn.elapsed_s` must be in `(0, leg.tof_s)`.
/// - `almanac` must contain required frames and force models.
///
/// # Errors
/// - [`ValidationError`] if impulse application, dynamics setup, or propagation fails.
fn propagate_leg(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    leg: &ManeuverLeg,
    burn: Option<&MidLegBurn>,
    ctx: &LegPropagationCtx<'_>,
) -> Result<LegPropOutput, ValidationError> {
    // Build dynamics once; clone is cheap (Arc ref-count bumps).
    let dynamics = build_full_physics_dynamics(ctx.almanac)?;
    let deputy_post_departure =
        apply_impulse(deputy_state, chief_state, &leg.departure_maneuver.dv_ric_km_s)?;

    let Some(burn) = burn else {
        // No split: single uniform segment, exactly matches the old
        // `propagate_leg_parallel` behavior.
        let (chief_results, deputy_results) = propagate_segment_parallel(
            chief_state,
            &deputy_post_departure,
            leg.tof_s,
            ctx.samples_per_leg,
            &dynamics,
            ctx,
        )?;
        return Ok(LegPropOutput {
            chief_results,
            deputy_results,
            split_index: None,
        });
    };

    debug_assert!(
        burn.elapsed_s > 0.0 && burn.elapsed_s < leg.tof_s,
        "propagate_leg: burn elapsed_s {} must be within (0, tof_s = {})",
        burn.elapsed_s,
        leg.tof_s,
    );

    // Split-sample allocation: identical to the old propagate_leg_with_cola.
    // n1 is the smallest integer such that n1 * step_s >= burn.elapsed_s;
    // n2 is the remainder (min 1). Stays in u32 domain -- no f64->integer cast.
    let step_s = leg.tof_s / f64::from(ctx.samples_per_leg);
    let n1 = (1..=ctx.samples_per_leg)
        .find(|&k| f64::from(k) * step_s >= burn.elapsed_s)
        .unwrap_or(ctx.samples_per_leg);
    let n2 = ctx.samples_per_leg.saturating_sub(n1).max(1);

    // Segment 1: [0, burn.elapsed_s] -- chief/deputy in parallel.
    let (chief_seg1, deputy_seg1) = propagate_segment_parallel(
        chief_state,
        &deputy_post_departure,
        burn.elapsed_s,
        n1,
        &dynamics,
        ctx,
    )?;

    let chief_at_split = chief_seg1
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();
    let deputy_at_split = deputy_seg1
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();

    // Apply the mid-leg impulse to the deputy if one was supplied.
    let deputy_seg2_start = match burn.impulse_dv_ric_km_s {
        Some(ref dv) => apply_impulse(&deputy_at_split, &chief_at_split, dv)?,
        None => deputy_at_split,
    };

    // Segment 2: [burn.elapsed_s, tof_s] -- chief/deputy in parallel.
    let remaining_s = leg.tof_s - burn.elapsed_s;
    let (chief_seg2, deputy_seg2) = propagate_segment_parallel(
        &chief_at_split,
        &deputy_seg2_start,
        remaining_s,
        n2,
        &dynamics,
        ctx,
    )?;

    // Merge segments: adjust segment 2 elapsed times to leg-local.
    let split_index = chief_seg1.len();
    let mut chief_results = chief_seg1;
    chief_results.reserve(chief_seg2.len());
    for mut s in chief_seg2 {
        s.elapsed_s += burn.elapsed_s;
        chief_results.push(s);
    }
    let mut deputy_results = deputy_seg1;
    deputy_results.reserve(deputy_seg2.len());
    for mut s in deputy_seg2 {
        s.elapsed_s += burn.elapsed_s;
        deputy_results.push(s);
    }

    Ok(LegPropOutput {
        chief_results,
        deputy_results,
        split_index: Some(split_index),
    })
}

/// Shared context for per-leg nyx propagation.
///
/// Groups the sampling density, spacecraft properties, and almanac that
/// are threaded through the per-leg propagators ([`propagate_leg`],
/// [`propagate_leg_parallel`], [`propagate_leg_with_cola`]) and the
/// pre-COLA safety pass ([`compute_pre_cola_safety`]), avoiding parameter
/// sprawl.
struct LegPropagationCtx<'a> {
    /// Number of intermediate comparison samples per leg.
    samples_per_leg: u32,
    /// Chief spacecraft properties.
    chief_config: &'a SpacecraftConfig,
    /// Deputy spacecraft properties.
    deputy_config: &'a SpacecraftConfig,
    /// ANISE almanac with ephemeris and frame data.
    almanac: &'a Arc<Almanac>,
}

/// Output from propagating a single leg with a mid-coast COLA impulse.
struct ColaLegOutput {
    /// Chief ECI states sampled across both segments.
    chief_results: Vec<TimedState>,
    /// Deputy ECI states sampled across both segments.
    deputy_results: Vec<TimedState>,
    /// Index in results where post-COLA samples begin.
    cola_split_index: usize,
}

/// Propagate a single leg with a mid-coast COLA impulse.
///
/// Thin wrapper over [`propagate_leg`] with `burn = Some({ elapsed_s,
/// impulse = Some(dv) })`. The unified `propagate_leg` handles the two-segment
/// split, parallel propagation, and impulse application; this wrapper just
/// repackages the output into the existing [`ColaLegOutput`] shape.
///
/// # Errors
/// Returns [`ValidationError`] if dynamics setup, propagation, or impulse application fails.
fn propagate_leg_with_cola(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    leg: &ManeuverLeg,
    cola: &ColaBurn,
    ctx: &LegPropagationCtx<'_>,
) -> Result<ColaLegOutput, ValidationError> {
    let burn = MidLegBurn {
        elapsed_s: cola.elapsed_s,
        impulse_dv_ric_km_s: Some(cola.dv_ric_km_s),
    };
    let out = propagate_leg(chief_state, deputy_state, leg, Some(&burn), ctx)?;
    Ok(ColaLegOutput {
        chief_results: out.chief_results,
        deputy_results: out.deputy_results,
        cola_split_index: out.split_index.ok_or(ValidationError::EmptyTrajectory)?,
    })
}

/// Compute the minimum chief-deputy distance in the post-COLA segment.
///
/// Iterates ECI position pairs from `cola_split_index` onward and returns
/// `min(||r_deputy_eci - r_chief_eci||)`. Returns `None` if no post-COLA
/// samples exist or if `cola_split_index` is out of bounds.
///
/// The Euclidean norm is frame-invariant, so ECI distance equals RIC distance —
/// no frame conversion is needed.
///
/// # Arguments
///
/// * `chief_results` — Chief ECI trajectory samples for the leg
/// * `deputy_results` — Deputy ECI trajectory samples for the leg (must be
///   same length as `chief_results`; samples are time-aligned by index)
/// * `cola_split_index` — Index of the first post-COLA sample
fn compute_post_cola_min_distance(
    chief_results: &[TimedState],
    deputy_results: &[TimedState],
    cola_split_index: usize,
) -> Option<f64> {
    let chief_post = chief_results.get(cola_split_index..)?;
    let deputy_post = deputy_results.get(cola_split_index..)?;

    if chief_post.is_empty() {
        return None;
    }

    chief_post
        .iter()
        .zip(deputy_post.iter())
        .map(|(c, d)| (d.state.position_eci_km - c.state.position_eci_km).norm())
        .reduce(f64::min)
}

/// Record COLA effectiveness for a leg that had a COLA burn injected.
///
/// Computes the post-COLA minimum distance from propagation results and pushes
/// a [`ColaEffectivenessEntry`] comparing against the analytical prediction.
///
/// # Invariants
/// - `out` must contain results from a COLA-split propagation (two segments)
/// - `cola.analytical_maneuvers` may be empty (produces `None` for the analytical field)
fn collect_cola_effectiveness(
    out: &ColaLegOutput,
    leg_idx: usize,
    cola: &ColaValidationInput,
    effectiveness: &mut Vec<ColaEffectivenessEntry>,
) {
    if let Some(nyx_min) = compute_post_cola_min_distance(
        &out.chief_results, &out.deputy_results, out.cola_split_index,
    ) {
        let analytical = cola.analytical_maneuvers.iter().find(|m| m.leg_index == leg_idx);
        effectiveness.push(ColaEffectivenessEntry {
            leg_index: leg_idx,
            analytical_post_cola_poca_km: analytical.map(|m| m.post_avoidance_poca_km),
            nyx_post_cola_min_distance_km: nyx_min,
            target_distance_km: cola.target_distance_km,
            threshold_met: cola.target_distance_km.map(|t| nyx_min >= t),
        });
    }
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
    let estimated_samples = ctx.samples_per_leg as usize // u32 -> usize: safe (usize >= 32 bits)
        * mission.legs.len();
    let mut chief = chief_initial.clone();
    let mut deputy = deputy_initial.clone();
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
                    chief: c.state.clone(),
                    deputy: d.state.clone(),
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
    mission: &WaypointMission,
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    config: &ValidationConfig,
    cola: &ColaValidationInput,
    almanac: &Arc<Almanac>,
) -> Result<ValidationReport, ValidationError> {
    if mission.legs.is_empty() {
        return Err(ValidationError::EmptyTrajectory);
    }

    let mut chief_state = chief_initial.clone();
    let mut deputy_state = deputy_initial.clone();
    let mut cumulative_time = 0.0_f64;
    let mut leg_points = Vec::with_capacity(mission.legs.len());
    let mut cola_effectiveness: Vec<ColaEffectivenessEntry> =
        Vec::with_capacity(cola.burns.len());
    let estimated_total_samples = config.samples_per_leg as usize // u32 -> usize: safe (usize >= 32 bits)
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
        let primary_cola = cola.burns.iter().find(|c| c.leg_index == leg_idx);

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

    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use crate::nyx_bridge;
    use rpo_core::mission::config::MissionConfig;
    use rpo_core::mission::types::Waypoint;
    use rpo_core::propagation::propagator::PropagationModel;
    use rpo_core::test_helpers::{
        iss_like_elements, test_epoch, DMF_RATE_NONZERO_LOWER_BOUND, DMF_RATE_UPPER_BOUND,
    };
    use rpo_core::types::{QuasiNonsingularROE, SpacecraftConfig};

    use crate::validation::test_scenario;
    use rpo_core::mission::types::{ColaEffectivenessEntry, SafetyMetrics};

    /// Assert that analytical and numerical safety metrics agree within the
    /// module-scoped tolerances (`SAFETY_RC_RELATIVE_TOL`,
    /// `SAFETY_3D_ABSOLUTE_TOL_KM`, `SAFETY_EI_RELATIVE_TOL`). Also emits
    /// a comparison banner to stderr for post-run eyeballing.
    ///
    /// R/C and e/i comparisons are relative and are skipped when either
    /// reference value is below its `*_NEAR_ZERO_KM` threshold (e.g. a
    /// V-bar configuration has ~zero R/C, making relative error undefined).
    fn assert_safety_agreement(analytical: &SafetyMetrics, numerical: &SafetyMetrics) {
        eprintln!("Safety comparison (analytical vs numerical):");
        eprintln!(
            "  R/C separation:  {:.4} km vs {:.4} km",
            analytical.operational.min_rc_separation_km,
            numerical.operational.min_rc_separation_km,
        );
        eprintln!(
            "  3D distance:     {:.4} km vs {:.4} km",
            analytical.operational.min_distance_3d_km,
            numerical.operational.min_distance_3d_km,
        );
        eprintln!(
            "  e/i separation:  {:.4} km vs {:.4} km",
            analytical.passive.min_ei_separation_km,
            numerical.passive.min_ei_separation_km,
        );
        eprintln!(
            "  |de|:            {:.6} vs {:.6}",
            analytical.passive.de_magnitude, numerical.passive.de_magnitude,
        );
        eprintln!(
            "  |di|:            {:.6} vs {:.6}",
            analytical.passive.di_magnitude, numerical.passive.di_magnitude,
        );
        eprintln!(
            "  e/i phase angle: {:.4} rad vs {:.4} rad",
            analytical.passive.ei_phase_angle_rad, numerical.passive.ei_phase_angle_rad,
        );

        // R/C separation: relative agreement, skip if both near zero.
        let rc_ref = analytical
            .operational
            .min_rc_separation_km
            .max(numerical.operational.min_rc_separation_km);
        if rc_ref > SAFETY_RC_NEAR_ZERO_KM {
            let rc_rel_err = (analytical.operational.min_rc_separation_km
                - numerical.operational.min_rc_separation_km)
                .abs()
                / rc_ref;
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

        // 3D distance: absolute agreement.
        let dist_3d_err = (analytical.operational.min_distance_3d_km
            - numerical.operational.min_distance_3d_km)
            .abs();
        eprintln!("  3D distance absolute error: {dist_3d_err:.4} km");
        assert!(
            dist_3d_err < SAFETY_3D_ABSOLUTE_TOL_KM,
            "3D distance error = {dist_3d_err:.4} km (expected < {SAFETY_3D_ABSOLUTE_TOL_KM})",
        );

        // e/i separation: relative agreement, skip if both near zero.
        let ei_ref = analytical
            .passive
            .min_ei_separation_km
            .max(numerical.passive.min_ei_separation_km);
        if ei_ref > SAFETY_EI_NEAR_ZERO_KM {
            let ei_rel_err = (analytical.passive.min_ei_separation_km
                - numerical.passive.min_ei_separation_km)
                .abs()
                / ei_ref;
            eprintln!("  e/i relative error: {ei_rel_err:.2}");
            assert!(
                ei_rel_err < SAFETY_EI_RELATIVE_TOL,
                "e/i separation relative error = {ei_rel_err:.2} (expected < {SAFETY_EI_RELATIVE_TOL})",
            );
        }
    }

    /// Per-leg COLA effectiveness assertion: checks that nyx's post-COLA
    /// minimum distance exceeds `COLA_EFFECTIVENESS_THRESHOLD_FRACTION` of
    /// the target, and that the analytical vs nyx relative disagreement is
    /// within `COLA_EFFECTIVENESS_RELATIVE_TOL` when an analytical estimate
    /// is available. Emits a one-line summary per entry to stderr.
    fn assert_cola_effectiveness(eff: &ColaEffectivenessEntry) {
        eprintln!(
            "Leg {}: analytical POCA={:.1}m, nyx min={:.1}m, target={:.1}m, met={:?}",
            eff.leg_index + 1,
            eff.analytical_post_cola_poca_km.unwrap_or(0.0) * 1000.0,
            eff.nyx_post_cola_min_distance_km * 1000.0,
            eff.target_distance_km.unwrap_or(0.0) * 1000.0,
            eff.threshold_met,
        );

        let target = eff.target_distance_km.expect("target should be set");
        assert!(
            eff.nyx_post_cola_min_distance_km > target * COLA_EFFECTIVENESS_THRESHOLD_FRACTION,
            "Nyx post-COLA min ({:.1}m) should exceed {:.0}% of target ({:.1}m)",
            eff.nyx_post_cola_min_distance_km * 1000.0,
            COLA_EFFECTIVENESS_THRESHOLD_FRACTION * 100.0,
            target * 1000.0,
        );

        if let Some(analytical) = eff.analytical_post_cola_poca_km {
            if analytical > 0.0 {
                let relative_diff =
                    (eff.nyx_post_cola_min_distance_km - analytical).abs() / analytical;
                eprintln!("  analytical vs nyx relative diff: {:.1}%", relative_diff * 100.0);
                assert!(
                    relative_diff < COLA_EFFECTIVENESS_RELATIVE_TOL,
                    "Analytical ({:.1}m) vs nyx ({:.1}m) relative diff {:.0}% exceeds {:.0}% tolerance",
                    analytical * 1000.0,
                    eff.nyx_post_cola_min_distance_km * 1000.0,
                    relative_diff * 100.0,
                    COLA_EFFECTIVENESS_RELATIVE_TOL * 100.0,
                );
            }
        }
    }

    // =========================================================================
    // COLA Burn Conversion Tests (pure logic, no nyx)
    // =========================================================================

    /// Build a minimal `AvoidanceManeuver` at the given epoch and leg index.
    fn make_avoidance(epoch: hifitime::Epoch, leg_index: usize) -> rpo_core::mission::avoidance::AvoidanceManeuver {
        rpo_core::mission::avoidance::AvoidanceManeuver {
            epoch,
            dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
            maneuver_location_rad: 0.0,
            post_avoidance_poca_km: 0.5,
            fuel_cost_km_s: 0.001,
            correction_type: rpo_core::mission::avoidance::CorrectionType::InPlane,
            leg_index,
        }
    }

    /// Build a minimal `WaypointMission` with one leg of given TOF starting at `dep_epoch`.
    fn make_one_leg_mission(dep_epoch: hifitime::Epoch, tof_s: f64) -> rpo_core::mission::types::WaypointMission {
        use rpo_core::mission::types::{Maneuver, ManeuverLeg};
        use rpo_core::types::QuasiNonsingularROE;
        let zero_dv = Vector3::zeros();
        let arr_epoch = dep_epoch + hifitime::Duration::from_seconds(tof_s);

        rpo_core::mission::types::WaypointMission {
            legs: vec![ManeuverLeg {
                departure_maneuver: Maneuver { dv_ric_km_s: zero_dv, epoch: dep_epoch },
                arrival_maneuver: Maneuver { dv_ric_km_s: zero_dv, epoch: arr_epoch },
                tof_s,
                total_dv_km_s: 0.0,
                pre_departure_roe: QuasiNonsingularROE::default(),
                post_departure_roe: QuasiNonsingularROE::default(),
                departure_chief_mean: rpo_core::test_helpers::iss_like_elements(),
                pre_arrival_roe: QuasiNonsingularROE::default(),
                post_arrival_roe: QuasiNonsingularROE::default(),
                arrival_chief_mean: rpo_core::test_helpers::iss_like_elements(),
                trajectory: vec![],
                from_position_ric_km: Vector3::zeros(),
                to_position_ric_km: Vector3::zeros(),
                target_velocity_ric_km_s: Vector3::zeros(),
                iterations: 0,
                position_error_km: 0.0,
            }],
            total_dv_km_s: 0.0,
            total_duration_s: tof_s,
            safety: None,
            covariance: None,
            eclipse: None,
        }
    }

    /// Verify `convert_cola_to_burns` computes correct `elapsed_s` from epoch delta.
    #[test]
    fn test_convert_cola_to_burns_epoch_to_elapsed() {
        let dep_epoch = test_epoch();
        let tof_s = 4200.0;
        let mission = make_one_leg_mission(dep_epoch, tof_s);

        // COLA maneuver at 2000s into the leg
        let cola_epoch = dep_epoch + hifitime::Duration::from_seconds(2000.0);
        let maneuvers = vec![make_avoidance(cola_epoch, 0)];

        let burns = super::convert_cola_to_burns(Some(&maneuvers), &mission)
            .expect("conversion should succeed");

        assert_eq!(burns.len(), 1);
        assert!(
            (burns[0].elapsed_s - 2000.0).abs() < rpo_core::constants::ELAPSED_TIME_TOL_S,
            "elapsed_s should be ~2000",
        );
        assert_eq!(burns[0].leg_index, 0);
    }

    /// Verify `convert_cola_to_burns` returns error for out-of-bounds epochs.
    #[test]
    fn test_convert_cola_to_burns_out_of_bounds() {
        let dep_epoch = test_epoch();
        let tof_s = 4200.0;
        let mission = make_one_leg_mission(dep_epoch, tof_s);

        // Epoch before leg start -> elapsed_s <= 0
        let before = dep_epoch - hifitime::Duration::from_seconds(100.0);
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(before, 0)]), &mission);
        assert!(result.is_err(), "epoch before leg start should fail");

        // Epoch after leg end -> elapsed_s >= tof_s
        let after = dep_epoch + hifitime::Duration::from_seconds(tof_s + 100.0);
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(after, 0)]), &mission);
        assert!(result.is_err(), "epoch after leg end should fail");

        // Epoch exactly at departure -> elapsed_s = 0 (boundary, should fail)
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(dep_epoch, 0)]), &mission);
        assert!(result.is_err(), "epoch at exact departure should fail");
    }

    /// Verify `convert_cola_to_burns` returns empty vec for None input.
    #[test]
    fn test_convert_cola_to_burns_none_returns_empty() {
        let mission = make_one_leg_mission(test_epoch(), 4200.0);
        let burns = super::convert_cola_to_burns(None, &mission)
            .expect("None input should succeed");
        assert!(burns.is_empty());
    }

    // =========================================================================
    // Post-COLA Minimum Distance Extraction Tests
    // =========================================================================

    use crate::nyx_bridge::TimedState;
    use rpo_core::types::state::StateVector;

    /// Tolerance for exact distance comparisons in synthetic axis-aligned test geometries.
    /// Computed norms are exact to machine precision; this guards against floating-point
    /// representation noise only.
    const SYNTHETIC_DISTANCE_TOL: f64 = 1e-12;

    /// Build a `TimedState` with the given ECI position (km) and elapsed time (s).
    fn make_timed_state(elapsed_s: f64, position_eci_km: Vector3<f64>) -> TimedState {
        TimedState {
            elapsed_s,
            state: StateVector {
                epoch: rpo_core::test_helpers::test_epoch(),
                position_eci_km,
                velocity_eci_km_s: Vector3::zeros(),
            },
        }
    }

    #[test]
    fn post_cola_min_distance_basic() {
        // Chief at origin, deputy at varying distances
        let chief: Vec<TimedState> = (0..10)
            .map(|i| make_timed_state(f64::from(i) * 100.0, Vector3::zeros()))
            .collect();

        // Deputy: pre-COLA far away, post-COLA closer with known minimum
        let mut deputy: Vec<TimedState> = Vec::with_capacity(10);
        for i in 0..5 {
            // Pre-COLA: 10 km away
            deputy.push(make_timed_state(
                f64::from(i) * 100.0,
                Vector3::new(10.0, 0.0, 0.0),
            ));
        }
        // Post-COLA: distances [3.0, 1.5, 0.8, 2.0, 4.0] km
        let post_cola_distances = [3.0, 1.5, 0.8, 2.0, 4.0];
        for (j, &d) in post_cola_distances.iter().enumerate() {
            deputy.push(make_timed_state(
                f64::from(u32::try_from(5 + j).unwrap()) * 100.0,
                Vector3::new(d, 0.0, 0.0),
            ));
        }

        let cola_split_index = 5;
        let min_dist = super::compute_post_cola_min_distance(&chief, &deputy, cola_split_index);
        assert!(min_dist.is_some(), "should have post-COLA samples");

        let expected_min = 0.8;
        let actual = min_dist.unwrap();
        assert!(
            (actual - expected_min).abs() < SYNTHETIC_DISTANCE_TOL,
            "min distance should be {expected_min}, got {actual}",
        );
    }

    #[test]
    fn post_cola_min_distance_empty_returns_none() {
        let chief = vec![make_timed_state(0.0, Vector3::zeros())];
        let deputy = vec![make_timed_state(0.0, Vector3::new(5.0, 0.0, 0.0))];

        // cola_split_index at end of array — no post-COLA samples
        let result = super::compute_post_cola_min_distance(&chief, &deputy, 1);
        assert!(result.is_none(), "should return None when no post-COLA samples");
    }

    #[test]
    fn post_cola_min_distance_3d_norm() {
        // Verify 3D norm computation (not just x-component)
        let chief = vec![
            make_timed_state(0.0, Vector3::zeros()),
            make_timed_state(100.0, Vector3::zeros()),
        ];
        let deputy = vec![
            make_timed_state(0.0, Vector3::new(10.0, 0.0, 0.0)),
            make_timed_state(100.0, Vector3::new(0.3, 0.4, 0.0)), // norm = 0.5
        ];

        let min_dist = super::compute_post_cola_min_distance(&chief, &deputy, 1);
        assert!(min_dist.is_some());
        let actual = min_dist.unwrap();
        assert!(
            (actual - 0.5).abs() < SYNTHETIC_DISTANCE_TOL,
            "3D norm should be 0.5, got {actual}",
        );
    }

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

    /// Relative tolerance for R/C separation comparison (analytical vs numerical).
    /// Different sampling density + mean/osculating offset justify 50%.
    const SAFETY_RC_RELATIVE_TOL: f64 = 0.50;

    /// Absolute tolerance for 3D distance comparison (km).
    /// Same effects as R/C; absolute because 3D distance can be small.
    const SAFETY_3D_ABSOLUTE_TOL_KM: f64 = 0.5;

    /// Relative tolerance for e/i vector separation comparison.
    /// ROE-level drift O(1e-5) over 1-3 orbits.
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

    /// Maximum relative difference between analytical and nyx post-COLA minimum distance.
    /// Analytical uses linearized GVE (inverse Gauss variational equations) while nyx uses
    /// full nonlinear dynamics with J2, drag, SRP, and third-body perturbations.
    /// The 50% tolerance accommodates this fundamental model fidelity gap.
    const COLA_EFFECTIVENESS_RELATIVE_TOL: f64 = 0.50;

    /// Nyx post-COLA minimum distance must exceed this fraction of the target distance.
    /// Set at 50% to account for: (a) discrete sampling resolution (50 points/leg may
    /// miss the true minimum), (b) nonlinear dynamics divergence from the linearized
    /// avoidance solution, and (c) unmodeled perturbations (SRP, third-body).
    const COLA_EFFECTIVENESS_THRESHOLD_FRACTION: f64 = 0.50;

    /// Integrator restart tolerance (km) for `propagate_leg_with_cola_zero_dv_matches_single_segment`:
    /// two independent nyx segment propagations vs one continuous propagation accumulate
    /// different rounding noise. 1 m is well within operational significance.
    const RESTART_TOL_KM: f64 = 0.001;

    /// COLA sample-split fraction for `propagate_leg_with_cola_sample_split`,
    /// expressed as an integer percentage so the expected split count can be
    /// computed with pure integer arithmetic (no f64 → usize cast).
    /// The synthetic COLA fires at this fraction of the way through the leg.
    const COLA_SAMPLE_PERCENT: u32 = 30;

    /// Absolute tolerance (samples) on the pre-COLA / post-COLA split count. Allows
    /// +/-5 samples of slop for discrete step rounding inside the nyx sampler.
    const COLA_SAMPLE_SPLIT_SLOP: usize = 5;

    /// Upper-bound slop on total samples returned by `propagate_leg_with_cola`: the
    /// nyx sampler may emit up to 4 extra points for endpoint inclusion across the
    /// two segments, so the total can exceed `samples_per_leg` by at most this many.
    const COLA_TOTAL_SAMPLE_SLOP: usize = 4;

    /// Default nyx sample count per mission leg used by the full-physics
    /// validation tests. 50 samples gives enough resolution to characterize
    /// per-sample position error without making runs prohibitively slow.
    const DEFAULT_VALIDATION_SAMPLES_PER_LEG: u32 = 50;

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
            iss_formation_roe, plan_and_validate, PlanAndValidateInput, ValidationContext,
        };

        let formation_roe = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);
        let period = ctx.chief_ke.period().unwrap();
        let waypoints = [Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        }];

        let (_, report) = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                formation_roe,
                waypoints: &waypoints,
                config: &MissionConfig::default(),
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &super::ColaValidationInput::default(),
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
            iss_formation_roe, plan_and_validate, PlanAndValidateInput, ValidationContext,
        };

        let formation_roe = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);
        let period = ctx.chief_ke.period().unwrap();
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

        let (_, report) = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                formation_roe,
                waypoints: &waypoints,
                config: &MissionConfig::default(),
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &super::ColaValidationInput::default(),
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
        use test_scenario::{plan_and_validate, PlanAndValidateInput, ValidationContext};

        let ctx = ValidationContext::iss_colocated();
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig {
            dry_mass_kg: 200.0,
            drag_area_m2: 2.0,
            ..SpacecraftConfig::SERVICER_500KG
        };

        // Step 1: Extract DMF rates (uses the shared almanac from `ctx`).
        let drag = nyx_bridge::extract_dmf_rates(
            &ctx.chief_sv,
            &ctx.deputy_sv,
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
        let period = ctx.chief_ke.period().unwrap();
        let waypoints = [Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        }];
        let config = MissionConfig::default();
        let default_cola = super::ColaValidationInput::default();

        let drag_propagator = PropagationModel::J2DragStm { drag };
        let (_, drag_report) = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                formation_roe: QuasiNonsingularROE::default(),
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
        let (_, j2_report) = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                formation_roe: QuasiNonsingularROE::default(),
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
            iss_formation_roe, plan_and_validate, PlanAndValidateInput, ValidationContext,
        };

        // Perpendicular e/i vectors: dex along ex, diy along iy -> meaningful separation.
        let formation_roe = iss_formation_roe(0.5, 0.0, 0.0, 0.5);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);

        let period = ctx.chief_ke.period().unwrap();
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

        let (_, report) = plan_and_validate(
            &ctx,
            &PlanAndValidateInput {
                formation_roe,
                waypoints: &waypoints,
                config: &config,
                propagator: &PropagationModel::J2Stm,
                chief_config: SpacecraftConfig::SERVICER_500KG,
                deputy_config: SpacecraftConfig::SERVICER_500KG,
                samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
                cola_input: &super::ColaValidationInput::default(),
            },
        );

        // Analytical safety must be present (we enabled it via SafetyConfig).
        let analytical = report
            .analytical_safety
            .as_ref()
            .expect("analytical safety should be present (SafetyConfig enabled)");
        assert_safety_agreement(analytical, &report.numerical_safety);
    }

    // =========================================================================
    // COLA Two-Segment Propagation Tests (require nyx)
    // =========================================================================

    /// Two-segment propagation with zero COLA delta-v should produce the same
    /// trajectory as single-segment `propagate_leg_parallel` within tolerance.
    ///
    /// This is the fundamental invariant of `propagate_leg_with_cola`: when
    /// the COLA impulse is zero, splitting the leg into two segments and
    /// re-joining should not alter the trajectory beyond floating-point noise.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn propagate_leg_with_cola_zero_dv_matches_single_segment() {
        use rpo_core::mission::waypoints::plan_waypoint_mission;
        use rpo_core::test_helpers::deputy_from_roe;
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        let formation_roe = QuasiNonsingularROE {
            da: 0.0, dlambda: 0.0,
            dex: 0.3 / a, dey: -0.2 / a,
            dix: 0.2 / a, diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState { roe: formation_roe, chief: chief_ke, epoch };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let mission = plan_waypoint_mission(&departure, &[waypoint], &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let samples: u32 = 50;
        let leg = &mission.legs[0];

        let ctx = super::LegPropagationCtx {
            samples_per_leg: samples,
            chief_config: &SpacecraftConfig::SERVICER_500KG,
            deputy_config: &SpacecraftConfig::SERVICER_500KG,
            almanac: &almanac,
        };

        // Single-segment baseline (no COLA)
        let (chief_baseline, deputy_baseline) = super::propagate_leg_parallel(
            &chief_sv, &deputy_sv, leg, &ctx,
        ).expect("baseline propagation should succeed");

        // Two-segment with zero COLA delta-v at mid-leg
        let zero_cola = super::ColaBurn {
            leg_index: 0,
            elapsed_s: leg.tof_s * 0.5,
            dv_ric_km_s: Vector3::zeros(),
        };
        let cola_out = super::propagate_leg_with_cola(
            &chief_sv, &deputy_sv, leg, &zero_cola, &ctx,
        ).expect("COLA propagation should succeed");

        // Compare final states -- should match within nyx integrator tolerance.
        // The two-segment approach re-initializes the integrator at the split point,
        // so we allow a small tolerance for integrator restart transients.
        let chief_final_baseline = &chief_baseline.last().unwrap().state;
        let chief_final_cola = &cola_out.chief_results.last().unwrap().state;
        let deputy_final_baseline = &deputy_baseline.last().unwrap().state;
        let deputy_final_cola = &cola_out.deputy_results.last().unwrap().state;

        let chief_pos_diff = (chief_final_baseline.position_eci_km - chief_final_cola.position_eci_km).norm();
        let deputy_pos_diff = (deputy_final_baseline.position_eci_km - deputy_final_cola.position_eci_km).norm();

        eprintln!("Zero-COLA vs baseline: chief dpos={chief_pos_diff:.6} km, deputy dpos={deputy_pos_diff:.6} km");

        assert!(
            chief_pos_diff < RESTART_TOL_KM,
            "chief final position diverged: {chief_pos_diff:.6} km (expected < {RESTART_TOL_KM})"
        );
        assert!(
            deputy_pos_diff < RESTART_TOL_KM,
            "deputy final position diverged: {deputy_pos_diff:.6} km (expected < {RESTART_TOL_KM})"
        );
    }

    /// Verify sample allocation in `propagate_leg_with_cola` produces
    /// approximately `samples_per_leg` total samples split proportionally
    /// across the two segments.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn propagate_leg_with_cola_sample_split() {
        use rpo_core::mission::waypoints::plan_waypoint_mission;
        use rpo_core::test_helpers::deputy_from_roe;
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        let formation_roe = QuasiNonsingularROE {
            da: 0.0, dlambda: 0.0,
            dex: 0.3 / a, dey: -0.2 / a,
            dix: 0.2 / a, diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState { roe: formation_roe, chief: chief_ke, epoch };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let mission = plan_waypoint_mission(&departure, &[waypoint], &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let samples: u32 = 50;
        let leg = &mission.legs[0];

        let ctx = super::LegPropagationCtx {
            samples_per_leg: samples,
            chief_config: &SpacecraftConfig::SERVICER_500KG,
            deputy_config: &SpacecraftConfig::SERVICER_500KG,
            almanac: &almanac,
        };

        // COLA fires at COLA_SAMPLE_PERCENT % of the leg tof.
        let cola = super::ColaBurn {
            leg_index: 0,
            elapsed_s: leg.tof_s * f64::from(COLA_SAMPLE_PERCENT) / 100.0,
            dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
        };
        let out = super::propagate_leg_with_cola(
            &chief_sv, &deputy_sv, leg, &cola, &ctx,
        ).expect("COLA propagation should succeed");

        // cola_split_index marks where segment 2 begins
        let n1 = out.cola_split_index;
        let n_total = out.chief_results.len();
        let n2 = n_total - n1;

        eprintln!("Sample split: n1={n1}, n2={n2}, total={n_total}, cola_split_index={}", out.cola_split_index);

        // n1 + n2 should be close to samples_per_leg (nyx may return up to
        // COLA_TOTAL_SAMPLE_SLOP extra points for endpoint inclusion).
        let samples_usize = usize::try_from(samples).unwrap();
        assert!(
            n_total >= samples_usize && n_total <= samples_usize + COLA_TOTAL_SAMPLE_SLOP,
            "total samples {n_total} should be approximately {samples} (+/-{COLA_TOTAL_SAMPLE_SLOP})"
        );

        let expected_n1 = usize::try_from(rpo_core::constants::round_half_up_percent(
            samples,
            COLA_SAMPLE_PERCENT,
        ))
        .unwrap();
        assert!(
            n1.abs_diff(expected_n1) <= COLA_SAMPLE_SPLIT_SLOP,
            "n1={n1} should be approximately {expected_n1} (COLA at {COLA_SAMPLE_PERCENT}% of leg)"
        );
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
            iss_formation_roe, plan_mission, validate_planned, PlanAndValidateInput,
            ValidationContext,
        };

        // Deputy: formation sized so POCA ~ 0.1 km (below 0.2 km threshold).
        // Minimum distance ~ a * min(de, di). For de=di=0.1/a: min distance ~ 0.1 km.
        // This gives scale = target/poca ~ 2, keeping COLA delta-v affordable.
        let formation_roe = iss_formation_roe(0.1, 0.0, 0.0, 0.1);
        let ctx = ValidationContext::iss_with_formation(&formation_roe);

        // Waypoint: along-track displacement, ~0.8 period.
        // Along-track (T) motion is cheapest for near-circular chief orbits.
        let period = ctx.chief_ke.period().unwrap();
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
        let default_cola = super::ColaValidationInput::default();
        let mut input = PlanAndValidateInput {
            formation_roe,
            waypoints: &waypoints,
            config: &config,
            propagator: &propagator,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
            samples_per_leg: DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            cola_input: &default_cola,
        };
        let mission = plan_mission(&ctx, &input);

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
        let cola_burns = super::convert_cola_to_burns(Some(&maneuvers), &mission)
            .expect("COLA burn conversion should succeed");
        let cola_input = super::ColaValidationInput {
            burns: cola_burns,
            analytical_maneuvers: maneuvers.clone(),
            target_distance_km: Some(cola_config.target_distance_km),
        };

        // Now validate the (same) planned mission against nyx, passing the
        // populated COLA input so the report carries effectiveness data.
        input.cola_input = &cola_input;
        let report = validate_planned(&ctx, &mission, &input);

        // Assert effectiveness data is populated, then check each entry.
        assert!(
            !report.cola_effectiveness.is_empty(),
            "COLA effectiveness should be populated when COLA burns are present",
        );
        for eff in &report.cola_effectiveness {
            assert_cola_effectiveness(eff);
        }
    }

    // =========================================================================
    // Unified propagate_leg Sampling Parity Tests (Task 1 of CLI report audit)
    // =========================================================================

    /// Build a minimal single-leg `ManeuverLeg` with the given TOF (s) and
    /// zero departure/arrival impulses. Chief mean elements are ISS-like.
    fn make_zero_dv_iss_leg(tof_s: f64) -> rpo_core::mission::types::ManeuverLeg {
        use rpo_core::mission::types::{Maneuver, ManeuverLeg};
        use rpo_core::types::QuasiNonsingularROE;
        let dep_epoch = test_epoch();
        let arr_epoch = dep_epoch + hifitime::Duration::from_seconds(tof_s);
        ManeuverLeg {
            departure_maneuver: Maneuver { dv_ric_km_s: Vector3::zeros(), epoch: dep_epoch },
            arrival_maneuver: Maneuver { dv_ric_km_s: Vector3::zeros(), epoch: arr_epoch },
            tof_s,
            total_dv_km_s: 0.0,
            pre_departure_roe: QuasiNonsingularROE::default(),
            post_departure_roe: QuasiNonsingularROE::default(),
            departure_chief_mean: iss_like_elements(),
            pre_arrival_roe: QuasiNonsingularROE::default(),
            post_arrival_roe: QuasiNonsingularROE::default(),
            arrival_chief_mean: iss_like_elements(),
            trajectory: vec![],
            from_position_ric_km: Vector3::zeros(),
            to_position_ric_km: Vector3::zeros(),
            target_velocity_ric_km_s: Vector3::zeros(),
            iterations: 0,
            position_error_km: 0.0,
        }
    }

    /// Build chief/deputy initial ECI states for an ISS-like ~300m formation
    /// with nonzero `dex`/`dey`/`dix`. Used by the sampling-parity tests.
    fn make_iss_like_formation_states() -> (StateVector, StateVector) {
        use rpo_core::test_helpers::deputy_from_roe;
        use rpo_core::types::QuasiNonsingularROE;

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;
        let formation_roe = QuasiNonsingularROE {
            da: 0.0, dlambda: 0.0,
            dex: 0.3 / a, dey: -0.2 / a,
            dix: 0.2 / a, diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();
        (chief_sv, deputy_sv)
    }

    /// Unit test: segment 1 of `propagate_leg` must be bit-identical (within
    /// float noise) across burn = None, burn = Some(zero), burn = Some(real),
    /// given the same `elapsed_s`. This is the load-bearing invariant that
    /// guarantees pre-COLA and post-COLA paths see the same sampling grid and
    /// the same physical trajectory in the pre-burn window.
    #[test]
    #[ignore = "Requires MetaAlmanac (network on first run)"]
    fn propagate_leg_segment_1_is_impulse_independent() {
        /// Position round-off budget. Segment 1 samples must be bit-identical
        /// across burn variants; this only absorbs f64 accumulation noise over
        /// a few hundred RK steps at LEO scale (sub-picometer).
        const FLOAT_NOISE_KM: f64 = 1.0e-12;
        /// Epoch round-off budget. Guards against a future "optimization"
        /// that shifts sample times based on impulse presence -- f64
        /// accumulation over `0..tof_s` is sub-nanosecond.
        const EPOCH_NOISE_S: f64 = 1.0e-9;

        let (chief_state, deputy_state) = make_iss_like_formation_states();
        let leg = make_zero_dv_iss_leg(4200.0);
        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let ctx = super::LegPropagationCtx {
            samples_per_leg: 50,
            chief_config: &SpacecraftConfig::SERVICER_500KG,
            deputy_config: &SpacecraftConfig::SERVICER_500KG,
            almanac: &almanac,
        };

        let burn_none = super::MidLegBurn {
            elapsed_s: 709.0,
            impulse_dv_ric_km_s: None,
        };
        let burn_zero = super::MidLegBurn {
            elapsed_s: 709.0,
            impulse_dv_ric_km_s: Some(Vector3::zeros()),
        };
        let burn_real = super::MidLegBurn {
            elapsed_s: 709.0,
            impulse_dv_ric_km_s: Some(Vector3::new(0.001, 0.0, 0.0)),
        };

        let out_none = super::propagate_leg(&chief_state, &deputy_state, &leg, Some(&burn_none), &ctx)
            .expect("impulse=None should succeed");
        let out_zero = super::propagate_leg(&chief_state, &deputy_state, &leg, Some(&burn_zero), &ctx)
            .expect("impulse=zero should succeed");
        let out_real = super::propagate_leg(&chief_state, &deputy_state, &leg, Some(&burn_real), &ctx)
            .expect("impulse=real should succeed");

        let split = out_none.split_index.expect("burn Some => split_index Some");
        assert_eq!(out_zero.split_index, Some(split), "same burn elapsed_s => same split index");
        assert_eq!(out_real.split_index, Some(split), "same burn elapsed_s => same split index");

        // Sample counts must agree across the three burn variants.
        assert_eq!(out_none.chief_results.len(), out_zero.chief_results.len());
        assert_eq!(out_none.chief_results.len(), out_real.chief_results.len());
        assert_eq!(out_none.deputy_results.len(), out_zero.deputy_results.len());
        assert_eq!(out_none.deputy_results.len(), out_real.deputy_results.len());

        // Segment 1 (0..split) must be bit-identical (float noise only) across
        // all three variants. Chief never receives the deputy's impulse, so
        // chief is even post-split impulse-independent -- but we only assert
        // the invariant we care about: pre-split identity.
        for i in 0..split {
            let chief_diff_zero_km = (out_none.chief_results[i].state.position_eci_km
                - out_zero.chief_results[i].state.position_eci_km).norm();
            let chief_diff_real_km = (out_none.chief_results[i].state.position_eci_km
                - out_real.chief_results[i].state.position_eci_km).norm();
            let deputy_diff_zero_km = (out_none.deputy_results[i].state.position_eci_km
                - out_zero.deputy_results[i].state.position_eci_km).norm();
            let deputy_diff_real_km = (out_none.deputy_results[i].state.position_eci_km
                - out_real.deputy_results[i].state.position_eci_km).norm();

            assert!(chief_diff_zero_km < FLOAT_NOISE_KM, "chief sample {i} diverges None vs zero ({chief_diff_zero_km:e} km)");
            assert!(chief_diff_real_km < FLOAT_NOISE_KM, "chief sample {i} diverges None vs real ({chief_diff_real_km:e} km)");
            assert!(deputy_diff_zero_km < FLOAT_NOISE_KM, "deputy sample {i} diverges None vs zero ({deputy_diff_zero_km:e} km)");
            assert!(deputy_diff_real_km < FLOAT_NOISE_KM, "deputy sample {i} diverges None vs real ({deputy_diff_real_km:e} km)");

            // Sample epoch match -- guards against any future attempt to
            // "optimize" by shifting sample times based on impulse presence.
            assert!((out_none.chief_results[i].elapsed_s - out_zero.chief_results[i].elapsed_s).abs() < EPOCH_NOISE_S);
            assert!((out_none.chief_results[i].elapsed_s - out_real.chief_results[i].elapsed_s).abs() < EPOCH_NOISE_S);
        }

        // Sanity: segment 2 deputy should diverge between None and real
        // (impulse actually did something). Not the invariant under test,
        // just a guard against an accidental no-op implementation.
        //
        // Note: nyx samples are `n_samples + 1` points (initial + n samples),
        // so `chief_results[split]` is the first sample of segment 2 at its
        // local t=0, i.e. the *position* at the split point. Impulse changes
        // velocity, not position, so we must look at the next sample (at
        // split+1) where the velocity difference has had time to integrate
        // into a nonzero position difference.
        let last_idx = out_real.deputy_results.len() - 1;
        if split < last_idx {
            let post_burn_divergence_km = (out_none.deputy_results[split + 1].state.position_eci_km
                - out_real.deputy_results[split + 1].state.position_eci_km).norm();
            assert!(
                post_burn_divergence_km > 0.0,
                "segment 2 must show the impulse effect; real impulse produced no divergence at split+1 \
                 (delta = {post_burn_divergence_km:e} km)"
            );
        }
    }

    /// End-to-end regression: pre-COLA baseline safety must match post-COLA
    /// safety in the pre-burn window, since COLA cannot affect the pre-burn
    /// portion of the trajectory. This is the test that would have caught
    /// the sampling artifact where post-COLA looked worse than pre-COLA in
    /// the validate.md Safety Comparison table.
    #[test]
    #[ignore = "Requires MetaAlmanac (network on first run)"]
    fn pre_cola_and_post_cola_pre_burn_window_match_within_sampling_tolerance() {
        use rpo_core::mission::waypoints::plan_waypoint_mission;
        use rpo_core::test_helpers::deputy_from_roe;
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::types::{DepartureState, QuasiNonsingularROE};

        /// Sampling-grid regression guard. With the unified `propagate_leg`,
        /// the pre-burn window should be bit-identical; this tolerance only
        /// absorbs f64 round-off across the trajectory-safety reduction
        /// (norm + min). A millimeter is ~6 orders of magnitude tighter than
        /// the sampling artifact we are regressing against (~16 m).
        const SAMPLING_REGRESSION_TOL_KM: f64 = 1.0e-6;

        /// Post-refactor numerical-noise floor. With unified sampling the
        /// pre-burn window should be bit-identical; this absorbs only the
        /// f64 accumulation that remains in `analyze_trajectory_safety`'s
        /// norm + min reduction. 1e-9 km = 1 micron, which is ~7 orders of
        /// magnitude tighter than the ~16 m artifact the unification removes.
        const NUMERICAL_NOISE_TOL_KM: f64 = 1.0e-9;

        // Build a single-leg mission with a nonzero formation so the ROE
        // trajectory is interesting. COLA burn will be injected at mid-leg.
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;
        let formation_roe = QuasiNonsingularROE {
            da: 0.0, dlambda: 0.0,
            dex: 0.3 / a, dey: -0.2 / a,
            dix: 0.2 / a, diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };
        let departure = DepartureState { roe: formation_roe, chief: chief_ke, epoch };
        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let mission = plan_waypoint_mission(&departure, &[waypoint], &mission_config, &propagator)
            .expect("mission planning should succeed");

        // COLA burn at 709s (synthetic small burn; we only care about sampling
        // grid parity in the pre-burn window, not that COLA actually resolves
        // anything). The dv is small enough not to blow up the trajectory but
        // large enough that the post-burn segment diverges from the baseline.
        let leg_tof_s = mission.legs[0].tof_s;
        assert!(709.0 < leg_tof_s, "test burn at 709s must be inside leg tof");
        let cola = super::ColaValidationInput {
            burns: vec![super::ColaBurn {
                leg_index: 0,
                elapsed_s: 709.0,
                dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
            }],
            analytical_maneuvers: vec![],
            target_distance_km: None,
        };

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let val_config = super::ValidationConfig {
            samples_per_leg: 50,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let report = crate::validation::validate_mission_nyx(
            &mission, &chief_sv, &deputy_sv, &val_config, &cola, &almanac,
        )
        .expect("validation should succeed");

        let pre_cola = report
            .pre_cola_numerical_safety
            .as_ref()
            .expect("pre_cola_numerical_safety must be populated when COLA is present");
        let post_cola_min_3d_km = report.numerical_safety.operational.min_distance_3d_km;
        let pre_cola_min_3d_km = pre_cola.operational.min_distance_3d_km;

        eprintln!(
            "Pre-COLA min 3D: {pre_cola_min_3d_km:.12} km; Post-COLA min 3D: {post_cola_min_3d_km:.12} km"
        );

        // COLA can only reduce the post-burn minimum; it cannot affect
        // pre-burn. So pre_cola_min_3d must be <= post_cola_min_3d +
        // sampling_tolerance.
        let delta_km = (pre_cola_min_3d_km - post_cola_min_3d_km).abs();
        assert!(
            delta_km < SAMPLING_REGRESSION_TOL_KM
                || pre_cola_min_3d_km <= post_cola_min_3d_km + SAMPLING_REGRESSION_TOL_KM,
            "pre-COLA min ({pre_cola_min_3d_km:.9} km) must be <= post-COLA min ({post_cola_min_3d_km:.9} km) \
             within sampling tolerance; delta = {delta_km:.9} km"
        );

        // Additional assertion: this guards against any future change that
        // re-introduces sampling asymmetry. Micron-scale tolerance -- purely
        // numerical noise from the safety reduction.
        assert!(
            (pre_cola_min_3d_km - post_cola_min_3d_km).abs() < NUMERICAL_NOISE_TOL_KM
                || pre_cola_min_3d_km <= post_cola_min_3d_km,
            "pre-COLA min must equal post-COLA min or be strictly less (COLA only acts post-split); \
             pre = {pre_cola_min_3d_km:.12} km, post = {post_cola_min_3d_km:.12} km"
        );
    }
}

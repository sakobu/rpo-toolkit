//! Per-leg nyx propagation machinery: single- and two-segment (COLA-split)
//! trajectory generation, comparison-point assembly, and COLA effectiveness
//! collection.

use std::sync::Arc;

use anise::prelude::Almanac;
use nalgebra::Vector3;
use nyx_space::md::prelude::SpacecraftDynamics;

use rpo_core::elements::eci_ric_dcm::eci_to_ric_relative;
use rpo_core::mission::types::{ColaEffectivenessEntry, ManeuverLeg, ValidationPoint};
use rpo_core::propagation::propagator::PropagatedState;
use rpo_core::types::{SpacecraftConfig, StateVector};

use super::super::eclipse::EclipseSample;
use super::super::errors::ValidationError;
use super::super::statistics::find_closest_analytical_ric;
use super::cola::{ColaBurn, ColaValidationInput};
use crate::nyx_bridge::{
    apply_impulse, nyx_propagate_segment, query_anise_eclipse, state_to_orbit,
    ChiefDeputySnapshot, TimedState,
};

/// Output from comparing nyx-propagated states against analytical trajectory for one leg.
pub(super) struct LegComparisonOutput {
    /// Per-sample validation comparison points.
    pub(super) points: Vec<ValidationPoint>,
    /// Chief/deputy ECI snapshot pairs for safety analysis.
    pub(super) safety_pairs: Vec<ChiefDeputySnapshot>,
    /// Eclipse samples for validation (empty if eclipse validation disabled).
    pub(super) eclipse_samples: Vec<EclipseSample>,
}

/// Extract final chief/deputy ECI states from propagation results and apply arrival impulse.
///
/// Used at leg boundaries to thread state into the next leg or return updated
/// states from per-leg validation.
pub(super) fn advance_leg_states(
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

/// Build comparison points for a single leg of the mission.
///
/// Compares nyx-propagated chief/deputy states against analytical trajectory,
/// collects safety pairs and eclipse samples for downstream analysis.
///
/// When `cola_split_index` is `Some(n)`, points at index `>= n` are tagged
/// `post_cola = true` -- they follow a COLA burn and should be excluded from
/// fidelity statistics.
pub(super) fn build_leg_comparison_points(
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
    let mut eclipse_samples = if earth_frame.is_some() {
        Vec::with_capacity(chief_results.len())
    } else {
        Vec::new()
    };

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
pub(super) fn propagate_leg_parallel(
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
pub(super) struct MidLegBurn {
    /// Time from leg departure to the burn split point (seconds).
    pub(super) elapsed_s: f64,
    /// Delta-v in RIC frame (km/s), or `None` for a sampling-only split.
    pub(super) impulse_dv_ric_km_s: Option<Vector3<f64>>,
}

/// Output from [`propagate_leg`]: per-sample chief/deputy states and an
/// optional split index marking where segment 2 begins when a split was used.
pub(super) struct LegPropOutput {
    /// Chief ECI states sampled across the leg (single or two segments).
    pub(super) chief_results: Vec<TimedState>,
    /// Deputy ECI states sampled across the leg (single or two segments).
    pub(super) deputy_results: Vec<TimedState>,
    /// Index of the first post-split sample when `burn` was `Some`; `None`
    /// otherwise.
    pub(super) split_index: Option<usize>,
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
    ctx: &LegPropagationCtx<'_>,
) -> Result<(Vec<TimedState>, Vec<TimedState>), ValidationError> {
    let (chief_result, deputy_result) = rayon::join(
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                chief_state,
                duration_s,
                n_samples,
                ctx.chief_config,
                ctx.dynamics.clone(),
                ctx.almanac,
            )?)
        },
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                deputy_state,
                duration_s,
                n_samples,
                ctx.deputy_config,
                ctx.dynamics.clone(),
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
pub(super) fn propagate_leg(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    leg: &ManeuverLeg,
    burn: Option<&MidLegBurn>,
    ctx: &LegPropagationCtx<'_>,
) -> Result<LegPropOutput, ValidationError> {
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
/// pre-COLA safety pass (`compute_pre_cola_safety`), avoiding parameter
/// sprawl.
pub(in crate::validation) struct LegPropagationCtx<'a> {
    /// Number of intermediate comparison samples per leg.
    pub(in crate::validation) samples_per_leg: u32,
    /// Chief spacecraft properties.
    pub(in crate::validation) chief_config: &'a SpacecraftConfig,
    /// Deputy spacecraft properties.
    pub(in crate::validation) deputy_config: &'a SpacecraftConfig,
    /// ANISE almanac with ephemeris and frame data.
    pub(in crate::validation) almanac: &'a Arc<Almanac>,
    /// Pre-built full-physics dynamics (hoisted out of per-leg construction).
    pub(in crate::validation) dynamics: &'a SpacecraftDynamics,
}

/// Output from propagating a single leg with a mid-coast COLA impulse.
pub(super) struct ColaLegOutput {
    /// Chief ECI states sampled across both segments.
    pub(super) chief_results: Vec<TimedState>,
    /// Deputy ECI states sampled across both segments.
    pub(super) deputy_results: Vec<TimedState>,
    /// Index in results where post-COLA samples begin.
    pub(super) cola_split_index: usize,
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
pub(super) fn propagate_leg_with_cola(
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
pub(super) fn collect_cola_effectiveness(
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

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use rpo_core::constants::{
        TEST_F64_EPOCH_NOISE_S, TEST_F64_POSITION_NOISE_KM, TEST_INTEGRATOR_RESTART_TOL_KM,
    };

    use crate::nyx_bridge::TimedState;
    use rpo_core::test_helpers::{iss_like_elements, test_epoch};
    use rpo_core::types::state::StateVector;
    use rpo_core::types::SpacecraftConfig;

    use crate::validation::test_scenario;

    // =========================================================================
    // Post-COLA Minimum Distance Extraction Tests
    // =========================================================================

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
                f64::from(5_u32 + u32::try_from(j).unwrap()) * 100.0,
                Vector3::new(d, 0.0, 0.0),
            ));
        }

        let cola_split_index = 5;
        let min_dist = super::compute_post_cola_min_distance(&chief, &deputy, cola_split_index);
        assert!(min_dist.is_some(), "should have post-COLA samples");

        let expected_min = 0.8;
        let actual = min_dist.unwrap();
        assert!(
            (actual - expected_min).abs() < TEST_F64_POSITION_NOISE_KM,
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
            (actual - 0.5).abs() < TEST_F64_POSITION_NOISE_KM,
            "3D norm should be 0.5, got {actual}",
        );
    }

    // =========================================================================
    // COLA Two-Segment Propagation Tests (require nyx)
    // =========================================================================

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

    /// Two-segment propagation with zero COLA delta-v should produce the same
    /// trajectory as single-segment `propagate_leg_parallel` within tolerance.
    ///
    /// This is the fundamental invariant of `propagate_leg_with_cola`: when
    /// the COLA impulse is zero, splitting the leg into two segments and
    /// re-joining should not alter the trajectory beyond floating-point noise.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn propagate_leg_with_cola_zero_dv_matches_single_segment() {
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::propagation::propagator::PropagationModel;

        use test_scenario::{
            iss_formation_roe, leg_propagation_ctx_from_scenario, plan_mission,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG, PlanAndValidateInput, ValidationContext,
        };

        let formation = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation);

        let period = ctx.chief_elements.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };

        let default_cola = super::super::cola::ColaValidationInput::default();
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
        let leg = &mission.legs[0];

        let chief_cfg = SpacecraftConfig::SERVICER_500KG;
        let deputy_cfg = SpacecraftConfig::SERVICER_500KG;
        let dynamics = crate::nyx_bridge::build_full_physics_dynamics(&ctx.almanac)
            .expect("dynamics should build from test almanac");
        let leg_ctx = leg_propagation_ctx_from_scenario(
            &ctx,
            &chief_cfg,
            &deputy_cfg,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            &dynamics,
        );

        // Single-segment baseline (no COLA).
        let (chief_baseline, deputy_baseline) = super::propagate_leg_parallel(
            &ctx.chief_state,
            &ctx.deputy_state,
            leg,
            &leg_ctx,
        )
        .expect("baseline propagation should succeed");

        // Two-segment with zero COLA delta-v at mid-leg.
        let zero_cola = super::super::cola::ColaBurn {
            leg_index: 0,
            elapsed_s: leg.tof_s * 0.5,
            dv_ric_km_s: Vector3::zeros(),
        };
        let cola_out = super::propagate_leg_with_cola(
            &ctx.chief_state,
            &ctx.deputy_state,
            leg,
            &zero_cola,
            &leg_ctx,
        )
        .expect("COLA propagation should succeed");

        let chief_final_baseline = &chief_baseline.last().unwrap().state;
        let chief_final_cola = &cola_out.chief_results.last().unwrap().state;
        let deputy_final_baseline = &deputy_baseline.last().unwrap().state;
        let deputy_final_cola = &cola_out.deputy_results.last().unwrap().state;

        let chief_pos_diff =
            (chief_final_baseline.position_eci_km - chief_final_cola.position_eci_km).norm();
        let deputy_pos_diff =
            (deputy_final_baseline.position_eci_km - deputy_final_cola.position_eci_km).norm();

        assert!(
            chief_pos_diff < TEST_INTEGRATOR_RESTART_TOL_KM,
            "chief final position diverged: {chief_pos_diff:.6} km \
             (expected < {TEST_INTEGRATOR_RESTART_TOL_KM})"
        );
        assert!(
            deputy_pos_diff < TEST_INTEGRATOR_RESTART_TOL_KM,
            "deputy final position diverged: {deputy_pos_diff:.6} km \
             (expected < {TEST_INTEGRATOR_RESTART_TOL_KM})"
        );
    }

    /// Verify sample allocation in `propagate_leg_with_cola` produces
    /// approximately `samples_per_leg` total samples split proportionally
    /// across the two segments.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn propagate_leg_with_cola_sample_split() {
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::propagation::propagator::PropagationModel;

        use test_scenario::{
            iss_formation_roe, leg_propagation_ctx_from_scenario, plan_mission,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG, PlanAndValidateInput, ValidationContext,
        };

        let formation = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation);

        let period = ctx.chief_elements.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(0.8 * period),
        };

        let default_cola = super::super::cola::ColaValidationInput::default();
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
        let leg = &mission.legs[0];

        let chief_cfg = SpacecraftConfig::SERVICER_500KG;
        let deputy_cfg = SpacecraftConfig::SERVICER_500KG;
        let dynamics = crate::nyx_bridge::build_full_physics_dynamics(&ctx.almanac)
            .expect("dynamics should build from test almanac");
        let leg_ctx = leg_propagation_ctx_from_scenario(
            &ctx,
            &chief_cfg,
            &deputy_cfg,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            &dynamics,
        );

        // COLA fires at COLA_SAMPLE_PERCENT % of the leg tof.
        let cola = super::super::cola::ColaBurn {
            leg_index: 0,
            elapsed_s: leg.tof_s * f64::from(COLA_SAMPLE_PERCENT) / 100.0,
            dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
        };
        let out = super::propagate_leg_with_cola(
            &ctx.chief_state,
            &ctx.deputy_state,
            leg,
            &cola,
            &leg_ctx,
        )
        .expect("COLA propagation should succeed");

        let n1 = out.cola_split_index;
        let n_total = out.chief_results.len();

        let samples_usize = usize::try_from(DEFAULT_VALIDATION_SAMPLES_PER_LEG).unwrap();
        assert!(
            n_total >= samples_usize && n_total <= samples_usize + COLA_TOTAL_SAMPLE_SLOP,
            "total samples {n_total} should be approximately \
             {DEFAULT_VALIDATION_SAMPLES_PER_LEG} (+/-{COLA_TOTAL_SAMPLE_SLOP})"
        );

        let expected_n1 = usize::try_from(rpo_core::constants::round_half_up_percent(
            DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            COLA_SAMPLE_PERCENT,
        ))
        .unwrap();
        assert!(
            n1.abs_diff(expected_n1) <= COLA_SAMPLE_SPLIT_SLOP,
            "n1={n1} should be approximately {expected_n1} \
             (COLA at {COLA_SAMPLE_PERCENT}% of leg)"
        );
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

    /// Unit test: segment 1 of `propagate_leg` must be bit-identical (within
    /// float noise) across burn = None, burn = Some(zero), burn = Some(real),
    /// given the same `elapsed_s`. This is the load-bearing invariant that
    /// guarantees pre-COLA and post-COLA paths see the same sampling grid and
    /// the same physical trajectory in the pre-burn window.
    #[test]
    #[ignore = "requires MetaAlmanac (network on first run)"]
    fn propagate_leg_segment_1_is_impulse_independent() {
        use test_scenario::{
            iss_formation_roe, leg_propagation_ctx_from_scenario,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG, ValidationContext,
        };

        /// Mid-coast burn time, prime-ish to avoid coinciding with sampling grid points.
        const MID_COAST_BURN_ELAPSED_S: f64 = 709.0;

        let formation = iss_formation_roe(0.3, -0.2, 0.2, 0.0);
        let ctx = ValidationContext::iss_with_formation(&formation);
        let leg = make_zero_dv_iss_leg(4200.0);

        let chief_cfg = SpacecraftConfig::SERVICER_500KG;
        let deputy_cfg = SpacecraftConfig::SERVICER_500KG;
        let dynamics = crate::nyx_bridge::build_full_physics_dynamics(&ctx.almanac)
            .expect("dynamics should build from test almanac");
        let leg_ctx = leg_propagation_ctx_from_scenario(
            &ctx,
            &chief_cfg,
            &deputy_cfg,
            DEFAULT_VALIDATION_SAMPLES_PER_LEG,
            &dynamics,
        );

        let burn_none = super::MidLegBurn {
            elapsed_s: MID_COAST_BURN_ELAPSED_S,
            impulse_dv_ric_km_s: None,
        };
        let burn_zero = super::MidLegBurn {
            elapsed_s: MID_COAST_BURN_ELAPSED_S,
            impulse_dv_ric_km_s: Some(Vector3::zeros()),
        };
        let burn_real = super::MidLegBurn {
            elapsed_s: MID_COAST_BURN_ELAPSED_S,
            impulse_dv_ric_km_s: Some(Vector3::new(0.001, 0.0, 0.0)),
        };

        let out_none = super::propagate_leg(
            &ctx.chief_state, &ctx.deputy_state, &leg, Some(&burn_none), &leg_ctx,
        )
        .expect("impulse=None should succeed");
        let out_zero = super::propagate_leg(
            &ctx.chief_state, &ctx.deputy_state, &leg, Some(&burn_zero), &leg_ctx,
        )
        .expect("impulse=zero should succeed");
        let out_real = super::propagate_leg(
            &ctx.chief_state, &ctx.deputy_state, &leg, Some(&burn_real), &leg_ctx,
        )
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

            assert!(chief_diff_zero_km < TEST_F64_POSITION_NOISE_KM, "chief sample {i} diverges None vs zero ({chief_diff_zero_km:e} km)");
            assert!(chief_diff_real_km < TEST_F64_POSITION_NOISE_KM, "chief sample {i} diverges None vs real ({chief_diff_real_km:e} km)");
            assert!(deputy_diff_zero_km < TEST_F64_POSITION_NOISE_KM, "deputy sample {i} diverges None vs zero ({deputy_diff_zero_km:e} km)");
            assert!(deputy_diff_real_km < TEST_F64_POSITION_NOISE_KM, "deputy sample {i} diverges None vs real ({deputy_diff_real_km:e} km)");

            // Sample epoch match -- guards against any future attempt to
            // "optimize" by shifting sample times based on impulse presence.
            assert!((out_none.chief_results[i].elapsed_s - out_zero.chief_results[i].elapsed_s).abs() < TEST_F64_EPOCH_NOISE_S);
            assert!((out_none.chief_results[i].elapsed_s - out_real.chief_results[i].elapsed_s).abs() < TEST_F64_EPOCH_NOISE_S);
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
}

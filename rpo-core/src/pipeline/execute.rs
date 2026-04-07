//! Pipeline execution: `execute_mission_from_transfer`, `replan_from_transfer`, `build_output`.
//!
//! These compose the core planning primitives (classify, waypoint
//! targeting, covariance, eclipse) into a complete mission pipeline.
//! Server-only wrappers (`compute_transfer`, `execute_mission`, `replan_mission`)
//! that require nyx-space live in `rpo-nyx`.

use crate::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;
use crate::mission::cola_assessment::{assess_cola, ColaAssessment};
use crate::mission::avoidance::ColaConfig;
use crate::mission::closest_approach::{find_closest_approaches, ClosestApproach};
use crate::mission::errors::MissionError;
use crate::elements::roe_to_ric::roe_to_ric;
use crate::mission::formation::{
    DriftPrediction, EnrichedWaypoint, FormationDesignReport,
    PerchEnrichmentResult, SafetyRequirements, TransitSafetyReport,
};
use crate::mission::formation::transit::{ei_separation_after, enrich_with_drift_compensation};
use crate::mission::formation::types::DriftCompensationStatus;
use super::types::EnrichmentSuggestion;
use crate::mission::formation::perch::enrich_perch;
use crate::mission::formation::safety_envelope::enrich_waypoint;
use crate::mission::formation::transit::assess_transit_safety;
use crate::mission::free_drift::{compute_free_drift, FreeDriftAnalysis};
use crate::mission::planning::compute_transfer_eclipse;
use crate::mission::types::PerchGeometry;
use crate::mission::waypoints::plan_waypoint_mission;
use crate::mission::waypoints::replan_from_waypoint;
use crate::propagation::covariance::{
    ric_accuracy_to_roe_covariance, CovarianceError, ManeuverUncertainty,
    MissionCovarianceReport, NavigationAccuracy,
};
use crate::propagation::propagator::{DragConfig, PropagationModel};
use crate::types::elements::KeplerianElements;
use crate::types::roe::QuasiNonsingularROE;
use crate::types::DepartureState;

use crate::mission::config::SafetyConfig;

use super::convert::{to_propagation_model, to_waypoints};
use super::errors::PipelineError;
use super::projections::{LeanPlanResult, TransferSummary};
use super::types::{PipelineInput, PipelineOutput, SafetyAnalysis, TransferResult, WaypointInput};

/// Number of eclipse evaluation points along the Lambert transfer arc.
const DEFAULT_TRANSFER_ECLIPSE_SAMPLES: u32 = 200;

/// Default delta-v budget for auto-triggered COLA (km/s).
/// 10 m/s = 0.01 km/s — matches CLI default, sufficient for typical
/// proximity-phase collision avoidance maneuvers.
const AUTO_COLA_BUDGET_KM_S: f64 = 0.01;

/// Compute free-drift analysis for each leg of a waypoint mission.
///
/// For each leg, propagates from the pre-departure ROE (burn skipped) for
/// the same TOF and runs safety analysis on the resulting trajectory.
///
/// Returns `None` if any leg fails (non-fatal).
#[must_use]
pub fn compute_free_drift_analysis(
    mission: &crate::mission::types::WaypointMission,
    propagator: &PropagationModel,
) -> Option<Vec<FreeDriftAnalysis>> {
    let mut results = Vec::with_capacity(mission.legs.len());
    for leg in &mission.legs {
        match compute_free_drift(
            &leg.pre_departure_roe,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            leg.tof_s,
            propagator,
            leg.trajectory.len().saturating_sub(1).max(2),
        ) {
            Ok(fd) => results.push(fd),
            Err(_) => return None,
        }
    }
    Some(results)
}

/// Compute refined closest-approach (POCA) for each leg of a waypoint mission.
///
/// For each leg, runs Brent-method refinement on the grid-sampled trajectory
/// to find exact conjunction time/position/distance. Returns `None` if any
/// leg fails (non-fatal).
#[must_use]
pub fn compute_poca_analysis(
    mission: &crate::mission::types::WaypointMission,
    propagator: &PropagationModel,
) -> Option<Vec<Vec<ClosestApproach>>> {
    let mut results = Vec::with_capacity(mission.legs.len());
    for (i, leg) in mission.legs.iter().enumerate() {
        match find_closest_approaches(
            &leg.trajectory,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            propagator,
            &leg.post_departure_roe,
            i,
        ) {
            Ok(pocas) => results.push(pocas),
            Err(_) => return None,
        }
    }
    flag_global_minimum(&mut results);
    Some(results)
}

/// Compute refined closest-approach (POCA) on free-drift trajectories.
///
/// For each free-drift trajectory, runs bracket detection + Brent refinement
/// using the pre-departure ROE (the free-drift departure state). Returns
/// `None` if any leg fails (non-fatal).
#[must_use]
pub fn compute_free_drift_poca(
    free_drift: &[FreeDriftAnalysis],
    legs: &[crate::mission::types::ManeuverLeg],
    propagator: &PropagationModel,
) -> Option<Vec<Vec<ClosestApproach>>> {
    let mut results = Vec::with_capacity(free_drift.len());
    for (i, (fd, leg)) in free_drift.iter().zip(legs.iter()).enumerate() {
        match find_closest_approaches(
            &fd.trajectory,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            propagator,
            &leg.pre_departure_roe,
            i,
        ) {
            Ok(pocas) => results.push(pocas),
            Err(_) => return None,
        }
    }
    flag_global_minimum(&mut results);
    Some(results)
}

/// Set `is_global_minimum` on the single closest approach across all legs.
fn flag_global_minimum(results: &mut [Vec<ClosestApproach>]) {
    let mut best_dist = f64::INFINITY;
    let mut best_leg = 0;
    let mut best_idx = 0;
    for (leg_i, pocas) in results.iter().enumerate() {
        for (poca_i, poca) in pocas.iter().enumerate() {
            if poca.distance_km < best_dist {
                best_dist = poca.distance_km;
                best_leg = leg_i;
                best_idx = poca_i;
            }
        }
    }
    if best_dist < f64::INFINITY {
        results[best_leg][best_idx].is_global_minimum = true;
    }
}

/// Arguments for assembling the final [`PipelineOutput`].
///
/// Groups the pipeline-context parameters that feed into [`build_output`],
/// keeping the function signature concise.
pub struct BuildOutputCtx<'a> {
    /// Transfer result from `compute_transfer()`.
    pub transfer: &'a TransferResult,
    /// Original pipeline input (for covariance/eclipse config).
    pub input: &'a PipelineInput,
    /// Resolved propagation model.
    pub propagator: &'a PropagationModel,
    /// Auto-derived drag config, if any.
    pub auto_drag: Option<DragConfig>,
    /// Enrichment suggestion, if formation design was requested.
    pub suggestion: Option<EnrichmentSuggestion>,
    /// Pre-computed safety assessment (free-drift, POCA, COLA).
    pub safety: SafetyAnalysis,
    /// Pre-computed covariance report (skips on-demand computation when present).
    pub precomputed_covariance: Option<MissionCovarianceReport>,
}

/// Assemble a [`PipelineOutput`] from pipeline components.
///
/// Runs covariance propagation (if `navigation_accuracy` is present)
/// and eclipse computation. Covariance and eclipse failures are non-fatal —
/// the result is returned without that data.
#[must_use]
pub fn build_output(
    ctx: BuildOutputCtx<'_>,
    wp_mission: crate::mission::types::WaypointMission,
) -> PipelineOutput {
    let transfer_eclipse = ctx.transfer.plan.transfer.as_ref().and_then(|t| {
        compute_transfer_eclipse(t, &ctx.input.chief, DEFAULT_TRANSFER_ECLIPSE_SAMPLES).ok()
    });

    let total_dv_km_s = ctx.transfer.lambert_dv_km_s + wp_mission.total_dv_km_s;
    let lambert_tof_s = ctx.transfer
        .plan
        .transfer
        .as_ref()
        .map_or(0.0, |t| t.tof_s);
    let total_duration_s = lambert_tof_s + wp_mission.total_duration_s;

    let covariance = ctx.precomputed_covariance.or_else(|| {
        ctx.input.navigation_accuracy.as_ref().and_then(|nav| {
            compute_mission_covariance(
                &wp_mission,
                &ctx.transfer.plan.chief_at_arrival,
                nav,
                ctx.input.maneuver_uncertainty.as_ref(),
                ctx.propagator,
            )
            .ok()
        })
    });

    let formation_design = ctx.suggestion.map(|s| {
        let waypoints = to_waypoints(&ctx.input.waypoints);
        compute_formation_report(s.perch, s.requirements, &wp_mission.legs, &waypoints)
    });

    PipelineOutput {
        phase: ctx.transfer.plan.phase.clone(),
        transfer: ctx.transfer.plan.transfer.clone(),
        transfer_eclipse,
        perch_roe: ctx.transfer.plan.perch_roe,
        mission: wp_mission,
        total_dv_km_s,
        total_duration_s,
        auto_drag_config: ctx.auto_drag,
        covariance,
        monte_carlo: None,
        safety: ctx.safety,
        formation_design,
    }
}

/// Compute safety assessment from a planned mission.
///
/// COLA config resolution happens here: explicit `cola` takes priority,
/// then auto-derives from `safety.min_distance_3d_km`, then `None`.
#[must_use]
pub fn compute_safety_analysis(
    wp_mission: &crate::mission::types::WaypointMission,
    safety: Option<&SafetyConfig>,
    cola: Option<&ColaConfig>,
    propagator: &PropagationModel,
) -> SafetyAnalysis {
    let free_drift = safety.and_then(|_| {
        compute_free_drift_analysis(wp_mission, propagator)
    });

    let poca = safety.and_then(|_| {
        compute_poca_analysis(wp_mission, propagator)
    });

    let free_drift_poca = free_drift.as_ref().and_then(|fd| {
        compute_free_drift_poca(fd, &wp_mission.legs, propagator)
    });

    // COLA: explicit config takes priority; auto-derive from safety thresholds
    // otherwise. When safety is enabled and POCA detects violations, COLA
    // auto-computes avoidance using min_distance_3d_km as the target separation.
    let cola_config = cola.copied().or_else(|| {
        safety.map(|s| ColaConfig {
            target_distance_km: s.min_distance_3d_km,
            max_dv_km_s: AUTO_COLA_BUDGET_KM_S,
        })
    });

    let (cola_maneuvers, secondary_conjunctions, cola_skipped) = cola_config
        .zip(poca.as_ref())
        .map_or((None, None, None), |(config, poca_data)| {
            let decision =
                assess_cola(wp_mission, poca_data, propagator, &config);
            match decision {
                ColaAssessment::Nominal => (None, None, None),
                ColaAssessment::Avoidance { maneuvers, skipped } => {
                    let skipped_opt =
                        if skipped.is_empty() { None } else { Some(skipped) };
                    (Some(maneuvers), None, skipped_opt)
                }
                ColaAssessment::SecondaryConjunction {
                    maneuvers,
                    secondary_violations,
                    skipped,
                } => {
                    let skipped_opt =
                        if skipped.is_empty() { None } else { Some(skipped) };
                    (Some(maneuvers), Some(secondary_violations), skipped_opt)
                }
            }
        });

    SafetyAnalysis {
        free_drift,
        poca,
        free_drift_poca,
        cola: cola_maneuvers,
        secondary_conjunctions,
        cola_skipped,
    }
}

/// Compute mission covariance from navigation accuracy and a planned mission.
///
/// Composes [`ric_accuracy_to_roe_covariance`] and
/// [`propagate_mission_covariance`](crate::mission::covariance::propagate_mission_covariance)
/// into a single call. Callers decide error policy: fatal callers use `?`,
/// non-fatal callers use `.ok()`.
///
/// # Errors
///
/// Returns [`CovarianceError`] if initial covariance conversion or propagation fails.
pub fn compute_mission_covariance(
    mission: &crate::mission::types::WaypointMission,
    chief_at_arrival: &KeplerianElements,
    navigation_accuracy: &NavigationAccuracy,
    maneuver_uncertainty: Option<&ManeuverUncertainty>,
    propagator: &PropagationModel,
) -> Result<MissionCovarianceReport, CovarianceError> {
    let initial_p = ric_accuracy_to_roe_covariance(navigation_accuracy, chief_at_arrival)?;
    crate::mission::covariance::propagate_mission_covariance(
        mission,
        &initial_p,
        navigation_accuracy,
        maneuver_uncertainty,
        propagator,
        DEFAULT_COVARIANCE_SAMPLES_PER_LEG,
    )
}

/// Build the [`DepartureState`] from a transfer result's perch ROE and chief elements.
fn departure_from_transfer(transfer: &TransferResult) -> DepartureState {
    DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    }
}

/// Plan waypoints from a transfer result.
///
/// Builds the [`DepartureState`] from the transfer's perch ROE and chief elements,
/// converts the pipeline waypoint inputs to core [`crate::mission::types::Waypoint`]s, and runs
/// waypoint targeting. Consolidates boilerplate shared by CLI and API handlers.
///
/// # Errors
///
/// Returns [`PipelineError`] if waypoint targeting fails.
pub fn plan_waypoints_from_transfer(
    transfer: &TransferResult,
    input: &PipelineInput,
    propagator: &PropagationModel,
) -> Result<crate::mission::types::WaypointMission, PipelineError> {
    let waypoints = to_waypoints(&input.waypoints);
    let departure = departure_from_transfer(transfer);
    Ok(plan_waypoint_mission(
        &departure,
        &waypoints,
        &input.config,
        propagator,
    )?)
}

/// Core enrichment computation shared by CLI pipeline and API handler.
///
/// Attempts [`enrich_perch()`]; on failure, falls back to unenriched ROE.
/// Both [`suggest_enrichment()`] and the API handler delegate to this.
#[must_use]
pub fn suggest_enrichment_from_parts(
    perch: &PerchGeometry,
    chief_at_arrival: &KeplerianElements,
    unenriched_roe: QuasiNonsingularROE,
    requirements: &SafetyRequirements,
) -> EnrichmentSuggestion {
    let perch_result = match enrich_perch(perch, chief_at_arrival, requirements) {
        Ok(safe_perch) => PerchEnrichmentResult::Enriched(safe_perch),
        Err(e) => PerchEnrichmentResult::Fallback {
            unenriched_roe,
            reason: e.into(),
        },
    };
    EnrichmentSuggestion {
        perch: perch_result,
        requirements: *requirements,
    }
}

/// Compute enrichment suggestion from a [`PipelineInput`] without mutating the transfer.
///
/// Returns `None` if `safety_requirements` is not set on the input.
/// Delegates to [`suggest_enrichment_from_parts()`].
#[must_use]
pub fn suggest_enrichment(
    transfer: &TransferResult,
    input: &PipelineInput,
) -> Option<EnrichmentSuggestion> {
    input.safety_requirements.as_ref().map(|reqs| {
        suggest_enrichment_from_parts(
            &input.perch,
            &transfer.plan.chief_at_arrival,
            transfer.plan.perch_roe,
            reqs,
        )
    })
}

/// Apply accepted perch enrichment to the transfer result.
///
/// If the suggestion contains an `Enriched` perch, replaces
/// `transfer.plan.perch_roe` with the enriched ROE.
/// Must be called before [`plan_waypoints_from_transfer()`].
pub fn apply_perch_enrichment(
    transfer: &mut TransferResult,
    suggestion: &EnrichmentSuggestion,
) {
    if let PerchEnrichmentResult::Enriched(ref safe_perch) = suggestion.perch {
        transfer.plan.perch_roe = safe_perch.roe;
    }
}

/// Accept an enriched ROE at a specific waypoint and replan from there.
///
/// Converts the enriched ROE to a concrete RIC velocity via `roe_to_ric`
/// (D'Amico Eq. 2.17), updates the waypoint's velocity target, and
/// replans from that waypoint onward.
///
/// This is the waypoint-level analog of [`apply_perch_enrichment`] +
/// `replan_mission`: the user sees the enriched suggestion, accepts it,
/// and the system replans with the enriched target.
///
/// # Arguments
/// * `input` — mutable pipeline input (waypoint velocity will be updated)
/// * `transfer` — mutable transfer result; passed through to replanning
///   and modified in-place if enrichment applies
/// * `waypoint_index` — which waypoint to enrich (0-indexed)
/// * `enriched_roe` — the accepted enriched ROE from `enrich_waypoint`
/// * `chief_at_waypoint` — chief mean elements at the waypoint epoch
///
/// # Invariants
/// - `chief_at_waypoint` must be valid mean Keplerian elements
///   (`a_km > 0`, `0 ≤ e < 1`) at the waypoint epoch.
/// - ROE linearization assumes small relative separation
///   (dimensionless ROE norm ≪ 1); see `roe_to_ric` (D'Amico Eq. 2.17).
/// - `transfer` is modified in-place by enrichment; caller must not
///   assume it is unchanged after this call.
///
/// # Errors
/// Returns [`PipelineError`] if `waypoint_index` is out of bounds,
/// if ROE-to-RIC conversion fails, or if replanning fails.
pub fn accept_waypoint_enrichment(
    input: &mut PipelineInput,
    transfer: &mut TransferResult,
    waypoint_index: usize,
    enriched_roe: &QuasiNonsingularROE,
    chief_at_waypoint: &KeplerianElements,
) -> Result<PipelineOutput, PipelineError> {
    // Bounds check using the existing mission-level error variant.
    if waypoint_index >= input.waypoints.len() {
        return Err(PipelineError::Mission(
            MissionError::InvalidReplanIndex {
                index: waypoint_index,
                num_waypoints: input.waypoints.len(),
            },
        ));
    }

    // Convert enriched ROE → RIC state (position preserved by null-space, velocity derived).
    let ric = roe_to_ric(enriched_roe, chief_at_waypoint)?;

    // Update the waypoint's velocity: None → Some(enriched_velocity).
    input.waypoints[waypoint_index].velocity_ric_km_s = Some(ric.velocity_ric_km_s.into());

    // Replan from the modified waypoint.
    replan_from_transfer(transfer, input, waypoint_index, None)
}

/// Build a [`FormationDesignReport`] from a perch enrichment result, safety
/// requirements, and completed maneuver legs.
///
/// Computes transit e/i monitoring for each leg and aggregates into a
/// report suitable for serialization to CLI or API clients.
///
/// Applies R2 (resolve alignment once at perch, propagate to all waypoints),
/// then enriches each waypoint and assesses transit safety per leg.
/// Uses per-leg chief mean elements (which evolve under J2) rather than a
/// single chief epoch, so that the T-matrix and e/i geometry are evaluated
/// at the correct epoch for each leg.
///
/// # Arguments
/// * `perch_result` — perch enrichment outcome (baseline or enriched ROE).
/// * `reqs` — safety requirements (min separation, alignment preference).
/// * `legs` — completed maneuver legs from targeting (1:1 with `waypoints`).
/// * `waypoints` — domain waypoints with optional velocity constraints.
///
/// # Invariants
/// - `legs` must be non-empty (post-targeting output).
/// - `legs.len() == waypoints.len()` (structurally guaranteed by
///   `plan_waypoint_mission`, which produces one leg per waypoint).
/// - `reqs.min_separation_km > 0`.
#[must_use]
pub fn compute_formation_report(
    perch_result: PerchEnrichmentResult,
    reqs: SafetyRequirements,
    legs: &[crate::mission::types::ManeuverLeg],
    waypoints: &[crate::mission::types::Waypoint],
) -> FormationDesignReport {
    debug_assert_eq!(
        legs.len(),
        waypoints.len(),
        "legs and waypoints must be 1:1 (plan_waypoint_mission guarantees this)"
    );

    // R2: resolve alignment once at perch, propagate to all waypoints.
    // If perch enrichment succeeded, use the resolved concrete alignment
    // (Auto → Parallel or AntiParallel). On fallback, use original reqs.
    let resolved_reqs = perch_result.resolve_requirements(&reqs);

    // Enrich each waypoint using the correct mode per waypoint:
    // - `None` velocity → position-only (3-DOF null-space, actionable suggestion).
    //   Pre-rotate e/i phases by half the J2 perigee drift over the coast arc
    //   that FOLLOWS this waypoint (legs[i+1].tof_s), so the aligned geometry
    //   lands at mid-transit of the next leg (D'Amico Eq. 2.30). The last
    //   waypoint has no following arc → fall back to uncompensated enrichment.
    // - `Some(v)` velocity → velocity-constrained (0-DOF, advisory comparison).
    // Uses per-leg arrival chief mean so T-matrix matches J2-evolved epoch.
    // Indexed by leg — None if enrichment failed for that waypoint.
    let enriched_waypoints: Vec<Option<EnrichedWaypoint>> = legs.iter().enumerate()
        .zip(waypoints.iter())
        .map(|((i, leg), wp)| match wp.velocity_ric_km_s {
            None => legs.get(i + 1).map_or_else(
                || enrich_waypoint(
                    &leg.to_position_ric_km,
                    None,
                    &leg.arrival_chief_mean,
                    &resolved_reqs,
                ).ok(),
                |next| enrich_with_drift_compensation(
                    &leg.to_position_ric_km,
                    &leg.arrival_chief_mean,
                    next.tof_s,
                    &resolved_reqs,
                )
                .map(|(enriched, _status)| enriched)
                .ok(),
            ),
            Some(ref vel) => enrich_waypoint(
                &leg.to_position_ric_km,
                Some(vel),
                &leg.arrival_chief_mean,
                &resolved_reqs,
            ).ok(),
        }).collect();

    // Advisory: transit e/i separation per leg.
    // assess_transit_safety reads per-sample chief from trajectory internally.
    // Indexed by leg — None if assessment failed for that leg.
    let transit_safety: Vec<Option<TransitSafetyReport>> = legs.iter().map(|leg| {
        assess_transit_safety(&leg.trajectory, &resolved_reqs).ok()
    }).collect();

    let mission_min_ei_separation_km = transit_safety.iter()
        .filter_map(|t| t.as_ref())
        .map(|t| t.min_ei_separation_km)
        .fold(None, |acc, v| Some(acc.map_or(v, |a: f64| a.min(v))));

    // Drift prediction: propagate the compensated ROE to mid-transit before
    // reading e/i — the departure-epoch values stored on EnrichedWaypoint
    // reflect the pre-rotated (lagged) phase, not the aligned phase that
    // compensation targets.
    let drift_prediction = legs.first().and_then(|leg| {
        match enrich_with_drift_compensation(
            &leg.from_position_ric_km,
            &leg.departure_chief_mean,
            leg.tof_s,
            &resolved_reqs,
        ) {
            Ok((enriched, DriftCompensationStatus::Applied)) => {
                ei_separation_after(&enriched.roe, &leg.departure_chief_mean, leg.tof_s * 0.5)
                    .ok()
                    .map(|ei_mid| DriftPrediction {
                        predicted_min_ei_km: ei_mid.min_separation_km,
                        predicted_phase_angle_rad: ei_mid.phase_angle_rad,
                    })
            }
            _ => None,
        }
    });

    FormationDesignReport {
        perch: perch_result,
        waypoints: enriched_waypoints,
        transit_safety,
        mission_min_ei_separation_km,
        drift_prediction,
    }
}

/// Construct a [`LeanPlanResult`] summary from a completed transfer, mission,
/// and waypoint inputs.
///
/// Used by both CLI `build_output()` and API handlers to produce the lean
/// projection sent to frontends.
#[must_use]
pub fn build_lean_plan_result(
    transfer: &TransferResult,
    wp_mission: &crate::mission::types::WaypointMission,
    waypoint_inputs: &[WaypointInput],
) -> LeanPlanResult {
    let transfer_summary = transfer.plan.transfer.as_ref().map(|lambert| TransferSummary {
        total_dv_km_s: transfer.lambert_dv_km_s,
        tof_s: lambert.tof_s,
        direction: lambert.direction,
        arrival_epoch: transfer.arrival_epoch,
    });

    let total_dv_km_s = transfer.lambert_dv_km_s + wp_mission.total_dv_km_s;
    let total_duration_s = transfer
        .plan
        .transfer
        .as_ref()
        .map_or(0.0, |t| t.tof_s)
        + wp_mission.total_duration_s;

    let legs = wp_mission
        .legs
        .iter()
        .enumerate()
        .map(|(i, leg)| {
            let label = waypoint_inputs.get(i).and_then(|wp| wp.label.clone());
            leg.to_summary(label)
        })
        .collect();

    LeanPlanResult {
        phase: transfer.plan.phase.clone(),
        transfer_summary,
        perch_roe: transfer.plan.perch_roe,
        legs,
        total_dv_km_s,
        total_duration_s,
        safety: wp_mission.safety,
    }
}

/// Plan a mission from a pre-computed transfer result.
///
/// WASM-eligible: accepts a [`TransferResult`] (from the server's `compute_transfer`
/// or from client-side proximity classification) and runs the full planning
/// pipeline: enrichment, waypoint targeting, safety analysis, output assembly.
///
/// # Arguments
///
/// * `transfer` — pre-computed transfer result; modified in-place if perch
///   enrichment applies (see [`apply_perch_enrichment`]).
/// * `input` — pipeline input defining waypoints, propagator, safety config,
///   and optional COLA/covariance settings.
///
/// # Invariants
///
/// - `transfer.plan.perch_roe` must satisfy ROE linearization validity
///   (dimensionless norm well below 1); see `roe_to_ric` (D'Amico Eq. 2.17).
/// - `input.waypoints` must be non-empty for a meaningful mission.
///
/// # Errors
///
/// Returns [`PipelineError::Mission`] if waypoint targeting fails.
/// Returns [`PipelineError::Propagation`] if STM propagation fails.
/// Returns [`PipelineError::Covariance`] if covariance propagation fails.
pub fn execute_mission_from_transfer(
    transfer: &mut TransferResult,
    input: &PipelineInput,
) -> Result<PipelineOutput, PipelineError> {
    let propagator = to_propagation_model(&input.propagator);

    let suggestion = suggest_enrichment(transfer, input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(transfer, s);
    }

    let wp_mission = plan_waypoints_from_transfer(transfer, input, &propagator)?;

    let safety = compute_safety_analysis(
        &wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &propagator,
    );

    Ok(build_output(
        BuildOutputCtx {
            transfer,
            input,
            propagator: &propagator,
            auto_drag: None,
            suggestion,
            safety,
            precomputed_covariance: None,
        },
        wp_mission,
    ))
}

/// Replan a mission from a pre-computed transfer result.
///
/// WASM-eligible: the client holds the [`TransferResult`] and optional cached
/// mission from previous planning. Only the changed waypoint triggers replanning.
///
/// When `cached_mission` is provided, reuses the kept legs (`0..modified_index`)
/// without re-solving them. When `None`, plans the full mission first.
///
/// # Arguments
///
/// * `transfer` — pre-computed transfer result; modified in-place if perch
///   enrichment applies.
/// * `input` — pipeline input with current waypoints.
/// * `modified_index` — 0-based index of the waypoint that changed.
/// * `cached_mission` — optional previously planned mission; legs before
///   `modified_index` are reused without re-solving.
///
/// # Invariants
///
/// - `modified_index < input.waypoints.len()`.
/// - If `cached_mission` is `Some`, its legs must correspond to the same
///   waypoint sequence (prior to modification).
///
/// # Errors
///
/// Returns [`PipelineError::Mission`] if waypoint targeting or replanning fails.
/// Returns [`PipelineError::Propagation`] if STM propagation fails.
/// Returns [`PipelineError::Covariance`] if covariance propagation fails.
pub fn replan_from_transfer(
    transfer: &mut TransferResult,
    input: &PipelineInput,
    modified_index: usize,
    cached_mission: Option<crate::mission::types::WaypointMission>,
) -> Result<PipelineOutput, PipelineError> {
    let propagator = to_propagation_model(&input.propagator);

    let suggestion = suggest_enrichment(transfer, input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(transfer, s);
    }

    let waypoints = to_waypoints(&input.waypoints);
    let departure = departure_from_transfer(transfer);

    let base_mission = match cached_mission {
        Some(m) => m,
        None => plan_waypoint_mission(&departure, &waypoints, &input.config, &propagator)?,
    };

    let wp_mission = replan_from_waypoint(
        base_mission,
        modified_index,
        &waypoints,
        &departure,
        &input.config,
        &propagator,
    )?;

    let safety = compute_safety_analysis(
        &wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &propagator,
    );

    Ok(build_output(
        BuildOutputCtx {
            transfer,
            input,
            propagator: &propagator,
            auto_drag: None,
            suggestion,
            safety,
            precomputed_covariance: None,
        },
        wp_mission,
    ))
}


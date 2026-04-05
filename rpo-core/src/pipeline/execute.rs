//! Pipeline execution: `compute_transfer`, `execute_mission`, `replan_mission`, `build_output`.
//!
//! These compose the core planning primitives (classify, Lambert, waypoint
//! targeting, covariance, eclipse) into a complete mission pipeline.

use crate::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;
use crate::mission::cola_assessment::{assess_cola, ColaAssessment};
use crate::mission::avoidance::ColaConfig;
use crate::mission::closest_approach::{find_closest_approaches, ClosestApproach};
use crate::mission::formation::{
    DriftPrediction, EnrichedWaypoint, FormationDesignReport,
    PerchEnrichmentResult, SafetyRequirements, TransitSafetyReport,
};
use crate::mission::formation::transit::enrich_with_drift_compensation;
use crate::mission::formation::types::DriftCompensationStatus;
use super::types::EnrichmentSuggestion;
use crate::mission::formation::perch::enrich_perch;
use crate::mission::formation::safety_envelope::enrich_waypoint;
use crate::mission::formation::transit::assess_transit_safety;
use crate::mission::free_drift::{compute_free_drift, FreeDriftAnalysis};
use crate::mission::planning::{compute_transfer_eclipse, plan_mission};
use crate::mission::types::PerchGeometry;
use crate::mission::waypoints::{plan_waypoint_mission, replan_from_waypoint};
use crate::propagation::covariance::{
    ric_accuracy_to_roe_covariance, CovarianceError, ManeuverUncertainty,
    MissionCovarianceReport, NavigationAccuracy,
};
use crate::propagation::keplerian::propagate_keplerian;
use crate::propagation::propagator::{DragConfig, PropagationModel};
use crate::types::elements::KeplerianElements;
use crate::types::roe::QuasiNonsingularROE;
use crate::types::{DepartureState, StateVector};

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

/// Classify separation, solve Lambert if far-field, compute perch ECI states.
///
/// This is the first phase of the pipeline: resolve defaults, classify the
/// chief/deputy separation, solve Lambert if far-field, and compute the
/// perch handoff states.
///
/// Public primitive — used directly by validate/MC handlers that need
/// intermediate states for progress injection and propagator resolution.
///
/// # Errors
///
/// Returns [`PipelineError`] if classification or Lambert solving fails,
/// or if the chief trajectory is empty after Lambert propagation.
pub fn compute_transfer(input: &PipelineInput) -> Result<TransferResult, PipelineError> {
    let plan = plan_mission(
        &input.chief,
        &input.deputy,
        &input.perch,
        &input.proximity,
        input.lambert_tof_s,
        &input.lambert_config,
    )?;

    let lambert_dv_km_s = plan.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);

    let (perch_chief, perch_deputy, arrival_epoch) = if let Some(ref transfer) = plan.transfer {
        let arrival_epoch =
            input.chief.epoch + hifitime::Duration::from_seconds(input.lambert_tof_s);
        let chief_traj = propagate_keplerian(&input.chief, input.lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or(PipelineError::EmptyTrajectory)?
            .clone();

        let deputy_at_perch = StateVector {
            epoch: arrival_epoch,
            position_eci_km: transfer.arrival_state.position_eci_km,
            velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
                + transfer.arrival_dv_eci_km_s,
        };

        (chief_at_arrival, deputy_at_perch, arrival_epoch)
    } else {
        (input.chief.clone(), input.deputy.clone(), input.chief.epoch)
    };

    Ok(TransferResult {
        plan,
        perch_chief,
        perch_deputy,
        arrival_epoch,
        lambert_dv_km_s,
    })
}

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
        compute_formation_report(s.perch, s.requirements, &wp_mission.legs)
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

/// Compute safety analysis and derive COLA burns for validation injection.
///
/// Combines [`compute_safety_analysis`] and [`crate::mission::convert_cola_to_burns`] into a
/// single call, ensuring COLA burns are always derived consistently from the
/// safety analysis. Used by both CLI and API validate handlers.
///
/// # Errors
/// Returns [`crate::mission::ValidationError`] if a COLA burn epoch is out of bounds.
pub fn compute_validation_burns(
    wp_mission: &crate::mission::types::WaypointMission,
    safety: Option<&SafetyConfig>,
    cola: Option<&ColaConfig>,
    propagator: &PropagationModel,
) -> Result<(SafetyAnalysis, Vec<crate::mission::validation::ColaBurn>), crate::mission::ValidationError> {
    let analysis = compute_safety_analysis(wp_mission, safety, cola, propagator);
    let burns = crate::mission::validation::convert_cola_to_burns(
        analysis.cola.as_deref(),
        wp_mission,
    )?;
    Ok((analysis, burns))
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
/// # Invariants
/// - `legs` must be non-empty (post-targeting output).
/// - `reqs.min_separation_km > 0`.
#[must_use]
pub fn compute_formation_report(
    perch_result: PerchEnrichmentResult,
    reqs: SafetyRequirements,
    legs: &[crate::mission::types::ManeuverLeg],
) -> FormationDesignReport {
    // R2: resolve alignment once at perch, propagate to all waypoints.
    // If perch enrichment succeeded, use the resolved concrete alignment
    // (Auto → Parallel or AntiParallel). On fallback, use original reqs.
    let resolved_reqs = perch_result.resolve_requirements(&reqs);

    // Advisory: enrich each waypoint (velocity-constrained path — all
    // waypoints currently carry velocity via to_waypoints() zero-fill).
    // Uses per-leg arrival chief mean so T-matrix matches J2-evolved epoch.
    // Indexed by leg — None if enrichment failed for that waypoint.
    let waypoints: Vec<Option<EnrichedWaypoint>> = legs.iter().map(|leg| {
        enrich_waypoint(
            &leg.to_position_ric_km,
            Some(&leg.target_velocity_ric_km_s),
            &leg.arrival_chief_mean,
            &resolved_reqs,
        ).ok()
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

    // Drift prediction: predict mid-transit e/i for the leg-1 coast arc
    // using J2 drift compensation. Computed here (orchestration layer) rather
    // than inside perch enrichment to keep geometric and temporal concerns separate.
    let drift_prediction = legs.first().and_then(|leg| {
        match enrich_with_drift_compensation(
            &leg.from_position_ric_km,
            &leg.departure_chief_mean,
            leg.tof_s,
            &PropagationModel::J2Stm,
            &resolved_reqs,
        ) {
            Ok((enriched, DriftCompensationStatus::Applied)) => Some(DriftPrediction {
                predicted_min_ei_km: enriched.enriched_ei.min_separation_km,
                predicted_phase_angle_rad: enriched.enriched_ei.phase_angle_rad,
            }),
            _ => None,
        }
    });

    FormationDesignReport {
        perch: perch_result,
        waypoints,
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

/// Execute the full mission pipeline: classify → Lambert → waypoints → covariance → eclipse.
///
/// Single entry point for the CLI `mission` command and the API `PlanMission` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn execute_mission(input: &PipelineInput) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);

    let suggestion = suggest_enrichment(&transfer, input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(&mut transfer, s);
    }

    let wp_mission = plan_waypoints_from_transfer(&transfer, input, &propagator)?;

    let safety = compute_safety_analysis(
        &wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &propagator,
    );

    Ok(build_output(
        BuildOutputCtx {
            transfer: &transfer,
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

/// Incremental replan pipeline: classify → Lambert → replan from index.
///
/// When `cached_mission` is provided, reuses the kept legs (`0..modified_index`)
/// without re-solving them. When `None`, plans the full mission first as a
/// fallback (prior behavior).
///
/// Used by the API's `MoveWaypoint` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn replan_mission(
    input: &PipelineInput,
    modified_index: usize,
    cached_mission: Option<crate::mission::types::WaypointMission>,
) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);

    let suggestion = suggest_enrichment(&transfer, input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(&mut transfer, s);
    }

    let waypoints = to_waypoints(&input.waypoints);
    let departure = departure_from_transfer(&transfer);

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
            transfer: &transfer,
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

#[cfg(test)]
mod tests {
    use crate::mission::config::ProximityConfig;
    use crate::propagation::lambert::LambertConfig;

    use super::*;

    /// Tolerance for pure data-copy fidelity tests (no arithmetic, exact IEEE 754 roundtrip).
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;

    /// Minimum e/i magnitude expected after enrichment (dimensionless ROE).
    /// For a 100m separation at a = 7000 km, d_min/a ~ 1.4e-5. 1e-10 provides
    /// 5 orders of margin above floating-point noise.
    const ENRICHMENT_NONZERO_TOL: f64 = 1e-10;

    /// Exact-equality tolerance for ROE that should be bitwise identical.
    /// The enriched ROE is assigned directly (no arithmetic), so the only
    /// difference is f64 representation noise from serialization roundtrips.
    const ROE_IDENTITY_TOL: f64 = 1e-15;

    /// Build a minimal `PipelineInput` from the standard far-field test scenario.
    fn far_field_input() -> PipelineInput {
        use crate::types::StateVector;
        use hifitime::Epoch;
        use nalgebra::Vector3;

        let chief = StateVector {
            epoch: Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap(),
            position_eci_km: Vector3::new(5876.261, 3392.661, 0.0),
            velocity_eci_km_s: Vector3::new(-2.380512, 4.123167, 6.006917),
        };

        let deputy = StateVector {
            epoch: Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap(),
            position_eci_km: Vector3::new(5199.839421, 4281.648523, 1398.070066),
            velocity_eci_km_s: Vector3::new(-3.993103, 2.970313, 5.764540),
        };

        PipelineInput {
            chief,
            deputy,
            perch: crate::pipeline::types::default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![
                crate::pipeline::types::WaypointInput {
                    position_ric_km: [0.5, 2.0, 0.5],
                    velocity_ric_km_s: None,
                    tof_s: Some(4200.0),
                    label: Some("WP1".into()),
                },
                crate::pipeline::types::WaypointInput {
                    position_ric_km: [0.5, 0.5, 0.5],
                    velocity_ric_km_s: Some([0.0, 0.001, 0.0]),
                    tof_s: Some(4200.0),
                    label: Some("WP2".into()),
                },
            ],
            proximity: ProximityConfig::default(),
            config: crate::mission::config::MissionConfig::default(),
            propagator: crate::pipeline::types::PropagatorChoice::J2,
            chief_config: None,
            deputy_config: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo: None,
            cola: None,
            safety_requirements: None,
        }
    }

    #[test]
    fn test_compute_transfer_far_field() {
        let input = far_field_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");

        // Far-field scenario should produce a Lambert transfer
        assert!(result.plan.transfer.is_some());
        assert!(result.lambert_dv_km_s > 0.0);
    }

    #[test]
    fn test_execute_mission_far_field() {
        let input = far_field_input();
        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Should have a Lambert transfer and waypoint legs
        assert!(output.transfer.is_some());
        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
        assert!(output.total_duration_s > 0.0);
    }

    #[test]
    fn test_replan_mission() {
        let input = far_field_input();
        let output = replan_mission(&input, 0, None).expect("replan_mission should succeed");

        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
    }

    #[test]
    fn test_replan_mission_with_cache() {
        let input = far_field_input();
        // First run a full mission to get a cached result
        let full_output = execute_mission(&input).expect("execute_mission should succeed");

        // Replan from waypoint 1 using the cached mission
        let output = replan_mission(&input, 1, Some(full_output.mission))
            .expect("replan_mission with cache should succeed");

        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
    }

    /// Build a proximity-regime `PipelineInput` (deputy 1 km higher than chief).
    fn proximity_input() -> PipelineInput {
        use crate::elements::keplerian_conversions::keplerian_to_state;
        use crate::test_helpers::iss_like_elements;
        use hifitime::Epoch;

        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher → Proximity regime
        deputy_ke.mean_anomaly_rad += 0.01;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        PipelineInput {
            chief,
            deputy,
            perch: crate::pipeline::types::default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![crate::pipeline::types::WaypointInput {
                position_ric_km: [0.5, 2.0, 0.5],
                velocity_ric_km_s: None,
                tof_s: Some(4200.0),
                label: Some("WP1".into()),
            }],
            proximity: ProximityConfig::default(),
            config: crate::mission::config::MissionConfig::default(),
            propagator: crate::pipeline::types::PropagatorChoice::J2,
            chief_config: None,
            deputy_config: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo: None,
            cola: None,
            safety_requirements: None,
        }
    }

    #[test]
    fn test_compute_transfer_proximity_arrival_epoch() {
        let input = proximity_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");

        // Proximity scenario: no Lambert transfer
        assert!(result.plan.transfer.is_none());
        assert!(
            result.lambert_dv_km_s.abs() < f64::EPSILON,
            "proximity should have zero Lambert delta-v"
        );

        // arrival_epoch must equal the chief epoch (no Lambert offset)
        assert_eq!(
            result.arrival_epoch, input.chief.epoch,
            "proximity arrival_epoch should equal chief epoch, not chief + lambert_tof_s"
        );
    }

    #[test]
    fn test_pipeline_input_serde_roundtrip() {
        let input = far_field_input();
        let json = serde_json::to_string(&input).expect("serialize");
        let roundtrip: PipelineInput = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(input.chief.epoch, roundtrip.chief.epoch);
        assert_eq!(input.waypoints.len(), roundtrip.waypoints.len());
        assert!(
            (input.lambert_tof_s - roundtrip.lambert_tof_s).abs() < COPY_FIDELITY_TOL,
            "lambert_tof_s serde roundtrip mismatch"
        );
    }

    #[test]
    fn test_transfer_result_serde_roundtrip() {
        let input = far_field_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");
        let json = serde_json::to_string(&result).expect("serialize TransferResult");
        let roundtrip: TransferResult =
            serde_json::from_str(&json).expect("deserialize TransferResult");

        assert_eq!(result.arrival_epoch, roundtrip.arrival_epoch);
        assert!(
            (result.lambert_dv_km_s - roundtrip.lambert_dv_km_s).abs() < COPY_FIDELITY_TOL,
            "lambert_dv_km_s serde roundtrip mismatch"
        );
        assert!(result.plan.transfer.is_some() == roundtrip.plan.transfer.is_some());
    }

    #[test]
    fn test_to_waypoints_conversion() {
        let inputs = vec![
            crate::pipeline::types::WaypointInput {
                position_ric_km: [1.0, 2.0, 3.0],
                velocity_ric_km_s: Some([0.1, 0.2, 0.3]),
                tof_s: Some(100.0),
                label: None,
            },
            crate::pipeline::types::WaypointInput {
                position_ric_km: [4.0, 5.0, 6.0],
                velocity_ric_km_s: None,
                tof_s: None,
                label: None,
            },
        ];

        let waypoints = to_waypoints(&inputs);
        assert_eq!(waypoints.len(), 2);
        assert!(
            (waypoints[0].position_ric_km.x - 1.0).abs() < COPY_FIDELITY_TOL,
            "position_ric_km.x copy mismatch"
        );
        assert!(
            (waypoints[0].velocity_ric_km_s.y - 0.2).abs() < COPY_FIDELITY_TOL,
            "velocity_ric_km_s.y copy mismatch"
        );
        assert!(
            (waypoints[0].tof_s.unwrap() - 100.0).abs() < COPY_FIDELITY_TOL,
            "tof_s copy mismatch"
        );
        // Second waypoint should default velocity to zero
        assert!(
            waypoints[1].velocity_ric_km_s.x.abs() < COPY_FIDELITY_TOL,
            "default velocity should be zero"
        );
        assert!(waypoints[1].tof_s.is_none());
    }

    #[test]
    fn test_execute_mission_with_formation_design() {
        use crate::mission::formation::{
            EiAlignment, PerchEnrichmentResult, SafetyRequirements,
        };

        let mut input = far_field_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.1,
            alignment: EiAlignment::Parallel,
        });

        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Formation design report must be present
        let report = output.formation_design
            .as_ref()
            .expect("formation_design should be Some when safety_requirements is set");

        // Perch should be enriched (V-bar perch has zero e/i → enrichment adds them)
        assert!(
            matches!(report.perch, PerchEnrichmentResult::Enriched(_)),
            "V-bar perch should enrich successfully"
        );

        // Enriched perch ROE should have nonzero e/i vectors
        let roe = &output.perch_roe;
        let ei_magnitude = (roe.dex * roe.dex + roe.dey * roe.dey
            + roe.dix * roe.dix + roe.diy * roe.diy).sqrt();

        assert!(
            ei_magnitude > ENRICHMENT_NONZERO_TOL,
            "enriched perch should have nonzero e/i vectors, got {ei_magnitude}"
        );

        // Enforced enrichment must flow through: output.perch_roe == report enriched ROE
        if let PerchEnrichmentResult::Enriched(ref sp) = report.perch {
            let diff = (sp.roe.to_vector() - roe.to_vector()).norm();
            assert!(
                diff < ROE_IDENTITY_TOL,
                "output.perch_roe must match enriched perch ROE, diff = {diff:.2e}"
            );
        }

        // Vectors indexed by leg — same length as legs
        assert_eq!(
            report.waypoints.len(),
            output.mission.legs.len(),
            "waypoints should have one entry per leg"
        );
        assert_eq!(
            report.transit_safety.len(),
            output.mission.legs.len(),
            "transit_safety should have one entry per leg"
        );

        // All entries should be Some (no failures in this scenario)
        assert!(
            report.waypoints.iter().all(Option::is_some),
            "all waypoint enrichments should succeed"
        );
        assert!(
            report.transit_safety.iter().all(Option::is_some),
            "all transit assessments should succeed"
        );

        // Mission-wide minimum e/i separation should be present and positive
        let mission_min = report.mission_min_ei_separation_km
            .expect("mission_min should be Some when transit assessments succeed");
        assert!(
            mission_min > 0.0,
            "mission min e/i separation should be positive, got {mission_min}"
        );
    }

    /// Formation report includes drift prediction for leg-1 coast arc.
    ///
    /// proximity_input has tof_s = 4200s (~0.78 orbits of ISS-like chief),
    /// well within the 10-orbit drift compensation regime, so a prediction
    /// must be present.
    #[test]
    fn formation_report_has_drift_prediction() {
        use crate::mission::formation::{EiAlignment, SafetyRequirements};

        let mut input = proximity_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.10,
            alignment: EiAlignment::Parallel,
        });

        let output = execute_mission(&input).expect("execute_mission should succeed");
        let report = output.formation_design
            .as_ref()
            .expect("formation_design should be present");

        assert!(
            report.drift_prediction.is_some(),
            "drift prediction should be present for sub-orbit arc with non-critical inclination"
        );
        let pred = report.drift_prediction.as_ref().unwrap();
        assert!(
            pred.predicted_min_ei_km > 0.0,
            "predicted min e/i must be positive"
        );
        assert!(
            pred.predicted_phase_angle_rad >= 0.0,
            "predicted phase angle must be non-negative"
        );
    }

    // ---- suggest + apply tests ----

    fn proximity_input_with_enrichment() -> PipelineInput {
        use crate::mission::formation::{EiAlignment, SafetyRequirements};

        let mut input = proximity_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        });
        input
    }

    #[test]
    fn test_suggest_enrichment_does_not_mutate() {
        let input = proximity_input_with_enrichment();
        let transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input);

        assert!(suggestion.is_some(), "should produce suggestion when safety_requirements set");
        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
            "suggest_enrichment must not mutate transfer"
        );
    }

    #[test]
    fn test_suggest_enrichment_none_without_requirements() {
        let input = far_field_input();
        let transfer = compute_transfer(&input).expect("transfer");
        let suggestion = suggest_enrichment(&transfer, &input);
        assert!(suggestion.is_none());
    }

    #[test]
    fn test_apply_perch_enrichment_mutates() {
        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input).expect("suggestion");

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.dey - original_perch_roe.dey).abs()
                > ENRICHMENT_NONZERO_TOL,
            "apply should mutate perch_roe dey"
        );
    }

    #[test]
    fn test_apply_perch_enrichment_fallback_no_mutate() {
        use crate::mission::formation::{EiAlignment, PerchFallbackReason, SafetyRequirements};

        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;

        let suggestion = EnrichmentSuggestion {
            perch: PerchEnrichmentResult::Fallback {
                unenriched_roe: original_perch_roe,
                reason: PerchFallbackReason::SingularGeometry {
                    mean_arg_lat_rad: 0.0,
                },
            },
            requirements: SafetyRequirements {
                min_separation_km: 0.15,
                alignment: EiAlignment::Parallel,
            },
        };

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
            "fallback should not mutate perch_roe"
        );
    }

    /// Verify `compute_safety_analysis` produces identical results to the
    /// safety computation that was previously inlined in `build_output`.
    ///
    /// Uses a far-field input with safety config enabled. The safety assessment
    /// should produce free-drift, POCA, and COLA results that match what
    /// `execute_mission` embeds in its `PipelineOutput`.
    #[test]
    fn test_compute_safety_analysis_matches_build_output() {
        use crate::mission::config::SafetyConfig;

        let mut input = far_field_input();
        input.config.safety = Some(SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.05,
        });

        // execute_mission calls compute_safety_analysis internally and
        // unpacks it into PipelineOutput. Verify the fields match.
        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Separately compute safety assessment from the same mission
        let transfer = compute_transfer(&input).expect("transfer");
        let propagator = to_propagation_model(&input.propagator);
        let mut transfer2 = transfer;
        let suggestion = suggest_enrichment(&transfer2, &input);
        if let Some(ref s) = suggestion {
            apply_perch_enrichment(&mut transfer2, s);
        }
        let wp_mission = plan_waypoints_from_transfer(&transfer2, &input, &propagator)
            .expect("waypoints");
        let safety = compute_safety_analysis(
            &wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &propagator,
        );

        // Free-drift: both should be Some when safety is enabled
        assert_eq!(
            output.safety.free_drift.is_some(),
            safety.free_drift.is_some(),
            "free_drift presence should match"
        );

        // POCA: both should be Some when safety is enabled
        assert_eq!(
            output.safety.poca.is_some(),
            safety.poca.is_some(),
            "poca presence should match"
        );

        // Free-drift POCA
        assert_eq!(
            output.safety.free_drift_poca.is_some(),
            safety.free_drift_poca.is_some(),
            "free_drift_poca presence should match"
        );

        // COLA maneuvers (may or may not be present depending on POCA distances)
        assert_eq!(
            output.safety.cola.is_some(),
            safety.cola.is_some(),
            "cola presence should match"
        );

        // If POCA data exists, check leg counts match
        if let (Some(out_poca), Some(sa_poca)) = (&output.safety.poca, &safety.poca) {
            assert_eq!(out_poca.len(), sa_poca.len(), "POCA leg count should match");
        }
    }
}

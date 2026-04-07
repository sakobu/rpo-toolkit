//! Mission planning handlers: set waypoints, update config, get trajectory/covariance/eclipse.
//!
//! These operate on the session's stored transfer and mission state.

use serde::Serialize;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::formation::{
    FormationDesignReport, PerchEnrichmentResult, SafetyRequirements,
};
use rpo_core::mission::planning::compute_transfer_eclipse;
use rpo_core::mission::types::{MissionPhase, Waypoint, WaypointMission};
use rpo_core::mission::waypoints::{plan_waypoint_mission, replan_from_waypoint};
use rpo_core::mission::ProximityConfig;
use rpo_core::pipeline::convert::to_waypoints;
use rpo_core::pipeline::{
    build_lean_plan_result, compute_formation_report, compute_mission_covariance,
    suggest_enrichment_from_parts, EnrichmentSuggestion, LeanPlanResult, LegTrajectory,
    PlanVariant, TransferResult, WaypointInput,
};
use rpo_nyx::pipeline::compute_transfer;
use rpo_core::propagation::covariance::{
    ManeuverUncertainty, MissionCovarianceReport, NavigationAccuracy,
};
use rpo_core::types::eclipse::{MissionEclipseData, TransferEclipseData};
use rpo_core::types::DepartureState;

use crate::error::{ApiError, InvalidInputError};
use crate::session::Session;

/// Number of eclipse evaluation points along the Lambert transfer arc.
const DEFAULT_TRANSFER_ECLIPSE_SAMPLES: u32 = 200;

/// Eclipse response combining transfer and mission eclipse data.
#[derive(Debug, Clone, Serialize)]
pub struct EclipseResponse {
    /// Transfer-phase eclipse data (None if proximity).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer: Option<TransferEclipseData>,
    /// Mission-phase eclipse data (None if no mission planned).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission: Option<MissionEclipseData>,
}

/// Combined plan result with baseline and optional enriched plan.
pub struct PlanResponse {
    /// Baseline plan (unenriched geometric perch).
    pub baseline: LeanPlanResult,
    /// Enriched plan (safe e/i vectors). Present when enrichment succeeded.
    pub enriched: Option<LeanPlanResult>,
    /// Baseline formation design report.
    pub baseline_formation: Option<FormationDesignReport>,
    /// Enriched formation design report.
    pub enriched_formation: Option<FormationDesignReport>,
}

/// Plan (or replan) the waypoint mission from the session's stored transfer.
///
/// Produces both a baseline plan (unenriched) and an enriched plan (if
/// `safety_requirements` is set and enrichment succeeds). Both plans support
/// incremental replan via `changed_from`.
///
/// # Errors
///
/// Returns [`ApiError`] if states are missing, transfer is required but absent,
/// or waypoint planning fails.
pub fn handle_set_waypoints(
    session: &mut Session,
    changed_from: Option<usize>,
    cached_baseline: Option<WaypointMission>,
    cached_enriched: Option<WaypointMission>,
) -> Result<PlanResponse, ApiError> {
    // 1. Ensure we have a transfer. If none, try auto-compute.
    if session.transfer.is_none() {
        let phase = super::handle_classify(session)?;
        match phase {
            MissionPhase::Proximity { .. } => {
                let input = session.assemble_pipeline_input()?;
                let transfer = compute_transfer(&input)?;
                session.store_transfer(transfer);
            }
            MissionPhase::FarField { .. } => {
                return Err(ApiError::InvalidInput(
                    InvalidInputError::MissingSessionState {
                        missing: "transfer",
                        context: "far-field scenario requires compute_transfer before planning",
                    },
                ));
            }
        }
    }

    // 2. Compute enrichment suggestion via shared pipeline kernel (read-only)
    let suggestion = session.safety_requirements.as_ref().and_then(|reqs| {
        session.transfer.as_ref().map(|t| {
            suggest_enrichment_from_parts(
                &session.perch,
                &t.plan.chief_at_arrival,
                t.plan.perch_roe,
                reqs,
            )
        })
    });

    // 3. Shared targeting inputs
    let transfer = session.require_transfer()?;
    let waypoints = to_waypoints(&session.waypoints);
    let propagator = session.resolve_propagation_model();
    let ctx = PlanContext {
        waypoints: &waypoints,
        config: &session.config,
        propagator: &propagator,
        waypoint_inputs: &session.waypoints,
        changed_from,
    };

    // 4. Build BASELINE plan
    let baseline_formation_ctx = suggestion.as_ref().map(|s| {
        (PerchEnrichmentResult::Baseline(transfer.plan.perch_roe), s.requirements)
    });
    let (baseline_mission, baseline_plan, baseline_formation) =
        build_plan_variant(transfer, cached_baseline, &ctx, baseline_formation_ctx)?;

    // 5. Build ENRICHED plan (if enrichment succeeded)
    let (enriched_mission, enriched_plan, enriched_formation) =
        if let Some(EnrichmentSuggestion {
            perch: PerchEnrichmentResult::Enriched(ref safe_perch),
            ref requirements,
        }) = suggestion
        {
            let mut enriched_transfer = session.require_transfer()?.clone();
            enriched_transfer.plan.perch_roe = safe_perch.roe;

            let formation_ctx = Some((
                PerchEnrichmentResult::Enriched(safe_perch.clone()),
                *requirements,
            ));
            let (mission, plan, formation) =
                build_plan_variant(&enriched_transfer, cached_enriched, &ctx, formation_ctx)?;
            (Some(mission), Some(plan), formation)
        } else {
            (None, None, None)
        };

    // 6. Store both missions in session
    session.store_missions(baseline_mission, enriched_mission, suggestion);

    Ok(PlanResponse {
        baseline: baseline_plan,
        enriched: enriched_plan,
        baseline_formation,
        enriched_formation,
    })
}

/// Select which plan variant (baseline or enriched) is active for downstream ops.
///
/// # Errors
///
/// Returns [`ApiError`] if `Enriched` is requested but no enriched plan is available.
pub fn handle_select_plan(
    session: &mut Session,
    variant: PlanVariant,
) -> Result<PlanVariant, ApiError> {
    if variant == PlanVariant::Enriched && !session.has_enriched_plan() {
        return Err(ApiError::InvalidInput(InvalidInputError::MissingSessionState {
            missing: "enriched_mission",
            context: "no enriched plan available — set safety_requirements and call set_waypoints first",
        }));
    }
    session.select_plan(variant);
    Ok(variant)
}

/// Configuration update fields for `handle_update_config`.
pub struct ConfigUpdate {
    /// Updated solver configuration.
    pub config: Option<MissionConfig>,
    /// Whether the propagator was changed (signals replan needed).
    pub propagator_changed: bool,
    /// Updated proximity thresholds.
    pub proximity: Option<ProximityConfig>,
    /// Updated navigation accuracy (overlay, no replan).
    pub navigation_accuracy: Option<NavigationAccuracy>,
    /// Updated maneuver uncertainty (overlay, no replan).
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
}

/// Apply configuration updates to the session, replanning if mission-affecting fields changed.
///
/// Returns `Some(PlanResponse)` if the mission was replanned, `None` if only overlays changed.
///
/// # Errors
///
/// Returns [`ApiError`] if replanning fails.
pub fn handle_update_config(
    session: &mut Session,
    update: ConfigUpdate,
) -> Result<Option<PlanResponse>, ApiError> {
    // Apply overlay-only fields (no invalidation)
    if let Some(nav) = update.navigation_accuracy {
        session.set_navigation_accuracy(nav);
    }
    if let Some(mu) = update.maneuver_uncertainty {
        session.set_maneuver_uncertainty(mu);
    }

    // Check if any mission-affecting field was provided
    let needs_replan = update.config.is_some()
        || update.propagator_changed
        || update.proximity.is_some();

    // Check mission existence BEFORE applying setters that clear it
    let had_mission = session.has_baseline_mission();

    // Apply mission-affecting fields (these may clear session missions)
    if let Some(config) = update.config {
        session.set_config(config);
    }
    if let Some(proximity) = update.proximity {
        session.proximity = proximity;
    }

    // Replan if mission-affecting fields changed AND a mission existed
    if needs_replan && had_mission {
        let result = handle_set_waypoints(session, None, None, None)?;
        Ok(Some(result))
    } else {
        Ok(None)
    }
}

/// Inputs shared across baseline and enriched plan construction.
struct PlanContext<'a> {
    /// Resolved waypoint positions (from session waypoint inputs).
    waypoints: &'a [Waypoint],
    /// Solver configuration.
    config: &'a MissionConfig,
    /// Resolved propagation model.
    propagator: &'a rpo_core::propagation::propagator::PropagationModel,
    /// Original waypoint inputs (for labels in lean plan result).
    waypoint_inputs: &'a [WaypointInput],
    /// Incremental replan start index (if any).
    changed_from: Option<usize>,
}

/// Plan a single variant and produce its lean result + optional formation report.
///
/// Shared by baseline and enriched plan construction in [`handle_set_waypoints()`].
fn build_plan_variant(
    transfer: &TransferResult,
    cached: Option<WaypointMission>,
    ctx: &PlanContext<'_>,
    formation: Option<(PerchEnrichmentResult, SafetyRequirements)>,
) -> Result<(WaypointMission, LeanPlanResult, Option<FormationDesignReport>), ApiError> {
    let departure = departure_from_transfer(transfer);
    let mission = match (cached, ctx.changed_from) {
        (Some(cached_mission), Some(from)) => replan_from_waypoint(
            cached_mission, from, ctx.waypoints, &departure, ctx.config, ctx.propagator,
        )?,
        _ => plan_waypoint_mission(&departure, ctx.waypoints, ctx.config, ctx.propagator)?,
    };
    let plan = build_lean_plan_result(transfer, &mission, ctx.waypoint_inputs);
    let report = formation.map(|(perch, reqs)| {
        compute_formation_report(perch, reqs, &mission.legs, ctx.waypoints)
    });
    Ok((mission, plan, report))
}

/// Fetch trajectory data for visualization.
///
/// Filters to requested leg indices (or all) and optionally resamples.
///
/// # Errors
///
/// Returns [`ApiError`] if no mission has been planned.
pub fn handle_get_trajectory(
    session: &Session,
    legs: Option<&[usize]>,
    max_points: Option<u32>,
) -> Result<Vec<LegTrajectory>, ApiError> {
    let mission = session.require_active_mission()?;

    let leg_indices: Vec<usize> = match legs {
        Some(indices) => indices
            .iter()
            .copied()
            .filter(|&i| i < mission.legs.len())
            .collect(),
        None => (0..mission.legs.len()).collect(),
    };

    let mut result = Vec::with_capacity(leg_indices.len());
    for &idx in &leg_indices {
        let leg = &mission.legs[idx];
        let points = match max_points {
            Some(max) => leg.resample_trajectory(max),
            None => leg.to_trajectory_points(),
        };
        result.push(LegTrajectory {
            leg_index: idx,
            points,
        });
    }

    Ok(result)
}

/// Compute mission covariance from session state.
///
/// Requires a planned mission, stored transfer, and navigation accuracy.
///
/// # Errors
///
/// Returns [`ApiError`] if mission, transfer, or navigation accuracy is missing,
/// or if covariance computation fails.
pub fn handle_get_covariance(session: &Session) -> Result<MissionCovarianceReport, ApiError> {
    let mission = session.require_active_mission()?;
    let transfer = session.require_transfer()?;
    let nav = session.require_navigation_accuracy()?;
    let propagator = session.resolve_propagation_model();

    let report = compute_mission_covariance(
        mission,
        &transfer.plan.chief_at_arrival,
        nav,
        session.maneuver_uncertainty.as_ref(),
        &propagator,
    )?;

    Ok(report)
}

/// Fetch eclipse data for transfer and mission phases.
///
/// Transfer eclipse is computed on-demand from the stored Lambert transfer.
/// Mission eclipse is read from the pre-computed field on the stored mission.
///
/// # Errors
///
/// Returns [`ApiError`] if neither transfer nor mission is available.
pub fn handle_get_eclipse(session: &Session) -> Result<EclipseResponse, ApiError> {
    // Transfer eclipse: computed on-demand if a Lambert arc exists
    let transfer_eclipse = if let Some(ref transfer) = session.transfer {
        transfer.plan.transfer.as_ref().and_then(|lambert| {
            // Need the chief state — use perch_chief only if proximity,
            // otherwise use original chief (transfer was from original state)
            let chief = session.chief.as_ref().unwrap_or(&transfer.perch_chief);
            compute_transfer_eclipse(lambert, chief, DEFAULT_TRANSFER_ECLIPSE_SAMPLES).ok()
        })
    } else {
        None
    };

    // Mission eclipse: pre-computed on the WaypointMission
    let mission_eclipse = session
        .mission()
        .and_then(|m| m.eclipse.clone());

    if transfer_eclipse.is_none() && mission_eclipse.is_none() {
        return Err(ApiError::InvalidInput(
            crate::error::InvalidInputError::MissingSessionState {
                missing: "transfer or mission",
                context: "compute_transfer or set_waypoints before requesting eclipse data",
            },
        ));
    }

    Ok(EclipseResponse {
        transfer: transfer_eclipse,
        mission: mission_eclipse,
    })
}

/// Build the [`DepartureState`] from a transfer result's perch ROE and chief elements.
fn departure_from_transfer(transfer: &TransferResult) -> DepartureState {
    DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    }
}


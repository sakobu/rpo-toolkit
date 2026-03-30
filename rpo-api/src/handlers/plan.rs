//! Mission planning handlers: set waypoints, update config, get trajectory/covariance/eclipse.
//!
//! These operate on the session's stored transfer and mission state.

use serde::Serialize;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::planning::compute_transfer_eclipse;
use rpo_core::mission::types::{MissionPhase, WaypointMission};
use rpo_core::mission::waypoints::{plan_waypoint_mission, replan_from_waypoint};
use rpo_core::mission::ProximityConfig;
use rpo_core::pipeline::convert::to_waypoints;
use rpo_core::pipeline::{
    compute_mission_covariance, compute_transfer, LeanPlanResult, LegTrajectory, TransferResult,
    TransferSummary, WaypointInput,
};
use rpo_core::propagation::covariance::{
    ManeuverUncertainty, MissionCovarianceReport, NavigationAccuracy,
};
use rpo_core::types::eclipse::{MissionEclipseData, TransferEclipseData};
use rpo_core::types::DepartureState;

use crate::error::ApiError;
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

/// Plan (or replan) the waypoint mission from the session's stored transfer.
///
/// If no transfer is stored, attempts auto-compute for proximity scenarios.
/// For far-field without a stored transfer, returns a `MissingSessionState` error.
///
/// When `changed_from` is `Some(index)`, attempts incremental replan from that
/// waypoint index, reusing earlier legs from the cached mission.
///
/// # Errors
///
/// Returns [`ApiError`] if states are missing, transfer is required but absent,
/// or waypoint planning fails.
pub fn handle_set_waypoints(
    session: &mut Session,
    changed_from: Option<usize>,
    cached_mission: Option<WaypointMission>,
) -> Result<LeanPlanResult, ApiError> {
    // Ensure we have a transfer. If none, try auto-compute.
    if session.transfer.is_none() {
        let phase = super::handle_classify(session)?;
        match phase {
            MissionPhase::Proximity { .. } => {
                // Auto-compute transfer for proximity regime
                let input = session.assemble_pipeline_input()?;
                let transfer = compute_transfer(&input)?;
                session.store_transfer(transfer);
            }
            MissionPhase::FarField { .. } => {
                return Err(ApiError::InvalidInput(
                    crate::error::InvalidInputError::MissingSessionState {
                        missing: "transfer",
                        context: "far-field scenario requires compute_transfer before planning",
                    },
                ));
            }
        }
    }

    let transfer = session.require_transfer()?;
    let departure = departure_from_transfer(transfer);
    let waypoints = to_waypoints(&session.waypoints);
    let propagator = session.resolve_propagation_model();

    let wp_mission = if let Some(index) = changed_from {
        if let Some(cached) = cached_mission {
            replan_from_waypoint(
                cached,
                index,
                &waypoints,
                &departure,
                &session.config,
                &propagator,
            )?
        } else {
            plan_waypoint_mission(&departure, &waypoints, &session.config, &propagator)?
        }
    } else {
        plan_waypoint_mission(&departure, &waypoints, &session.config, &propagator)?
    };

    let lean = build_lean_result(transfer, &wp_mission, &session.waypoints);
    session.store_mission(wp_mission);
    Ok(lean)
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
/// Returns `Some(LeanPlanResult)` if the mission was replanned, `None` if only overlays changed.
///
/// # Errors
///
/// Returns [`ApiError`] if replanning fails.
pub fn handle_update_config(
    session: &mut Session,
    update: ConfigUpdate,
) -> Result<Option<LeanPlanResult>, ApiError> {
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
    let had_mission = session.mission.is_some();

    // Apply mission-affecting fields (these may clear session.mission)
    if let Some(config) = update.config {
        session.set_config(config);
    }
    if let Some(proximity) = update.proximity {
        session.proximity = proximity;
    }

    // Replan if mission-affecting fields changed AND a mission existed
    if needs_replan && had_mission {
        let result = handle_set_waypoints(session, None, None)?;
        Ok(Some(result))
    } else {
        Ok(None)
    }
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
    let mission = session.require_mission()?;

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
    let mission = session.require_mission()?;
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
        .mission
        .as_ref()
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

/// Build a lean plan result from the full mission and transfer.
fn build_lean_result(
    transfer: &TransferResult,
    wp_mission: &WaypointMission,
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
            let label = waypoint_inputs
                .get(i)
                .and_then(|wp| wp.label.clone());
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

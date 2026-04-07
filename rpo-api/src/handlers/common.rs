//! Shared helpers for API handlers.

use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::types::WaypointMission;
use rpo_core::mission::waypoints::plan_waypoint_mission;
use rpo_core::pipeline::convert::to_waypoints;
use rpo_core::pipeline::{resolve_propagator, PropagatorChoice, TransferResult, WaypointInput};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_nyx::nyx_bridge::extract_dmf_rates;
use rpo_core::types::{DepartureState, SpacecraftConfig};

use crate::error::{ApiError, InvalidInputError};
use crate::protocol::{ProgressPhase, ProgressUpdate, PropagatorToggle};

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via nyx DMF.
/// Returns the propagation model and optional derived drag config.
///
/// # Errors
///
/// Returns [`ApiError`] if drag extraction fails.
pub fn resolve_drag_and_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    base_propagator: &PropagationModel,
) -> Result<(PropagationModel, Option<DragConfig>), ApiError> {
    let auto_drag_config = if auto_drag {
        Some(extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            chief_config,
            deputy_config,
            almanac,
        )?)
    } else {
        None
    };
    Ok(resolve_propagator(auto_drag_config, base_propagator.clone()))
}

/// Resolve a [`PropagatorToggle`] to a [`PropagatorChoice`], requiring drag config for `J2Drag`.
///
/// # Errors
///
/// Returns [`ApiError::InvalidInput`] if `J2Drag` is selected but no drag config is available.
pub fn resolve_propagator_toggle(
    toggle: PropagatorToggle,
    drag_config: Option<DragConfig>,
) -> Result<PropagatorChoice, ApiError> {
    match toggle {
        PropagatorToggle::J2 => Ok(PropagatorChoice::J2),
        PropagatorToggle::J2Drag => {
            let drag = drag_config.ok_or(ApiError::InvalidInput(
                InvalidInputError::MissingSessionState {
                    missing: "drag_config",
                    context: "call extract_drag before selecting j2_drag propagator",
                },
            ))?;
            Ok(PropagatorChoice::J2Drag { drag })
        }
    }
}

/// Best-effort progress update (dropped if channel is full).
pub(crate) fn send_progress(
    tx: &mpsc::Sender<ProgressUpdate>,
    phase: ProgressPhase,
    detail: &str,
    fraction: f64,
) {
    if let Err(e) = tx.try_send(ProgressUpdate {
        phase,
        detail: Some(detail.into()),
        fraction: Some(fraction),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }
}

/// If auto-drag extracted new rates, replan the mission with the updated propagator.
///
/// Returns the original mission unchanged when `auto_drag` is false or no drag was derived.
///
/// # Errors
///
/// Returns [`ApiError`] if replanning fails.
pub(crate) fn replan_if_drag_changed(
    auto_drag: bool,
    derived_drag: Option<&DragConfig>,
    mission: WaypointMission,
    transfer: &TransferResult,
    waypoints: &[WaypointInput],
    config: &MissionConfig,
    propagator: &PropagationModel,
) -> Result<WaypointMission, ApiError> {
    if auto_drag && derived_drag.is_some() {
        let departure = DepartureState {
            roe: transfer.plan.perch_roe,
            chief: transfer.plan.chief_at_arrival,
            epoch: transfer.arrival_epoch,
        };
        let wp = to_waypoints(waypoints);
        Ok(plan_waypoint_mission(&departure, &wp, config, propagator)?)
    } else {
        Ok(mission)
    }
}

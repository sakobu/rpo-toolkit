//! Validation handler: per-leg nyx validation with progress streaming.
//!
//! Designed to be called from `spawn_blocking`. Sends `Progress` updates
//! between legs and checks `cancel` between legs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::types::WaypointMission;
use rpo_core::mission::{validate_mission_nyx, ValidationConfig, ValidationReport};
use rpo_core::pipeline::{TransferResult, WaypointInput};
use rpo_core::propagation::PropagationModel;
use rpo_core::types::{SpacecraftConfig, StateVector};

use super::common::{replan_if_drag_changed, resolve_drag_and_propagator, send_progress};
use crate::error::ApiError;
use crate::protocol::{ProgressPhase, ProgressUpdate};

/// Request payload for validation, cloned from session state.
///
/// Contains all data needed to run validation independently of the session,
/// since validation runs on a blocking thread.
pub struct ValidateRequest {
    /// The planned waypoint mission to validate.
    pub mission: WaypointMission,
    /// Chief ECI state at perch (mission start).
    pub perch_chief: StateVector,
    /// Deputy ECI state at perch (mission start).
    pub perch_deputy: StateVector,
    /// Chief spacecraft physical properties.
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft physical properties.
    pub deputy_config: SpacecraftConfig,
    /// Number of nyx sample points per leg.
    pub samples_per_leg: u32,
    /// Auto-derive drag rates before validation.
    pub auto_drag: bool,
    /// Waypoint inputs (needed for drag-based replan).
    pub waypoints: Vec<WaypointInput>,
    /// Mission solver configuration (needed for drag-based replan).
    pub config: MissionConfig,
    /// Propagation model (needed for drag-based replan).
    pub propagator: PropagationModel,
    /// Transfer result (needed for departure state in replan).
    pub transfer: TransferResult,
}

/// Run per-leg nyx validation with progress streaming and cancellation.
///
/// Uses the pre-planned mission from the request. When `auto_drag` is true,
/// extracts drag rates and replans with a drag propagator before validating.
///
/// # Errors
/// Returns [`ApiError`] if planning, propagation, or validation fails,
/// or if the operation is cancelled.
pub fn handle_validate(
    request: ValidateRequest,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
) -> Result<ValidationReport, ApiError> {
    send_progress(progress_tx, ProgressPhase::Validate, "Planning mission...", 0.0);

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Resolve propagator (with optional auto-drag)
    let (propagator, derived_drag) = resolve_drag_and_propagator(
        request.auto_drag,
        &request.transfer,
        &request.chief_config,
        &request.deputy_config,
        almanac,
        &request.propagator,
    )?;

    // If auto_drag changed the propagator, replan the mission
    let mission = replan_if_drag_changed(
        request.auto_drag,
        derived_drag.as_ref(),
        request.mission,
        &request.transfer,
        &request.waypoints,
        &request.config,
        &propagator,
    )?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    send_progress(progress_tx, ProgressPhase::Validate, "Running nyx validation...", 0.1);

    let val_config = ValidationConfig {
        samples_per_leg: request.samples_per_leg,
        chief_config: request.chief_config,
        deputy_config: request.deputy_config,
    };
    let report = validate_mission_nyx(
        &mission,
        &request.perch_chief,
        &request.perch_deputy,
        &val_config,
        almanac,
    )?;

    send_progress(progress_tx, ProgressPhase::Validate, "Validation complete", 1.0);

    Ok(report)
}


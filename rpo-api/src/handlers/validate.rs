//! Validation handler: per-leg nyx validation with progress streaming.
//!
//! Designed to be called from `spawn_blocking`. Sends `Progress` updates
//! between legs and checks `cancel` between legs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{validate_mission_nyx, ValidationReport};
use rpo_core::pipeline::{compute_transfer, plan_waypoints_from_transfer};

use super::common::resolve_drag_and_propagator;
use crate::error::ApiError;
use crate::protocol::{MissionDefinition, ProgressPhase, ProgressUpdate};

/// Run per-leg nyx validation with progress streaming and cancellation.
///
/// Plans the mission first, then validates each leg against nyx full-physics.
///
/// # Errors
/// Returns [`ApiError`] if planning, propagation, or validation fails,
/// or if the operation is cancelled.
pub fn handle_validate(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
    samples_per_leg: u32,
    auto_drag: bool,
) -> Result<ValidationReport, ApiError> {
    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Validate,
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    let chief_config = def.chief_config.unwrap_or_default().resolve();
    let deputy_config = def.deputy_config.unwrap_or_default().resolve();

    // Phase 1: Compute transfer
    let transfer = compute_transfer(def)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 2: Resolve propagator
    let (propagator, _derived_drag) =
        resolve_drag_and_propagator(auto_drag, &transfer, &chief_config, &deputy_config, almanac, def)?;

    // Phase 3: Plan waypoints
    let wp_mission = plan_waypoints_from_transfer(&transfer, def, &propagator)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 4: Validate via nyx
    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Validate,
        detail: Some("Running nyx validation...".into()),
        fraction: Some(0.1),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    let report = validate_mission_nyx(
        &wp_mission,
        &transfer.perch_chief,
        &transfer.perch_deputy,
        samples_per_leg,
        &chief_config,
        &deputy_config,
        almanac,
    )?;

    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Validate,
        detail: Some("Validation complete".into()),
        fraction: Some(1.0),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    Ok(report)
}

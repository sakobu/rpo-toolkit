//! Validation handler: per-leg nyx validation with progress streaming.
//!
//! Designed to be called from `spawn_blocking`. Sends `Progress` updates
//! between legs and checks `cancel` between legs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{validate_mission_nyx, ValidationReport};
use rpo_core::pipeline::{
    compute_transfer, plan_waypoints_from_transfer, resolve_propagator, to_propagation_model,
};
use rpo_core::propagation::extract_dmf_rates;

use crate::error::ApiError;
use crate::protocol::MissionDefinition;

/// Progress update sent from the validation handler to the WebSocket loop.
pub struct ProgressUpdate {
    /// Phase label.
    pub phase: String,
    /// Human-readable detail.
    pub detail: Option<String>,
    /// Fraction complete (0.0 to 1.0).
    pub fraction: Option<f64>,
}

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
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    });

    let chief_config = def.chief_config.unwrap_or_default();
    let deputy_config = def.deputy_config.unwrap_or_default();

    // Phase 1: Compute transfer
    let transfer = compute_transfer(def)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 2: Resolve propagator
    let auto_drag_config = if auto_drag {
        Some(extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            &chief_config,
            &deputy_config,
            almanac,
        )?)
    } else {
        None
    };
    let (propagator, _derived_drag) =
        resolve_propagator(auto_drag_config, to_propagation_model(&def.propagator));

    // Phase 3: Plan waypoints
    let wp_mission = plan_waypoints_from_transfer(&transfer, def, &propagator)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 4: Validate via nyx
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Running nyx validation...".into()),
        fraction: Some(0.1),
    });

    let report = validate_mission_nyx(
        &wp_mission,
        &transfer.perch_chief,
        &transfer.perch_deputy,
        samples_per_leg,
        &chief_config,
        &deputy_config,
        almanac,
    )?;

    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Validation complete".into()),
        fraction: Some(1.0),
    });

    Ok(report)
}

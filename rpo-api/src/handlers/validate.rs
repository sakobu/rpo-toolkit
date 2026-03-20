//! Validation handler: per-leg nyx validation with progress streaming.
//!
//! Designed to be called from `spawn_blocking`. Sends `Progress` updates
//! between legs and checks `cancel` between legs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{validate_mission_nyx, ValidationReport};

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
/// Plans the mission first (same as `handle_plan`), then validates each leg
/// against nyx full-physics. Sends progress updates between legs.
///
/// For simplicity, falls back to `validate_mission_nyx()` (which includes
/// eclipse validation). The per-leg API exists for future fine-grained progress.
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

    let prepared = super::common::plan_and_prepare(
        def, almanac, cancel, auto_drag, &chief_config, &deputy_config,
    )?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Validate via nyx (full mission — includes eclipse)
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Running nyx validation...".into()),
        fraction: Some(0.1),
    });

    let report = validate_mission_nyx(
        &prepared.wp_mission,
        &prepared.perch_chief,
        &prepared.perch_deputy,
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

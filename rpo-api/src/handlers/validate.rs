//! Validation handler: per-leg nyx validation with progress streaming.
//!
//! Designed to be called from `spawn_blocking`. Sends `Progress` updates
//! between legs and checks `cancel` between legs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{
    plan_mission, plan_waypoint_mission, validate_mission_nyx, ValidationReport,
};
use rpo_core::propagation::propagate_keplerian;
use rpo_core::types::{DepartureState, StateVector};

use crate::convert::{resolve_propagator, to_propagation_model, to_waypoints};
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
pub fn handle_validate(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
    progress_tx: mpsc::Sender<ProgressUpdate>,
    cancel: Arc<AtomicBool>,
    samples_per_leg: u32,
    auto_drag: bool,
) -> Result<ValidationReport, ApiError> {
    // Phase 1: Plan the mission
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    });

    let perch = def
        .perch
        .clone()
        .unwrap_or(rpo_core::mission::PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = def.proximity.unwrap_or_default();
    let lambert_tof_s = def.lambert_tof_s.unwrap_or(3600.0);
    let lambert_cfg = def.lambert_config.clone().unwrap_or_default();

    let plan = plan_mission(
        &def.chief,
        &def.deputy,
        &perch,
        &proximity,
        lambert_tof_s,
        &lambert_cfg,
    )?;

    let arrival_epoch =
        def.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);

    let (perch_chief, perch_deputy) = if let Some(ref transfer) = plan.transfer {
        let chief_traj = propagate_keplerian(&def.chief, lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or_else(|| {
                ApiError::InvalidMessage("empty chief trajectory".into())
            })?
            .clone();
        let deputy_at_perch = StateVector {
            epoch: arrival_epoch,
            position_eci_km: transfer.arrival_state.position_eci_km,
            velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
                + transfer.arrival_dv_eci_km_s,
        };
        (chief_at_arrival, deputy_at_perch)
    } else {
        (def.chief.clone(), def.deputy.clone())
    };

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 2: Resolve propagator (optional auto-drag)
    let chief_config = def.chief_config.unwrap_or_default();
    let deputy_config = def.deputy_config.unwrap_or_default();

    let (propagator, _derived_drag) = resolve_propagator(
        auto_drag,
        &perch_chief,
        &perch_deputy,
        &chief_config,
        &deputy_config,
        almanac,
        to_propagation_model(&def.propagator),
    )?;

    // Phase 3: Plan waypoints
    let waypoints = to_waypoints(&def.waypoints);
    let departure = DepartureState {
        roe: plan.perch_roe,
        chief: plan.chief_at_arrival,
        epoch: arrival_epoch,
    };
    let wp_mission =
        plan_waypoint_mission(&departure, &waypoints, &def.config, &propagator)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 4: Validate via nyx (full mission — includes eclipse)
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "validate".into(),
        detail: Some("Running nyx validation...".into()),
        fraction: Some(0.1),
    });

    let report = validate_mission_nyx(
        &wp_mission,
        &perch_chief,
        &perch_deputy,
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

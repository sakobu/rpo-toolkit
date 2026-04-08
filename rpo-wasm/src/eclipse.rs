//! Eclipse computation along transfer arcs and mission legs.

use wasm_bindgen::prelude::*;

use rpo_core::mission::types::WaypointMission;
use rpo_core::propagation::lambert::LambertTransfer;
use rpo_core::types::eclipse::{MissionEclipseData, TransferEclipseData};
use rpo_core::types::StateVector;

use crate::error::WasmError;

/// Compute eclipse data along a Lambert transfer arc.
///
/// # Arguments
///
/// * `transfer` — Solved Lambert transfer (departure/arrival states, TOF).
/// * `chief` — Chief ECI state at departure (used for Sun/Moon geometry).
/// * `arc_steps` — Number of sample points along the transfer arc.
///
/// # Invariants
///
/// - `arc_steps > 0`
///
/// # Errors
///
/// Returns [`WasmError`] if eclipse computation fails (e.g. degenerate geometry).
#[wasm_bindgen]
pub fn compute_transfer_eclipse(
    transfer: LambertTransfer,
    chief: StateVector,
    arc_steps: u32,
) -> Result<TransferEclipseData, WasmError> {
    rpo_core::mission::planning::compute_transfer_eclipse(&transfer, &chief, arc_steps)
        .map_err(WasmError::from)
}

/// Compute per-leg eclipse data for a planned mission.
///
/// # Arguments
///
/// * `mission` — Planned waypoint mission with per-leg trajectories.
///
/// # Errors
///
/// Returns [`WasmError`] if eclipse computation fails.
#[wasm_bindgen]
pub fn compute_mission_eclipse(
    mission: WaypointMission,
) -> Result<MissionEclipseData, WasmError> {
    rpo_core::mission::compute_mission_eclipse(&mission.legs).map_err(WasmError::from)
}

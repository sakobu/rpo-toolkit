//! Mission state queries at arbitrary elapsed times.

use wasm_bindgen::prelude::*;

use rpo_core::mission::types::WaypointMission;
use rpo_core::pipeline::types::PropagatorChoice;
use rpo_core::pipeline::to_propagation_model;
use rpo_core::propagation::PropagatedState;

use crate::error::WasmError;

/// Query the mission state at an arbitrary elapsed time.
///
/// Returns the propagated deputy state at the requested time, or `None`
/// if the elapsed time is outside the mission duration.
///
/// # Arguments
///
/// * `mission` — Planned waypoint mission with per-leg maneuver data.
/// * `elapsed_s` — Elapsed time from mission start (seconds).
/// * `propagator` — Propagation model choice (J2 or J2+drag).
///
/// # Invariants
///
/// - `elapsed_s >= 0.0`
///
/// # Errors
///
/// Returns [`WasmError`] if propagation fails for the requested time.
#[wasm_bindgen]
pub fn get_mission_state_at_time(
    mission: WaypointMission,
    elapsed_s: f64,
    propagator: PropagatorChoice,
) -> Result<Option<PropagatedState>, WasmError> {
    let model = to_propagation_model(&propagator);
    rpo_core::mission::waypoints::get_mission_state_at_time(&mission, elapsed_s, &model)
        .map_err(WasmError::from)
}

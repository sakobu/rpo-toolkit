//! Classification and waypoint mission planning.

use wasm_bindgen::prelude::*;

use rpo_core::mission::config::ProximityConfig;
use rpo_core::mission::types::{MissionPhase, Waypoint, WaypointMission};
use rpo_core::pipeline::{to_propagation_model, to_waypoints};
use rpo_core::types::DepartureState;
use rpo_core::mission::config::MissionConfig;
use rpo_core::pipeline::types::{PropagatorChoice, WaypointInput};

use crate::error::WasmError;

/// Classify the separation between chief and deputy spacecraft.
///
/// Returns a [`MissionPhase`] (either `far_field` or `proximity`) with
/// orbital elements, separation distance, and dimensionless ratio.
///
/// # Errors
///
/// Returns [`WasmError`] if classification encounters invalid orbital elements.
#[wasm_bindgen]
pub fn classify_separation(
    chief: rpo_core::types::StateVector,
    deputy: rpo_core::types::StateVector,
    config: ProximityConfig,
) -> Result<MissionPhase, WasmError> {
    rpo_core::mission::planning::classify_separation(&chief, &deputy, &config)
        .map_err(WasmError::from)
}

/// Plan a multi-waypoint mission from a departure state.
///
/// # Arguments
///
/// * `departure` — Deputy departure state (ROE + chief elements).
/// * `waypoints` — `Vec<WaypointInput>` as `JsValue`. Uses serde-wasm-bindgen
///   because tsify cannot generate `FromWasmAbi` for bare `Vec<T>` parameters.
/// * `config` — Mission configuration (targeting, TOF, safety thresholds).
/// * `propagator` — Propagator selection (J2 or J2+drag).
///
/// # Invariants
///
/// - `waypoints` must deserialize to a non-empty `Vec<WaypointInput>`
///
/// # Errors
///
/// Returns [`WasmError`] if targeting fails or waypoints are empty.
#[wasm_bindgen]
pub fn plan_waypoint_mission(
    departure: DepartureState,
    waypoints: JsValue,
    config: MissionConfig,
    propagator: PropagatorChoice,
) -> Result<WaypointMission, WasmError> {
    let inputs: Vec<WaypointInput> =
        crate::error::deserialize_js(waypoints, "waypoints")?;
    let wps: Vec<Waypoint> = to_waypoints(&inputs);
    let model = to_propagation_model(&propagator);
    rpo_core::mission::waypoints::plan_waypoint_mission(&departure, &wps, &config, &model)
        .map_err(WasmError::from)
}

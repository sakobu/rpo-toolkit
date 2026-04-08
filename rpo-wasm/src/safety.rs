//! Safety analysis, COLA assessment, and avoidance maneuver computation.

use wasm_bindgen::prelude::*;

use rpo_core::mission::avoidance::ColaConfig;
use rpo_core::mission::closest_approach::ClosestApproach;
use rpo_core::mission::cola_assessment::ColaAssessment;
use rpo_core::mission::types::WaypointMission;
use rpo_core::mission::AvoidanceManeuver;
use rpo_core::pipeline::types::{PropagatorChoice, SafetyAnalysis};
use rpo_core::pipeline::to_propagation_model;
use rpo_core::types::{KeplerianElements, QuasiNonsingularROE};

use crate::error::WasmError;

/// Compute the full safety analysis for a planned mission.
///
/// Returns free-drift analysis, POCA refinement, and optional COLA
/// assessment — the same computation done inside `build_output`.
///
/// # Arguments
///
/// * `safety` — Safety thresholds for POCA alert detection. `None` skips
///   the POCA alert threshold check (raw POCA data is still computed).
/// * `cola` — COLA avoidance configuration. `None` skips avoidance
///   maneuver computation entirely.
#[must_use]
#[wasm_bindgen]
pub fn compute_safety_analysis(
    mission: WaypointMission,
    safety: Option<rpo_core::mission::config::SafetyConfig>,
    cola: Option<ColaConfig>,
    propagator: PropagatorChoice,
) -> SafetyAnalysis {
    let model = to_propagation_model(&propagator);
    rpo_core::pipeline::compute_safety_analysis(&mission, safety.as_ref(), cola.as_ref(), &model)
}

/// Assess COLA threat level from pre-computed POCA data.
///
/// # Arguments
///
/// * `poca` — `Vec<Vec<ClosestApproach>>` as `JsValue` (outer = legs, inner = POCAs).
///   Uses serde-wasm-bindgen because tsify cannot generate `FromWasmAbi` for nested `Vec<Vec<T>>`.
///
/// # Errors
///
/// Returns [`WasmError`] if deserialization of POCA data fails.
#[wasm_bindgen]
pub fn assess_cola(
    mission: WaypointMission,
    poca: JsValue,
    propagator: PropagatorChoice,
    config: ColaConfig,
) -> Result<ColaAssessment, WasmError> {
    let poca_data: Vec<Vec<ClosestApproach>> =
        crate::error::deserialize_js(poca, "poca")?;
    let model = to_propagation_model(&propagator);
    Ok(rpo_core::mission::cola_assessment::assess_cola(
        &mission,
        &poca_data,
        &model,
        &config,
    ))
}

/// Compute a single avoidance maneuver for a POCA violation.
///
/// # Arguments
///
/// * `departure_epoch` — ISO 8601 epoch string (parsed to `hifitime::Epoch`).
///
/// # Invariants
///
/// - `departure_epoch` must be a valid ISO 8601 epoch string
/// - `tof_s > 0`
///
/// # Errors
///
/// Returns [`WasmError`] if epoch parsing fails or the avoidance solver fails.
#[wasm_bindgen]
pub fn compute_avoidance(
    poca: ClosestApproach,
    roe: QuasiNonsingularROE,
    chief_mean: KeplerianElements,
    departure_epoch: String,
    tof_s: f64,
    propagator: PropagatorChoice,
    config: ColaConfig,
) -> Result<AvoidanceManeuver, WasmError> {
    let epoch: hifitime::Epoch = departure_epoch.parse().map_err(|e| WasmError {
        code: crate::error::WasmErrorCode::Deserialization,
        message: format!("departure_epoch is not valid ISO 8601: {e}"),
        details: None,
    })?;
    let model = to_propagation_model(&propagator);
    rpo_core::mission::avoidance::compute_avoidance(
        &poca,
        &roe,
        &chief_mean,
        epoch,
        tof_s,
        &model,
        &config,
    )
    .map_err(WasmError::from)
}

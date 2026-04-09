//! Covariance, free-drift, POCA, and trajectory resampling analysis.

use serde::{Deserialize, Serialize};
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::mission::closest_approach::ClosestApproach;
use rpo_core::mission::free_drift::FreeDriftAnalysis;
use rpo_core::mission::types::{ManeuverLeg, WaypointMission};
use rpo_core::pipeline::types::PropagatorChoice;
use rpo_core::pipeline::to_propagation_model;
use rpo_core::propagation::covariance::types::{
    ManeuverUncertainty, MissionCovarianceReport, NavigationAccuracy,
};
use rpo_core::propagation::PropagatedState;
use rpo_core::types::KeplerianElements;

use crate::error::WasmError;

/// Wrapper for `Vec<FreeDriftAnalysis>` (wasm-bindgen compatibility).
#[derive(Debug, Serialize, Deserialize, Tsify)]
// Output-only: no from_wasm_abi needed (never passed from JS to Rust).
#[tsify(into_wasm_abi)]
pub struct FreeDriftResult {
    /// Per-leg free-drift analysis results.
    pub analyses: Vec<FreeDriftAnalysis>,
}

/// Wrapper for `Vec<Vec<ClosestApproach>>` (wasm-bindgen compatibility).
#[derive(Debug, Serialize, Deserialize, Tsify)]
// Output-only: no from_wasm_abi needed (never passed from JS to Rust).
#[tsify(into_wasm_abi)]
pub struct PocaResult {
    /// Per-leg POCA results (outer = legs, inner = closest approaches).
    pub legs: Vec<Vec<ClosestApproach>>,
}

/// Wrapper for resampled trajectory states (wasm-bindgen compatibility).
#[derive(Debug, Serialize, Deserialize, Tsify)]
// Output-only: no from_wasm_abi needed (never passed from JS to Rust).
#[tsify(into_wasm_abi)]
pub struct ResampledTrajectory {
    /// Resampled propagated states along the leg.
    pub states: Vec<PropagatedState>,
}

/// Compute mission-level covariance propagation.
///
/// Composes initial RIC→ROE covariance conversion and multi-leg propagation
/// in a single call.
///
/// # Arguments
///
/// * `mission` — Planned waypoint mission with per-leg maneuver data.
/// * `chief_at_arrival` — Chief mean Keplerian elements at the arrival epoch.
/// * `navigation_accuracy` — 1-sigma RIC position/velocity uncertainties.
/// * `maneuver_uncertainty` — Per-burn execution error model. `None` assumes
///   perfect maneuver execution (no covariance injection at burn epochs).
/// * `propagator` — Propagation model choice (J2 or J2+drag).
///
/// # Errors
///
/// Returns [`WasmError`] if covariance conversion or propagation fails.
#[wasm_bindgen]
pub fn compute_mission_covariance(
    mission: WaypointMission,
    chief_at_arrival: KeplerianElements,
    navigation_accuracy: NavigationAccuracy,
    maneuver_uncertainty: Option<ManeuverUncertainty>,
    propagator: PropagatorChoice,
) -> Result<MissionCovarianceReport, WasmError> {
    let model = to_propagation_model(&propagator);
    rpo_core::pipeline::compute_mission_covariance(
        &mission,
        &chief_at_arrival,
        &navigation_accuracy,
        maneuver_uncertainty.as_ref(),
        &model,
    )
    .map_err(WasmError::from)
}

/// Compute free-drift (abort-case) analysis for all mission legs.
///
/// # Arguments
///
/// * `mission` — Planned waypoint mission with per-leg maneuver data.
/// * `propagator` — Propagation model choice (J2 or J2+drag).
///
/// Returns `None` if any leg's free-drift computation fails (graceful degradation).
#[must_use]
#[wasm_bindgen]
pub fn compute_free_drift_analysis(
    mission: WaypointMission,
    propagator: PropagatorChoice,
) -> Option<FreeDriftResult> {
    let model = to_propagation_model(&propagator);
    rpo_core::pipeline::compute_free_drift_analysis(&mission, &model)
        .map(|analyses| FreeDriftResult { analyses })
}

/// Compute refined closest-approach (POCA) analysis for all mission legs.
///
/// # Arguments
///
/// * `mission` — Planned waypoint mission with per-leg maneuver data.
/// * `propagator` — Propagation model choice (J2 or J2+drag).
///
/// Returns `None` if any leg's POCA computation fails (graceful degradation).
#[must_use]
#[wasm_bindgen]
pub fn compute_poca_analysis(
    mission: WaypointMission,
    propagator: PropagatorChoice,
) -> Option<PocaResult> {
    let model = to_propagation_model(&propagator);
    rpo_core::pipeline::compute_poca_analysis(&mission, &model)
        .map(|legs| PocaResult { legs })
}

/// Resample a single leg trajectory with a different step count.
///
/// Useful for visualization at a different resolution than the planning step.
///
/// # Invariants
///
/// - `n_steps > 0`
///
/// # Errors
///
/// Returns [`WasmError`] if propagation fails.
#[wasm_bindgen]
pub fn resample_leg_trajectory(
    leg: ManeuverLeg,
    n_steps: usize,
    propagator: PropagatorChoice,
) -> Result<ResampledTrajectory, WasmError> {
    let model = to_propagation_model(&propagator);
    rpo_core::mission::waypoints::resample_leg_trajectory(&leg, n_steps, &model)
        .map(|states| ResampledTrajectory { states })
        .map_err(WasmError::from)
}

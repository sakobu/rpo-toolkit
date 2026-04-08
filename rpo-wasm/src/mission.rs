//! Mission execution and replanning from a pre-computed transfer.

use serde::{Deserialize, Serialize};
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::mission::types::WaypointMission;
use rpo_core::pipeline::types::{PipelineInput, PipelineOutput, TransferResult};

use crate::error::WasmError;

/// Combined output returning both the pipeline output and the mutated transfer.
///
/// Core functions take `&mut TransferResult`; WASM cannot pass mutable
/// references across the boundary, so this returns owned copies of both.
#[derive(Serialize, Deserialize, Tsify)]
// Output-only: no from_wasm_abi needed (never passed from JS to Rust).
#[tsify(into_wasm_abi)]
pub struct MissionResult {
    /// Pipeline output (mission, safety, covariance, etc.).
    pub output: PipelineOutput,
    /// Transfer result (potentially mutated by enrichment).
    pub transfer: TransferResult,
}

/// Execute a full mission from a pre-computed Lambert transfer.
///
/// # Errors
///
/// Returns [`WasmError`] on targeting, propagation, or covariance failure.
#[wasm_bindgen]
pub fn execute_mission_from_transfer(
    mut transfer: TransferResult,
    input: PipelineInput,
) -> Result<MissionResult, WasmError> {
    let output =
        rpo_core::pipeline::execute_mission_from_transfer(&mut transfer, &input)
            .map_err(WasmError::from)?;
    Ok(MissionResult { output, transfer })
}

/// Re-execute a mission after modifying a waypoint.
///
/// Preserves converged legs before `modified_index` when a cached mission
/// is provided, avoiding redundant re-targeting.
///
/// # Arguments
///
/// * `cached_mission` — Previous mission result for incremental replanning.
///   `None` = full re-plan from scratch. `Some(m)` = reuse converged legs
///   before `modified_index`, only re-targeting from the modified waypoint onward.
///
/// # Invariants
///
/// - `modified_index` must be within the waypoint count of the input
///
/// # Errors
///
/// Returns [`WasmError`] on targeting or propagation failure.
#[wasm_bindgen]
pub fn replan_from_transfer(
    mut transfer: TransferResult,
    input: PipelineInput,
    modified_index: usize,
    cached_mission: Option<WaypointMission>,
) -> Result<MissionResult, WasmError> {
    let output = rpo_core::pipeline::replan_from_transfer(
        &mut transfer,
        &input,
        modified_index,
        cached_mission,
    )
    .map_err(WasmError::from)?;
    Ok(MissionResult { output, transfer })
}

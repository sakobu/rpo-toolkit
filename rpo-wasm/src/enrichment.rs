//! Formation design enrichment: suggest, apply, accept, and per-waypoint enrichment.

use serde::{Deserialize, Serialize};
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::mission::formation::{EnrichedWaypoint, SafetyRequirements};
use rpo_core::pipeline::types::{
    EnrichmentSuggestion, PipelineInput, PipelineOutput, TransferResult,
};
use rpo_core::types::{KeplerianElements, QuasiNonsingularROE};

use crate::error::WasmError;

/// Combined output from accepting waypoint enrichment.
///
/// Core functions take `&mut PipelineInput` and `&mut TransferResult`;
/// WASM cannot pass mutable references across the boundary, so this
/// returns owned copies of all three values.
#[derive(Debug, Serialize, Deserialize, Tsify)]
// Output-only: no from_wasm_abi needed (never passed from JS to Rust).
#[tsify(into_wasm_abi)]
pub struct EnrichmentAcceptResult {
    /// Pipeline output from re-execution with enriched waypoint.
    pub output: PipelineOutput,
    /// Mutated pipeline input (waypoint ROE target updated).
    pub input: PipelineInput,
    /// Mutated transfer result.
    pub transfer: TransferResult,
}

/// Check whether perch enrichment is available for this transfer + input.
///
/// # Arguments
///
/// * `transfer` — Pre-computed transfer result (classification + perch ROE).
/// * `input` — Full pipeline input (must include `safety_requirements` for enrichment).
///
/// Returns `None` if safety requirements are not configured or enrichment
/// is not applicable.
#[must_use]
#[wasm_bindgen]
pub fn suggest_enrichment(
    transfer: TransferResult,
    input: PipelineInput,
) -> Option<EnrichmentSuggestion> {
    rpo_core::pipeline::suggest_enrichment(&transfer, &input)
}

/// Apply a perch enrichment suggestion to the transfer result.
///
/// Mutates the transfer's perch ROE to the enriched safe-perch values.
/// Returns the updated transfer.
///
/// # Arguments
///
/// * `transfer` — Transfer result to enrich (consumed and returned with updated perch ROE).
/// * `suggestion` — Enrichment suggestion from [`suggest_enrichment`].
#[must_use]
#[wasm_bindgen]
pub fn apply_perch_enrichment(
    mut transfer: TransferResult,
    suggestion: EnrichmentSuggestion,
) -> TransferResult {
    rpo_core::pipeline::apply_perch_enrichment(&mut transfer, &suggestion);
    transfer
}

/// Accept enrichment for a specific waypoint and re-execute the mission.
///
/// Updates the waypoint's ROE target to the enriched values, then
/// re-runs the full mission pipeline.
///
/// # Invariants
///
/// - `waypoint_index` must be within the waypoint count of `input`
///
/// # Errors
///
/// Returns [`WasmError`] on targeting or propagation failure.
#[wasm_bindgen]
pub fn accept_waypoint_enrichment(
    mut input: PipelineInput,
    mut transfer: TransferResult,
    waypoint_index: usize,
    enriched_roe: QuasiNonsingularROE,
    chief_at_waypoint: KeplerianElements,
) -> Result<EnrichmentAcceptResult, WasmError> {
    let output = rpo_core::pipeline::accept_waypoint_enrichment(
        &mut input,
        &mut transfer,
        waypoint_index,
        &enriched_roe,
        &chief_at_waypoint,
    )
    .map_err(WasmError::from)?;
    Ok(EnrichmentAcceptResult {
        output,
        input,
        transfer,
    })
}

/// Compute a safe enriched waypoint for a single position.
///
/// On-demand per-waypoint computation showing baseline vs enriched e/i
/// vectors for the "safe alternative" card.
///
/// # Arguments
///
/// * `position_ric_km` — `[f64; 3]` as `JsValue`. Uses serde-wasm-bindgen
///   because tsify cannot generate `FromWasmAbi` for bare fixed-size arrays.
/// * `velocity_ric_km_s` — `Option<[f64; 3]>` as `JsValue` (same reason).
/// * `chief_mean` — Chief mean Keplerian elements at the waypoint.
/// * `requirements` — Safety requirements (e/i threshold, alignment).
///
/// # Invariants
///
/// - `position_ric_km` must deserialize to a 3-element `[f64; 3]` array
/// - `velocity_ric_km_s` must deserialize to `Option<[f64; 3]>`
///
/// # Errors
///
/// Returns [`WasmError`] if enrichment computation fails.
#[wasm_bindgen]
pub fn enrich_waypoint(
    position_ric_km: JsValue,
    velocity_ric_km_s: JsValue,
    chief_mean: KeplerianElements,
    requirements: SafetyRequirements,
) -> Result<EnrichedWaypoint, WasmError> {
    let pos: [f64; 3] = crate::error::deserialize_js(position_ric_km, "position_ric_km")?;
    let vel: Option<[f64; 3]> =
        crate::error::deserialize_js(velocity_ric_km_s, "velocity_ric_km_s")?;

    let pos_vec = nalgebra::Vector3::new(pos[0], pos[1], pos[2]);
    let vel_vec = vel.map(|v| nalgebra::Vector3::new(v[0], v[1], v[2]));

    rpo_core::mission::formation::safety_envelope::enrich_waypoint(
        &pos_vec,
        vel_vec.as_ref(),
        &chief_mean,
        &requirements,
    )
    .map_err(WasmError::from)
}

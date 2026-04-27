//! WASM binding for the full transfer-computation pipeline:
//! classify → Lambert → perch geometry → arc densification (+ optional enrichment).
//!
//! Replaces the previous `compute_transfer` WebSocket route now that the
//! orchestration is fully WASM-eligible (Lambert moved to `rpo-core` in-tree).

use serde::Serialize;
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::mission::formation::SafetyRequirements;
use rpo_core::pipeline::types::{EnrichmentSuggestion, TransferComputationInput, TransferResult};

use crate::error::WasmError;

/// Combined transfer + enrichment result, matching the pre-port
/// `ServerMessage::TransferResult` wire shape.
///
/// `enrichment` is `None` when `safety_requirements` is omitted from the
/// input, mirroring `serde(skip_serializing_if = "Option::is_none")` on the
/// old WebSocket variant.
#[derive(Debug, Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct ComputeTransferOutput {
    /// Transfer solution (classification + optional Lambert + perch states).
    pub transfer: TransferResult,
    /// Enrichment outcome when `safety_requirements` was supplied.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enrichment: Option<EnrichmentSuggestion>,
}

/// Compute a transfer and, when `safety_requirements` is supplied, the
/// matching perch enrichment suggestion.
///
/// Thin pass-through to [`rpo_core::pipeline::compute_transfer_with_enrichment`];
/// the orchestration lives in `rpo-core` so CLI and tests can exercise the
/// same code path.
///
/// # Errors
///
/// Returns [`WasmError`] on classification, Lambert, propagation, or
/// arc-densification failure.
#[wasm_bindgen]
pub fn compute_transfer_with_enrichment(
    input: TransferComputationInput,
    safety_requirements: Option<SafetyRequirements>,
) -> Result<ComputeTransferOutput, WasmError> {
    let (transfer, enrichment) = rpo_core::pipeline::compute_transfer_with_enrichment(
        &input,
        safety_requirements.as_ref(),
    )
    .map_err(WasmError::from)?;
    Ok(ComputeTransferOutput {
        transfer,
        enrichment,
    })
}

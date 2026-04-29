//! WASM binding for the full transfer-computation pipeline:
//! classify → Lambert → perch geometry → arc densification (+ optional enrichment).

use serde::Serialize;
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::mission::formation::{EnrichmentSuggestion, SafetyRequirements};
use rpo_core::pipeline::types::{TransferComputationInput, TransferResult};

use crate::error::WasmError;

/// Combined transfer + enrichment result.
///
/// `enrichment` is always present: when `safety_requirements` is omitted it
/// is [`EnrichmentSuggestion::Baseline`] carrying the geometric perch ROE;
/// when supplied, the engine returns [`EnrichmentSuggestion::Enriched`] on
/// success or surfaces a [`crate::error::WasmErrorCode::Formation`] error
/// on failure.
#[derive(Debug, Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct ComputeTransferOutput {
    /// Transfer solution (classification + optional Lambert + perch states).
    pub transfer: TransferResult,
    /// Enrichment outcome (Baseline if no requirements, Enriched on success).
    pub enrichment: EnrichmentSuggestion,
}

/// Compute a transfer and its perch enrichment outcome.
///
/// Thin pass-through to [`rpo_core::pipeline::compute_transfer_with_enrichment`];
/// the orchestration lives in `rpo-core` so CLI and tests can exercise the
/// same code path.
///
/// # Errors
///
/// Returns [`WasmError`] on classification, Lambert, propagation,
/// arc-densification, or perch-enrichment failure.
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

//! Shared helpers for API handlers.

use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::pipeline::{resolve_propagator, to_propagation_model, PipelineInput, TransferResult};
use rpo_core::propagation::{extract_dmf_rates, DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;

use crate::error::ApiError;

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via nyx DMF.
/// Returns the propagation model and optional derived drag config.
///
/// # Errors
///
/// Returns [`ApiError`] if drag extraction fails.
pub fn resolve_drag_and_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    input: &PipelineInput,
) -> Result<(PropagationModel, Option<DragConfig>), ApiError> {
    let auto_drag_config = if auto_drag {
        Some(extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            chief_config,
            deputy_config,
            almanac,
        )?)
    } else {
        None
    };
    Ok(resolve_propagator(
        auto_drag_config,
        to_propagation_model(&input.propagator),
    ))
}

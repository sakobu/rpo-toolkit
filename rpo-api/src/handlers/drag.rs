//! Drag extraction handler — blocking, ~3 seconds.

use crate::error::ServerError;
use anise::prelude::Almanac;
use rpo_core::propagation::propagator::DragConfig;
use rpo_core::types::spacecraft::SpacecraftConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::nyx_bridge::extract_dmf_rates;
use std::sync::Arc;

/// Handle an `ExtractDrag` message.
///
/// Short-circuits to `DragConfig::zero()` when spacecraft configs are identical
/// (no differential drag). Otherwise delegates to nyx full-physics propagation.
///
/// # Errors
///
/// Returns [`ServerError::NyxBridge`] if nyx propagation fails (almanac load,
/// dynamics setup, or propagation error).
pub fn handle_extract_drag(
    chief: &StateVector,
    deputy: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<DragConfig, ServerError> {
    if chief_config == deputy_config {
        return Ok(DragConfig::zero());
    }
    Ok(extract_dmf_rates(chief, deputy, chief_config, deputy_config, almanac)?)
}

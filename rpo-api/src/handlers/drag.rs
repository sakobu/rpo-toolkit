//! Drag extraction handler — blocking, ~3 seconds.

use crate::error::ServerError;
use anise::prelude::Almanac;
use rpo_core::propagation::propagator::DragConfig;
use rpo_core::types::spacecraft::SpacecraftConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::nyx_bridge::{NyxBridgeError, extract_dmf_rates_with_cancel};
use std::sync::atomic::{AtomicBool, Ordering};
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
    chief_eci: &StateVector,
    deputy_eci: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    cancel: &AtomicBool,
) -> Result<DragConfig, ServerError> {
    if cancel.load(Ordering::Relaxed) {
        return Err(ServerError::Cancelled);
    }

    if chief_config.bit_eq(deputy_config) {
        return Ok(DragConfig::zero());
    }

    extract_dmf_rates_with_cancel(chief_eci, deputy_eci, chief_config, deputy_config, almanac, cancel)
        .map_err(|e| match e {
            NyxBridgeError::Cancelled => ServerError::Cancelled,
            other => ServerError::from(other),
        })
}

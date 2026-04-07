//! Drag extraction handler: derive differential drag rates via nyx.
//!
//! Runs nyx full-physics dynamics for ~3 seconds. Designed to be called
//! from `spawn_blocking`.

use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::propagation::DragConfig;
use rpo_nyx::nyx_bridge::extract_dmf_rates;
use rpo_core::types::{SpacecraftConfig, StateVector};

use crate::error::ApiError;

/// Extract differential drag rates from spacecraft properties via nyx DMF estimation.
///
/// When chief and deputy spacecraft configs are identical (by `PartialEq`),
/// returns `DragConfig::zero()` without running nyx — there is no differential
/// drag between identical spacecraft.
///
/// Takes ~3 seconds (nyx short simulation) when configs differ.
///
/// # Errors
/// Returns [`ApiError`] if DMF extraction fails.
pub fn handle_extract_drag(
    chief: &StateVector,
    deputy: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<DragConfig, ApiError> {
    // Identical configs produce zero differential drag — skip nyx.
    if chief_config == deputy_config {
        return Ok(DragConfig::zero());
    }

    let drag = extract_dmf_rates(chief, deputy, chief_config, deputy_config, almanac)?;
    Ok(drag)
}

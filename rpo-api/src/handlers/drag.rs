//! Drag extraction handler: derive differential drag rates via nyx.
//!
//! Runs nyx full-physics dynamics for ~3 seconds. Designed to be called
//! from `spawn_blocking`.

use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::propagation::{extract_dmf_rates, DragConfig};

use crate::error::{require_field, ApiError};
use crate::protocol::MissionDefinition;

/// Extract differential drag rates from spacecraft properties via nyx DMF estimation.
///
/// Requires `chief_config` and `deputy_config` in the mission definition.
/// Takes ~3 seconds (nyx short simulation).
///
/// # Errors
/// Returns [`ApiError`] if spacecraft configs are missing or DMF extraction fails.
pub fn handle_extract_drag(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
) -> Result<DragConfig, ApiError> {
    let chief_config = require_field(def.chief_config, "chief_config", "drag extraction")?;
    let deputy_config = require_field(def.deputy_config, "deputy_config", "drag extraction")?;

    let drag = extract_dmf_rates(
        &def.chief,
        &def.deputy,
        &chief_config,
        &deputy_config,
        almanac,
    )?;
    Ok(drag)
}

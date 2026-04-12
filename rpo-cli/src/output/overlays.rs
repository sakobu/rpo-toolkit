//! CLI overlay flags and propagator resolution.

use std::sync::Arc;

use anise::prelude::Almanac;
use indicatif::ProgressBar;

use rpo_core::mission::{ColaConfig, EiAlignment, SafetyRequirements};
use rpo_core::pipeline::{
    resolve_propagator, to_propagation_model, PipelineInput, TransferResult,
};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;
use rpo_nyx::nyx_bridge::extract_dmf_rates;

use crate::error::CliError;

use super::io::status;

/// Default delta-v budget for CLI COLA overlay (km/s).
/// 10 m/s — matches the pipeline auto-COLA default in `execute.rs`.
const DEFAULT_COLA_BUDGET_KM_S: f64 = 0.01;

/// Default formation design enrichment threshold (km).
/// 100 m — a reasonable passive safety margin for close proximity operations.
const DEFAULT_ENRICHMENT_THRESHOLD_KM: f64 = 0.1;

/// CLI overlay flags that mutate a [`PipelineInput`] before execution.
///
/// Groups COLA and formation design enrichment flags to avoid
/// long argument lists in porcelain command `run()` functions.
#[derive(Debug, Clone, Default)]
pub struct OverlayFlags {
    /// Target miss distance for collision avoidance (km).
    pub cola_threshold: Option<f64>,
    /// Maximum delta-v budget for COLA (km/s).
    pub cola_budget: Option<f64>,
    /// Enable formation design enrichment.
    pub auto_enrich: bool,
    /// Custom min separation threshold (km) for formation design.
    pub auto_enrich_threshold: Option<f64>,
}

/// Apply all CLI overlay flags onto a [`PipelineInput`].
///
/// Handles COLA configuration and formation design enrichment in one call.
pub fn apply_overlays(input: &mut PipelineInput, flags: &OverlayFlags) {
    // COLA overlay
    if let Some(threshold) = flags.cola_threshold {
        input.base.cola = Some(ColaConfig {
            target_distance_km: threshold,
            max_dv_km_s: flags.cola_budget.unwrap_or(DEFAULT_COLA_BUDGET_KM_S),
        });
    }

    // Formation design enrichment overlay
    let separation_km = match (flags.auto_enrich_threshold, flags.auto_enrich) {
        (Some(t), _) => Some(t),
        (None, true) => Some(DEFAULT_ENRICHMENT_THRESHOLD_KM),
        (None, false) => None,
    };
    if let Some(min_separation_km) = separation_km {
        input.base.safety_requirements = Some(SafetyRequirements {
            min_separation_km,
            alignment: EiAlignment::default(),
        });
    }
}

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via Nyx DMF
/// and prints them to stderr. Returns the propagation model and optional
/// derived drag config.
///
/// # Errors
///
/// Returns [`CliError`] if drag extraction or propagator resolution fails.
pub fn resolve_drag_and_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    input: &PipelineInput,
    spinner: Option<&ProgressBar>,
) -> Result<(PropagationModel, Option<DragConfig>), CliError> {
    let auto_drag_config = if auto_drag {
        status!(spinner, "Extracting differential drag rates via Nyx...");
        let drag = extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            chief_config,
            deputy_config,
            almanac,
        )?;
        eprintln!(
            "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
        Some(drag)
    } else {
        None
    };
    Ok(resolve_propagator(
        auto_drag_config,
        to_propagation_model(&input.base.propagator),
    ))
}

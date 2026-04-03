//! Shared orchestration for porcelain commands that need full-physics support.

use std::sync::Arc;

use anise::prelude::Almanac;
use indicatif::ProgressBar;

use rpo_core::mission::WaypointMission;
use rpo_core::pipeline::{
    apply_perch_enrichment, compute_transfer, plan_waypoints_from_transfer, suggest_enrichment,
    EnrichmentSuggestion, PipelineInput, TransferResult,
};
use rpo_core::propagation::{load_full_almanac, DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;

use crate::error::CliError;
use crate::output::common::{create_spinner, resolve_drag_and_propagator, status};

/// Result of the shared transfer → waypoint planning orchestration.
///
/// Groups everything that validate and MC commands need from the common
/// pipeline prefix: transfer result, planned mission, propagation model,
/// optional drag config, enrichment suggestion, almanac, and spinner.
pub struct PlannedMission {
    /// Transfer result (Lambert + perch handoff).
    pub transfer: TransferResult,
    /// Waypoint mission plan.
    pub wp_mission: WaypointMission,
    /// Propagation model used for waypoint targeting.
    pub propagator: PropagationModel,
    /// Auto-derived drag configuration, if `auto_drag` was set.
    pub derived_drag: Option<DragConfig>,
    /// Enrichment suggestion applied to the perch, if any.
    pub suggestion: Option<EnrichmentSuggestion>,
    /// ANISE almanac (shared across downstream operations).
    pub almanac: Arc<Almanac>,
    /// CLI spinner for status messages. `None` if stderr is not a terminal.
    pub spinner: Option<ProgressBar>,
}

/// Run the shared pipeline prefix: transfer → enrich → almanac → drag → waypoints.
///
/// This extracts the identical orchestration logic shared by `validate` and `mc`
/// commands. Both commands need full-physics support (almanac, optional drag),
/// which `mission` does not.
///
/// # Errors
///
/// Returns [`CliError`] if any pipeline step fails.
pub fn plan_with_physics(
    input: &PipelineInput,
    auto_drag: bool,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
) -> Result<PlannedMission, CliError> {
    let spinner = create_spinner();

    status!(spinner, "Classification + Lambert transfer...");
    let mut transfer = compute_transfer(input)?;
    let suggestion = suggest_enrichment(&transfer, input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(&mut transfer, s);
    }

    status!(spinner, "Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()?;

    let (propagator, derived_drag) = resolve_drag_and_propagator(
        auto_drag,
        &transfer,
        chief_config,
        deputy_config,
        &almanac,
        input,
        spinner.as_ref(),
    )?;

    status!(spinner, "Waypoint targeting...");
    let wp_mission = plan_waypoints_from_transfer(&transfer, input, &propagator)?;

    Ok(PlannedMission {
        transfer,
        wp_mission,
        propagator,
        derived_drag,
        suggestion,
        almanac,
        spinner,
    })
}

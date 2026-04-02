//! Porcelain: mission + Nyx high-fidelity validation.

use std::path::Path;

use rpo_core::mission::{validate_mission_nyx, ValidationConfig, ValidationReport};
use rpo_core::pipeline::{
    apply_perch_enrichment, compute_transfer, plan_waypoints_from_transfer, suggest_enrichment,
    PipelineInput, PipelineOutput,
};
use rpo_core::propagation::load_full_almanac;

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{
    apply_overlays, create_spinner, print_insights, resolve_drag_and_propagator, status,
    write_json_report, write_report, OverlayFlags, SafetyTier, ValidationTier,
};
use crate::output::markdown_fmt::{self, ValidationContext};
use crate::output::mission_fmt::{
    print_mission_human, print_mission_verdict, print_validation_details,
};

/// Combined mission + validation output for JSON serialization.
///
/// Borrows both structs to avoid the deep clone that `serde_json::json!` would perform.
#[derive(serde::Serialize)]
struct ValidateReport<'a> {
    mission: &'a PipelineOutput,
    validation: &'a ValidationReport,
}

/// Run mission + nyx validation pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    samples_per_leg: u32,
    auto_drag: bool,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path)?;
    apply_overlays(&mut input, flags);

    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    let spinner = create_spinner(mode.suppress_interactive());

    status!(spinner, "Classification + Lambert transfer...");
    let mut transfer = compute_transfer(&input)?;
    let suggestion = suggest_enrichment(&transfer, &input);
    if let Some(ref s) = suggestion {
        apply_perch_enrichment(&mut transfer, s);
    }

    status!(spinner, "Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()?;

    let (prop, derived_drag) = resolve_drag_and_propagator(
        auto_drag, &transfer, &chief_config, &deputy_config, &almanac, &input, spinner.as_ref(),
    )?;

    status!(spinner, "Waypoint targeting...");
    let wp_mission = plan_waypoints_from_transfer(&transfer, &input, &prop)?;

    status!(
        spinner,
        "Nyx validation ({samples_per_leg} samples/leg)..."
    );
    let val_config = ValidationConfig {
        samples_per_leg,
        chief_config,
        deputy_config,
    };
    let report = validate_mission_nyx(
        &wp_mission,
        &transfer.perch_chief,
        &transfer.perch_deputy,
        &val_config,
        &almanac,
    )?;

    if let Some(s) = spinner {
        s.finish_and_clear();
    }
    eprintln!("Validation complete.");

    let output = rpo_core::pipeline::build_output(
        &transfer,
        wp_mission,
        &input,
        &prop,
        derived_drag,
        suggestion,
    );

    match mode {
        OutputMode::Json => {
            let combined = ValidateReport {
                mission: &output,
                validation: &report,
            };
            write_json_report("validate", &combined)
        }
        OutputMode::Markdown => {
            let ctx = ValidationContext {
                propagator: &prop,
                auto_drag,
                samples_per_leg,
                derived_drag: derived_drag.as_ref(),
            };
            let md = markdown_fmt::validation_to_markdown(
                &output,
                &input,
                &report,
                &ctx,
            );
            write_report("validate", &md)
        }
        OutputMode::Human => {
            print_mission_human(&output, &input, &prop, auto_drag, "Mission + Validation", SafetyTier::Baseline(ValidationTier::Nyx));
            print_validation_details(
                &output,
                &input,
                &report,
                samples_per_leg,
                derived_drag.as_ref(),
            );

            let sc = input.config.safety.unwrap_or_default();
            let insight_lines =
                crate::output::insights::validation_insights(&report, &sc);
            print_insights(&insight_lines);

            print_mission_verdict(&output, &input, Some(&report));
            Ok(())
        }
    }
}

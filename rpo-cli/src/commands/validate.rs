//! Porcelain: mission + Nyx high-fidelity validation.

use std::path::Path;

use rpo_core::mission::validate_mission_nyx;
use rpo_core::pipeline::{compute_transfer, plan_waypoints_from_transfer, PipelineInput};
use rpo_core::propagation::load_full_almanac;

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{
    create_spinner, print_json, resolve_drag_and_propagator, status,
};
use crate::output::mission_fmt::{
    print_mission_human, print_mission_verdict, print_validation_details,
};

/// Run mission + nyx validation pipeline.
pub fn run(
    input_path: &Path,
    json: bool,
    samples_per_leg: u32,
    auto_drag: bool,
) -> Result<(), CliError> {
    let input: PipelineInput = load_json(input_path)?;

    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    let spinner = create_spinner(json);

    status!(spinner, "Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

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
    let report = validate_mission_nyx(
        &wp_mission,
        &transfer.perch_chief,
        &transfer.perch_deputy,
        samples_per_leg,
        &chief_config,
        &deputy_config,
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
    );

    if json {
        let combined = serde_json::json!({
            "mission": output,
            "validation": report,
        });
        return print_json(&combined);
    }

    print_mission_human(&output, &input, &prop, auto_drag, "Mission + Validation");
    print_validation_details(&output, &input, &report, samples_per_leg, derived_drag.as_ref());
    print_mission_verdict(&output, &input, Some(&report));
    Ok(())
}

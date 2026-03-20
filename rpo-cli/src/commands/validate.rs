//! Porcelain: mission + nyx high-fidelity validation.

use std::path::Path;

use rpo_core::mission::validate_mission_nyx;
use rpo_core::pipeline::{
    compute_transfer, plan_waypoints_from_transfer, resolve_propagator, to_propagation_model,
    PipelineInput,
};
use rpo_core::propagation::{extract_dmf_rates, load_full_almanac};

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;
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

    let chief_config = input.chief_config.unwrap_or_default();
    let deputy_config = input.deputy_config.unwrap_or_default();

    eprintln!("Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()?;

    eprintln!("Phase 1: Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

    // Resolve propagator (with optional auto-drag)
    let auto_drag_config = if auto_drag {
        eprintln!("Extracting differential drag rates via nyx...");
        let drag = extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            &chief_config,
            &deputy_config,
            &almanac,
        )?;
        eprintln!(
            "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
        Some(drag)
    } else {
        None
    };
    let (prop, derived_drag) =
        resolve_propagator(auto_drag_config, to_propagation_model(&input.propagator));

    eprintln!("Phase 2: Waypoint targeting...");
    let wp_mission = plan_waypoints_from_transfer(&transfer, &input, &prop)?;

    // Phase 3: Nyx validation
    eprintln!(
        "Phase 3: Nyx validation ({samples_per_leg} samples/leg)..."
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

    print_mission_human(&output, &input, &prop, auto_drag);
    print_validation_details(&output, &input, &report, samples_per_leg, derived_drag.as_ref());
    print_mission_verdict(&output, &input, Some(&report));
    Ok(())
}

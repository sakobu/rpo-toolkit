//! Porcelain: full-physics Monte Carlo ensemble analysis.

use std::path::Path;

use rpo_core::mission::{run_monte_carlo, MonteCarloInput};
use rpo_core::pipeline::{
    compute_mission_covariance, compute_transfer, plan_waypoints_from_transfer,
    resolve_propagator, to_propagation_model, PipelineInput,
};
use rpo_core::propagation::{extract_dmf_rates, load_full_almanac};

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{print_derived_drag, print_json};
use crate::output::mc_fmt::print_mc_report;

/// Run full-physics Monte Carlo pipeline.
#[allow(clippy::too_many_lines)]
pub fn run(input_path: &Path, json: bool, auto_drag: bool) -> Result<(), CliError> {
    let input: PipelineInput = load_json(input_path)?;

    let chief_config = input
        .chief_config
        .ok_or(CliError::MissingField { field: "chief_config", context: "mc" })?;
    let deputy_config = input
        .deputy_config
        .ok_or(CliError::MissingField { field: "deputy_config", context: "mc" })?;
    let mc_config = input
        .monte_carlo
        .as_ref()
        .ok_or(CliError::MissingField { field: "monte_carlo", context: "mc" })?;

    eprintln!("Phase 1: Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

    eprintln!("Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()?;

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

    // Optional covariance propagation
    let covariance_report = if let Some(ref nav) = input.navigation_accuracy {
        eprintln!("Running covariance propagation...");
        Some(compute_mission_covariance(
            &wp_mission,
            &transfer.plan.chief_at_arrival,
            nav,
            input.maneuver_uncertainty.as_ref(),
            &prop,
        )?)
    } else {
        None
    };

    // Run Monte Carlo
    eprintln!(
        "Phase 4: Running Monte Carlo ({} samples, {:?} mode)...",
        mc_config.num_samples, mc_config.mode,
    );

    let mc_input = MonteCarloInput {
        nominal_mission: &wp_mission,
        initial_chief: &transfer.perch_chief,
        initial_deputy: &transfer.perch_deputy,
        config: mc_config,
        mission_config: &input.config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &prop,
        almanac: &almanac,
        covariance_report: covariance_report.as_ref(),
        control: None,
    };

    let report = run_monte_carlo(&mc_input)?;
    eprintln!(
        "Monte Carlo complete ({:.1}s wall time).",
        report.elapsed_wall_s
    );

    if json {
        return print_json(&report);
    }

    print_mc_report(&report, transfer.lambert_dv_km_s);

    if let Some(ref drag) = derived_drag {
        print_derived_drag(drag);
    }

    Ok(())
}

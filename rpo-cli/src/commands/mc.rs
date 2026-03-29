//! Porcelain: full-physics Monte Carlo ensemble analysis.

use std::path::Path;

use rpo_core::mission::{run_monte_carlo, MissionPhase, MonteCarloInput};
use rpo_core::pipeline::{
    build_output, compute_mission_covariance, compute_transfer, plan_waypoints_from_transfer,
    PipelineInput, PipelineOutput,
};
use rpo_core::propagation::{load_full_almanac, DragConfig};

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{
    apply_cola_overlay, create_spinner, print_insights, resolve_drag_and_propagator, status,
    write_json_report, write_report, McBaseline, SafetyTier, ValidationTier,
};
use crate::output::markdown_fmt;
use crate::output::mc_fmt::{print_mc_report, print_mc_summary};
use crate::output::mission_fmt::print_mission_human;

/// Run full-physics Monte Carlo pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    auto_drag: bool,
    cola_threshold: Option<f64>,
    cola_budget: Option<f64>,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path)?;
    apply_cola_overlay(&mut input, cola_threshold, cola_budget);

    let chief_config = input
        .chief_config
        .ok_or(CliError::MissingField { field: "chief_config", context: "mc" })?
        .resolve();
    let deputy_config = input
        .deputy_config
        .ok_or(CliError::MissingField { field: "deputy_config", context: "mc" })?
        .resolve();
    let mc_config = input
        .monte_carlo
        .as_ref()
        .ok_or(CliError::MissingField { field: "monte_carlo", context: "mc" })?;

    let spinner = create_spinner(mode.suppress_interactive());

    status!(spinner, "Classification + Lambert transfer...");
    let transfer = compute_transfer(&input)?;

    status!(spinner, "Loading almanac (may download on first run)...");
    let almanac = load_full_almanac()?;

    let (prop, derived_drag) = resolve_drag_and_propagator(
        auto_drag, &transfer, &chief_config, &deputy_config, &almanac, &input, spinner.as_ref(),
    )?;

    status!(spinner, "Waypoint targeting...");
    let wp_mission = plan_waypoints_from_transfer(&transfer, &input, &prop)?;

    // Capture values before borrowing wp_mission into MonteCarloInput
    let baseline = McBaseline {
        lambert_dv_km_s: transfer.lambert_dv_km_s,
        lambert_tof_s: transfer.plan.transfer.as_ref().map_or(0.0, |t| t.tof_s),
        waypoint_dv_km_s: wp_mission.total_dv_km_s,
        waypoint_duration_s: wp_mission.total_duration_s,
        num_legs: wp_mission.legs.len(),
        is_far_field: matches!(transfer.plan.phase, MissionPhase::FarField { .. }),
    };

    // Optional covariance propagation
    let covariance_report = if let Some(ref nav) = input.navigation_accuracy {
        status!(spinner, "Running covariance propagation...");
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
    status!(
        spinner,
        "Running Monte Carlo ({} samples, {} mode)...",
        mc_config.num_samples, mc_config.mode
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

    if let Some(s) = spinner {
        s.finish_and_clear();
    }
    eprintln!(
        "Monte Carlo complete ({:.1}s wall time).",
        report.elapsed_wall_s
    );

    // Build canonical output with nominal safety context (free-drift, POCA).
    let mut output = build_output(&transfer, wp_mission, &input, &prop, derived_drag);
    output.monte_carlo = Some(report);

    print_mc_output(mode, &output, &input, &baseline, &prop, derived_drag.as_ref())
}

/// Format and print MC results in the requested output mode.
fn print_mc_output(
    mode: OutputMode,
    output: &PipelineOutput,
    input: &PipelineInput,
    baseline: &McBaseline,
    propagator: &rpo_core::propagation::PropagationModel,
    derived_drag: Option<&DragConfig>,
) -> Result<(), CliError> {
    let report = output
        .monte_carlo
        .as_ref()
        .expect("monte_carlo field must be set before print_mc_output");

    match mode {
        OutputMode::Json => write_json_report("mc", output),
        OutputMode::Markdown => {
            let md = markdown_fmt::mc_to_markdown(output, input, baseline, derived_drag);
            write_report("mc", &md)
        }
        OutputMode::Human => {
            // Nominal mission context (Transfer, Waypoints, Safety, POCA, Free-Drift)
            print_mission_human(
                output,
                input,
                propagator,
                derived_drag.is_some(),
                "Mission + Monte Carlo",
                SafetyTier::Baseline(ValidationTier::MonteCarlo),
            );

            // MC ensemble statistics
            print_mc_report(report, baseline.lambert_dv_km_s, derived_drag);

            let safety_config = input.config.safety.unwrap_or_default();
            let insight_lines =
                crate::output::insights::mc_insights(report, &safety_config);
            print_insights(&insight_lines);

            print_mc_summary(report, baseline.lambert_dv_km_s, &safety_config, derived_drag.is_some());

            Ok(())
        }
    }
}

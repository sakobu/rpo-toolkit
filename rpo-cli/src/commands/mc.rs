//! Porcelain: full-physics Monte Carlo ensemble analysis.

use std::path::Path;

use rpo_core::mission::{run_monte_carlo, MissionPhase, MonteCarloInput};
use rpo_core::pipeline::{build_output, compute_mission_covariance, PipelineInput};

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{
    apply_overlays, output_json, output_text, status, McBaseline, OverlayFlags,
};
use crate::output::markdown_fmt;

use super::common::plan_with_physics;

/// Run full-physics Monte Carlo pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    output: Option<&Path>,
    auto_drag: bool,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path)?;
    apply_overlays(&mut input, flags);

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

    let plan = plan_with_physics(&input, auto_drag, &chief_config, &deputy_config)?;

    // Capture baseline values before borrowing wp_mission into MonteCarloInput
    let baseline = McBaseline {
        lambert_dv_km_s: plan.transfer.lambert_dv_km_s,
        lambert_tof_s: plan.transfer.plan.transfer.as_ref().map_or(0.0, |t| t.tof_s),
        waypoint_dv_km_s: plan.wp_mission.total_dv_km_s,
        waypoint_duration_s: plan.wp_mission.total_duration_s,
        num_legs: plan.wp_mission.legs.len(),
        is_far_field: matches!(plan.transfer.plan.phase, MissionPhase::FarField { .. }),
    };

    // Optional covariance propagation
    let covariance_report = if let Some(ref nav) = input.navigation_accuracy {
        status!(plan.spinner, "Running covariance propagation...");
        Some(compute_mission_covariance(
            &plan.wp_mission,
            &plan.transfer.plan.chief_at_arrival,
            nav,
            input.maneuver_uncertainty.as_ref(),
            &plan.propagator,
        )?)
    } else {
        None
    };

    // Run Monte Carlo
    status!(
        plan.spinner,
        "Running Monte Carlo ({} samples, {} mode)...",
        mc_config.num_samples, mc_config.mode
    );

    let mc_input = MonteCarloInput {
        nominal_mission: &plan.wp_mission,
        initial_chief: &plan.transfer.perch_chief,
        initial_deputy: &plan.transfer.perch_deputy,
        config: mc_config,
        mission_config: &input.config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &plan.propagator,
        almanac: &plan.almanac,
        covariance_report: covariance_report.as_ref(),
        control: None,
    };

    let report = run_monte_carlo(&mc_input)?;

    if let Some(s) = plan.spinner {
        s.finish_and_clear();
    }
    eprintln!(
        "Monte Carlo complete ({:.1}s wall time).",
        report.elapsed_wall_s
    );

    // Build canonical output with nominal safety context (free-drift, POCA).
    let mut result = build_output(
        &plan.transfer,
        plan.wp_mission,
        &input,
        &plan.propagator,
        plan.derived_drag,
        plan.suggestion,
    );

    match mode {
        OutputMode::Json => {
            result.monte_carlo = Some(report);
            output_json(&result, output)
        }
        OutputMode::Summary => {
            let md = markdown_fmt::mc_to_markdown(
                &result,
                &input,
                &report,
                &baseline,
                plan.derived_drag.as_ref(),
            );
            output_text(&md, output)
        }
    }
}

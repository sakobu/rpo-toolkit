//! Porcelain: full-physics Monte Carlo ensemble analysis.

use std::path::Path;

use rpo_core::mission::MissionPhase;
use rpo_core::pipeline::{build_output, compute_mission_covariance, compute_safety_analysis, PipelineInput};
use rpo_nyx::monte_carlo::{run_monte_carlo, MonteCarloInput};

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

    // Resolve MC dispersions: derive from top-level nav/maneuver if not explicitly set
    let resolved_mc = mc_config.with_resolved_dispersions(
        input.navigation_accuracy.as_ref(),
        input.maneuver_uncertainty.as_ref(),
    );

    // Run Monte Carlo
    status!(
        plan.spinner,
        "Running Monte Carlo ({} samples, {} mode)...",
        resolved_mc.num_samples, resolved_mc.mode
    );

    let mc_input = MonteCarloInput {
        nominal_mission: &plan.wp_mission,
        initial_chief: &plan.transfer.perch_chief,
        initial_deputy: &plan.transfer.perch_deputy,
        config: &resolved_mc,
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
    let safety = compute_safety_analysis(
        &plan.wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &plan.propagator,
    );
    let mut result = build_output(
        rpo_core::pipeline::BuildOutputCtx {
            transfer: &plan.transfer,
            input: &input,
            propagator: &plan.propagator,
            auto_drag: plan.derived_drag,
            suggestion: plan.suggestion,
            safety,
            precomputed_covariance: covariance_report,
        },
        plan.wp_mission,
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

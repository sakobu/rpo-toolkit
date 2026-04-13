//! Porcelain: mission + Nyx high-fidelity validation.

use std::path::Path;

use rpo_core::mission::ValidationReport;
use rpo_core::pipeline::PipelineOutput;
use rpo_nyx::nyx_bridge::build_full_physics_dynamics;
use rpo_nyx::pipeline::compute_validation_burns;
use rpo_nyx::validation::{
    validate_mission_nyx, ColaValidationInput, ValidationConfig, ValidationPipelineCtx,
};

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::io::{output_json, output_text, status};
use crate::output::overlays::{apply_overlays, OverlayFlags};
use crate::output::report::{self, ValidationContext};

use super::common::plan_with_physics;

/// Combined mission + validation output for JSON serialization.
///
/// Borrows both structs to avoid the deep clone that `serde_json::json!` would perform.
#[derive(serde::Serialize)]
struct ValidateReport<'a> {
    /// Analytical mission pipeline output.
    mission: &'a PipelineOutput,
    /// Nyx full-physics validation results.
    validation: &'a ValidationReport,
}

/// Run mission + nyx validation pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    output: Option<&Path>,
    samples_per_leg: u32,
    auto_drag: bool,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input = load_json(input_path, None)?;
    apply_overlays(&mut input, flags);

    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    let plan = plan_with_physics(&input, auto_drag, &chief_config, &deputy_config)?;

    // Compute safety analysis and derive COLA burns for nyx injection.
    let (safety, cola_burns) = compute_validation_burns(
        &plan.wp_mission, input.base.config.safety.as_ref(), input.base.cola.as_ref(), &plan.propagator,
    )?;

    status!(
        plan.spinner,
        "Nyx validation ({samples_per_leg} samples/leg)..."
    );
    let val_config = ValidationConfig {
        samples_per_leg,
        chief_config,
        deputy_config,
    };
    let cola_input = ColaValidationInput {
        burns: cola_burns,
        analytical_maneuvers: safety.cola.clone().unwrap_or_default(),
        target_distance_km: input.base.cola.as_ref().map(|c| c.target_distance_km),
    };

    let dynamics = build_full_physics_dynamics(&plan.almanac)?;
    let pipeline = ValidationPipelineCtx {
        mission: &plan.wp_mission,
        chief_initial: &plan.transfer.perch_chief,
        deputy_initial: &plan.transfer.perch_deputy,
        config: &val_config,
        cola: &cola_input,
        almanac: &plan.almanac,
        dynamics: &dynamics,
    };
    let report = validate_mission_nyx(&pipeline)?;

    if let Some(s) = plan.spinner {
        s.finish_and_clear();
    }
    eprintln!("Validation complete.");

    let result = rpo_core::pipeline::build_output(
        rpo_core::pipeline::BuildOutputCtx {
            transfer: &plan.transfer,
            input: &input.base,
            propagator: &plan.propagator,
            auto_drag: plan.derived_drag,
            suggestion: plan.suggestion,
            safety,
            precomputed_covariance: None,
        },
        plan.wp_mission,
    );

    match mode {
        OutputMode::Json => {
            let combined = ValidateReport {
                mission: &result,
                validation: &report,
            };
            output_json(&combined, output)
        }
        OutputMode::Summary => {
            let ctx = ValidationContext {
                auto_drag,
                samples_per_leg,
            };
            let md = report::validation_to_markdown(&result, &input, &report, &ctx);
            output_text(&md, output)
        }
    }
}

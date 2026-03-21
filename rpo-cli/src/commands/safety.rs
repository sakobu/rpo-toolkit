//! Plumbing: safety analysis for a planned mission (JSON only).

use std::path::Path;

use rpo_core::mission::assess_safety;
use rpo_core::pipeline::{execute_mission, PipelineInput};

use crate::error::CliError;
use crate::input::load_json_with_hint;
use crate::output::common::print_json;

/// Run safety analysis on a planned mission and print JSON.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: PipelineInput = load_json_with_hint(
        input_path,
        "safety expects full PipelineInput (same as mission); see examples/mission.json",
    )?;
    let output = execute_mission(&input)?;

    let safety_config = input.config.safety.unwrap_or_default();
    let result = output.mission.safety.as_ref().map(|s| {
        let assessment = assess_safety(s, &safety_config);
        serde_json::json!({
            "safety": s,
            "assessment": {
                "overall_pass": assessment.overall_pass,
                "distance_3d_pass": assessment.distance_3d_pass,
                "ei_separation_pass": assessment.ei_separation_pass,
            },
        })
    });

    print_json(&result)
}

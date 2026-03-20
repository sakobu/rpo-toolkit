//! Plumbing: eclipse computation for a planned mission (JSON only).

use std::path::Path;

use rpo_core::pipeline::{execute_mission, PipelineInput};

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;

/// Compute eclipse data for a planned mission and print JSON.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: PipelineInput = load_json(input_path)?;
    let output = execute_mission(&input)?;

    let eclipse_data = serde_json::json!({
        "transfer_eclipse": output.transfer_eclipse,
        "mission_eclipse": output.mission.eclipse,
    });

    print_json(&eclipse_data)
}

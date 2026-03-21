//! Porcelain: end-to-end mission command.

use std::path::Path;

use rpo_core::pipeline::{execute_mission, to_propagation_model, PipelineInput};

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;
use crate::output::mission_fmt::{print_mission_human, print_mission_verdict};

/// Run the end-to-end mission pipeline.
pub fn run(input_path: &Path, json: bool) -> Result<(), CliError> {
    let input: PipelineInput = load_json(input_path)?;
    let output = execute_mission(&input)?;

    if json {
        return print_json(&output);
    }

    let propagator = to_propagation_model(&input.propagator);
    print_mission_human(&output, &input, &propagator, false, "Mission");
    print_mission_verdict(&output, &input, None);
    Ok(())
}

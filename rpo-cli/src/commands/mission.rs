//! Porcelain: end-to-end mission command.

use std::path::Path;

use rpo_core::pipeline::{execute_mission, to_propagation_model, PipelineInput};

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{apply_overlays, write_json_report, write_report, OverlayFlags, SafetyTier};
use crate::output::markdown_fmt;
use crate::output::mission_fmt::{print_mission_human, print_mission_verdict};

/// Run the end-to-end mission pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path)?;
    apply_overlays(&mut input, flags);

    let output = execute_mission(&input)?;

    match mode {
        OutputMode::Json => write_json_report("mission", &output),
        OutputMode::Markdown => {
            let propagator = to_propagation_model(&input.propagator);
            let md = markdown_fmt::mission_to_markdown(&output, &input, &propagator, false);
            write_report("mission", &md)
        }
        OutputMode::Human => {
            let propagator = to_propagation_model(&input.propagator);
            print_mission_human(&output, &input, &propagator, false, "Mission", SafetyTier::Governing);
            print_mission_verdict(&output, &input, None);
            Ok(())
        }
    }
}

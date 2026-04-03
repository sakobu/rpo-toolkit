//! Porcelain: end-to-end mission command.

use std::path::Path;

use rpo_core::pipeline::{execute_mission, to_propagation_model, PipelineInput};

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::{apply_overlays, output_json, output_text, OverlayFlags};
use crate::output::markdown_fmt;

/// Run the end-to-end mission pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    output: Option<&Path>,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path)?;
    apply_overlays(&mut input, flags);

    let result = execute_mission(&input)?;

    match mode {
        OutputMode::Json => output_json(&result, output),
        OutputMode::Summary => {
            let propagator = to_propagation_model(&input.propagator);
            let md = markdown_fmt::mission_to_markdown(&result, &input, &propagator, false);
            output_text(&md, output)
        }
    }
}

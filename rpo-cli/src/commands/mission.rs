//! Porcelain: end-to-end mission command.

use std::path::Path;

use rpo_core::pipeline::PipelineInput;
use rpo_nyx::pipeline::execute_mission;

use crate::cli::OutputMode;
use crate::error::CliError;
use crate::input::load_json;
use crate::output::io::{output_json, output_text};
use crate::output::overlays::{apply_overlays, OverlayFlags};
use crate::output::report;

/// Run the end-to-end mission pipeline.
pub fn run(
    input_path: &Path,
    mode: OutputMode,
    output: Option<&Path>,
    flags: &OverlayFlags,
) -> Result<(), CliError> {
    let mut input: PipelineInput = load_json(input_path, None)?;
    apply_overlays(&mut input, flags);

    let result = execute_mission(&input)?;

    match mode {
        OutputMode::Json => output_json(&result, output),
        OutputMode::Summary => {
            let md = report::mission_to_markdown(&result, &input, false);
            output_text(&md, output)
        }
    }
}

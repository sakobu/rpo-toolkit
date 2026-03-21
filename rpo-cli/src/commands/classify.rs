//! Plumbing: classify chief/deputy separation (JSON only).

use std::path::Path;

use serde::Deserialize;

use rpo_core::mission::{classify_separation, ProximityConfig};
use rpo_core::types::StateVector;

use crate::error::CliError;
use crate::input::load_json_with_hint;
use crate::output::common::print_json;

#[derive(Deserialize)]
struct ClassifyInput {
    chief: StateVector,
    deputy: StateVector,
    #[serde(default)]
    proximity: ProximityConfig,
}

/// Classify separation and print JSON result.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: ClassifyInput = load_json_with_hint(
        input_path,
        "classify expects { chief, deputy, proximity? }; see examples/classify.json",
    )?;
    let phase = classify_separation(&input.chief, &input.deputy, &input.proximity)?;
    print_json(&phase)
}

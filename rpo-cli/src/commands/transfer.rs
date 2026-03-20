//! Plumbing: solve Lambert transfer (JSON only).

use std::path::Path;

use serde::Deserialize;

use rpo_core::propagation::{solve_lambert_with_config, LambertConfig};
use rpo_core::types::StateVector;

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;

#[derive(Deserialize)]
struct TransferInput {
    departure: StateVector,
    arrival: StateVector,
    #[serde(default)]
    config: LambertConfig,
}

/// Solve Lambert transfer and print JSON result.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: TransferInput = load_json(input_path)?;
    let transfer = solve_lambert_with_config(&input.departure, &input.arrival, &input.config)?;
    print_json(&transfer)
}

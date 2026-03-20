//! Plumbing: compute ROE between chief and deputy states (JSON only).

use std::path::Path;

use serde::Deserialize;

use rpo_core::elements::{compute_roe, state_to_keplerian};
use rpo_core::types::StateVector;

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;

#[derive(Deserialize)]
struct RoeInput {
    chief: StateVector,
    deputy: StateVector,
}

/// Compute ROE between two states and print JSON.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: RoeInput = load_json(input_path)?;
    let chief_ke = state_to_keplerian(&input.chief)?;
    let deputy_ke = state_to_keplerian(&input.deputy)?;
    let roe = compute_roe(&chief_ke, &deputy_ke)?;
    print_json(&roe)
}

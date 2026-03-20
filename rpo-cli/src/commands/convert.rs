//! Plumbing: convert between state representations (JSON only).

use std::path::Path;

use rpo_core::elements::state_to_keplerian;
use rpo_core::types::StateVector;

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;

/// Convert ECI state to the specified representation and print JSON.
pub fn run(input_path: &Path, to: &str) -> Result<(), CliError> {
    let state: StateVector = load_json(input_path)?;

    match to {
        "keplerian" | "kep" => {
            let ke = state_to_keplerian(&state)?;
            print_json(&ke)?;
        }
        other => {
            return Err(CliError::UnknownFormat {
                format: other.to_string(),
                valid: &["keplerian"],
            });
        }
    }

    Ok(())
}

//! Plumbing: propagate ROE state via STM (JSON only).

use std::path::Path;

use serde::Deserialize;

use rpo_core::pipeline::{to_propagation_model, PropagatorChoice};
use rpo_core::types::{DepartureState, KeplerianElements, QuasiNonsingularROE};

use crate::error::CliError;
use crate::input::load_json;
use crate::output::common::print_json;

/// Epoch string that we parse manually since `epoch_serde` is `pub(crate)`.
#[derive(Deserialize)]
struct PropagateInput {
    roe: QuasiNonsingularROE,
    chief_mean: KeplerianElements,
    epoch: String,
    dt_s: f64,
    #[serde(default)]
    propagator: PropagatorChoice,
}

/// Propagate a ROE state and print JSON result.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: PropagateInput = load_json(input_path)?;
    let epoch = hifitime::Epoch::from_gregorian_str(&input.epoch).map_err(|e| {
        CliError::EpochParse {
            input: input.epoch.clone(),
            source: e.to_string(),
        }
    })?;
    let model = to_propagation_model(&input.propagator);
    let departure = DepartureState {
        roe: input.roe,
        chief: input.chief_mean,
        epoch,
    };
    let result = model.propagate(&departure.roe, &departure.chief, departure.epoch, input.dt_s)?;
    print_json(&result)
}

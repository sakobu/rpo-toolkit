//! Lambert transfer handler — synchronous, ~100ms.

use crate::error::ServerError;
use rpo_core::mission::config::ProximityConfig;
use rpo_core::mission::types::PerchGeometry;
use rpo_core::pipeline::types::{TransferComputationInput, TransferResult};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::pipeline::compute_transfer;

/// Handle a `ComputeTransfer` message.
///
/// Constructs a `TransferComputationInput` from the self-contained message
/// fields and delegates to `rpo_nyx::pipeline::compute_transfer()`.
///
/// # Errors
///
/// - [`ServerError::Lambert`] if the Lambert solver fails (convergence, degenerate
///   geometry, non-positive TOF).
/// - [`ServerError::PipelineFailure`] if classification fails (mission planning error).
pub fn handle_compute_transfer(
    chief: StateVector,
    deputy: StateVector,
    perch: PerchGeometry,
    proximity: ProximityConfig,
    lambert_tof_s: f64,
    lambert_config: LambertConfig,
) -> Result<TransferResult, ServerError> {
    let input = TransferComputationInput {
        chief,
        deputy,
        perch,
        proximity,
        lambert_tof_s,
        lambert_config,
    };
    Ok(compute_transfer(&input)?)
}

//! Lambert transfer handler — synchronous, ~100ms.

use crate::error::ServerError;
use rpo_core::mission::config::{MissionConfig, ProximityConfig};
use rpo_core::mission::types::PerchGeometry;
use rpo_core::pipeline::types::{PipelineInput, PropagatorChoice, TransferResult};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::pipeline::compute_transfer;

/// Handle a `ComputeTransfer` message.
///
/// Constructs a `PipelineInput` from the self-contained message fields and
/// delegates to `rpo_nyx::pipeline::compute_transfer()`. Unused pipeline
/// fields (waypoints, config, etc.) are set to defaults — `compute_transfer`
/// only reads the transfer-related fields.
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
    let input = PipelineInput {
        chief,
        deputy,
        perch,
        lambert_tof_s,
        lambert_config,
        proximity,
        waypoints: vec![],
        config: MissionConfig::default(),
        propagator: PropagatorChoice::default(),
        chief_config: None,
        deputy_config: None,
        navigation_accuracy: None,
        maneuver_uncertainty: None,
        monte_carlo: None,
        cola: None,
        safety_requirements: None,
    };
    Ok(compute_transfer(&input)?)
}

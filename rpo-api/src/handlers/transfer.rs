//! Transfer computation handler: Lambert transfer + perch handoff states.

use rpo_core::pipeline::compute_transfer;

use crate::error::ApiError;
use crate::protocol::{MissionDefinition, TransferResultPayload};

/// Compute Lambert transfer and perch handoff states.
///
/// Runs classification, solves Lambert (if far-field), and computes
/// the perch ECI states. Does NOT run waypoint targeting.
///
/// Pure function — runs in microseconds (proximity) to ~100ms (far-field Lambert).
///
/// # Errors
///
/// Returns [`ApiError`] if classification or Lambert solving fails.
pub fn handle_compute_transfer(
    def: &MissionDefinition,
) -> Result<TransferResultPayload, ApiError> {
    let result = compute_transfer(def)?;
    Ok(result)
}

//! Transfer computation handler: Lambert transfer + perch handoff states.

use rpo_core::pipeline::compute_transfer;

use crate::error::ApiError;
use crate::protocol::TransferResultPayload;
use crate::session::Session;

/// Compute Lambert transfer and store the result in the session.
///
/// Assembles a [`PipelineInput`](rpo_core::pipeline::PipelineInput) from the
/// current session state, runs classification + Lambert solving, and stores
/// the result. The stored transfer is the starting point for waypoint planning.
///
/// Pure function — runs in microseconds (proximity) to ~100ms (far-field Lambert).
///
/// # Errors
///
/// Returns [`ApiError`] if session states are missing, or classification/Lambert fails.
pub fn handle_compute_transfer(
    session: &mut Session,
) -> Result<TransferResultPayload, ApiError> {
    let input = session.assemble_pipeline_input()?;
    let result = compute_transfer(&input)?;
    session.store_transfer(result.clone());
    Ok(result)
}

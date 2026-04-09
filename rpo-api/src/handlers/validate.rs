//! Full-physics validation handler — blocking, seconds to minutes.

use crate::error::ServerError;
use crate::protocol::{ColaBurnInput, ProgressPhase, PROGRESS_COMPLETE, PROGRESS_EXECUTING, PROGRESS_START};
use anise::prelude::Almanac;
use rpo_core::mission::types::{ValidationReport, WaypointMission};
use rpo_core::types::spacecraft::SpacecraftConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::validation::{validate_mission_nyx, ValidationConfig};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::mpsc;

use super::send_progress;

/// Domain inputs for a validation job.
///
/// Groups the 7 domain parameters needed by [`handle_validate`], keeping the
/// handler signature at 4 arguments (input + almanac + progress + cancel).
pub(crate) struct ValidateJobInput {
    /// Analytical mission to validate against nyx full-physics propagation.
    pub mission: WaypointMission,
    /// Chief ECI state at mission start.
    pub chief: StateVector,
    /// Deputy ECI state at mission start.
    pub deputy: StateVector,
    /// Chief spacecraft physical properties.
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft physical properties.
    pub deputy_config: SpacecraftConfig,
    /// Intermediate comparison samples per leg.
    pub samples_per_leg: u32,
    /// Optional COLA avoidance burns to inject during nyx propagation.
    pub cola_burn_inputs: Vec<ColaBurnInput>,
}

/// Handle a `Validate` message on a blocking thread.
///
/// Sends progress updates via `progress_tx` and checks `cancel` between phases.
/// Returns the validation report on success.
///
/// # Errors
///
/// - [`ServerError::Cancelled`] if the cancel flag is set before validation starts.
/// - [`ServerError::Validation`] if nyx full-physics validation fails (empty trajectory,
///   nyx bridge error, frame conversion failure, COLA epoch out of bounds).
pub(crate) fn handle_validate(
    input: ValidateJobInput,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<crate::protocol::ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
) -> Result<ValidationReport, ServerError> {
    send_progress(progress_tx, ProgressPhase::Validate, "Starting validation...", PROGRESS_START);

    if cancel.load(Ordering::Relaxed) {
        return Err(ServerError::Cancelled);
    }

    let cola_burns: Vec<_> = input.cola_burn_inputs.into_iter().map(Into::into).collect();

    let config = ValidationConfig {
        samples_per_leg: input.samples_per_leg,
        chief_config: input.chief_config,
        deputy_config: input.deputy_config,
    };

    send_progress(progress_tx, ProgressPhase::Validate, "Running nyx validation...", PROGRESS_EXECUTING);

    let report = validate_mission_nyx(
        &input.mission,
        &input.chief,
        &input.deputy,
        &config,
        &cola_burns,
        almanac,
    )?;

    send_progress(progress_tx, ProgressPhase::Validate, "Validation complete", PROGRESS_COMPLETE);

    Ok(report)
}

//! Classification handler: determine proximity vs far-field.

use rpo_core::mission::{classify_separation, MissionPhase};

use crate::error::ApiError;
use crate::protocol::MissionDefinition;

/// Classify the separation between chief and deputy.
///
/// Pure function — runs in microseconds, always inline.
///
/// # Errors
/// Returns [`ApiError`] if the underlying classification fails.
pub fn handle_classify(def: &MissionDefinition) -> Result<MissionPhase, ApiError> {
    let phase = classify_separation(&def.chief, &def.deputy, &def.proximity)?;
    Ok(phase)
}

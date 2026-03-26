//! Classification handler: determine proximity vs far-field.

use rpo_core::mission::{classify_separation, MissionPhase};

use crate::error::ApiError;
use crate::session::Session;

/// Classify the separation between chief and deputy from session state.
///
/// Pure function — runs in microseconds, always inline.
///
/// # Errors
/// Returns [`ApiError`] if the chief/deputy states are missing or classification fails.
pub fn handle_classify(session: &Session) -> Result<MissionPhase, ApiError> {
    let chief = session.require_chief()?;
    let deputy = session.require_deputy()?;
    let phase = classify_separation(chief, deputy, &session.proximity)?;
    Ok(phase)
}

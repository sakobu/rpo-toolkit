//! Validation error types for nyx full-physics comparison.

use rpo_core::elements::eci_ric_dcm::DcmError;
use rpo_core::mission::safety::SafetyError;
use crate::nyx_bridge::NyxBridgeError;

/// Errors from nyx high-fidelity validation.
#[derive(Debug, thiserror::Error)]
pub enum ValidationError {
    /// Nyx bridge failure (almanac, dynamics, propagation, conversion).
    #[error("nyx bridge: {0}")]
    NyxBridge(#[source] Box<NyxBridgeError>),
    /// Safety analysis failure.
    #[error("safety analysis failed: {source}")]
    Safety {
        /// The underlying safety error.
        #[source]
        source: SafetyError,
    },
    /// No trajectory points to analyze.
    #[error("no trajectory points to analyze")]
    EmptyTrajectory,
    /// ECI-RIC frame conversion failed.
    #[error("frame conversion failed: {0}")]
    DcmFailure(#[from] DcmError),
    /// COLA burn epoch falls outside the valid range for its leg.
    #[error("COLA burn elapsed_s={elapsed_s:.3} outside (0, {tof_s:.3}) on leg {leg_index}")]
    ColaEpochOutOfBounds {
        /// Computed elapsed time from leg departure (seconds).
        elapsed_s: f64,
        /// Leg time-of-flight (seconds).
        tof_s: f64,
        /// Index of the leg.
        leg_index: usize,
    },
}

// Boxed target — hand-rolled per migration plan §4; thiserror's #[from]
// does not auto-box.
impl From<NyxBridgeError> for ValidationError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

// Struct-form target variant (Safety { source }) — #[from] only works on
// tuple variants, so this routing is hand-rolled.
impl From<SafetyError> for ValidationError {
    fn from(e: SafetyError) -> Self {
        Self::Safety { source: e }
    }
}

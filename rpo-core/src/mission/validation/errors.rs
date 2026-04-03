//! Validation error types for nyx full-physics comparison.

use hifitime::Epoch;

use crate::elements::eci_ric_dcm::DcmError;
use crate::mission::safety::SafetyError;
use crate::propagation::nyx_bridge::NyxBridgeError;

/// Errors from nyx high-fidelity validation.
#[derive(Debug)]
pub enum ValidationError {
    /// Nyx bridge failure (almanac, dynamics, propagation, conversion).
    NyxBridge(Box<NyxBridgeError>),
    /// Safety analysis failure.
    Safety {
        /// The underlying safety error.
        source: SafetyError,
    },
    /// No trajectory points to analyze.
    EmptyTrajectory,
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
    /// ANISE eclipse query failed at a specific epoch.
    EclipseQuery {
        /// The epoch at which the query failed.
        epoch: Epoch,
        /// The underlying error message.
        message: String,
    },
    /// COLA burn epoch falls outside the valid range for its leg.
    ColaEpochOutOfBounds {
        /// Computed elapsed time from leg departure (seconds).
        elapsed_s: f64,
        /// Leg time-of-flight (seconds).
        tof_s: f64,
        /// Index of the leg.
        leg_index: usize,
    },
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NyxBridge(e) => write!(f, "nyx bridge: {e}"),
            Self::Safety { source } => {
                write!(f, "safety analysis failed: {source}")
            }
            Self::EmptyTrajectory => {
                write!(f, "no trajectory points to analyze")
            }
            Self::DcmFailure(e) => write!(f, "frame conversion failed: {e}"),
            Self::EclipseQuery { epoch, message } => {
                write!(f, "eclipse query at {epoch} failed: {message}")
            }
            Self::ColaEpochOutOfBounds { elapsed_s, tof_s, leg_index } => {
                write!(
                    f,
                    "COLA burn elapsed_s={elapsed_s:.3} outside (0, {tof_s:.3}) on leg {leg_index}"
                )
            }
        }
    }
}

impl std::error::Error for ValidationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::NyxBridge(e) => Some(e.as_ref()),
            Self::Safety { source } => Some(source),
            Self::DcmFailure(e) => Some(e),
            Self::EmptyTrajectory
            | Self::EclipseQuery { .. }
            | Self::ColaEpochOutOfBounds { .. } => None,
        }
    }
}

impl From<NyxBridgeError> for ValidationError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<DcmError> for ValidationError {
    fn from(e: DcmError) -> Self {
        Self::DcmFailure(e)
    }
}

impl From<SafetyError> for ValidationError {
    fn from(e: SafetyError) -> Self {
        Self::Safety { source: e }
    }
}

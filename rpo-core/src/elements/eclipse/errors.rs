//! Eclipse computation error types.
//!
//! Eclipse computation is advisory — callers may convert these to `Option`
//! via `.ok()` when eclipse data is non-critical. The error preserves
//! diagnostic information for callers that need it.

use crate::elements::eci_ric_dcm::DcmError;
use crate::elements::keplerian_conversions::ConversionError;

/// Errors from eclipse computation.
///
/// Eclipse computation is advisory — callers may convert these to `Option`
/// via `.ok()` when eclipse data is non-critical. The error preserves
/// diagnostic information for callers that need it.
#[derive(Debug, Clone)]
pub enum EclipseComputeError {
    /// ECI ↔ Keplerian conversion failure (degenerate orbit geometry).
    Conversion(ConversionError),
    /// ECI ↔ RIC frame transformation failure during deputy eclipse
    /// reconstruction (degenerate chief state).
    Dcm(DcmError),
    /// No non-empty trajectory legs available for eclipse computation.
    EmptyTrajectory,
}

impl std::fmt::Display for EclipseComputeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Conversion(e) => write!(f, "EclipseComputeError: {e}"),
            Self::Dcm(e) => write!(f, "EclipseComputeError: {e}"),
            Self::EmptyTrajectory => {
                write!(f, "EclipseComputeError: no non-empty trajectory legs")
            }
        }
    }
}

impl std::error::Error for EclipseComputeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Conversion(e) => Some(e),
            Self::Dcm(e) => Some(e),
            Self::EmptyTrajectory => None,
        }
    }
}

impl From<ConversionError> for EclipseComputeError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion(e)
    }
}

impl From<DcmError> for EclipseComputeError {
    fn from(e: DcmError) -> Self {
        Self::Dcm(e)
    }
}

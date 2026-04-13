//! Eclipse geometry error type.

use crate::elements::{ConversionError, DcmError};

/// Errors from eclipse geometry computation (frame / element conversions).
///
/// Eclipse computation is advisory — callers may convert these to `Option`
/// via `.ok()` when eclipse data is non-critical. The error preserves
/// diagnostic information for callers that need it.
#[derive(Debug, Clone)]
pub enum EclipseGeometryError {
    /// ECI ↔ Keplerian conversion failure (degenerate orbit geometry).
    Conversion(ConversionError),
    /// ECI ↔ RIC frame transformation failure during deputy eclipse
    /// reconstruction (degenerate chief state).
    Dcm(DcmError),
}

impl std::fmt::Display for EclipseGeometryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Conversion(e) => write!(f, "{e}"),
            Self::Dcm(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for EclipseGeometryError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Conversion(e) => Some(e),
            Self::Dcm(e) => Some(e),
        }
    }
}

impl From<ConversionError> for EclipseGeometryError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion(e)
    }
}

impl From<DcmError> for EclipseGeometryError {
    fn from(e: DcmError) -> Self {
        Self::Dcm(e)
    }
}

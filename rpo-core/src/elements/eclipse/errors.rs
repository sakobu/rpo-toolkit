//! Eclipse geometry error type.

use crate::elements::{ConversionError, DcmError};

/// Errors from eclipse geometry computation (frame / element conversions).
///
/// Eclipse computation is advisory — callers may convert these to `Option`
/// via `.ok()` when eclipse data is non-critical. The error preserves
/// diagnostic information for callers that need it.
#[derive(Debug, Clone, thiserror::Error)]
pub enum EclipseGeometryError {
    /// ECI ↔ Keplerian conversion failure (degenerate orbit geometry).
    #[error(transparent)]
    Conversion(#[from] ConversionError),
    /// ECI ↔ RIC frame transformation failure during deputy eclipse
    /// reconstruction (degenerate chief state).
    #[error(transparent)]
    Dcm(#[from] DcmError),
}

//! Mission-level eclipse orchestration error type.

use crate::elements::{ConversionError, DcmError, EclipseGeometryError};

/// Errors from [`super::compute_mission_eclipse`] — orchestration-level
/// failures that aggregate per-leg geometry errors plus the empty-trajectory
/// case unique to mission assembly.
#[derive(Debug, Clone, thiserror::Error)]
pub enum MissionEclipseError {
    /// Per-point eclipse geometry computation failed.
    #[error(transparent)]
    Geometry(#[from] EclipseGeometryError),
    /// No non-empty trajectory legs available for eclipse computation.
    #[error("no non-empty trajectory legs")]
    EmptyTrajectory,
}

// Transitive tunnels through EclipseGeometryError — hand-rolled because
// thiserror's #[from] only synthesizes one-hop conversions.
impl From<ConversionError> for MissionEclipseError {
    fn from(e: ConversionError) -> Self {
        Self::Geometry(EclipseGeometryError::Conversion(e))
    }
}

impl From<DcmError> for MissionEclipseError {
    fn from(e: DcmError) -> Self {
        Self::Geometry(EclipseGeometryError::Dcm(e))
    }
}

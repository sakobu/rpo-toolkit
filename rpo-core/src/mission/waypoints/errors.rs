//! Mission-level eclipse orchestration error type.

use crate::elements::{ConversionError, DcmError, EclipseGeometryError};

/// Errors from [`super::compute_mission_eclipse`] — orchestration-level
/// failures that aggregate per-leg geometry errors plus the empty-trajectory
/// case unique to mission assembly.
#[derive(Debug, Clone)]
pub enum MissionEclipseError {
    /// Per-point eclipse geometry computation failed.
    Geometry(EclipseGeometryError),
    /// No non-empty trajectory legs available for eclipse computation.
    EmptyTrajectory,
}

impl std::fmt::Display for MissionEclipseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Geometry(e) => write!(f, "{e}"),
            Self::EmptyTrajectory => write!(f, "no non-empty trajectory legs"),
        }
    }
}

impl std::error::Error for MissionEclipseError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Geometry(e) => Some(e),
            Self::EmptyTrajectory => None,
        }
    }
}

impl From<EclipseGeometryError> for MissionEclipseError {
    fn from(e: EclipseGeometryError) -> Self {
        Self::Geometry(e)
    }
}

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

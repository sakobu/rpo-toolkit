//! Covariance propagation using STM-based linear mapping.
//!
//! Implements P₁ = Φ P₀ Φᵀ using J2/drag STMs, RIC↔ROE covariance
//! conversion via the T matrix, maneuver covariance updates via the
//! B matrix, and collision probability from Mahalanobis distance.
//!
//! Mission-level orchestration (`propagate_mission_covariance`) lives in
//! `mission::covariance` and threads these kernels across a multi-leg
//! waypoint mission.

pub mod propagate;
pub mod types;

use std::fmt;

use crate::elements::keplerian_conversions::ConversionError;
use crate::propagation::propagator::PropagationError;

pub use propagate::{
    compute_collision_metrics, propagate_covariance, propagate_covariance_with_drag,
    propagate_covariance_with_params, ric_accuracy_to_roe_covariance,
    roe_covariance_to_ric_position, update_covariance_at_maneuver,
};
pub use types::{
    CovarianceState, LegCovarianceReport, ManeuverUncertainty, MissionCovarianceReport,
    NavigationAccuracy,
};

// ── Error type ──────────────────────────────────────────────────────────────

/// Errors from covariance propagation.
#[derive(Debug, Clone)]
pub enum CovarianceError {
    /// T matrix is singular (cannot invert for ROE↔RIC conversion).
    SingularTMatrix,
    /// Underlying STM computation failed.
    StmFailure(PropagationError),
    /// Chief element validation failed (delegates to `ConversionError`).
    InvalidChiefElements(ConversionError),
    /// `n_steps` exceeds `u32::MAX` (would cause enormous loop / OOM).
    TooManySteps {
        /// The requested step count that exceeded `u32::MAX`.
        n_steps: usize,
    },
}

impl fmt::Display for CovarianceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SingularTMatrix => write!(f, "CovarianceError: T matrix is singular"),
            Self::StmFailure(e) => write!(f, "CovarianceError: STM computation failed — {e}"),
            Self::InvalidChiefElements(e) => {
                write!(f, "CovarianceError: invalid chief elements — {e}")
            }
            Self::TooManySteps { n_steps } => {
                write!(f, "CovarianceError: n_steps = {n_steps} exceeds u32::MAX")
            }
        }
    }
}

impl std::error::Error for CovarianceError {}

impl From<PropagationError> for CovarianceError {
    fn from(e: PropagationError) -> Self {
        Self::StmFailure(e)
    }
}

impl From<ConversionError> for CovarianceError {
    fn from(e: ConversionError) -> Self {
        Self::InvalidChiefElements(e)
    }
}

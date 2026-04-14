//! Covariance propagation using STM-based linear mapping.
//!
//! Implements P₁ = Φ P₀ Φᵀ using J2/drag STMs, RIC↔ROE covariance
//! conversion via the T matrix, maneuver covariance updates via the
//! B matrix, and Mahalanobis distance computation.
//!
//! Mission-level orchestration (`propagate_mission_covariance`) lives in
//! `mission::covariance` and threads these kernels across a multi-leg
//! waypoint mission.

pub mod propagate;
pub mod types;

use crate::elements::keplerian_conversions::ConversionError;
use crate::propagation::propagator::PropagationError;

pub use propagate::{
    compute_mahalanobis_distance, propagate_covariance, propagate_covariance_with_drag,
    propagate_covariance_with_params, ric_accuracy_to_roe_covariance,
    roe_covariance_to_ric_position, update_covariance_at_maneuver,
};
pub use types::{
    CovarianceState, LegCovarianceReport, ManeuverUncertainty, MissionCovarianceReport,
    NavigationAccuracy,
};

// ── Error type ──────────────────────────────────────────────────────────────

/// Errors from covariance propagation.
#[derive(Debug, Clone, thiserror::Error)]
pub enum CovarianceError {
    /// T matrix is singular (cannot invert for ROE↔RIC conversion).
    #[error("T matrix is singular")]
    SingularTMatrix,
    /// Underlying STM computation failed.
    #[error(transparent)]
    StmFailure(#[from] PropagationError),
    /// Chief element validation failed (delegates to `ConversionError`).
    #[error(transparent)]
    InvalidChiefElements(#[from] ConversionError),
    /// `n_steps` exceeds `u32::MAX` (would cause enormous loop / OOM).
    #[error("n_steps = {n_steps} exceeds u32::MAX")]
    TooManySteps {
        /// The requested step count that exceeded `u32::MAX`.
        n_steps: usize,
    },
}

//! Error types for formation design operations.

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::safety::SafetyError;
use crate::propagation::propagator::PropagationError;
use crate::types::KeplerError;

/// Errors from formation design operations.
#[derive(Debug, Clone, thiserror::Error)]
pub enum FormationDesignError {
    /// `T_pos` null-space computation produced degenerate geometry.
    #[error("singular T_pos geometry at mean arg lat = {mean_arg_lat_rad:.6} rad")]
    SingularGeometry {
        /// Chief mean argument of latitude where singularity occurs (rad).
        mean_arg_lat_rad: f64,
    },
    /// Requested `d_min` exceeds what the `T_pos` linearization regime
    /// supports for this chief geometry.
    ///
    /// Triggered when the perturbation norm produced by the safety
    /// projection in `safety_envelope` would exceed
    /// [`super::LINEARIZATION_PERTURBATION_BOUND`]. For V-bar/R-bar parallel
    /// zero-baseline perches the bound reduces to
    /// `min_separation_km ≤ a · LINEARIZATION_PERTURBATION_BOUND
    /// / VBAR_RBAR_PERTURBATION_RATIO`; Custom perches with non-zero
    /// baseline e/i have a geometry-dependent threshold and may hit this
    /// error at smaller requested separations.
    #[error(
        "requested separation {requested_km:.6} km exceeds achievable {achievable_km:.6} km within linearization bounds"
    )]
    SeparationUnachievable {
        /// Requested separation (km).
        requested_km: f64,
        /// Maximum achievable separation within linearization bound (km).
        achievable_km: f64,
    },
    /// Chief mean elements are invalid (e.g., `a_km` <= 0, e >= 1).
    #[error(transparent)]
    InvalidChiefElements(#[from] ConversionError),
    /// Safety analysis computation failed.
    #[error(transparent)]
    SafetyAnalysis(#[from] SafetyError),
    /// Transit trajectory has insufficient sampling density for reliable e/i monitoring.
    #[error("{total_samples} total samples < {required_per_orbit} required per orbit")]
    InsufficientSampling {
        /// Total samples in the provided trajectory.
        total_samples: u32,
        /// Required minimum samples per orbital period.
        required_per_orbit: u32,
    },
    /// J2 parameter computation failed.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Kepler equation or derived-quantity failure (period, mean motion).
    #[error(transparent)]
    KeplerFailure(#[from] KeplerError),
}

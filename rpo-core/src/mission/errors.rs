//! Mission planning error types.

use crate::elements::keplerian_conversions::ConversionError;
use crate::elements::roe_to_ric::RicError;
use crate::propagation::lambert::LambertError;
use crate::propagation::propagator::PropagationError;
use crate::types::KeplerError;

/// Errors from mission planning.
#[derive(Debug, Clone, thiserror::Error)]
pub enum MissionError {
    /// Propagation failure during proximity phase.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Lambert solver failure during transfer phase.
    #[error(transparent)]
    Lambert(#[from] LambertError),
    /// ECI ↔ Keplerian conversion failure.
    #[error(transparent)]
    Conversion(#[from] ConversionError),
    /// ROE ↔ RIC pseudo-inverse failure.
    #[error(transparent)]
    Ric(#[from] RicError),
    /// V-bar perch offset must be nonzero.
    #[error("invalid V-bar perch — along-track offset = {along_track_km:.6e} km (must be nonzero)")]
    InvalidVBarOffset {
        /// The invalid along-track offset (km).
        along_track_km: f64,
    },
    /// R-bar perch offset must be nonzero.
    #[error("invalid R-bar perch — radial offset = {radial_km:.6e} km (must be nonzero)")]
    InvalidRBarOffset {
        /// The invalid radial offset (km).
        radial_km: f64,
    },
    /// Targeting solver failed to converge.
    #[error("targeting failed to converge — error = {final_error_km:.6e} km after {iterations} iterations")]
    TargetingConvergence {
        /// Final position error (km)
        final_error_km: f64,
        /// Number of iterations completed
        iterations: u32,
    },
    /// Jacobian is singular and cannot be inverted.
    #[error("singular Jacobian in targeting solver")]
    SingularJacobian,
    /// No waypoints provided.
    #[error("no waypoints provided")]
    EmptyWaypoints,
    /// TOF optimization failed to find a valid solution.
    #[error(
        "TOF optimization failed — no valid TOF in [{min_tof:.1}, {max_tof:.1}] s ({num_starts} starts)"
    )]
    TofOptimizationFailure {
        /// Minimum TOF searched (seconds)
        min_tof: f64,
        /// Maximum TOF searched (seconds)
        max_tof: f64,
        /// Number of multi-start samples evaluated
        num_starts: u32,
    },
    /// Replan index is out of bounds for the waypoint list.
    #[error("replan index {index} out of bounds for {num_waypoints} waypoints")]
    InvalidReplanIndex {
        /// The invalid index provided
        index: usize,
        /// Total number of waypoints
        num_waypoints: usize,
    },
    /// Kepler equation or derived-quantity failure.
    #[error(transparent)]
    Kepler(#[from] KeplerError),
}

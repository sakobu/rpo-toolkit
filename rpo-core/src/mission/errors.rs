//! Mission planning error types.

use crate::elements::eci_ric_dcm::DcmError;
use crate::elements::keplerian_conversions::ConversionError;
use crate::propagation::lambert::LambertError;
use crate::propagation::propagator::PropagationError;
use crate::types::KeplerError;

/// Errors from mission planning.
#[derive(Debug, Clone)]
pub enum MissionError {
    /// Propagation failure during proximity phase.
    Propagation(PropagationError),
    /// Lambert solver failure during transfer phase.
    Lambert(LambertError),
    /// ECI ↔ Keplerian conversion failure.
    Conversion(ConversionError),
    /// V-bar perch offset must be nonzero.
    InvalidVBarOffset {
        /// The invalid along-track offset (km).
        along_track_km: f64,
    },
    /// R-bar perch offset must be nonzero.
    InvalidRBarOffset {
        /// The invalid radial offset (km).
        radial_km: f64,
    },
    /// Spacecraft are not in proximity for ROE-based operations.
    NotInProximity {
        /// Actual dimensionless separation δr/r
        delta_r_over_r: f64,
        /// Configured proximity threshold
        threshold: f64,
    },
    /// Targeting solver failed to converge.
    TargetingConvergence {
        /// Final position error (km)
        final_error_km: f64,
        /// Number of iterations completed
        iterations: u32,
    },
    /// Jacobian is singular and cannot be inverted.
    SingularJacobian,
    /// No waypoints provided.
    EmptyWaypoints,
    /// TOF optimization failed to find a valid solution.
    TofOptimizationFailure {
        /// Minimum TOF searched (seconds)
        min_tof: f64,
        /// Maximum TOF searched (seconds)
        max_tof: f64,
        /// Number of multi-start samples evaluated
        num_starts: u32,
    },
    /// Replan index is out of bounds for the waypoint list.
    InvalidReplanIndex {
        /// The invalid index provided
        index: usize,
        /// Total number of waypoints
        num_waypoints: usize,
    },
    /// Kepler equation or derived-quantity failure.
    Kepler(KeplerError),
}

impl std::fmt::Display for MissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Propagation(e) => write!(f, "MissionError: {e}"),
            Self::Lambert(e) => write!(f, "MissionError: {e}"),
            Self::Conversion(e) => write!(f, "MissionError: {e}"),
            Self::InvalidVBarOffset { along_track_km } => write!(
                f,
                "MissionError: invalid V-bar perch — along-track offset = {along_track_km:.6e} km (must be nonzero)"
            ),
            Self::InvalidRBarOffset { radial_km } => write!(
                f,
                "MissionError: invalid R-bar perch — radial offset = {radial_km:.6e} km (must be nonzero)"
            ),
            Self::NotInProximity { delta_r_over_r, threshold } => write!(
                f,
                "MissionError: not in proximity — δr/r = {delta_r_over_r:.6} exceeds threshold {threshold:.6}"
            ),
            Self::TargetingConvergence { final_error_km, iterations } => write!(
                f,
                "MissionError: targeting failed to converge — error = {final_error_km:.6e} km after {iterations} iterations"
            ),
            Self::SingularJacobian => write!(f, "MissionError: singular Jacobian in targeting solver"),
            Self::EmptyWaypoints => write!(f, "MissionError: no waypoints provided"),
            Self::TofOptimizationFailure { min_tof, max_tof, num_starts } => write!(
                f,
                "MissionError: TOF optimization failed — no valid TOF in [{min_tof:.1}, {max_tof:.1}] s ({num_starts} starts)"
            ),
            Self::InvalidReplanIndex { index, num_waypoints } => write!(
                f,
                "MissionError: replan index {index} out of bounds for {num_waypoints} waypoints"
            ),
            Self::Kepler(e) => write!(f, "MissionError: {e}"),
        }
    }
}

impl std::error::Error for MissionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Propagation(e) => Some(e),
            Self::Lambert(e) => Some(e),
            Self::Conversion(e) => Some(e),
            Self::Kepler(e) => Some(e),
            _ => None,
        }
    }
}

impl From<KeplerError> for MissionError {
    fn from(e: KeplerError) -> Self {
        Self::Kepler(e)
    }
}

impl From<PropagationError> for MissionError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<LambertError> for MissionError {
    fn from(e: LambertError) -> Self {
        Self::Lambert(e)
    }
}

impl From<ConversionError> for MissionError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion(e)
    }
}

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

//! Covariance propagation domain types.
//!
//! Navigation accuracy, maneuver uncertainty, and covariance state types
//! used by the covariance propagation pipeline.

use hifitime::Epoch;
use nalgebra::{SMatrix, Vector3};
use serde::{Deserialize, Serialize};

use crate::constants::{
    DEFAULT_MANEUVER_MAGNITUDE_SIGMA, DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
    DEFAULT_NAV_POSITION_SIGMA_KM, DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
};
use crate::types::Matrix6;

/// Navigation accuracy specification in RIC frame.
///
/// Users specify 1-sigma position and velocity accuracy per RIC axis.
/// Converted to ROE-space covariance internally via the T matrix inverse.
///
/// Defaults to typical LEO proximity operations navigation accuracy:
/// 100 m position, 0.1 m/s velocity per axis.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct NavigationAccuracy {
    /// 1-sigma position accuracy per RIC axis (km): [radial, in-track, cross-track]
    pub position_sigma_ric_km: Vector3<f64>,
    /// 1-sigma velocity accuracy per RIC axis (km/s): [radial, in-track, cross-track]
    pub velocity_sigma_ric_km_s: Vector3<f64>,
}

impl Default for NavigationAccuracy {
    fn default() -> Self {
        Self {
            position_sigma_ric_km: Vector3::new(
                DEFAULT_NAV_POSITION_SIGMA_KM,
                DEFAULT_NAV_POSITION_SIGMA_KM,
                DEFAULT_NAV_POSITION_SIGMA_KM,
            ),
            velocity_sigma_ric_km_s: Vector3::new(
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
            ),
        }
    }
}

/// Maneuver execution uncertainty model.
///
/// Magnitude error is proportional (e.g., 0.01 = 1% 1-sigma).
/// Pointing error is a scalar 1-sigma (rad), applied as a rotation
/// of the nominal Δv direction via Rodrigues' formula.
///
/// This is the covariance-layer representation. The MC-layer counterpart
/// is `ManeuverDispersion`, which uses the same (`magnitude_sigma`, `pointing_sigma_rad`)
/// representation for sampling.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ManeuverUncertainty {
    /// Proportional 1-sigma magnitude error (dimensionless, e.g. 0.01 = 1%)
    pub magnitude_sigma: f64,
    /// 1-sigma pointing error (rad), applied as isotropic rotation about Δv axis
    pub pointing_sigma_rad: f64,
}

impl Default for ManeuverUncertainty {
    fn default() -> Self {
        Self {
            magnitude_sigma: DEFAULT_MANEUVER_MAGNITUDE_SIGMA,
            pointing_sigma_rad: DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
        }
    }
}

/// Covariance state at a single epoch along the trajectory.
///
/// Contains the propagated covariance in ROE space, projected to RIC
/// position covariance, with derived uncertainty metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CovarianceState {
    /// Epoch
    #[serde(with = "crate::types::state::epoch_serde")]
    pub epoch: Epoch,
    /// Elapsed time since leg start (seconds)
    pub elapsed_s: f64,
    /// 6×6 ROE-space covariance [δa, δλ, δeₓ, δeᵧ, δiₓ, δiᵧ] (must be symmetric PSD)
    pub covariance_roe: Matrix6,
    /// 3×3 RIC position covariance (km²), projected from ROE via `T_pos`
    pub covariance_ric_position_km2: SMatrix<f64, 3, 3>,
    /// 3-sigma position bounds per RIC axis (km): sqrt(diag) × 3
    pub sigma3_position_ric_km: Vector3<f64>,
    /// Mahalanobis distance from chief (dimensionless).
    /// Measures deputy-chief separation in sigma-space; smaller values
    /// indicate closer proximity relative to the covariance ellipsoid.
    pub mahalanobis_distance: f64,
}

/// Covariance evolution summary for a single maneuver leg.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegCovarianceReport {
    /// Covariance states at sampled time points along the leg
    pub states: Vec<CovarianceState>,
    /// Maximum 3-sigma position uncertainty across leg (km, any axis)
    pub max_sigma3_position_km: f64,
    /// Minimum Mahalanobis distance across leg (closest approach in sigma-space)
    pub min_mahalanobis_distance: f64,
}

/// Complete mission covariance propagation report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionCovarianceReport {
    /// Per-leg covariance evolution
    pub legs: Vec<LegCovarianceReport>,
    /// Navigation accuracy used
    pub navigation_accuracy: NavigationAccuracy,
    /// Maneuver uncertainty used (if any)
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Overall maximum 3-sigma position uncertainty (km)
    pub max_sigma3_position_km: f64,
    /// Minimum Mahalanobis distance across entire mission (closest approach in sigma-space)
    pub min_mahalanobis_distance: f64,
    /// Predicted nominal RIC position at mission end (km).
    /// Used as the center for terminal 3-sigma box containment checks.
    pub terminal_position_ric_km: Vector3<f64>,
    /// Terminal 3-sigma position uncertainty per RIC axis (km) at mission end.
    /// Used for epoch-matched comparison against MC terminal dispersion.
    pub terminal_sigma3_position_ric_km: Vector3<f64>,
}

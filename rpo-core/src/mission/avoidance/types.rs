//! COLA type definitions, error types, and named constants.
//!
//! All constants that control solver behavior (tolerances, iteration limits,
//! singularity guards) are defined here with named identifiers and physical
//! justification, per the crate tolerance policy.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::closest_approach::PocaError;
use crate::propagation::propagator::PropagationError;

// ---------------------------------------------------------------------------
// Named tolerances
// ---------------------------------------------------------------------------

/// Distance-ratio convergence tolerance on POCA distance (km).
/// 0.05 km = 50 m, well below any operational safety margin. Accounts
/// for linearization error in the distance-ratio scaling approach where
/// the POCA distance responds nonlinearly to Δv scaling.
pub(super) const COLA_NEWTON_TOL_KM: f64 = 0.05;

/// Maximum Newton iterations for distance-ratio refinement.
/// 20 iterations handles nonlinear distance response with margin.
pub(super) const COLA_MAX_ITERATIONS: usize = 20;

/// Minimum ROE e/i vector magnitude for singularity avoidance.
/// Below this, the correction direction is undefined (degenerate geometry).
pub(super) const COLA_MIN_ROE_MAGNITUDE: f64 = 1e-10;

/// |R| / |C| dominance ratio for correction classification.
/// When the radial/in-track (eccentricity) component exceeds the cross-track
/// (inclination) component by this factor, in-plane correction is preferred.
pub(super) const COLA_DOMINANCE_RATIO: f64 = 2.0;

/// Number of trajectory samples for post-avoidance POCA verification.
pub(super) const COLA_VERIFICATION_STEPS: usize = 200;

/// Clamp factor for COLA burn time within a leg's TOF.
/// When the computed burn location wraps past the leg boundary due to
/// floating-point rounding, the burn time is clamped to this fraction
/// of the leg TOF to keep it within bounds.
pub(crate) const BURN_TIME_CLAMP_FRACTION: f64 = 0.99;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Configuration for collision avoidance maneuver design.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ColaConfig {
    /// Target POCA distance after avoidance (km).
    pub target_distance_km: f64,
    /// Maximum available delta-v budget (km/s).
    pub max_dv_km_s: f64,
}

/// Classification of the avoidance correction type.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CorrectionType {
    /// Eccentricity vector change only (D'Amico Eq. 2.41). No delta-a change — no drift.
    InPlane,
    /// Inclination vector change only (D'Amico Eq. 2.53).
    CrossTrack,
    /// Combined e + i correction with Newton refinement.
    Combined,
}

/// Avoidance maneuver result from COLA computation.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AvoidanceManeuver {
    /// Epoch of avoidance burn.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub epoch: Epoch,
    /// Delta-v in RIC frame (km/s).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub dv_ric_km_s: Vector3<f64>,
    /// Mean argument of latitude at burn (rad).
    pub maneuver_location_rad: f64,
    /// POCA distance after avoidance (km), verified by re-propagation.
    pub post_avoidance_poca_km: f64,
    /// Total fuel cost (km/s).
    pub fuel_cost_km_s: f64,
    /// Which correction type was applied.
    pub correction_type: CorrectionType,
    /// Leg index within the mission.
    pub leg_index: usize,
}

/// Errors from collision avoidance computation.
#[derive(Debug, Clone)]
pub enum AvoidanceError {
    /// POCA already satisfies the threshold; no avoidance needed.
    NoPocaViolation {
        /// Current POCA distance (km).
        poca_km: f64,
        /// Required threshold distance (km).
        threshold_km: f64,
    },
    /// Required delta-v exceeds the available budget.
    ExceedsBudget {
        /// Required delta-v magnitude (km/s).
        required_km_s: f64,
        /// Available budget (km/s).
        budget_km_s: f64,
    },
    /// Newton iteration did not converge within the iteration limit.
    NoConvergence {
        /// Number of iterations attempted.
        iterations: usize,
        /// Final residual distance (km).
        residual_km: f64,
    },
    /// ROE geometry is too small for meaningful correction direction.
    DegenerateGeometry {
        /// Eccentricity vector magnitude (dimensionless).
        de_mag: f64,
        /// Inclination vector magnitude (dimensionless).
        di_mag: f64,
    },
    /// Underlying POCA computation failed.
    PocaFailure(PocaError),
    /// Underlying propagation failed.
    PropagationFailure(PropagationError),
    /// GVE maneuver application failed.
    GveFailure(ConversionError),
}

impl std::fmt::Display for AvoidanceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoPocaViolation {
                poca_km,
                threshold_km,
            } => write!(
                f,
                "POCA distance {poca_km:.6} km already exceeds threshold {threshold_km:.6} km"
            ),
            Self::ExceedsBudget {
                required_km_s,
                budget_km_s,
            } => write!(
                f,
                "required delta-v {required_km_s:.6e} km/s exceeds budget {budget_km_s:.6e} km/s"
            ),
            Self::NoConvergence {
                iterations,
                residual_km,
            } => write!(
                f,
                "Newton iteration did not converge after {iterations} iterations (residual: {residual_km:.6e} km)"
            ),
            Self::DegenerateGeometry { de_mag, di_mag } => write!(
                f,
                "degenerate ROE geometry: |de| = {de_mag:.6e}, |di| = {di_mag:.6e}"
            ),
            Self::PocaFailure(e) => write!(f, "POCA computation failed: {e}"),
            Self::PropagationFailure(e) => write!(f, "propagation failed: {e}"),
            Self::GveFailure(e) => write!(f, "GVE maneuver application failed: {e}"),
        }
    }
}

impl std::error::Error for AvoidanceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::PocaFailure(e) => Some(e),
            Self::PropagationFailure(e) => Some(e),
            Self::GveFailure(e) => Some(e),
            _ => None,
        }
    }
}

impl From<PocaError> for AvoidanceError {
    fn from(e: PocaError) -> Self {
        Self::PocaFailure(e)
    }
}

impl From<PropagationError> for AvoidanceError {
    fn from(e: PropagationError) -> Self {
        Self::PropagationFailure(e)
    }
}

impl From<ConversionError> for AvoidanceError {
    fn from(e: ConversionError) -> Self {
        Self::GveFailure(e)
    }
}

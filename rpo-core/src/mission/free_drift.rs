//! Free-drift trajectory analysis for abort/contingency scenarios.
//!
//! For each maneuver leg, propagates a trajectory from the **pre-burn** ROE
//! state (burn skipped / failed) to visualize the abort geometry. Includes
//! a bounded-motion diagnostic (D'Amico Eq. 2.33) that explains whether the
//! formation remains passively safe in free drift.
//!
//! # References
//! - Bounded-motion condition: D'Amico Eq. 2.33
//! - γ parameter: D'Amico Eq. 2.25
//! - Secular ROE drift under J2: D'Amico Eq. 2.29
//!
//! # Validity
//! - The bounded-motion residual (Eq. 2.33) is J2-only. Differential drag
//!   causes additional secular δa drift (D'Amico Eqs. 2.36-2.37) not
//!   captured by this diagnostic.

use hifitime::Epoch;
use serde::{Deserialize, Serialize};

use crate::constants::{J2, R_EARTH};
use crate::mission::safety::{analyze_trajectory_safety, SafetyError};
use crate::mission::types::SafetyMetrics;
use crate::propagation::j2_params::compute_j2_params;
use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Free-drift analysis for a single maneuver leg.
///
/// Represents the abort/contingency trajectory if the departure burn is
/// skipped. Computed from the pre-burn ROE state.
///
/// # References
/// - Bounded-motion condition: D'Amico Eq. 2.33
/// - γ parameter: D'Amico Eq. 2.25
/// - Secular ROE drift under J2: D'Amico Eq. 2.29
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FreeDriftAnalysis {
    /// Propagated trajectory from pre-burn ROE state.
    pub trajectory: Vec<PropagatedState>,
    /// Safety metrics computed on the free-drift arc.
    pub safety: SafetyMetrics,
    /// Residual of D'Amico Eq. 2.33 bounded-motion condition.
    /// Zero = bounded relative motion under J2; nonzero = secular
    /// along-track drift. Dimensionless.
    ///
    /// Computed as: `γ · sin(2i) · δix + (1/7) · δa`
    /// where γ, i are from the chief mean elements and δix, δa
    /// are from the pre-burn ROE state.
    ///
    /// # Validity
    /// This diagnostic is J2-only. Differential drag causes additional
    /// secular δa drift (D'Amico Eqs. 2.36-2.37) not captured here.
    pub bounded_motion_residual: f64,
}

/// Errors from free-drift computation.
#[derive(Debug, Clone)]
pub enum FreeDriftError {
    /// Propagation of the free-drift trajectory failed.
    Propagation(PropagationError),
    /// Safety analysis on the free-drift trajectory failed.
    Safety(SafetyError),
}

impl std::fmt::Display for FreeDriftError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Propagation(e) => write!(f, "free-drift propagation failed: {e}"),
            Self::Safety(e) => write!(f, "free-drift safety analysis failed: {e}"),
        }
    }
}

impl std::error::Error for FreeDriftError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Propagation(e) => Some(e),
            Self::Safety(e) => Some(e),
        }
    }
}

impl From<PropagationError> for FreeDriftError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<SafetyError> for FreeDriftError {
    fn from(e: SafetyError) -> Self {
        Self::Safety(e)
    }
}

/// Divisor for the δa term in D'Amico Eq. 2.33 bounded-motion condition.
/// The condition is: γ · sin(2i) · δix + δa / 7 = 0.
const BOUNDED_MOTION_DA_DIVISOR: f64 = 7.0;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute the bounded-motion residual (D'Amico Eq. 2.33).
///
/// Returns the scalar residual of the J2-perturbed bounded-motion
/// condition: `γ · sin(2i) · δix + (1/7) · δa`.
///
/// # Arguments
/// * `roe` - Quasi-nonsingular ROE state (dimensionless components)
/// * `chief` - Chief mean Keplerian elements
///
/// # References
/// - D'Amico Eq. 2.33: closed orbit condition under J2
/// - D'Amico Eq. 2.25: γ = (J2/2) · (R\_E/a)² · (1/η⁴)
///
/// # Invariants
/// - `chief.a_km > 0`
/// - `chief.e` in [0, 1) (η must be real and positive)
///
/// # Singularities
/// - `e → 1`: η → 0, γ → ∞. Invalid for hyperbolic/parabolic orbits.
/// - `i = 0` or `i = π/2`: sin(2i) = 0, J2 coupling vanishes.
///   The condition degenerates to `δa/7` (only δa matters).
///
/// # Validity
/// This diagnostic is J2-only. Differential drag causes additional secular
/// δa drift (D'Amico Eqs. 2.36-2.37) not captured by this residual.
///
/// # Errors
/// Returns `PropagationError` if the chief elements are invalid
/// (e.g. `e >= 1` or `a_km <= 0`).
pub fn bounded_motion_residual(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
) -> Result<f64, PropagationError> {
    // Reuse J2Params for η, sin_i, cos_i validation
    let j2p = compute_j2_params(chief)?;

    // γ = (J2/2) · (R_E/a)² · (1/η⁴)   (D'Amico Eq. 2.25)
    let r_over_a = R_EARTH / chief.a_km;
    let eta4 = j2p.eta * j2p.eta * j2p.eta * j2p.eta;
    let gamma = (J2 / 2.0) * r_over_a * r_over_a / eta4;

    // sin(2i) = 2·sin(i)·cos(i)
    let sin_2i = 2.0 * j2p.sin_i * j2p.cos_i;

    // Eq. 2.33: γ · sin(2i) · δix + δa / BOUNDED_MOTION_DA_DIVISOR
    Ok(gamma * sin_2i * roe.dix + roe.da / BOUNDED_MOTION_DA_DIVISOR)
}

/// Compute free-drift trajectory and safety analysis for a maneuver leg.
///
/// Propagates from the pre-burn ROE state (departure burn skipped) for
/// the same time-of-flight as the nominal leg. Runs safety analysis on
/// the resulting trajectory and computes the bounded-motion diagnostic
/// (D'Amico Eq. 2.33).
///
/// # Arguments
/// * `departure_roe` - Pre-burn ROE state (before departure maneuver)
/// * `chief_mean` - Chief mean Keplerian elements at departure epoch
/// * `departure_epoch` - Epoch of the (skipped) departure burn
/// * `tof_s` - Time of flight for the leg (seconds)
/// * `model` - Propagation model (J2 or J2+drag)
/// * `num_steps` - Number of trajectory sample points
///
/// # Invariants
/// - `tof_s > 0`
/// - `chief_mean.a_km > 0`
/// - `chief_mean.e` in [0, 1)
/// - `num_steps >= 2`
///
/// # Validity regime
/// - Near-circular chief orbit (`e < ~0.1`): the ROE linearization
///   and J2 STM assume near-circular orbits. Accuracy degrades for
///   higher eccentricities.
/// - Small relative separation (`δr/r << 1`): the ROE-to-RIC mapping
///   is a first-order linearization. Free-drift trajectories for large
///   initial separations should be validated with full physics.
/// - `e → 1`: bounded-motion residual's γ diverges (η → 0). The
///   propagation itself may still succeed but the diagnostic is
///   unreliable for high-eccentricity orbits.
///
/// # Errors
/// - `FreeDriftError::Propagation`: propagation failed
/// - `FreeDriftError::Safety`: trajectory safety analysis failed
pub fn compute_free_drift(
    departure_roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    departure_epoch: Epoch,
    tof_s: f64,
    model: &PropagationModel,
    num_steps: usize,
) -> Result<FreeDriftAnalysis, FreeDriftError> {
    let trajectory = model.propagate_with_steps(
        departure_roe,
        chief_mean,
        departure_epoch,
        tof_s,
        num_steps,
    )?;

    let safety = analyze_trajectory_safety(&trajectory)?;
    let residual = bounded_motion_residual(departure_roe, chief_mean)?;

    Ok(FreeDriftAnalysis {
        trajectory,
        safety,
        bounded_motion_residual: residual,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::j2_params::compute_j2_params;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{
        damico_table21_case1_roe, damico_table21_chief, propagate_test_trajectory_at,
    };
    use crate::types::QuasiNonsingularROE;

    use hifitime::Epoch;

    /// Machine-epsilon accumulation tolerance for identity tests.
    /// Two propagations from identical initial state — only IEEE 754 rounding.
    const IDENTITY_TOL: f64 = 1e-14;
    /// Tolerance for bounded-motion residual when condition is exactly satisfied.
    /// Eq. 2.33 evaluates to zero analytically; residual from floating-point products.
    const RESIDUAL_TOL: f64 = 1e-12;
    /// Tolerance for gamma cross-check (algebraic identity, no approximation).
    const GAMMA_TOL: f64 = 1e-14;
    /// Minimum absolute residual that indicates secular δλ drift.
    /// Below this, bounded-motion condition is effectively satisfied.
    const DIVERGING_DRIFT_THRESHOLD: f64 = 1e-6;
    /// Tolerance for near-equatorial/polar degeneration tests.
    /// At i = ±0.001° from singularity, sin(2i) ≈ 3.5e-5 so the
    /// δix coupling contributes O(1e-9) to the residual.
    const DEGENERATION_TOL: f64 = 1e-8;

    fn test_epoch() -> Epoch {
        Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap()
    }

    /// Zero-burn identity: free-drift trajectory from the post-departure ROE
    /// should match a nominal propagation when departure Δv is zero
    /// (i.e. pre_departure_roe == post_departure_roe).
    #[test]
    fn test_zero_burn_identity() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;
        let period_s = chief.period().expect("valid chief orbit");
        let num_steps = 100;

        // "Nominal" propagation (same ROE, same model)
        let nominal = propagate_test_trajectory_at(&roe, &chief, epoch, period_s, num_steps);

        // Free-drift propagation (same ROE — zero-burn case)
        let free = compute_free_drift(&roe, &chief, epoch, period_s, &model, num_steps)
            .expect("free-drift propagation");

        assert_eq!(nominal.len(), free.trajectory.len());
        for (n, f) in nominal.iter().zip(free.trajectory.iter()) {
            let pos_diff = (n.ric.position_ric_km - f.ric.position_ric_km).norm();
            assert!(
                pos_diff < IDENTITY_TOL,
                "position mismatch at t={}: {pos_diff}",
                n.elapsed_s
            );
        }
    }

    /// Passively-stable residual: D'Amico Table 2.1 Case 1 has parallel e/i
    /// vectors with δa=0, δix=0. The bounded-motion condition (Eq. 2.33)
    /// should be satisfied (residual ≈ 0).
    #[test]
    fn test_passively_stable_residual() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();

        // Case 1: δa = 0, δix = 0 → residual = γ·sin(2i)·0 + 0/7 = 0
        let residual = bounded_motion_residual(&roe, &chief).expect("residual computation");
        assert!(
            residual.abs() < RESIDUAL_TOL,
            "passively-stable residual should be near zero, got {residual}"
        );
    }

    /// Diverging drift: inject δa ≠ 0 → residual nonzero.
    #[test]
    fn test_diverging_drift_residual() {
        let chief = damico_table21_chief();
        let mut roe = damico_table21_case1_roe();
        // Inject δa offset: 100m / a ≈ 1.41e-5
        roe.da = 100.0 / chief.a_km;

        let residual = bounded_motion_residual(&roe, &chief).expect("residual computation");
        // residual should be dominated by δa/7 term
        assert!(
            residual.abs() > DIVERGING_DRIFT_THRESHOLD,
            "diverging drift residual should be > {DIVERGING_DRIFT_THRESHOLD}, got {residual}"
        );
    }

    /// γ cross-check: γ = κ / ((3/4)·n), where κ and n come from J2Params.
    #[test]
    fn test_gamma_cross_check() {
        let chief = damico_table21_chief();
        let j2p = compute_j2_params(&chief).expect("j2 params");

        // γ from definition: (J2/2) · (R_E/a)² · (1/η⁴)
        let r_over_a = R_EARTH / chief.a_km;
        let eta4 = j2p.eta.powi(4);
        let gamma_direct = (J2 / 2.0) * r_over_a * r_over_a / eta4;

        // γ from κ: κ = (3/4)·n·J2·(R_E/p)² → γ = κ / ((3/4)·n)
        // But κ uses p = a(1-e²) while γ uses a and η⁴.
        // Cross-check: κ/(0.75·n) should equal J2·(R_E/p)² = J2·(R_E/(a·η²))²
        //   = J2·R_E²/(a²·η⁴) = 2·γ
        // So γ = κ / (1.5·n)
        let gamma_from_kappa = j2p.kappa / (1.5 * j2p.n_rad_s);

        assert!(
            (gamma_direct - gamma_from_kappa).abs() < GAMMA_TOL,
            "γ direct ({gamma_direct}) ≠ γ from κ ({gamma_from_kappa})"
        );
    }

    /// Near-equatorial: i ≈ 0 → sin(2i) ≈ 0, residual dominated by δa/7.
    /// The sin(2i)·δix contribution is suppressed but nonzero at 0.001°.
    /// Tolerance: sin(2·0.001°) ≈ 3.5e-5 → γ·sin(2i)·δix ≈ O(1e-9).
    #[test]
    fn test_near_equatorial_residual() {
        let mut chief = damico_table21_chief();
        chief.i_rad = 0.001_f64.to_radians(); // near-equatorial

        let mut roe = QuasiNonsingularROE::default();
        roe.da = 100.0 / chief.a_km;
        roe.dix = 200.0 / chief.a_km;

        let residual = bounded_motion_residual(&roe, &chief).expect("residual");
        let expected = roe.da / BOUNDED_MOTION_DA_DIVISOR;

        assert!(
            (residual - expected).abs() < DEGENERATION_TOL,
            "near-equatorial: residual ({residual}) should ≈ δa/7 ({expected})"
        );
    }

    /// Near-polar: i ≈ π/2 → sin(2i) ≈ 0, same degeneration as equatorial.
    #[test]
    fn test_near_polar_residual() {
        let mut chief = damico_table21_chief();
        chief.i_rad = std::f64::consts::FRAC_PI_2 - 0.001_f64.to_radians(); // near-polar

        let mut roe = QuasiNonsingularROE::default();
        roe.da = 100.0 / chief.a_km;
        roe.dix = 200.0 / chief.a_km;

        let residual = bounded_motion_residual(&roe, &chief).expect("residual");
        let expected = roe.da / BOUNDED_MOTION_DA_DIVISOR;

        assert!(
            (residual - expected).abs() < DEGENERATION_TOL,
            "near-polar: residual ({residual}) should ≈ δa/7 ({expected})"
        );
    }

    /// Zero num_steps → propagation error (ZeroSteps).
    #[test]
    fn test_zero_num_steps() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;

        let result = compute_free_drift(&roe, &chief, epoch, 1000.0, &model, 0);
        assert!(result.is_err(), "num_steps=0 should fail");
        assert!(
            matches!(result.unwrap_err(), FreeDriftError::Propagation(_)),
            "should be Propagation error"
        );
    }

    /// Safety metrics populated: free-drift analysis returns valid SafetyMetrics.
    #[test]
    fn test_safety_metrics_populated() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;

        let analysis =
            compute_free_drift(&roe, &chief, epoch, chief.period().expect("valid chief orbit"), &model, 100)
                .expect("free-drift");

        // Safety metrics should be populated with real values
        assert!(analysis.safety.operational.min_distance_3d_km > 0.0);
        assert!(analysis.safety.operational.min_rc_separation_km >= 0.0);
        assert!(analysis.safety.passive.de_magnitude >= 0.0);
        assert!(analysis.safety.passive.di_magnitude >= 0.0);
    }
}

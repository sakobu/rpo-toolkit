//! Gauss Variational Equations for impulsive maneuvers (D'Amico Eq. 2.38).
//!
//! Provides the B matrix mapping an impulsive Δv in the RIC frame to
//! instantaneous changes in quasi-nonsingular ROE.

use nalgebra::{SMatrix, Vector3};

use crate::elements::conversions::{validate_elements, ConversionError};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

/// Compute the 6×3 GVE control input matrix (D'Amico Eq. 2.38).
///
/// Maps `[δv_R, δv_I, δv_C]` → `[δ(δa), δ(δλ), δ(δex), δ(δey), δ(δix), δ(δiy)]`.
///
/// This is the instantaneous B matrix for near-circular orbits evaluated at
/// the chief mean argument of latitude `u`.
///
/// # Arguments
/// * `chief` - Chief Keplerian elements (provides `a`, `n`, and `u`)
///
/// # Invariants
/// - `chief.a_km > 0` (appears as `n * a` in denominator; zero → division by zero)
/// - Near-circular assumption: derived for `e ≈ 0`; accuracy degrades for `e > 0.1`
///
/// # Errors
/// Returns `ConversionError::InvalidSemiMajorAxis` if `chief.a_km <= 0`.
/// Returns `ConversionError::InvalidEccentricity` if `chief.e` is outside [0, 1).
pub fn compute_b_matrix(chief: &KeplerianElements) -> Result<SMatrix<f64, 6, 3>, ConversionError> {
    validate_elements(chief)?;
    let a = chief.a_km;
    let n = chief.mean_motion();
    let u = chief.mean_arg_of_lat();
    let (sin_u, cos_u) = u.sin_cos();
    let inv_na = 1.0 / (n * a);

    let mut b = SMatrix::<f64, 6, 3>::zeros();

    // δ(δa): only affected by along-track (tangential) thrust
    // a·δa = 2·δv_t/n  →  δa = 2·δv_t/(n·a)
    b[(0, 1)] = 2.0 * inv_na;

    // δ(δλ): only affected by radial thrust (instantaneous, drift term = 0)
    // a·δλ = -2·δv_r/n  →  δλ = -2·δv_r/(n·a)
    b[(1, 0)] = -2.0 * inv_na;

    // δ(δex): radial + along-track
    // a·δex = δv_r·sin(u)/n + 2·δv_t·cos(u)/n
    b[(2, 0)] = sin_u * inv_na;
    b[(2, 1)] = 2.0 * cos_u * inv_na;

    // δ(δey): radial + along-track
    // a·δey = -δv_r·cos(u)/n + 2·δv_t·sin(u)/n
    b[(3, 0)] = -cos_u * inv_na;
    b[(3, 1)] = 2.0 * sin_u * inv_na;

    // δ(δix): only cross-track
    // a·δix = δv_n·cos(u)/n
    b[(4, 2)] = cos_u * inv_na;

    // δ(δiy): only cross-track
    // a·δiy = δv_n·sin(u)/n
    b[(5, 2)] = sin_u * inv_na;

    Ok(b)
}

/// Apply an impulsive maneuver to a ROE state using the GVE B matrix.
///
/// Returns the post-maneuver ROE state.
///
/// # Arguments
/// * `roe` - Pre-maneuver ROE state
/// * `dv_ric` - Δv in RIC frame (km/s): [radial, in-track, cross-track]
/// * `chief` - Chief Keplerian elements at maneuver epoch
///
/// # Invariants
/// - `chief.a_km > 0` (delegates to `compute_b_matrix`)
/// - Near-circular assumption: accurate for `chief.e < ~0.1`
/// - `chief` must be at the maneuver epoch
///
/// # Errors
/// Returns `ConversionError` if `chief` has invalid SMA or eccentricity.
pub fn apply_maneuver(
    roe: &QuasiNonsingularROE,
    dv_ric: &Vector3<f64>,
    chief: &KeplerianElements,
) -> Result<QuasiNonsingularROE, ConversionError> {
    let b = compute_b_matrix(chief)?;
    let delta_roe = b * dv_ric;
    let new_roe = roe.to_vector() + delta_roe;
    Ok(QuasiNonsingularROE::from_vector(&new_roe))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;

    #[test]
    fn b_matrix_negative_sma_returns_error() {
        let chief = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = compute_b_matrix(&chief);
        assert!(
            matches!(result, Err(crate::elements::conversions::ConversionError::InvalidSemiMajorAxis { .. })),
            "Negative SMA should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    /// Along-track Δv should primarily change δa and δex/δey.
    #[test]
    fn along_track_dv_changes_da() {
        let chief = iss_like_elements();
        let b = compute_b_matrix(&chief).unwrap();

        // Pure along-track impulse
        let dv = Vector3::new(0.0, 0.001, 0.0); // 1 m/s along-track
        let delta = b * dv;

        // δa should be positive (raising orbit)
        assert!(
            delta[0] > 0.0,
            "Along-track Δv should increase δa, got {}",
            delta[0]
        );
        // δix and δiy should be zero (decoupled)
        assert!(delta[4].abs() < 1e-15, "δix should be zero");
        assert!(delta[5].abs() < 1e-15, "δiy should be zero");
    }

    /// Radial Δv should change δλ and δex/δey but NOT δa.
    #[test]
    fn radial_dv_does_not_change_da() {
        let chief = iss_like_elements();
        let b = compute_b_matrix(&chief).unwrap();

        let dv = Vector3::new(0.001, 0.0, 0.0); // 1 m/s radial
        let delta = b * dv;

        // δa should be zero for pure radial
        assert!(delta[0].abs() < 1e-15, "Radial Δv should not change δa");
        // δλ should be nonzero
        assert!(delta[1].abs() > 1e-10, "Radial Δv should change δλ");
    }

    /// Cross-track Δv should only affect δix and δiy.
    #[test]
    fn cross_track_dv_changes_only_di() {
        let chief = iss_like_elements();
        let b = compute_b_matrix(&chief).unwrap();

        let dv = Vector3::new(0.0, 0.0, 0.001); // 1 m/s cross-track
        let delta = b * dv;

        // In-plane should be zero
        assert!(delta[0].abs() < 1e-15, "δa should be zero for cross-track");
        assert!(delta[1].abs() < 1e-15, "δλ should be zero for cross-track");
        assert!(delta[2].abs() < 1e-15, "δex should be zero for cross-track");
        assert!(delta[3].abs() < 1e-15, "δey should be zero for cross-track");

        // Out-of-plane should be nonzero
        let di_mag = (delta[4] * delta[4] + delta[5] * delta[5]).sqrt();
        assert!(di_mag > 1e-10, "Cross-track Δv should change δi vector");
    }

    /// In-plane / out-of-plane decoupling.
    #[test]
    fn inplane_outofplane_decoupled() {
        let chief = iss_like_elements();
        let b = compute_b_matrix(&chief).unwrap();

        // B matrix should have zeros enforcing decoupling:
        // Rows 4,5 (δix, δiy) columns 0,1 (radial, along-track) = 0
        assert!(b[(4, 0)].abs() < 1e-15);
        assert!(b[(4, 1)].abs() < 1e-15);
        assert!(b[(5, 0)].abs() < 1e-15);
        assert!(b[(5, 1)].abs() < 1e-15);

        // Rows 0-3 (δa, δλ, δex, δey) column 2 (cross-track) = 0
        assert!(b[(0, 2)].abs() < 1e-15);
        assert!(b[(1, 2)].abs() < 1e-15);
        assert!(b[(2, 2)].abs() < 1e-15);
        assert!(b[(3, 2)].abs() < 1e-15);
    }

    /// D'Amico Eq. 2.52: optimal two-impulse Δv for eccentricity change.
    /// At u=0: along-track Δv gives `δex = 2·Δv/(n·a)`, so `Δv = n·a·δex/2`.
    /// Verifies B-matrix application matches the analytical formula.
    #[test]
    fn damico_eq252_optimal_dv_cost() {
        use crate::test_helpers::damico_table21_chief;

        let chief = damico_table21_chief(); // aop=0, M=0 → u=0
        let a = chief.a_km;
        let n = chief.mean_motion();

        // Target: δex corresponding to 400m physical separation
        let target_dex = 0.400 / a; // dimensionless

        // D'Amico Eq. 2.52: Δv_I = n·a·|δex|/2 for optimal eccentricity change at u=0
        let dv_analytical = n * a * target_dex / 2.0;

        // Apply via B-matrix: B * [0, Δv_I, 0]^T
        let b = compute_b_matrix(&chief).unwrap();
        let dv = Vector3::new(0.0, dv_analytical, 0.0);
        let delta_roe = b * dv;

        // δex from B-matrix should match target
        assert!(
            (delta_roe[2] - target_dex).abs() < 1e-10,
            "δex from B-matrix = {}, expected {target_dex}",
            delta_roe[2]
        );

        // At u=0: B[2,1] = 2·cos(0)/(n·a) = 2/(n·a), so δex = 2·Δv/(n·a) = target_dex ✓
        let expected_b21 = 2.0 / (n * a);
        assert!(
            (b[(2, 1)] - expected_b21).abs() < 1e-15,
            "B[2,1] = {}, expected {expected_b21}",
            b[(2, 1)]
        );

        // Structural: B[0,0]=0 means pure radial Δv does NOT change δa.
        // But along-track Δv DOES change δa: B[0,1]=2/(n·a), so δa = 2·Δv/(n·a).
        let expected_da = 2.0 * dv_analytical / (n * a);
        assert!(
            (delta_roe[0] - expected_da).abs() < 1e-10,
            "δa from along-track Δv = {}, expected {expected_da}",
            delta_roe[0]
        );

        // Verify B[0,0]=0: radial Δv does NOT change δa
        assert!(
            b[(0, 0)].abs() < 1e-15,
            "B[0,0] should be 0 (radial Δv doesn't change δa), got {}",
            b[(0, 0)]
        );

        // δey at u=0: B[3,1] = 2·sin(0)/(n·a) = 0
        assert!(
            delta_roe[3].abs() < 1e-15,
            "δey should be zero at u=0, got {}",
            delta_roe[3]
        );

        // Out-of-plane should be zero (decoupled)
        assert!(delta_roe[4].abs() < 1e-15, "δix should be zero");
        assert!(delta_roe[5].abs() < 1e-15, "δiy should be zero");
    }

    /// Zero Δv should not change ROE.
    #[test]
    fn zero_dv_identity() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0002,
            dey: -0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let result = apply_maneuver(&roe, &Vector3::zeros(), &chief).unwrap();
        let diff = (result.to_vector() - roe.to_vector()).norm();
        assert!(diff < 1e-15, "Zero Δv should not change ROE, diff={diff}");
    }

    /// Apply + reverse maneuver should recover original ROE.
    #[test]
    fn apply_reverse_roundtrip() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let dv = Vector3::new(0.001, 0.002, 0.0005);
        let after_fwd = apply_maneuver(&roe, &dv, &chief).unwrap();
        let after_rev = apply_maneuver(&after_fwd, &(-dv), &chief).unwrap();

        let diff = (after_rev.to_vector() - roe.to_vector()).norm();
        assert!(
            diff < 1e-14,
            "Apply+reverse should recover original ROE, diff={diff}"
        );
    }
}

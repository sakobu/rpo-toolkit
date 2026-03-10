//! J2-perturbed quasi-nonsingular state transition matrix (Koenig Eq. A6).
//!
//! Implements the closed-form STM for propagating relative orbital elements
//! under J2-perturbed motion, directly from the Koenig et al. (JGCD 2017)
//! Appendix A formulas.

use nalgebra::SMatrix;

use crate::constants::TWO_PI;
use crate::propagation::j2_params::{compute_j2_params, J2Params};
use crate::propagation::propagator::PropagationError;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

/// 6×6 matrix type alias.
type Matrix6 = SMatrix<f64, 6, 6>;

/// Compute the closed-form J2-perturbed QNS STM (Koenig Eq. A6).
///
/// Directly implements the state transition matrix `Φ_J2^qns` from the paper's
/// Appendix A, using substitutions from Eqs. A1-A2 and auxiliaries from Eqs. 14-16.
///
/// # Arguments
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `tau` - Propagation time (seconds)
///
/// # Invariants
/// - `chief_mean.a_km > 0`
/// - `0 <= chief_mean.e < 1`
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `tau` must be finite
///
/// # Errors
/// Returns `PropagationError` if eccentricity or SMA are out of range.
pub fn compute_stm(chief_mean: &KeplerianElements, tau: f64) -> Result<Matrix6, PropagationError> {
    let j2p = compute_j2_params(chief_mean)?;
    Ok(compute_stm_with_params(&j2p, chief_mean, tau))
}

/// Compute the J2-perturbed QNS STM using pre-computed [`J2Params`] (Koenig Eq. A6).
///
/// Use this variant when you already have [`J2Params`] to avoid recomputing them.
///
/// # Arguments
/// * `j2p` - Pre-computed J2 perturbation parameters
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `tau` - Propagation time (seconds)
///
/// # Invariants
/// - `j2p` must correspond to `chief_mean` (caller responsibility)
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `tau` must be finite
#[must_use]
#[allow(clippy::similar_names)]
pub fn compute_stm_with_params(j2p: &J2Params, chief_mean: &KeplerianElements, tau: f64) -> Matrix6 {
    let n = j2p.n_rad_s;
    let kappa = j2p.kappa;
    let e = chief_mean.e;
    let big_e = j2p.big_e;
    let big_f = j2p.big_f; // F = 4 + 3η
    let big_g = j2p.big_g; // G = 1/(η²(1+η))
    let big_p = j2p.big_p; // P = 3cos²i - 1
    let big_q = j2p.big_q; // Q = 5cos²i - 1
    let big_s = j2p.big_s; // S = sin(2i)
    let big_t = j2p.big_t; // T = sin²(i)

    // Eq. A1: secular angle rates and propagated angles
    let omega_dot = j2p.aop_dot_rad_s; // ω̇ = κQ
    let omega_i = chief_mean.aop_rad;
    let omega_f = omega_i + omega_dot * tau;
    let d_omega = omega_dot * tau; // ω̇τ

    // Eq. A2: eccentricity vector components at initial and final times
    let ex_i = j2p.ex;
    let ey_i = j2p.ey;
    let ex_f = e * omega_f.cos();
    let ey_f = e * omega_f.sin();

    // Build the STM (Eq. A6)
    let mut phi = Matrix6::zeros();

    // Row 0: δa is constant
    phi[(0, 0)] = 1.0;

    // Row 1: δλ (relative mean longitude)
    phi[(1, 0)] = -(1.5 * n + 3.5 * kappa * big_e * big_p) * tau;
    phi[(1, 1)] = 1.0;
    phi[(1, 2)] = kappa * ex_i * big_f * big_g * big_p * tau;
    phi[(1, 3)] = kappa * ey_i * big_f * big_g * big_p * tau;
    phi[(1, 4)] = -kappa * big_f * big_s * tau;
    phi[(1, 5)] = 0.0;

    // Row 2: δex (relative eccentricity vector x)
    phi[(2, 0)] = 3.5 * kappa * ey_f * big_q * tau;
    phi[(2, 1)] = 0.0;
    phi[(2, 2)] = d_omega.cos() - 4.0 * kappa * ex_i * ey_f * big_g * big_q * tau;
    phi[(2, 3)] = -d_omega.sin() - 4.0 * kappa * ey_i * ey_f * big_g * big_q * tau;
    phi[(2, 4)] = 5.0 * kappa * ey_f * big_s * tau;
    phi[(2, 5)] = 0.0;

    // Row 3: δey (relative eccentricity vector y)
    phi[(3, 0)] = -3.5 * kappa * ex_f * big_q * tau;
    phi[(3, 1)] = 0.0;
    phi[(3, 2)] = d_omega.sin() + 4.0 * kappa * ex_i * ex_f * big_g * big_q * tau;
    phi[(3, 3)] = d_omega.cos() + 4.0 * kappa * ey_i * ex_f * big_g * big_q * tau;
    phi[(3, 4)] = -5.0 * kappa * ex_f * big_s * tau;
    phi[(3, 5)] = 0.0;

    // Row 4: δix is constant
    phi[(4, 4)] = 1.0;

    // Row 5: δiy (relative inclination vector y)
    phi[(5, 0)] = 3.5 * kappa * big_s * tau;
    phi[(5, 1)] = 0.0;
    phi[(5, 2)] = -4.0 * kappa * ex_i * big_g * big_s * tau;
    phi[(5, 3)] = -4.0 * kappa * ey_i * big_g * big_s * tau;
    phi[(5, 4)] = 2.0 * kappa * big_t * tau;
    phi[(5, 5)] = 1.0;

    phi
}

/// Propagate ROE using the J2-perturbed STM.
///
/// Returns the propagated ROE and the propagated chief mean elements.
///
/// # Arguments
/// * `roe` - Initial quasi-nonsingular ROE
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `tau` - Propagation time (seconds)
///
/// # Invariants
/// - `chief_mean.a_km > 0`
/// - `0 <= chief_mean.e < 1`
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - ROE must satisfy linearization validity (`dimensionless_norm() < ~0.01`)
///
/// # Errors
/// Returns `PropagationError` if eccentricity or SMA are out of range.
pub fn propagate_roe_stm(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Result<(QuasiNonsingularROE, KeplerianElements), PropagationError> {
    let j2p = compute_j2_params(chief_mean)?;
    let stm = compute_stm_with_params(&j2p, chief_mean, tau);
    let roe_vec = roe.to_vector();
    let propagated = stm * roe_vec;

    let chief_prop = propagate_chief_mean(chief_mean, &j2p, tau);

    Ok((QuasiNonsingularROE::from_vector(&propagated), chief_prop))
}

/// Advance chief mean elements by secular J2 rates.
///
/// Updates M, ω, Ω using the secular rates from J2 perturbations.
///
/// # Invariants
/// - `j2p` must correspond to `chief` (caller responsibility)
/// - `chief` must be **mean** Keplerian elements, not osculating
/// - `tau` must be finite
#[must_use]
pub fn propagate_chief_mean(
    chief: &KeplerianElements,
    j2p: &J2Params,
    tau: f64,
) -> KeplerianElements {
    KeplerianElements {
        a_km: chief.a_km,
        e: chief.e,
        i_rad: chief.i_rad,
        raan_rad: (chief.raan_rad + j2p.raan_dot_rad_s * tau).rem_euclid(TWO_PI),
        aop_rad: (chief.aop_rad + j2p.aop_dot_rad_s * tau).rem_euclid(TWO_PI),
        mean_anomaly_rad: (chief.mean_anomaly_rad + j2p.m_dot_rad_s * tau).rem_euclid(TWO_PI),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::roe::wrap_angle;
    use crate::test_helpers::{
        iss_like_elements, koenig_table2_case1, koenig_table2_case1_roe, koenig_table2_case2,
    };

    #[test]
    fn zero_tau_gives_near_identity() {
        let chief = iss_like_elements();
        let stm = compute_stm(&chief, 0.0).unwrap();
        let identity = Matrix6::identity();

        let diff = (stm - identity).norm();
        assert!(
            diff < 1e-10,
            "STM at tau=0 should be near identity, diff={diff}"
        );
    }

    #[test]
    fn forward_backward_symmetry() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let tau = 3600.0; // 1 hour
        let (roe_fwd, chief_fwd) = propagate_roe_stm(&roe, &chief, tau).unwrap();
        let (roe_back, _) = propagate_roe_stm(&roe_fwd, &chief_fwd, -tau).unwrap();

        assert!(
            (roe_back.da - roe.da).abs() < 1e-8,
            "da not recovered: {} vs {}",
            roe_back.da,
            roe.da
        );
        assert!(
            wrap_angle(roe_back.dlambda - roe.dlambda).abs() < 1e-6,
            "dlambda not recovered: {} vs {}",
            roe_back.dlambda,
            roe.dlambda
        );
        assert!(
            (roe_back.dex - roe.dex).abs() < 1e-8,
            "dex not recovered"
        );
        assert!(
            (roe_back.dey - roe.dey).abs() < 1e-8,
            "dey not recovered"
        );
        assert!(
            (roe_back.dix - roe.dix).abs() < 1e-8,
            "dix not recovered"
        );
        assert!(
            (roe_back.diy - roe.diy).abs() < 1e-8,
            "diy not recovered"
        );
    }

    #[test]
    fn pure_da_keplerian_drift() {
        // Pure δa should produce along-track drift: δλ(t) ≈ δλ(0) - 3/2·n·δa·t
        let chief = KeplerianElements {
            a_km: 7000.0,
            e: 0.001,
            i_rad: 0.1_f64.to_radians(), // near-equatorial to minimize J2 coupling
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };

        let da = 0.001 / chief.a_km; // very small δa to stay linear
        let roe = QuasiNonsingularROE {
            da,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let n = chief.mean_motion();
        let tau = 1000.0; // short enough to stay in linear regime

        let (roe_prop, _) = propagate_roe_stm(&roe, &chief, tau).unwrap();

        // Expected drift includes both Keplerian and J2 terms:
        // Φ[1,0] = -(3/2·n + 7/2·κ·E·P)·τ
        let j2p = compute_j2_params(&chief).unwrap();
        let expected_dlambda =
            -(1.5 * n + 3.5 * j2p.kappa * j2p.big_e * j2p.big_p) * tau * da;

        assert!(
            (roe_prop.dlambda - expected_dlambda).abs()
                < (expected_dlambda.abs() * 0.1).max(1e-10),
            "Along-track drift mismatch: got {}, expected {expected_dlambda}",
            roe_prop.dlambda
        );

        // δa should remain constant
        assert!(
            (roe_prop.da - da).abs() < 1e-12,
            "da should be constant: {} vs {da}",
            roe_prop.da
        );
    }

    #[test]
    fn stm_preserves_da() {
        let chief = iss_like_elements();
        let da = 1.0 / chief.a_km;
        let roe = QuasiNonsingularROE {
            da,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let period = TWO_PI / chief.mean_motion();
        let (roe_prop, _) = propagate_roe_stm(&roe, &chief, period).unwrap();

        assert!(
            (roe_prop.da - da).abs() < 1e-8,
            "da should be preserved over one orbit: {} vs {da}",
            roe_prop.da
        );
    }

    // --- Paper-traced regression tests (Koenig Eq. A6) ---

    /// Koenig Eq. A6: STM structure for Table 2 Case 1 (i=30°, e=0.005), tau = 1 period.
    /// Validates row 0/row 4 identity structure and key matrix entries against
    /// hand-computed values from Eq. A6 using precomputed J2Params.
    #[test]
    fn koenig_eqa6_stm_elements_case1() {
        let chief = koenig_table2_case1();
        let period = chief.period();
        let j2p = compute_j2_params(&chief).unwrap();
        let stm = compute_stm(&chief, period).unwrap();

        // Row 0: δa is conserved → phi[0,0]=1, all others = 0
        assert!((stm[(0, 0)] - 1.0).abs() < 1e-14, "phi[0,0] should be 1.0");
        for c in 1..6 {
            assert!(stm[(0, c)].abs() < 1e-14, "phi[0,{c}] should be 0, got {}", stm[(0, c)]);
        }

        // Row 4: δix is conserved → phi[4,4]=1, all others = 0
        assert!((stm[(4, 4)] - 1.0).abs() < 1e-14, "phi[4,4] should be 1.0");
        for c in 0..6 {
            if c != 4 {
                assert!(stm[(4, c)].abs() < 1e-14, "phi[4,{c}] should be 0, got {}", stm[(4, c)]);
            }
        }

        // phi[1,0] = -(1.5·n + 3.5·κ·E·P)·τ
        let expected_10 = -(1.5 * j2p.n_rad_s + 3.5 * j2p.kappa * j2p.big_e * j2p.big_p) * period;
        assert!(
            (stm[(1, 0)] - expected_10).abs() < 1e-6,
            "phi[1,0]: got {}, expected {expected_10}", stm[(1, 0)]
        );

        // phi[5,0] = 3.5·κ·S·τ
        let expected_50 = 3.5 * j2p.kappa * j2p.big_s * period;
        assert!(
            (stm[(5, 0)] - expected_50).abs() < 1e-6,
            "phi[5,0]: got {}, expected {expected_50}", stm[(5, 0)]
        );

        // phi[2,2] ≈ cos(ω̇·τ) at leading order (ey_i ≈ 0 so secular term vanishes)
        let d_omega = j2p.aop_dot_rad_s * period;
        assert!(
            (stm[(2, 2)] - d_omega.cos()).abs() < 1e-4,
            "phi[2,2]: got {}, expected ~cos(dω)={}", stm[(2, 2)], d_omega.cos()
        );

        // phi[1,3] ≈ 0 because ey_i ≈ 0 at aop=180°
        assert!(
            stm[(1, 3)].abs() < 1e-12,
            "phi[1,3] should be ~0 (ey_i≈0), got {}", stm[(1, 3)]
        );

        // --- Velocity block entries (Eq. A6 rows 2-3, 5) ---

        // phi[2,3] = -sin(ω̇τ) - 4κ·ey_i·ey_f·G·Q·τ
        // Since ey_i ≈ 0 (aop=180°), the secular term vanishes: phi[2,3] ≈ -sin(dω)
        let omega_dot = j2p.aop_dot_rad_s;
        let omega_f = chief.aop_rad + omega_dot * period;
        let ey_f = chief.e * omega_f.sin();
        let ex_f = chief.e * omega_f.cos();

        let expected_23 = -d_omega.sin()
            - 4.0 * j2p.kappa * j2p.ey * ey_f * j2p.big_g * j2p.big_q * period;
        assert!(
            (stm[(2, 3)] - expected_23).abs() < 1e-6,
            "phi[2,3]: got {}, expected {expected_23}", stm[(2, 3)]
        );

        // phi[3,2] = sin(ω̇τ) + 4κ·ex_i·ex_f·G·Q·τ
        let expected_32 = d_omega.sin()
            + 4.0 * j2p.kappa * j2p.ex * ex_f * j2p.big_g * j2p.big_q * period;
        assert!(
            (stm[(3, 2)] - expected_32).abs() < 1e-6,
            "phi[3,2]: got {}, expected {expected_32}", stm[(3, 2)]
        );

        // phi[3,3] = cos(ω̇τ) + 4κ·ey_i·ex_f·G·Q·τ
        let expected_33 = d_omega.cos()
            + 4.0 * j2p.kappa * j2p.ey * ex_f * j2p.big_g * j2p.big_q * period;
        assert!(
            (stm[(3, 3)] - expected_33).abs() < 1e-6,
            "phi[3,3]: got {}, expected {expected_33}", stm[(3, 3)]
        );

        // phi[5,5] = 1.0 (δiy diagonal, identity)
        assert!(
            (stm[(5, 5)] - 1.0).abs() < 1e-14,
            "phi[5,5] should be 1.0, got {}", stm[(5, 5)]
        );

        // phi[5,2] = -4κ·ex_i·G·S·τ
        let expected_52 = -4.0 * j2p.kappa * j2p.ex * j2p.big_g * j2p.big_s * period;
        assert!(
            (stm[(5, 2)] - expected_52).abs() < 1e-6,
            "phi[5,2]: got {}, expected {expected_52}", stm[(5, 2)]
        );

        // phi[5,3] = -4κ·ey_i·G·S·τ ≈ 0 (ey_i ≈ 0)
        let expected_53 = -4.0 * j2p.kappa * j2p.ey * j2p.big_g * j2p.big_s * period;
        assert!(
            (stm[(5, 3)] - expected_53).abs() < 1e-12,
            "phi[5,3]: got {}, expected ~0 (ey_i≈0)", stm[(5, 3)]
        );
    }

    /// Koenig Eq. A6: STM for Table 2 Case 2 (e=0.2, aop=120°).
    /// Nonzero ey_i = 0.2·sin(120°) ≈ 0.1732 makes phi[1,3] substantial.
    #[test]
    fn koenig_eqa6_stm_elements_case2() {
        let chief = koenig_table2_case2();
        let period = chief.period();
        let j2p = compute_j2_params(&chief).unwrap();
        let stm = compute_stm(&chief, period).unwrap();

        // Verify ey_i is substantial (nonzero aop)
        assert!(
            j2p.ey.abs() > 0.17,
            "ey_i should be ~0.1732, got {}", j2p.ey
        );

        // phi[1,3] = κ·ey_i·F·G·P·τ should be nonzero
        let expected_13 = j2p.kappa * j2p.ey * j2p.big_f * j2p.big_g * j2p.big_p * period;
        assert!(
            (stm[(1, 3)] - expected_13).abs() < 1e-6,
            "phi[1,3]: got {}, expected {expected_13}", stm[(1, 3)]
        );

        // phi[1,2] = κ·ex_i·F·G·P·τ should also be nonzero (ex_i = -0.1)
        let expected_12 = j2p.kappa * j2p.ex * j2p.big_f * j2p.big_g * j2p.big_p * period;
        assert!(
            (stm[(1, 2)] - expected_12).abs() < 1e-6,
            "phi[1,2]: got {}, expected {expected_12}", stm[(1, 2)]
        );

        // Both ex and ey columns contribute substantially
        assert!(
            stm[(1, 2)].abs() > 1e-3,
            "phi[1,2] should be substantial, got {}", stm[(1, 2)]
        );
        assert!(
            stm[(1, 3)].abs() > 1e-3,
            "phi[1,3] should be substantial, got {}", stm[(1, 3)]
        );
    }

    /// Koenig Table 4: 10-orbit propagation bounds for Case 1.
    /// Verifies δa conservation and successful propagation over long intervals.
    /// Koenig Table 4 reports J2 STM errors of 38.5m (δa), 1808.8m (δλ) over
    /// 10 orbits vs. 20×20 numerical integrator; our STM is self-consistent
    /// and these errors represent linearization + truncation limits.
    #[test]
    fn koenig_table4_propagation_bounds_case1() {
        let chief = koenig_table2_case1();
        let roe = koenig_table2_case1_roe();
        let period = chief.period();
        let tau = 10.0 * period;

        let (roe_prop, _) = propagate_roe_stm(&roe, &chief, tau).unwrap();

        // δa is a first-order constant in the J2 STM → should be conserved
        assert!(
            (roe_prop.da - roe.da).abs() < 1e-8,
            "δa should be conserved over 10 orbits: initial={}, final={}",
            roe.da, roe_prop.da
        );

        // δix is also conserved (row 4 identity)
        assert!(
            (roe_prop.dix - roe.dix).abs() < 1e-8,
            "δix should be conserved over 10 orbits: initial={}, final={}",
            roe.dix, roe_prop.dix
        );

        // Eccentricity vector magnitude should be approximately conserved
        // (it rotates due to J2 but magnitude changes are second-order)
        let de_initial = (roe.dex.powi(2) + roe.dey.powi(2)).sqrt();
        let de_final = (roe_prop.dex.powi(2) + roe_prop.dey.powi(2)).sqrt();
        assert!(
            (de_final - de_initial).abs() / de_initial < 0.01,
            "δe magnitude should be ~conserved: initial={de_initial}, final={de_final}"
        );
    }
}

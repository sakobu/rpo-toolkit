//! J2-perturbed quasi-nonsingular state transition matrix (Koenig Eq. A6).
//!
//! Implements the closed-form STM for propagating relative orbital elements
//! under J2-perturbed motion, directly from the Koenig et al. (JGCD 2017)
//! Appendix A formulas.

use nalgebra::SMatrix;

use crate::constants::TWO_PI;
use crate::propagation::j2_params::{compute_j2_params, J2Params};
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
#[must_use]
pub fn compute_stm(chief_mean: &KeplerianElements, tau: f64) -> Matrix6 {
    let j2p = compute_j2_params(chief_mean);
    compute_stm_with_params(&j2p, chief_mean, tau)
}

/// Compute the J2-perturbed QNS STM using pre-computed [`J2Params`] (Koenig Eq. A6).
///
/// Use this variant when you already have [`J2Params`] to avoid recomputing them.
///
/// # Arguments
/// * `j2p` - Pre-computed J2 perturbation parameters
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `tau` - Propagation time (seconds)
#[must_use]
#[allow(clippy::similar_names)]
pub fn compute_stm_with_params(j2p: &J2Params, chief_mean: &KeplerianElements, tau: f64) -> Matrix6 {
    let n = j2p.n;
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
    let omega_dot = j2p.aop_dot; // ω̇ = κQ
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
#[must_use]
pub fn propagate_roe_stm(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> (QuasiNonsingularROE, KeplerianElements) {
    let j2p = compute_j2_params(chief_mean);
    let stm = compute_stm_with_params(&j2p, chief_mean, tau);
    let roe_vec = roe.to_vector();
    let propagated = stm * roe_vec;

    let chief_prop = propagate_chief_mean(chief_mean, &j2p, tau);

    (QuasiNonsingularROE::from_vector(&propagated), chief_prop)
}

/// Advance chief mean elements by secular J2 rates.
///
/// Updates M, ω, Ω using the secular rates from J2 perturbations.
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
        raan_rad: (chief.raan_rad + j2p.raan_dot * tau).rem_euclid(TWO_PI),
        aop_rad: (chief.aop_rad + j2p.aop_dot * tau).rem_euclid(TWO_PI),
        mean_anomaly_rad: (chief.mean_anomaly_rad + j2p.m_dot * tau).rem_euclid(TWO_PI),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::roe::wrap_angle;
    use crate::test_helpers::iss_like_elements;

    #[test]
    fn zero_tau_gives_near_identity() {
        let chief = iss_like_elements();
        let stm = compute_stm(&chief, 0.0);
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
        let (roe_fwd, chief_fwd) = propagate_roe_stm(&roe, &chief, tau);
        let (roe_back, _) = propagate_roe_stm(&roe_fwd, &chief_fwd, -tau);

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

        let (roe_prop, _) = propagate_roe_stm(&roe, &chief, tau);

        // Expected drift includes both Keplerian and J2 terms:
        // Φ[1,0] = -(3/2·n + 7/2·κ·E·P)·τ
        let j2p = compute_j2_params(&chief);
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
        let (roe_prop, _) = propagate_roe_stm(&roe, &chief, period);

        assert!(
            (roe_prop.da - da).abs() < 1e-8,
            "da should be preserved over one orbit: {} vs {da}",
            roe_prop.da
        );
    }
}

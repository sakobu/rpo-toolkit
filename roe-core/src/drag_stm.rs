//! J2 + differential drag quasi-nonsingular state transition matrix (Koenig Appendix D).
//!
//! Extends the J2-only 6×6 STM from `stm.rs` to a 9×9 STM that includes
//! density-model-free (DMF) differential drag effects. The augmented state
//! is `[δa, δλ, δex, δey, δix, δiy, δȧ, δėx, δėy]` where the last three
//! elements are constant drag rates (Koenig Sec. VIII, Eqs. 73-77).

use nalgebra::{SMatrix, SVector};

use crate::j2_params::{compute_j2_params, J2Params};
use crate::stm::{compute_stm_with_params, propagate_chief_mean};
use crate::types::{DragConfig, KeplerianElements, QuasiNonsingularROE};

/// 9×9 matrix type alias for the augmented J2+drag STM.
type Matrix9 = SMatrix<f64, 9, 9>;

/// Compute the J2+drag 9×9 QNS STM (Koenig Appendix D).
///
/// Convenience wrapper that computes [`J2Params`] internally.
///
/// # Arguments
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `drag` - DMF differential drag configuration
/// * `tau` - Propagation time (seconds)
#[must_use]
pub fn compute_j2_drag_stm(
    chief_mean: &KeplerianElements,
    _drag: &DragConfig,
    tau: f64,
) -> Matrix9 {
    let j2p = compute_j2_params(chief_mean);
    compute_j2_drag_stm_with_params(&j2p, chief_mean, tau)
}

/// Compute the J2+drag 9×9 QNS STM using pre-computed [`J2Params`] (Koenig Appendix D).
///
/// The 9×9 STM is structured as:
/// ```text
/// ┌─────────┬─────────┐
/// │  6×6    │  6×3    │
/// │  J2 STM │  Drag   │
/// │ (Eq.A6) │ (Eq.D2) │
/// ├─────────┼─────────┤
/// │  3×6    │  3×3    │
/// │  Zero   │ Identity│
/// └─────────┴─────────┘
/// ```
///
/// # Arguments
/// * `j2p` - Pre-computed J2 perturbation parameters
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `tau` - Propagation time (seconds)
#[must_use]
#[allow(clippy::similar_names)]
pub fn compute_j2_drag_stm_with_params(
    j2p: &J2Params,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Matrix9 {
    let kappa = j2p.kappa;
    let e = chief_mean.e;
    let n = j2p.n;
    let big_e = j2p.big_e;
    let big_f = j2p.big_f;
    let big_g = j2p.big_g;
    let big_p = j2p.big_p;
    let big_q = j2p.big_q;
    let big_s = j2p.big_s;

    let omega_f = chief_mean.aop + j2p.aop_dot * tau;
    let ex_f = e * omega_f.cos();
    let ey_f = e * omega_f.sin();

    let tau2 = tau * tau;

    let mut phi = Matrix9::zeros();

    // === Upper-left 6×6: reuse J2-only STM (Eq. A6) from stm.rs ===
    let j2_block = compute_stm_with_params(j2p, chief_mean, tau);
    phi.fixed_view_mut::<6, 6>(0, 0).copy_from(&j2_block);

    // === Upper-right 6×3: Drag coupling (Eq. D2) ===
    // Columns 6,7,8 correspond to δȧ, δėx, δėy

    // Row 0 (δa): only δȧ contributes
    phi[(0, 6)] = tau;

    // Row 1 (δλ): δȧ causes quadratic along-track drift
    phi[(1, 6)] = -(0.75 * n + 1.75 * kappa * big_e * big_p) * tau2;
    phi[(1, 7)] = 0.5 * kappa * big_f * big_g * big_p * e * tau2;

    // Row 2 (δex): drag coupling
    phi[(2, 6)] = 1.75 * kappa * ey_f * big_q * tau2;
    phi[(2, 7)] = omega_f.cos() * tau - 2.0 * kappa * e * ey_f * big_g * big_q * tau2;
    phi[(2, 8)] = -omega_f.sin() * tau;

    // Row 3 (δey): drag coupling
    phi[(3, 6)] = -1.75 * kappa * ex_f * big_q * tau2;
    phi[(3, 7)] = omega_f.sin() * tau + 2.0 * kappa * e * ex_f * big_g * big_q * tau2;
    phi[(3, 8)] = omega_f.cos() * tau;

    // Row 4 (δix): no drag coupling
    // (already zero)

    // Row 5 (δiy): drag coupling
    phi[(5, 6)] = 1.75 * kappa * big_s * tau2;
    phi[(5, 7)] = -2.0 * kappa * e * big_g * big_s * tau2;

    // === Lower-left 3×6: zeros (drag rates don't depend on ROE) ===
    // (already zero)

    // === Lower-right 3×3: identity (drag rates are constant) ===
    phi[(6, 6)] = 1.0;
    phi[(7, 7)] = 1.0;
    phi[(8, 8)] = 1.0;

    phi
}

/// Propagate ROE using the J2+drag 9×9 STM.
///
/// Returns the propagated ROE and the propagated chief mean elements.
/// The drag rates are treated as constant over the propagation interval.
///
/// # Arguments
/// * `roe` - Initial quasi-nonsingular ROE
/// * `chief_mean` - Chief mean Keplerian elements at epoch
/// * `drag` - DMF differential drag configuration
/// * `tau` - Propagation time (seconds)
#[must_use]
pub fn propagate_roe_j2_drag(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    drag: &DragConfig,
    tau: f64,
) -> (QuasiNonsingularROE, KeplerianElements) {
    let j2p = compute_j2_params(chief_mean);
    let stm = compute_j2_drag_stm_with_params(&j2p, chief_mean, tau);

    // Build 9-element augmented state vector
    let state = SVector::<f64, 9>::from_column_slice(&[
        roe.da,
        roe.dlambda,
        roe.dex,
        roe.dey,
        roe.dix,
        roe.diy,
        drag.da_dot,
        drag.dex_dot,
        drag.dey_dot,
    ]);

    let propagated = stm * state;

    let roe_prop =
        QuasiNonsingularROE::from_vector(&propagated.fixed_view::<6, 1>(0, 0).into_owned());

    let chief_prop = propagate_chief_mean(chief_mean, &j2p, tau);

    (roe_prop, chief_prop)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stm::compute_stm;
    use crate::test_helpers::{eccentric_elements, iss_like_elements, test_drag_config};

    #[test]
    fn zero_drag_equals_j2_stm() {
        let chief = iss_like_elements();
        let tau = 3600.0;
        let drag = DragConfig::zero();

        let stm_j2 = compute_stm(&chief, tau);
        let stm_drag = compute_j2_drag_stm(&chief, &drag, tau);

        // Upper-left 6x6 must match
        for r in 0..6 {
            for c in 0..6 {
                let diff = (stm_drag[(r, c)] - stm_j2[(r, c)]).abs();
                assert!(
                    diff < 1e-14,
                    "Mismatch at ({r},{c}): drag={}, j2={}, diff={diff}",
                    stm_drag[(r, c)],
                    stm_j2[(r, c)]
                );
            }
        }
    }

    #[test]
    fn zero_tau_gives_identity_9x9() {
        let chief = iss_like_elements();
        let drag = test_drag_config();

        let stm = compute_j2_drag_stm(&chief, &drag, 0.0);
        let identity = Matrix9::identity();

        let diff = (stm - identity).norm();
        assert!(
            diff < 1e-10,
            "STM at tau=0 should be near 9x9 identity, diff={diff}"
        );
    }

    #[test]
    fn forward_backward_symmetry_with_drag() {
        let chief = iss_like_elements();
        let drag = test_drag_config();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let tau = 3600.0;
        let (roe_fwd, chief_fwd) = propagate_roe_j2_drag(&roe, &chief, &drag, tau);
        let (roe_back, _) = propagate_roe_j2_drag(&roe_fwd, &chief_fwd, &drag, -tau);

        assert!(
            (roe_back.da - roe.da).abs() < 1e-6,
            "da not recovered: {} vs {}",
            roe_back.da,
            roe.da
        );
        assert!(
            (roe_back.dex - roe.dex).abs() < 1e-6,
            "dex not recovered: {} vs {}",
            roe_back.dex,
            roe.dex
        );
        assert!(
            (roe_back.dey - roe.dey).abs() < 1e-6,
            "dey not recovered"
        );
        assert!(
            (roe_back.dix - roe.dix).abs() < 1e-6,
            "dix not recovered"
        );
        assert!(
            (roe_back.diy - roe.diy).abs() < 1e-6,
            "diy not recovered"
        );
    }

    #[test]
    fn drag_produces_linear_da_drift() {
        let chief = iss_like_elements();
        let da_dot = -1e-10;
        let drag = DragConfig {
            da_dot,
            dex_dot: 0.0,
            dey_dot: 0.0,
        };
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let tau = 1000.0;
        let (roe_prop, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau);

        // da(τ) ≈ da_dot * τ (linear drift from drag)
        let expected_da = da_dot * tau;
        let rel_err = ((roe_prop.da - expected_da) / expected_da).abs();
        assert!(
            rel_err < 0.01,
            "da should drift linearly: got {}, expected {expected_da}, rel_err={rel_err}",
            roe_prop.da
        );
    }

    #[test]
    fn drag_produces_quadratic_dlambda() {
        let chief = iss_like_elements();
        let da_dot = -1e-10;
        let drag = DragConfig {
            da_dot,
            dex_dot: 0.0,
            dey_dot: 0.0,
        };
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let tau = 1000.0;
        let (roe_1, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau);
        let (roe_2, _) = propagate_roe_j2_drag(&roe, &chief, &drag, 2.0 * tau);

        // Quadratic: drift at 2τ ≈ 4× drift at τ
        let ratio = roe_2.dlambda / roe_1.dlambda;
        assert!(
            (ratio - 4.0).abs() < 0.5,
            "Along-track drift should be quadratic: ratio={ratio}, expected ~4.0"
        );
    }

    #[test]
    fn drag_drifts_eccentricity_vector() {
        let chief = iss_like_elements();
        let dex_dot = 1e-11;
        let dey_dot = -1e-11;
        let drag = DragConfig {
            da_dot: 0.0,
            dex_dot,
            dey_dot,
        };
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let tau = 1000.0;
        let (roe_prop, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau);

        // Eccentricity vector should have drifted from zero
        assert!(
            roe_prop.dex.abs() > 1e-15,
            "dex should have drifted, got {}",
            roe_prop.dex
        );
        assert!(
            roe_prop.dey.abs() > 1e-15,
            "dey should have drifted, got {}",
            roe_prop.dey
        );
    }

    #[test]
    fn eccentric_orbit_zero_drag() {
        let chief = eccentric_elements();
        let tau = 3600.0;
        let drag = DragConfig::zero();

        let stm_j2 = compute_stm(&chief, tau);
        let stm_drag = compute_j2_drag_stm(&chief, &drag, tau);

        for r in 0..6 {
            for c in 0..6 {
                let diff = (stm_drag[(r, c)] - stm_j2[(r, c)]).abs();
                assert!(
                    diff < 1e-14,
                    "Eccentric mismatch at ({r},{c}): drag={}, j2={}, diff={diff}",
                    stm_drag[(r, c)],
                    stm_j2[(r, c)]
                );
            }
        }
    }

    #[test]
    fn drag_config_serde_roundtrip() {
        let config = test_drag_config();
        let json = serde_json::to_string(&config).expect("serialize");
        let roundtrip: DragConfig = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(config, roundtrip);
    }
}

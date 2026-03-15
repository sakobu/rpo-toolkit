//! J2 + differential drag quasi-nonsingular state transition matrix (Koenig Appendix D).
//!
//! Extends the J2-only 6×6 STM from `stm.rs` to a 9×9 STM that includes
//! density-model-free (DMF) differential drag effects. The augmented state
//! is `[δa, δλ, δex, δey, δix, δiy, δȧ, δėx, δėy]` where the last three
//! elements are constant drag rates (Koenig Sec. VIII, Eqs. 73-77).

use nalgebra::SVector;

use crate::propagation::j2_params::{compute_j2_params, J2Params};
use crate::propagation::propagator::PropagationError;
use crate::propagation::stm::{compute_stm_with_params, propagate_chief_mean};
use super::propagator::DragConfig;
use crate::types::{KeplerianElements, Matrix9, QuasiNonsingularROE};

/// Compute the J2+drag 9×9 QNS STM (Koenig Appendix D).
///
/// Convenience wrapper that computes [`J2Params`] internally.
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
pub fn compute_j2_drag_stm(
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Result<Matrix9, PropagationError> {
    let j2p = compute_j2_params(chief_mean)?;
    Ok(compute_j2_drag_stm_with_params(&j2p, chief_mean, tau))
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
///
/// # Invariants
/// - `j2p` must correspond to `chief_mean` (caller responsibility)
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `tau` must be finite
#[must_use]
#[allow(clippy::similar_names)]
pub fn compute_j2_drag_stm_with_params(
    j2p: &J2Params,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Matrix9 {
    let kappa = j2p.kappa;
    let e = chief_mean.e;
    let n = j2p.n_rad_s;
    let big_e = j2p.big_e;
    let big_f = j2p.big_f;
    let big_g = j2p.big_g;
    let big_p = j2p.big_p;
    let big_q = j2p.big_q;
    let big_s = j2p.big_s;

    let omega_f = chief_mean.aop_rad + j2p.aop_dot_rad_s * tau;
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
///
/// # Invariants
/// - `chief_mean.a_km > 0`
/// - `0 <= chief_mean.e < 1`
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - Drag rates in `drag` are assumed constant over the propagation interval
/// - ROE must satisfy linearization validity (`dimensionless_norm() < ~0.01`)
///
/// # Errors
/// Returns `PropagationError` if eccentricity or SMA are out of range.
pub fn propagate_roe_j2_drag(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    drag: &DragConfig,
    tau: f64,
) -> Result<(QuasiNonsingularROE, KeplerianElements), PropagationError> {
    let j2p = compute_j2_params(chief_mean)?;
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

    Ok((roe_prop, chief_prop))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::stm::compute_stm;
    use crate::test_helpers::{eccentric_elements, iss_like_elements, koenig_table2_case1, test_drag_config};

    #[test]
    fn zero_drag_equals_j2_stm() {
        let chief = iss_like_elements();
        let tau = 3600.0;

        let stm_j2 = compute_stm(&chief, tau).unwrap();
        let stm_drag = compute_j2_drag_stm(&chief, tau).unwrap();

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

        let stm = compute_j2_drag_stm(&chief, 0.0).unwrap();
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
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let tau = 3600.0;
        let (roe_fwd, chief_fwd) = propagate_roe_j2_drag(&roe, &chief, &drag, tau).unwrap();
        let (roe_back, _) = propagate_roe_j2_drag(&roe_fwd, &chief_fwd, &drag, -tau).unwrap();

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
        let roe = QuasiNonsingularROE::default();

        let tau = 1000.0;
        let (roe_prop, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau).unwrap();

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
        let roe = QuasiNonsingularROE::default();

        let tau = 1000.0;
        let (roe_1, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau).unwrap();
        let (roe_2, _) = propagate_roe_j2_drag(&roe, &chief, &drag, 2.0 * tau).unwrap();

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
        let roe = QuasiNonsingularROE::default();

        let tau = 1000.0;
        let (roe_prop, _) = propagate_roe_j2_drag(&roe, &chief, &drag, tau).unwrap();

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

        let stm_j2 = compute_stm(&chief, tau).unwrap();
        let stm_drag = compute_j2_drag_stm(&chief, tau).unwrap();

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

    // --- Paper-traced regression tests (Koenig Appendix D) ---

    /// Koenig Appendix D: drag coupling block entries for Table 2 Case 1, tau = 1 period.
    /// Validates the 6×3 upper-right drag block and the identity/zero structure.
    #[test]
    fn koenig_appendix_d_drag_block_case1() {
        let chief = koenig_table2_case1();
        let period = chief.period();
        let j2p = compute_j2_params(&chief).unwrap();
        let stm = compute_j2_drag_stm(&chief, period).unwrap();

        // phi[0,6] = τ (linear δa from δȧ)
        assert!(
            (stm[(0, 6)] - period).abs() < 1e-4,
            "phi[0,6]: got {}, expected {period}", stm[(0, 6)]
        );

        // phi[1,6] = -(0.75·n + 1.75·κ·E·P)·τ²
        let tau2 = period * period;
        let expected_16 = -(0.75 * j2p.n_rad_s + 1.75 * j2p.kappa * j2p.big_e * j2p.big_p) * tau2;
        assert!(
            (stm[(1, 6)] - expected_16).abs() < 1e-2,
            "phi[1,6]: got {}, expected {expected_16}", stm[(1, 6)]
        );

        // Lower-right 3×3 identity block
        assert!((stm[(6, 6)] - 1.0).abs() < 1e-14, "phi[6,6] should be 1.0");
        assert!((stm[(7, 7)] - 1.0).abs() < 1e-14, "phi[7,7] should be 1.0");
        assert!((stm[(8, 8)] - 1.0).abs() < 1e-14, "phi[8,8] should be 1.0");

        // Lower-left 3×6 block should be all zeros
        for r in 6..9 {
            for c in 0..6 {
                assert!(
                    stm[(r, c)].abs() < 1e-14,
                    "phi[{r},{c}] should be 0, got {}", stm[(r, c)]
                );
            }
        }
    }

    /// Koenig Appendix D: δȧ → δλ quadratic coupling coefficient.
    /// Propagates zero ROE with δȧ=-1e-10 and verifies:
    /// (1) δλ(2T)/δλ(T) ≈ 4.0 (quadratic scaling)
    /// (2) coefficient matches -(0.75·n + 1.75·κ·E·P)
    #[test]
    fn koenig_appendix_d_quadratic_coefficient() {
        let chief = koenig_table2_case1();
        let da_dot = -1e-10;
        let drag = DragConfig {
            da_dot,
            dex_dot: 0.0,
            dey_dot: 0.0,
        };
        let roe = QuasiNonsingularROE::default();
        let period = chief.period();

        let (roe_1t, _) = propagate_roe_j2_drag(&roe, &chief, &drag, period).unwrap();
        let (roe_2t, _) = propagate_roe_j2_drag(&roe, &chief, &drag, 2.0 * period).unwrap();

        // Quadratic: δλ(2T) / δλ(T) ≈ 4.0
        let ratio = roe_2t.dlambda / roe_1t.dlambda;
        assert!(
            (ratio - 4.0).abs() < 0.1,
            "Quadratic ratio: got {ratio}, expected ~4.0"
        );

        // Verify coefficient: δλ ≈ coeff · δȧ · τ²
        // coeff = -(0.75·n + 1.75·κ·E·P)
        let j2p = compute_j2_params(&chief).unwrap();
        let expected_coeff = -(0.75 * j2p.n_rad_s + 1.75 * j2p.kappa * j2p.big_e * j2p.big_p);
        let actual_coeff = roe_1t.dlambda / (da_dot * period * period);
        assert!(
            (actual_coeff - expected_coeff).abs() / expected_coeff.abs() < 1e-6,
            "Quadratic coefficient: got {actual_coeff}, expected {expected_coeff}"
        );
    }

    // --- Koenig Table 3 regression tests (J2+drag STM physical validation) ---

    /// Koenig Table 3 structural validation: δȧ-driven drift over 1, 5, 10 orbital periods.
    ///
    /// Uses the Koenig Table 2 Case 1 chief orbit (a=6812 km, e=0.005, i=30°) with a
    /// differential drag rate δȧ = -1e-10 /s applied to an otherwise zero ROE.
    ///
    /// Physical properties verified (Koenig Sec. VIII):
    /// 1. δa drift is linear in time: δa(τ) ≈ δȧ·τ within 1%
    /// 2. δλ drift is quadratic: δλ(10T)/δλ(5T) ≈ 4.0 within 10%
    /// 3. δix is unaffected by differential drag: remains ≡ 0 for zero initial ROE
    #[test]
    fn koenig_table3_drag_drift_physical_properties() {
        use crate::propagation::j2_params::compute_j2_params;
        use crate::test_helpers::koenig_table2_case1;

        let chief = koenig_table2_case1();
        let da_dot = -1e-10_f64; // differential drag rate on SMA (1/s)
        let drag = DragConfig {
            da_dot,
            dex_dot: 0.0,
            dey_dot: 0.0,
        };
        let roe0 = QuasiNonsingularROE::default();
        let period = chief.period();

        // Propagate to 1, 5, and 10 orbital periods
        let tau_1t = period;
        let tau_5t = 5.0 * period;
        let tau_10t = 10.0 * period;

        let (roe_1t, _) = propagate_roe_j2_drag(&roe0, &chief, &drag, tau_1t).unwrap();
        let (roe_5t, _) = propagate_roe_j2_drag(&roe0, &chief, &drag, tau_5t).unwrap();
        let (roe_10t, _) = propagate_roe_j2_drag(&roe0, &chief, &drag, tau_10t).unwrap();

        // 1. δa drift is linear: δa(τ) ≈ δȧ·τ  (phi[0,6] = τ)
        //    Tolerance: 1% relative error at each epoch
        let expected_da_1t = da_dot * tau_1t;
        let expected_da_5t = da_dot * tau_5t;
        let expected_da_10t = da_dot * tau_10t;

        let rel_err_da_1t = ((roe_1t.da - expected_da_1t) / expected_da_1t).abs();
        let rel_err_da_5t = ((roe_5t.da - expected_da_5t) / expected_da_5t).abs();
        let rel_err_da_10t = ((roe_10t.da - expected_da_10t) / expected_da_10t).abs();

        assert!(
            rel_err_da_1t < 0.01,
            "δa linear drift at 1T: got {}, expected {expected_da_1t}, rel_err={rel_err_da_1t}",
            roe_1t.da
        );
        assert!(
            rel_err_da_5t < 0.01,
            "δa linear drift at 5T: got {}, expected {expected_da_5t}, rel_err={rel_err_da_5t}",
            roe_5t.da
        );
        assert!(
            rel_err_da_10t < 0.01,
            "δa linear drift at 10T: got {}, expected {expected_da_10t}, rel_err={rel_err_da_10t}",
            roe_10t.da
        );

        // 2. δλ drift is quadratic in time.
        //    For pure δȧ input: δλ(τ) = -(0.75·n + 1.75·κ·E·P)·δȧ·τ²
        //    Quadratic scaling implies δλ(10T)/δλ(5T) = (10T)²/(5T)² = 4.0
        //    Tolerance: 10% (allows for any small non-quadratic J2-driven cross-coupling)
        let ratio_dlambda = roe_10t.dlambda / roe_5t.dlambda;
        assert!(
            (ratio_dlambda - 4.0).abs() < 0.4,
            "δλ quadratic scaling: δλ(10T)/δλ(5T) = {ratio_dlambda}, expected ~4.0 (within 10%)"
        );

        // Also verify sign: negative δȧ with negative coupling coefficient → positive δλ.
        // phi[1,6] = -(0.75·n + 1.75·κ·E·P)·τ²  which is negative for i=30° (P=1.25 > 0).
        // δλ = phi[1,6] · δȧ = (negative) · (negative) = positive.
        // Physical meaning: SMA decay (lower orbit) causes along-track advancement.
        let j2p = compute_j2_params(&chief).unwrap();
        let dlambda_coeff = -(0.75 * j2p.n_rad_s + 1.75 * j2p.kappa * j2p.big_e * j2p.big_p);
        assert!(
            dlambda_coeff < 0.0,
            "δλ coupling coefficient must be negative for positive n and P (i=30° < 63.4°)"
        );
        assert!(
            roe_10t.dlambda > 0.0,
            "δλ should be positive for negative δȧ and negative coupling coefficient; got {}",
            roe_10t.dlambda
        );

        // 3. δix is unaffected by differential drag (Koenig Eq. D2, row 4 has no drag columns)
        //    The drag coupling block has phi[4, 6..9] = 0 by construction.
        //    Starting from zero ROE, δix must remain zero to machine precision.
        assert!(
            roe_1t.dix.abs() < 1e-30,
            "δix must remain 0 at 1T under pure drag input; got {}", roe_1t.dix
        );
        assert!(
            roe_10t.dix.abs() < 1e-30,
            "δix must remain 0 at 10T under pure drag input; got {}", roe_10t.dix
        );
    }

    /// Koenig Table 3 coefficient verification: phi[0,6] and phi[1,6] entries.
    ///
    /// Uses the Koenig Table 2 Case 1 chief orbit (a=6812 km, e=0.005, i=30°).
    ///
    /// Validates the analytical drag coupling coefficients from Koenig Appendix D
    /// against the directly assembled STM entries:
    /// - phi[0,6] = τ  (linear δȧ → δa coupling)
    /// - phi[1,6] = -(3/4·n + 7/4·κ·E·P)·τ²  (quadratic δȧ → δλ coupling)
    #[test]
    fn koenig_table3_coefficient_verification() {
        use crate::propagation::j2_params::compute_j2_params;
        use crate::test_helpers::koenig_table2_case1;

        let chief = koenig_table2_case1();
        let j2p = compute_j2_params(&chief).unwrap();
        let period = chief.period();

        // Build the STM directly for tau = 1 orbital period
        let stm = compute_j2_drag_stm(&chief, period).unwrap();

        // phi[0,6] = τ  (Koenig Appendix D, Eq. D2, row 0)
        // The δȧ coupling to δa is purely linear, with no J2 corrections.
        let expected_phi_0_6 = period;
        assert!(
            (stm[(0, 6)] - expected_phi_0_6).abs() < 1e-9,
            "phi[0,6] = tau: got {}, expected {expected_phi_0_6}",
            stm[(0, 6)]
        );

        // phi[1,6] = -(3/4·n + 7/4·κ·E·P)·τ²  (Koenig Appendix D, Eq. D2, row 1)
        // Note: the factor is written as -(0.75·n + 1.75·κ·E·P) in the implementation,
        // which matches the Koenig expression 3/4·n + 7/4·κ·E·P exactly.
        let tau2 = period * period;
        let expected_phi_1_6 =
            -(0.75 * j2p.n_rad_s + 1.75 * j2p.kappa * j2p.big_e * j2p.big_p) * tau2;
        assert!(
            (stm[(1, 6)] - expected_phi_1_6).abs() < 1e-4,
            "phi[1,6] = -(3/4·n + 7/4·κ·E·P)·τ²: got {}, expected {expected_phi_1_6}",
            stm[(1, 6)]
        );

        // Verify the sign of phi[1,6]: must be negative for i=30° (P=1.25 > 0) and
        // positive n, kappa. This is the fundamental along-track drift direction.
        assert!(
            stm[(1, 6)] < 0.0,
            "phi[1,6] must be negative for i=30° (prograde LEO); got {}",
            stm[(1, 6)]
        );

        // Verify the ratio of phi[1,6] to phi[0,6] reveals the characteristic
        // frequency scale: ratio = -(3/4·n + 7/4·κ·E·P)·τ
        let ratio = stm[(1, 6)] / stm[(0, 6)];
        let expected_ratio =
            -(0.75 * j2p.n_rad_s + 1.75 * j2p.kappa * j2p.big_e * j2p.big_p) * period;
        assert!(
            (ratio - expected_ratio).abs() / expected_ratio.abs() < 1e-9,
            "phi[1,6]/phi[0,6] ratio: got {ratio}, expected {expected_ratio}"
        );
    }
}

//! Quasi-nonsingular relative orbital element (ROE) computation.

use crate::constants::TWO_PI;
use crate::elements::keplerian_conversions::ConversionError;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

/// Compute quasi-nonsingular relative orbital elements (Koenig Eq. 2).
///
/// The ROEs are normalized by the chief semi-major axis.
///
/// # Invariants
/// - `chief.a_km > 0` (used as normalizing denominator)
/// - Both elements must be at the same epoch
/// - Near-equatorial orbits (`i ≈ 0`): `diy` degrades as `sin(i) → 0`
///
/// # Errors
/// Returns `ConversionError::KeplerFailure` if `chief.a_km <= 0` or `chief.e` is outside [0, 1).
pub fn compute_roe(
    chief: &KeplerianElements,
    deputy: &KeplerianElements,
) -> Result<QuasiNonsingularROE, ConversionError> {
    chief.validate()?;

    let da = (deputy.a_km - chief.a_km) / chief.a_km;

    // δλ = (M_d + ω_d) - (M_c + ω_c) + (Ω_d - Ω_c) * cos(i_c)
    let lambda_d = deputy.mean_anomaly_rad + deputy.aop_rad;
    let lambda_c = chief.mean_anomaly_rad + chief.aop_rad;
    let d_raan = deputy.raan_rad - chief.raan_rad;
    let dlambda = wrap_angle(lambda_d - lambda_c + d_raan * chief.i_rad.cos());

    let (sin_aop_d, cos_aop_d) = deputy.aop_rad.sin_cos();
    let (sin_aop_c, cos_aop_c) = chief.aop_rad.sin_cos();
    let dex = deputy.e * cos_aop_d - chief.e * cos_aop_c;
    let dey = deputy.e * sin_aop_d - chief.e * sin_aop_c;

    let dix = deputy.i_rad - chief.i_rad;
    let diy = d_raan * chief.i_rad.sin();

    Ok(QuasiNonsingularROE {
        da,
        dlambda,
        dex,
        dey,
        dix,
        diy,
    })
}

/// Wrap angle to [-π, π].
///
/// # Invariants
/// - `angle` must be finite (NaN/Inf inputs produce undefined results)
/// - Output is in the half-open interval `(-π, π]`
#[must_use]
pub fn wrap_angle(angle: f64) -> f64 {
    let a = angle.rem_euclid(TWO_PI);
    if a > std::f64::consts::PI {
        a - TWO_PI
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;

    #[test]
    fn roe_negative_sma_returns_error() {
        let chief = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let deputy = iss_like_elements();
        let result = compute_roe(&chief, &deputy);
        assert!(
            matches!(result, Err(crate::elements::keplerian_conversions::ConversionError::KeplerFailure(
                crate::types::KeplerError::InvalidSemiMajorAxis { .. }
            ))),
            "Negative SMA should return KeplerFailure(InvalidSemiMajorAxis), got {result:?}"
        );
    }

    #[test]
    fn roe_identical_orbits_are_zero() {
        let chief = iss_like_elements();
        let roe = compute_roe(&chief, &chief).unwrap();
        assert!(roe.da.abs() < 1e-15);
        assert!(roe.dlambda.abs() < 1e-15);
        assert!(roe.dex.abs() < 1e-15);
        assert!(roe.dey.abs() < 1e-15);
        assert!(roe.dix.abs() < 1e-15);
        assert!(roe.diy.abs() < 1e-15);
    }

    #[test]
    fn roe_sma_offset() {
        let chief = KeplerianElements {
            a_km: 6786.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 30.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let mut deputy = chief;
        deputy.a_km = 6787.0; // 1 km higher

        let roe = compute_roe(&chief, &deputy).unwrap();
        let expected_da = 1.0 / 6786.0;
        assert!(
            (roe.da - expected_da).abs() < 1e-12,
            "da mismatch: {} vs {expected_da}",
            roe.da
        );
    }

    /// D'Amico Table 2.1 Case 1: construct deputy from known ROE, then verify
    /// `compute_roe()` recovers the published ROE values.
    /// This is the first test validating `compute_roe()` against published chief/deputy → ROE data.
    #[test]
    fn damico_table21_case1_compute_roe() {
        use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief, deputy_from_roe};

        let chief = damico_table21_chief();
        let expected_roe = damico_table21_case1_roe();

        // Construct deputy by inverting the ROE formulas from the known Table 2.1 Case 1 ROE
        let deputy = deputy_from_roe(&chief, &expected_roe);

        // Compute ROE from the constructed chief/deputy pair
        let computed_roe = compute_roe(&chief, &deputy).unwrap();

        // Assert each ROE component matches within 1e-10 (dimensionless)
        assert!(
            (computed_roe.da - expected_roe.da).abs() < 1e-10,
            "δa: computed={}, expected={}", computed_roe.da, expected_roe.da
        );
        assert!(
            (computed_roe.dlambda - expected_roe.dlambda).abs() < 1e-10,
            "δλ: computed={}, expected={}", computed_roe.dlambda, expected_roe.dlambda
        );
        assert!(
            (computed_roe.dex - expected_roe.dex).abs() < 1e-10,
            "δex: computed={}, expected={}", computed_roe.dex, expected_roe.dex
        );
        assert!(
            (computed_roe.dey - expected_roe.dey).abs() < 1e-10,
            "δey: computed={}, expected={}", computed_roe.dey, expected_roe.dey
        );
        assert!(
            (computed_roe.dix - expected_roe.dix).abs() < 1e-10,
            "δix: computed={}, expected={}", computed_roe.dix, expected_roe.dix
        );
        assert!(
            (computed_roe.diy - expected_roe.diy).abs() < 1e-10,
            "δiy: computed={}, expected={}", computed_roe.diy, expected_roe.diy
        );
    }

    /// Near-equatorial orbits: diy degrades as sin(i) → 0.
    /// Verify that δiy is still computed (no panic/NaN) but may lose precision.
    #[test]
    fn near_equatorial_diy_degrades() {
        let chief = KeplerianElements {
            a_km: 7000.0,
            e: 0.001,
            i_rad: 0.001_f64.to_radians(), // 0.001° ≈ near-equatorial
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let mut deputy = chief;
        deputy.raan_rad = 0.01; // small RAAN offset

        let roe = compute_roe(&chief, &deputy).unwrap();

        // δiy = ΔΩ·sin(i_c) — for very small i, this is ≈ 0 even with substantial ΔΩ
        // The result should be finite (no NaN/Inf) but very small
        assert!(roe.diy.is_finite(), "diy should be finite for near-equatorial");
        assert!(
            roe.diy.abs() < 1e-4,
            "diy should be small for near-equatorial orbit (sin(i)≈0), got {}", roe.diy
        );
    }

    #[test]
    fn roe_inclination_offset() {
        let chief = KeplerianElements {
            a_km: 6786.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 30.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let mut deputy = chief;
        deputy.i_rad = chief.i_rad + 0.001; // small inclination offset

        let roe = compute_roe(&chief, &deputy).unwrap();
        assert!(
            (roe.dix - 0.001).abs() < 1e-12,
            "dix mismatch: {}",
            roe.dix
        );
        assert!(roe.diy.abs() < 1e-15, "diy should be zero for RAAN-only offset");
    }
}

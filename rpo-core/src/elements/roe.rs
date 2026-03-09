//! Quasi-nonsingular relative orbital element (ROE) computation.

use crate::constants::TWO_PI;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

/// Compute quasi-nonsingular relative orbital elements (Koenig Eq. 2).
///
/// The ROEs are normalized by the chief semi-major axis.
#[must_use]
pub fn compute_roe(
    chief: &KeplerianElements,
    deputy: &KeplerianElements,
) -> QuasiNonsingularROE {
    debug_assert!(chief.a > 0.0, "chief semi-major axis must be positive, got {}", chief.a);

    let da = (deputy.a - chief.a) / chief.a;

    // δλ = (M_d + ω_d) - (M_c + ω_c) + (Ω_d - Ω_c) * cos(i_c)
    let lambda_d = deputy.mean_anomaly + deputy.aop;
    let lambda_c = chief.mean_anomaly + chief.aop;
    let d_raan = deputy.raan - chief.raan;
    let dlambda = wrap_angle(lambda_d - lambda_c + d_raan * chief.i.cos());

    let (sin_aop_d, cos_aop_d) = deputy.aop.sin_cos();
    let (sin_aop_c, cos_aop_c) = chief.aop.sin_cos();
    let dex = deputy.e * cos_aop_d - chief.e * cos_aop_c;
    let dey = deputy.e * sin_aop_d - chief.e * sin_aop_c;

    let dix = deputy.i - chief.i;
    let diy = d_raan * chief.i.sin();

    QuasiNonsingularROE {
        da,
        dlambda,
        dex,
        dey,
        dix,
        diy,
    }
}

/// Wrap angle to [-π, π]
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
    fn roe_identical_orbits_are_zero() {
        let chief = iss_like_elements();
        let roe = compute_roe(&chief, &chief);
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
            a: 6786.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 30.0_f64.to_radians(),
            aop: 0.0,
            mean_anomaly: 0.0,
        };
        let mut deputy = chief.clone();
        deputy.a = 6787.0; // 1 km higher

        let roe = compute_roe(&chief, &deputy);
        let expected_da = 1.0 / 6786.0;
        assert!(
            (roe.da - expected_da).abs() < 1e-12,
            "da mismatch: {} vs {expected_da}",
            roe.da
        );
    }

    #[test]
    fn roe_inclination_offset() {
        let chief = KeplerianElements {
            a: 6786.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 30.0_f64.to_radians(),
            aop: 0.0,
            mean_anomaly: 0.0,
        };
        let mut deputy = chief.clone();
        deputy.i = chief.i + 0.001; // small inclination offset

        let roe = compute_roe(&chief, &deputy);
        assert!(
            (roe.dix - 0.001).abs() < 1e-12,
            "dix mismatch: {}",
            roe.dix
        );
        assert!(roe.diy.abs() < 1e-15, "diy should be zero for RAAN-only offset");
    }
}

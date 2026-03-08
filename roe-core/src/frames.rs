//! Linearized ROE-to-RIC frame mapping (D'Amico Eq. 2.17).

use nalgebra::Vector3;

use crate::types::{KeplerianElements, QuasiNonsingularROE, RICState};

/// Convert quasi-nonsingular ROEs to RIC-frame relative state (D'Amico Eq. 2.17).
///
/// This is the linearized mapping valid for near-circular chief orbits.
/// For a static snapshot at epoch, the along-track secular drift term (-1.5*da*Δu) is zero.
///
/// # Arguments
/// * `roe` - Quasi-nonsingular relative orbital elements
/// * `chief` - Chief Keplerian elements (used for a and mean argument of latitude u)
#[must_use]
pub fn roe_to_ric(roe: &QuasiNonsingularROE, chief: &KeplerianElements) -> RICState {
    debug_assert!(chief.a > 0.0, "chief semi-major axis must be positive");
    let a = chief.a;
    let u = chief.mean_arg_of_lat();
    let cos_u = u.cos();
    let sin_u = u.sin();

    // D'Amico Eq. 2.17 — position components
    // At epoch (no elapsed time), the secular along-track drift is zero
    let r_radial = a * (roe.da - roe.dex * cos_u - roe.dey * sin_u);
    let r_along = a * (roe.dlambda + 2.0 * roe.dex * sin_u - 2.0 * roe.dey * cos_u);
    let r_cross = a * (roe.dix * sin_u - roe.diy * cos_u);

    // Velocity components from time derivative of position
    // Using mean motion n = sqrt(μ/a³) and du/dt = n (for near-circular)
    let n = chief.mean_motion();
    let v_radial = a * n * (roe.dex * sin_u - roe.dey * cos_u);
    let v_along = a * n * (-1.5 * roe.da + 2.0 * roe.dex * cos_u + 2.0 * roe.dey * sin_u);
    let v_cross = a * n * (roe.dix * cos_u + roe.diy * sin_u);

    RICState {
        position: Vector3::new(r_radial, r_along, r_cross),
        velocity: Vector3::new(v_radial, v_along, v_cross),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_roe_gives_zero_ric() {
        let chief = KeplerianElements {
            a: 6786.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 30.0_f64.to_radians(),
            aop: 0.0,
            mean_anomaly: 0.0,
        };
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let ric = roe_to_ric(&roe, &chief);
        assert!(ric.position.norm() < 1e-12);
        assert!(ric.velocity.norm() < 1e-12);
    }

    #[test]
    fn pure_da_gives_radial_offset() {
        // A pure δa offset should produce a radial offset at u=0 and along-track drift
        let chief = KeplerianElements {
            a: 6786.0,
            e: 0.0,
            i: 51.6_f64.to_radians(),
            raan: 0.0,
            aop: 0.0,
            mean_anomaly: 0.0, // u = 0
        };
        let da = 1.0 / 6786.0; // ~1 km offset in SMA
        let roe = QuasiNonsingularROE {
            da,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let ric = roe_to_ric(&roe, &chief);

        // Radial position should be a * da = 1 km
        assert!(
            (ric.position.x - 1.0).abs() < 1e-8,
            "Radial offset should be ~1 km, got {}",
            ric.position.x
        );
        // Cross-track should be zero
        assert!(ric.position.z.abs() < 1e-12);
    }

    #[test]
    fn pure_dix_gives_cross_track() {
        let chief = KeplerianElements {
            a: 6786.0,
            e: 0.0,
            i: 51.6_f64.to_radians(),
            raan: 0.0,
            aop: 0.0,
            mean_anomaly: 90.0_f64.to_radians(), // u = π/2
        };
        let dix = 0.001_f64; // small inclination offset
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix,
            diy: 0.0,
        };
        let ric = roe_to_ric(&roe, &chief);

        // At u=π/2, cross-track = a * dix * sin(u) = a * dix
        let expected_cross = chief.a * dix;
        assert!(
            (ric.position.z - expected_cross).abs() < 1e-8,
            "Cross-track should be ~{expected_cross} km, got {}",
            ric.position.z
        );
    }
}

//! Shared test fixtures for unit tests.

use hifitime::Epoch;
use nalgebra::Vector3;

use crate::constants::{J2, MU_EARTH, R_EARTH};
use crate::propagation::propagator::DragConfig;
use crate::types::{KeplerianElements, QuasiNonsingularROE, StateVector};

/// Standard test epoch: 2024-01-01 00:00:00 UTC.
pub fn test_epoch() -> Epoch {
    Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0)
}

/// ISS-like orbit: 408 km altitude, 51.6 deg inclination.
pub fn iss_like_elements() -> KeplerianElements {
    KeplerianElements {
        a_km: 6786.0,
        e: 0.0001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 30.0_f64.to_radians(),
        aop_rad: 45.0_f64.to_radians(),
        mean_anomaly_rad: 60.0_f64.to_radians(),
    }
}

/// Eccentric test orbit (e = 0.3).
pub fn eccentric_elements() -> KeplerianElements {
    KeplerianElements {
        a_km: 10000.0,
        e: 0.3,
        i_rad: 45.0_f64.to_radians(),
        raan_rad: 120.0_f64.to_radians(),
        aop_rad: 200.0_f64.to_radians(),
        mean_anomaly_rad: 150.0_f64.to_radians(),
    }
}

/// LEO 400 km altitude orbit for Lambert tests: ISS-like inclination, near-circular.
pub fn leo_400km_elements() -> KeplerianElements {
    KeplerianElements {
        a_km: 6378.137 + 400.0,
        e: 0.001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 0.0,
        aop_rad: 0.0,
        mean_anomaly_rad: 0.0,
    }
}

/// LEO 800 km altitude target orbit for Lambert tests: coplanar with `leo_400km_elements`,
/// 120° mean anomaly offset.
pub fn leo_800km_target_elements() -> KeplerianElements {
    KeplerianElements {
        a_km: 6378.137 + 800.0,
        e: 0.001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 0.0,
        aop_rad: 0.0,
        mean_anomaly_rad: 120.0_f64.to_radians(),
    }
}

/// Test drag configuration with small nonzero rates.
pub fn test_drag_config() -> DragConfig {
    DragConfig {
        da_dot: -1e-10,
        dex_dot: 1e-11,
        dey_dot: -1e-11,
    }
}

/// Koenig Table 2 Case 1 chief orbit: a=6812 km, e=0.005, i=30°, Ω=60°, ω=180°, M=180°.
/// Used to validate J2 STM propagation errors and perturbation parameters.
pub fn koenig_table2_case1() -> KeplerianElements {
    KeplerianElements {
        a_km: 6812.0,
        e: 0.005,
        i_rad: 30.0_f64.to_radians(),
        raan_rad: 60.0_f64.to_radians(),
        aop_rad: 180.0_f64.to_radians(),
        mean_anomaly_rad: 180.0_f64.to_radians(),
    }
}

/// Koenig Table 2 Case 2 chief orbit: a=8348 km, e=0.2, i=1°, Ω=120°, ω=120°, M=180°.
/// Near-equatorial with moderate eccentricity; exercises nonzero ey component.
pub fn koenig_table2_case2() -> KeplerianElements {
    KeplerianElements {
        a_km: 8348.0,
        e: 0.2,
        i_rad: 1.0_f64.to_radians(),
        raan_rad: 120.0_f64.to_radians(),
        aop_rad: 120.0_f64.to_radians(),
        mean_anomaly_rad: 180.0_f64.to_radians(),
    }
}

/// Koenig Table 2 Case 3 chief orbit: a=13256 km, e=0.5, i=45°, Ω=80°, ω=60°, M=180°.
/// High eccentricity exercises large G and eta corrections.
pub fn koenig_table2_case3() -> KeplerianElements {
    KeplerianElements {
        a_km: 13256.0,
        e: 0.5,
        i_rad: 45.0_f64.to_radians(),
        raan_rad: 80.0_f64.to_radians(),
        aop_rad: 60.0_f64.to_radians(),
        mean_anomaly_rad: 180.0_f64.to_radians(),
    }
}

/// Koenig Table 2 Case 1 initial ROE: physical separations 200m in each component.
/// a·δex=200m, a·δey=-200m, a·δix=200m, a·δiy=-200m (a=6812 km).
pub fn koenig_table2_case1_roe() -> QuasiNonsingularROE {
    let a_km = 6812.0;
    QuasiNonsingularROE {
        da: 0.0,
        dlambda: 0.0,
        dex: 0.200 / a_km,
        dey: -0.200 / a_km,
        dix: 0.200 / a_km,
        diy: -0.200 / a_km,
    }
}

/// D'Amico Table 2.1 chief orbit: 700 km SSO (a=7078.135 km, e=0.001, i=98.19°).
/// Sun-synchronous orbit with ex=0.001, ey=0, u=0 (aop=M=0).
pub fn damico_table21_chief() -> KeplerianElements {
    KeplerianElements {
        a_km: 7078.135,
        e: 0.001,
        i_rad: 98.19_f64.to_radians(),
        raan_rad: 189.89086_f64.to_radians(),
        aop_rad: 0.0,
        mean_anomaly_rad: 0.0,
    }
}

/// D'Amico Table 2.1 Case 1 ROE: parallel e/i vectors along +y axis.
/// a·δey=400m, a·δiy=200m (all others zero); chief a=7078.135 km.
pub fn damico_table21_case1_roe() -> QuasiNonsingularROE {
    let a_km = 7078.135;
    QuasiNonsingularROE {
        da: 0.0,
        dlambda: 0.0,
        dex: 0.0,
        dey: 0.400 / a_km,
        dix: 0.0,
        diy: 0.200 / a_km,
    }
}

/// Koenig Table 2 Case 2 initial ROE: a=8348 km.
/// a·δa=25m, a·δλ=4000m, a·δex=-1000m, a·δey=1000m, a·δix=1000m, a·δiy=0m.
pub fn koenig_table2_case2_roe() -> QuasiNonsingularROE {
    let a_km = 8348.0;
    QuasiNonsingularROE {
        da: 25.0 / a_km,
        dlambda: 4000.0 / a_km,
        dex: -1000.0 / a_km,
        dey: 1000.0 / a_km,
        dix: 1000.0 / a_km,
        diy: 0.0,
    }
}

/// Koenig Table 2 Case 3 initial ROE: a=13256 km.
/// a·δa=100m, a·δλ=5000m, a·δex=5000m, a·δey=5000m, a·δix=-5000m, a·δiy=20000m.
pub fn koenig_table2_case3_roe() -> QuasiNonsingularROE {
    let a_km = 13256.0;
    QuasiNonsingularROE {
        da: 100.0 / a_km,
        dlambda: 5000.0 / a_km,
        dex: 5000.0 / a_km,
        dey: 5000.0 / a_km,
        dix: -5000.0 / a_km,
        diy: 20000.0 / a_km,
    }
}

/// Lower bound for DMF differential drag rate (nonzero sanity check).
/// Any physically meaningful differential drag rate should exceed this
/// threshold; values below indicate extraction failure or zero B* difference.
pub const DMF_RATE_NONZERO_LOWER_BOUND: f64 = 1e-16;

/// Upper bound for DMF differential drag rate (reasonableness check).
/// Differential semi-major axis decay of ~1e-6 km/s (~86 m/day) already
/// exceeds any LEO scenario; values above indicate extraction errors.
pub const DMF_RATE_UPPER_BOUND: f64 = 1e-6;

/// Minimum sin(i) for `deputy_from_roe` δiy inversion.
/// sin(0.001°) ≈ 1.7e-5; 1e-10 catches truly equatorial cases
/// while allowing near-equatorial chief orbits.
const NEAR_EQUATORIAL_SIN_I_THRESHOLD: f64 = 1e-10;

/// Construct deputy Keplerian elements from chief + ROE by inverting the QNS ROE formulas.
///
/// Inverts Koenig Eq. 2 to recover deputy elements from chief elements and ROE.
///
/// # Panics
/// Panics if chief inclination is near zero (`sin(i) < 1e-10`), making diy inversion singular.
/// This is a test-only function; panicking is acceptable for test fixtures.
#[allow(clippy::similar_names)]
pub fn deputy_from_roe(chief: &KeplerianElements, roe: &QuasiNonsingularROE) -> KeplerianElements {
    let a_d = chief.a_km * (1.0 + roe.da);
    let i_d = chief.i_rad + roe.dix;

    assert!(
        chief.i_rad.sin().abs() > NEAR_EQUATORIAL_SIN_I_THRESHOLD,
        "deputy_from_roe: chief inclination too close to zero for δiy inversion"
    );
    let d_raan = roe.diy / chief.i_rad.sin();
    let raan_d = chief.raan_rad + d_raan;

    let ex_c = chief.e * chief.aop_rad.cos();
    let ey_c = chief.e * chief.aop_rad.sin();
    let ex_d = ex_c + roe.dex;
    let ey_d = ey_c + roe.dey;
    let e_d = (ex_d * ex_d + ey_d * ey_d).sqrt();
    let aop_d = ey_d.atan2(ex_d);

    let lambda_c = chief.mean_anomaly_rad + chief.aop_rad;
    let lambda_d = lambda_c + roe.dlambda - d_raan * chief.i_rad.cos();
    let m_d = lambda_d - aop_d;

    KeplerianElements {
        a_km: a_d,
        e: e_d,
        i_rad: i_d,
        raan_rad: raan_d,
        aop_rad: aop_d.rem_euclid(crate::constants::TWO_PI),
        mean_anomaly_rad: m_d.rem_euclid(crate::constants::TWO_PI),
    }
}

// =========================================================================
// RK4 J2 Numerical Integrator (independent truth source for regression tests)
// =========================================================================

/// ECI acceleration: two-body + J2 zonal harmonic.
#[allow(clippy::doc_markdown)]
fn j2_acceleration(pos: &Vector3<f64>) -> Vector3<f64> {
    let r2 = pos.dot(pos);
    let r = r2.sqrt();
    let r3 = r * r2;
    let r5 = r3 * r2;
    let z2 = pos.z * pos.z;
    let z2_over_r2 = z2 / r2;

    // Two-body
    let a_2body = -MU_EARTH / r3 * pos;

    // J2 perturbation
    let j2_coeff = -1.5 * J2 * MU_EARTH * R_EARTH * R_EARTH / r5;
    let a_j2 = Vector3::new(
        j2_coeff * pos.x * (1.0 - 5.0 * z2_over_r2),
        j2_coeff * pos.y * (1.0 - 5.0 * z2_over_r2),
        j2_coeff * pos.z * (3.0 - 5.0 * z2_over_r2),
    );

    a_2body + a_j2
}

/// Propagate an ECI state forward using RK4 with J2 gravity.
///
/// Nominal time step `dt` (seconds); the last step is shortened to land
/// exactly on `duration_s`. State = [pos; vel] in ECI.
/// This is a test-only utility providing an independent numerical truth source
/// with no shared code paths with the analytical STM.
pub fn rk4_j2_propagate(sv: &StateVector, duration_s: f64, dt: f64) -> StateVector {
    assert!(duration_s >= 0.0 && dt > 0.0, "duration and dt must be positive");

    let mut pos = sv.position_eci_km;
    let mut vel = sv.velocity_eci_km_s;
    let mut t = 0.0;

    while t < duration_s {
        let step = dt.min(duration_s - t);

        // RK4 for d(pos)/dt = vel, d(vel)/dt = accel(pos)
        let k1v = j2_acceleration(&pos);
        let k1r = vel;

        let pos2 = pos + 0.5 * step * k1r;
        let vel2 = vel + 0.5 * step * k1v;
        let k2v = j2_acceleration(&pos2);
        let k2r = vel2;

        let pos3 = pos + 0.5 * step * k2r;
        let vel3 = vel + 0.5 * step * k2v;
        let k3v = j2_acceleration(&pos3);
        let k3r = vel3;

        let pos4 = pos + step * k3r;
        let vel4 = vel + step * k3v;
        let k4v = j2_acceleration(&pos4);
        let k4r = vel4;

        pos += step / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
        vel += step / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        t += step;
    }

    StateVector {
        epoch: sv.epoch + hifitime::Duration::from_seconds(duration_s),
        position_eci_km: pos,
        velocity_eci_km_s: vel,
    }
}

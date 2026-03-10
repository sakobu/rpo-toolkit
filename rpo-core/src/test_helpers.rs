//! Shared test fixtures for unit tests.

use hifitime::Epoch;

use crate::types::{DragConfig, KeplerianElements, QuasiNonsingularROE};

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

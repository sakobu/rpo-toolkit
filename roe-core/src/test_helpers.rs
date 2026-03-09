//! Shared test fixtures for unit tests.

use hifitime::Epoch;

use crate::types::{DragConfig, KeplerianElements};

/// Standard test epoch: 2024-01-01 00:00:00 UTC.
pub fn test_epoch() -> Epoch {
    Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0)
}

/// ISS-like orbit: 408 km altitude, 51.6 deg inclination.
pub fn iss_like_elements() -> KeplerianElements {
    KeplerianElements {
        a: 6786.0,
        e: 0.0001,
        i: 51.6_f64.to_radians(),
        raan: 30.0_f64.to_radians(),
        aop: 45.0_f64.to_radians(),
        mean_anomaly: 60.0_f64.to_radians(),
    }
}

/// Eccentric test orbit (e = 0.3).
pub fn eccentric_elements() -> KeplerianElements {
    KeplerianElements {
        a: 10000.0,
        e: 0.3,
        i: 45.0_f64.to_radians(),
        raan: 120.0_f64.to_radians(),
        aop: 200.0_f64.to_radians(),
        mean_anomaly: 150.0_f64.to_radians(),
    }
}

/// LEO 400 km altitude orbit for Lambert tests: ISS-like inclination, near-circular.
pub fn leo_400km_elements() -> KeplerianElements {
    KeplerianElements {
        a: 6378.137 + 400.0,
        e: 0.001,
        i: 51.6_f64.to_radians(),
        raan: 0.0,
        aop: 0.0,
        mean_anomaly: 0.0,
    }
}

/// LEO 800 km altitude target orbit for Lambert tests: coplanar with `leo_400km_elements`,
/// 120° mean anomaly offset.
pub fn leo_800km_target_elements() -> KeplerianElements {
    KeplerianElements {
        a: 6378.137 + 800.0,
        e: 0.001,
        i: 51.6_f64.to_radians(),
        raan: 0.0,
        aop: 0.0,
        mean_anomaly: 120.0_f64.to_radians(),
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

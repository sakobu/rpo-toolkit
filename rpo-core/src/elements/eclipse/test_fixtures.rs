//! Shared test fixtures for eclipse submodule tests.

use hifitime::Epoch;

use crate::constants::TWO_PI;
use crate::types::KeplerianElements;

/// ISS orbital period ~92.4 min = 5544 s. Published ~15.5 orbits/day.
pub(super) const ISS_PERIOD_S: f64 = 5544.0;

/// Build an ISS-like trajectory as (Epoch, KeplerianElements) pairs.
///
/// Generates `n_orbits` orbits at 360 points per orbit (1 deg mean anomaly steps).
/// Each point advances both the epoch and mean anomaly uniformly.
pub(super) fn iss_trajectory_points(n_orbits: u32) -> Vec<(Epoch, KeplerianElements)> {
    let base_elements = crate::test_helpers::iss_like_elements();
    let epoch = crate::test_helpers::test_epoch();
    let base_m = base_elements.mean_anomaly_rad;
    let n_steps_per_orbit: u32 = 360;
    let total_steps = n_orbits * n_steps_per_orbit;
    let dt_per_step = ISS_PERIOD_S / f64::from(n_steps_per_orbit);

    let mut points = Vec::with_capacity(total_steps as usize);
    for k in 0..total_steps {
        let t = f64::from(k) * dt_per_step;
        let m = base_m + f64::from(k) * TWO_PI / f64::from(n_steps_per_orbit);
        let mut elements = base_elements.clone();
        elements.mean_anomaly_rad = m.rem_euclid(TWO_PI);
        points.push((
            epoch + hifitime::Duration::from_seconds(t),
            elements,
        ));
    }
    points
}

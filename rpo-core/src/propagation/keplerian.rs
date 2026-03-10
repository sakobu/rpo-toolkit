//! Two-body Keplerian propagation for ECI trajectory generation.

use hifitime::Duration;

use crate::constants::TWO_PI;
use crate::elements::conversions::{keplerian_to_state, state_to_keplerian, ConversionError};
use crate::types::StateVector;

/// Propagate a state vector under two-body Keplerian dynamics, producing
/// a dense ECI trajectory of `n_steps + 1` states (including endpoints).
///
/// Uses the exact two-body solution: convert to Keplerian elements, advance
/// mean anomaly by `n · dt`, convert back to ECI.
///
/// # Invariants
/// - `initial` must represent a bound orbit (`e < 1`, `a > 0`)
/// - `duration_s` must be finite
///
/// # Errors
/// Returns `ConversionError` if the initial state represents an unbound orbit
/// or has a zero position vector.
pub fn propagate_keplerian(
    initial: &StateVector,
    duration_s: f64,
    n_steps: u32,
) -> Result<Vec<StateVector>, ConversionError> {
    if n_steps == 0 {
        return Ok(vec![initial.clone()]);
    }

    let ke = state_to_keplerian(initial)?;
    let n = ke.mean_motion();

    let step = duration_s / f64::from(n_steps);
    (0..=n_steps)
        .map(|k| {
            let dt = f64::from(k) * step;
            let epoch_k = initial.epoch + Duration::from_seconds(dt);
            let m_k = (ke.mean_anomaly_rad + n * dt).rem_euclid(TWO_PI);
            let ke_k = crate::types::KeplerianElements {
                mean_anomaly_rad: m_k,
                ..ke
            };
            keplerian_to_state(&ke_k, epoch_k)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::MU_EARTH;
    use crate::elements::conversions::keplerian_to_state;
    use crate::test_helpers::{eccentric_elements, iss_like_elements, test_epoch};

    #[test]
    fn propagate_keplerian_circular_closure() {
        let epoch = test_epoch();
        let ke = iss_like_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period();

        let trajectory = propagate_keplerian(&initial, period, 100).unwrap();

        assert_eq!(trajectory.len(), 101);

        let pos_err = (trajectory.last().unwrap().position_eci_km - initial.position_eci_km).norm();
        assert!(
            pos_err < 1e-10,
            "Circular orbit closure error: {pos_err} km"
        );
    }

    #[test]
    fn propagate_keplerian_eccentric_closure() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period();

        let trajectory = propagate_keplerian(&initial, period, 100).unwrap();

        let pos_err = (trajectory.last().unwrap().position_eci_km - initial.position_eci_km).norm();
        assert!(
            pos_err < 1e-10,
            "Eccentric orbit closure error: {pos_err} km"
        );
    }

    #[test]
    fn propagate_keplerian_zero_steps() {
        let epoch = test_epoch();
        let ke = iss_like_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();

        let trajectory = propagate_keplerian(&initial, 3600.0, 0).unwrap();
        assert_eq!(trajectory.len(), 1, "n_steps=0 should return 1 state");
        assert!(
            (trajectory[0].position_eci_km - initial.position_eci_km).norm() < 1e-15,
            "n_steps=0 should return the initial state"
        );
    }

    #[test]
    fn propagate_keplerian_energy_conservation() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period();

        let trajectory = propagate_keplerian(&initial, period, 200).unwrap();

        let energy_0 = {
            let r = initial.position_eci_km.norm();
            let v = initial.velocity_eci_km_s.norm();
            v * v / 2.0 - MU_EARTH / r
        };

        for (k, state) in trajectory.iter().enumerate() {
            let r = state.position_eci_km.norm();
            let v = state.velocity_eci_km_s.norm();
            let energy_k = v * v / 2.0 - MU_EARTH / r;
            let err = (energy_k - energy_0).abs();
            assert!(
                err < 1e-10,
                "Energy conservation violated at step {k}: error = {err} km²/s²"
            );
        }
    }
}

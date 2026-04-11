//! Post-plan trajectory queries: exact state evaluation and resampling.

use crate::mission::types::{ManeuverLeg, WaypointMission};
use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};

/// Evaluate the propagated state at a given elapsed time into the mission.
///
/// Re-evaluates the closed-form propagation model at the exact query time,
/// avoiding discretization error from sampled trajectory lookup. Uses the
/// explicit initial conditions (`post_departure_roe`, `departure_chief_mean`,
/// `departure_maneuver.epoch`) stored on each leg.
///
/// Returns `Ok(None)` exclusively when `elapsed_s` is out of bounds (negative
/// or past the mission end).
///
/// # Invariants
/// - `elapsed_s` must be finite
/// - `mission` must have been produced by `plan_waypoint_mission`
///
/// # Arguments
/// * `mission` — Completed waypoint mission
/// * `elapsed_s` — Elapsed time from mission start (seconds)
/// * `propagator` — Propagation model (must match the one used to plan)
///
/// # Errors
/// Returns `PropagationError` if propagation fails (e.g., invalid orbital elements).
pub fn get_mission_state_at_time(
    mission: &WaypointMission,
    elapsed_s: f64,
    propagator: &PropagationModel,
) -> Result<Option<PropagatedState>, PropagationError> {
    if elapsed_s < 0.0 {
        return Ok(None);
    }

    let mut t_offset = 0.0;
    for leg in &mission.legs {
        if elapsed_s <= t_offset + leg.tof_s {
            let local_t = elapsed_s - t_offset;
            return Ok(Some(propagator.propagate(
                &leg.post_departure_roe,
                &leg.departure_chief_mean,
                leg.departure_maneuver.epoch,
                local_t,
            )?));
        }
        t_offset += leg.tof_s;
    }

    Ok(None)
}

/// Resample a leg's trajectory at a specified number of steps using exact propagation.
///
/// Re-evaluates the closed-form propagation model at equally-spaced times,
/// avoiding discretization error from the original sampled trajectory.
/// Uses the explicit initial conditions on the leg, independent of the
/// stored `trajectory` vector.
///
/// `n_steps` intervals produces `n_steps + 1` states, including both the
/// t=0 and t=tof endpoints.
///
/// # Invariants
/// - `n_steps > 0`
/// - `leg` must have been produced by `solve_leg` or `plan_waypoint_mission`
///
/// # Arguments
/// * `leg` — Maneuver leg to resample
/// * `n_steps` — Number of time intervals (produces `n_steps + 1` states)
/// * `propagator` — Propagation model (must match the one used to plan)
///
/// # Errors
/// Returns `PropagationError::ZeroSteps` if `n_steps` is zero.
pub fn resample_leg_trajectory(
    leg: &ManeuverLeg,
    n_steps: usize,
    propagator: &PropagationModel,
) -> Result<Vec<PropagatedState>, PropagationError> {
    propagator.propagate_with_steps(
        &leg.post_departure_roe,
        &leg.departure_chief_mean,
        leg.departure_maneuver.epoch,
        leg.tof_s,
        n_steps,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::config::MissionConfig;
    use crate::mission::waypoints::plan_waypoint_mission;
    use crate::mission::types::Waypoint;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::{DepartureState, QuasiNonsingularROE};
    use nalgebra::Vector3;

    /// Tolerance for propagated state agreement when the same propagation model
    /// is evaluated at t=0 (identity) or at the exact same query time.
    /// Single STM evaluation — error is f64 arithmetic only.
    const SAME_PATH_PROPAGATION_TOL: f64 = 1e-12;

    /// Tolerance for cross-path propagation agreement: comparing a freshly
    /// propagated state against the last point of a discretized trajectory.
    /// Different evaluation paths (direct propagation vs. trajectory sampling
    /// at `n_steps` resolution) introduce ~1e-10 accumulated error.
    const CROSS_PATH_PROPAGATION_TOL: f64 = 1e-10;

    fn default_config() -> MissionConfig {
        MissionConfig::default()
    }

    fn zero_departure() -> DepartureState {
        DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: iss_like_elements(),
            epoch: test_epoch(),
        }
    }

    /// `get_mission_state_at_time` returns exact state via propagator.
    #[test]
    fn get_state_at_time() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // Mid-mission should return a state with exact elapsed_s
        let query_t = period / 2.0;
        let state = get_mission_state_at_time(&mission, query_t, &propagator).unwrap();
        assert!(state.is_some(), "Should find state at mid-mission");
        let s = state.unwrap();
        assert!(
            (s.elapsed_s - query_t).abs() < SAME_PATH_PROPAGATION_TOL,
            "elapsed_s should exactly match query time"
        );

        // Before mission should return None
        let state = get_mission_state_at_time(&mission, -1.0, &propagator).unwrap();
        assert!(state.is_none(), "Should return None before mission");

        // After mission should return None
        let state = get_mission_state_at_time(&mission, period * 2.0, &propagator).unwrap();
        assert!(state.is_none(), "Should return None after mission");
    }

    /// Exact evaluation at leg endpoints matches trajectory start/end.
    #[test]
    fn exact_evaluation_at_endpoints() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // At t=0, should match first trajectory point
        let at_start = get_mission_state_at_time(&mission, 0.0, &propagator)
            .unwrap().expect("t=0 should be valid");
        let first = &mission.legs[0].trajectory[0];
        assert!(
            (at_start.ric.position_ric_km - first.ric.position_ric_km).norm() < SAME_PATH_PROPAGATION_TOL,
            "Start position should match trajectory[0]"
        );

        // At t=tof, should match last trajectory point
        let at_end = get_mission_state_at_time(&mission, period, &propagator)
            .unwrap().expect("t=tof should be valid");
        let last = mission.legs[0].trajectory.last().unwrap();
        assert!(
            (at_end.ric.position_ric_km - last.ric.position_ric_km).norm() < CROSS_PATH_PROPAGATION_TOL,
            "End position should match trajectory.last()"
        );
    }

    /// Exact evaluation at an arbitrary mid-leg time returns exact `elapsed_s`.
    #[test]
    fn exact_evaluation_midpoint() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // Query a point in the second leg
        let query_t = tof + tof * 0.37; // 37% into second leg
        let state = get_mission_state_at_time(&mission, query_t, &propagator)
            .unwrap().expect("mid-leg query should succeed");

        // elapsed_s should be the local time within the second leg
        let expected_local_t = tof * 0.37;
        assert!(
            (state.elapsed_s - expected_local_t).abs() < CROSS_PATH_PROPAGATION_TOL,
            "elapsed_s should be local time within leg: got {} expected {}",
            state.elapsed_s,
            expected_local_t,
        );
    }

    /// Resample at higher density produces correct number of points.
    #[test]
    fn resample_leg_denser() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let resampled = resample_leg_trajectory(&mission.legs[0], 1000, &propagator)
            .expect("resample should succeed");

        assert_eq!(resampled.len(), 1001, "1000 steps → 1001 points");

        // First point should match trajectory[0]
        let first_orig = &mission.legs[0].trajectory[0];
        assert!(
            (resampled[0].ric.position_ric_km - first_orig.ric.position_ric_km).norm() < SAME_PATH_PROPAGATION_TOL,
            "Resampled first point should match original"
        );
    }

    /// Out-of-bounds queries return None.
    #[test]
    fn evaluate_out_of_bounds() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        assert!(
            get_mission_state_at_time(&mission, -1.0, &propagator).unwrap().is_none(),
            "Negative time should return None"
        );
        assert!(
            get_mission_state_at_time(&mission, period + 1.0, &propagator).unwrap().is_none(),
            "Past-end time should return None"
        );
    }

    /// Leg boundary ownership: intermediate and final boundaries are included.
    #[test]
    fn leg_boundary_ownership() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // At exactly tof (end of leg 1 / start of leg 2): should return Some,
        // owned by leg 1 (due to `elapsed_s <= t_offset + leg.tof_s`)
        let at_boundary = get_mission_state_at_time(&mission, tof, &propagator).unwrap();
        assert!(at_boundary.is_some(), "Boundary at tof should be included");

        // At exactly 2*tof (mission end): should return Some
        let at_end = get_mission_state_at_time(&mission, 2.0 * tof, &propagator).unwrap();
        assert!(at_end.is_some(), "Mission endpoint should be included");

        // At 2*tof + epsilon: should return None
        let past_end = get_mission_state_at_time(&mission, 2.0 * tof + crate::constants::ELAPSED_TIME_TOL_S, &propagator).unwrap();
        assert!(past_end.is_none(), "Past mission end should return None");
    }
}

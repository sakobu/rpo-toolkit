//! Waypoint-based mission chaining.
//!
//! Chains multiple two-burn transfer legs to visit a sequence of waypoints
//! in RIC space. The arrival state of each leg becomes the departure state
//! of the next.

use hifitime::Duration;

use crate::mission::targeting::{optimize_tof, solve_leg};
use crate::propagation::propagator::{PropagatedState, RelativePropagator};
use crate::types::{
    DepartureState, MissionError, SafetyConfig, TargetingConfig, TofOptConfig, Waypoint,
    WaypointMission,
};

/// Plan a multi-waypoint mission by chaining two-burn transfer legs.
///
/// For each waypoint, solves a two-burn transfer from the current state
/// to the target RIC position/velocity. The post-arrival ROE of leg N
/// becomes the departure ROE of leg N+1, and the chief/epoch advance
/// with the time of flight.
///
/// # Errors
/// Returns `MissionError::EmptyWaypoints` if no waypoints are provided,
/// or propagates targeting/propagation errors from individual legs.
pub fn plan_waypoint_mission(
    initial: &DepartureState,
    waypoints: &[Waypoint],
    targeting_config: &TargetingConfig,
    tof_config: &TofOptConfig,
    safety_config: Option<&SafetyConfig>,
    propagator: &dyn RelativePropagator,
) -> Result<WaypointMission, MissionError> {
    if waypoints.is_empty() {
        return Err(MissionError::EmptyWaypoints);
    }

    let mut legs = Vec::with_capacity(waypoints.len());
    let mut current = *initial;
    let mut total_dv = 0.0;
    let mut total_duration = 0.0;

    for wp in waypoints {
        let leg = if let Some(tof) = wp.tof_s {
            // Fixed TOF
            solve_leg(
                &current,
                &wp.position,
                &wp.velocity,
                tof,
                targeting_config,
                propagator,
            )?
        } else {
            // Optimize TOF
            let (_, leg) = optimize_tof(
                &current,
                &wp.position,
                &wp.velocity,
                targeting_config,
                tof_config,
                propagator,
            )?;
            leg
        };

        total_dv += leg.total_dv;
        total_duration += leg.tof_s;

        // Advance state for next leg: post-arrival becomes departure
        current = DepartureState {
            roe: leg.post_arrival_roe,
            chief: leg.arrival_chief_mean,
            epoch: current.epoch + Duration::from_seconds(leg.tof_s),
        };

        legs.push(leg);
    }

    // Compute safety if requested: worst-case across all leg trajectories
    let safety = safety_config.map(|sc| {
        let mut worst: Option<crate::types::SafetyMetrics> = None;
        for leg in &legs {
            if leg.trajectory.is_empty() {
                continue;
            }
            let m = crate::mission::safety::analyze_trajectory_safety(&leg.trajectory, sc);
            worst = Some(match worst {
                None => m,
                Some(w) if m.min_radial_crosstrack_km < w.min_radial_crosstrack_km => m,
                Some(w) => w,
            });
        }
        worst.unwrap_or(crate::types::SafetyMetrics {
            min_radial_crosstrack_km: 0.0,
            de_magnitude: 0.0,
            di_magnitude: 0.0,
            ei_phase_angle_rad: 0.0,
            is_safe: false,
        })
    });

    Ok(WaypointMission {
        legs,
        total_dv,
        total_duration_s: total_duration,
        safety,
    })
}

/// Get the propagated state at a given elapsed time into the mission.
///
/// Searches through all legs to find which leg contains the requested time,
/// then interpolates within that leg's trajectory.
///
/// Returns `None` if `elapsed_s` is outside the mission duration.
#[must_use]
pub fn get_mission_state_at_time(
    mission: &WaypointMission,
    elapsed_s: f64,
) -> Option<PropagatedState> {
    if elapsed_s < 0.0 {
        return None;
    }

    let mut t_offset = 0.0;
    for leg in &mission.legs {
        if elapsed_s <= t_offset + leg.tof_s {
            // This leg contains the requested time
            let local_t = elapsed_s - t_offset;
            // Find closest trajectory point
            return leg
                .trajectory
                .iter()
                .min_by(|a, b| {
                    let da = (a.elapsed_s - local_t).abs();
                    let db = (b.elapsed_s - local_t).abs();
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .cloned();
        }
        t_offset += leg.tof_s;
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::propagator::J2StmPropagator;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::QuasiNonsingularROE;
    use nalgebra::Vector3;

    fn default_configs() -> (TargetingConfig, TofOptConfig) {
        (TargetingConfig::default(), TofOptConfig::default())
    }

    fn zero_departure() -> DepartureState {
        DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: iss_like_elements(),
            epoch: test_epoch(),
        }
    }

    /// Single waypoint mission.
    #[test]
    fn single_waypoint() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &tc, &tof_c, None, &propagator)
                .expect("single waypoint should succeed");

        assert_eq!(mission.legs.len(), 1);
        assert!(mission.total_dv > 0.0);
        assert!(mission.total_duration_s > 0.0);
    }

    /// Three-waypoint chain (use 0.75-period TOFs to avoid CW singularity).
    #[test]
    fn three_waypoint_chain() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position: Vector3::new(0.0, 5.0, 0.0),
                velocity: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position: Vector3::new(2.0, 3.0, 0.0),
                velocity: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position: Vector3::new(0.0, 1.0, 0.0),
                velocity: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &tc, &tof_c, None, &propagator)
                .expect("3-waypoint chain should succeed");

        assert_eq!(mission.legs.len(), 3);
        assert!(mission.total_dv > 0.0);
        // Total duration should be sum of leg TOFs
        let sum_tof: f64 = mission.legs.iter().map(|l| l.tof_s).sum();
        assert!(
            (mission.total_duration_s - sum_tof).abs() < 1e-6,
            "Total duration should match sum of leg TOFs"
        );
    }

    /// State continuity: post-arrival epoch of leg N matches departure of leg N+1.
    #[test]
    fn state_continuity() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position: Vector3::new(0.0, 5.0, 0.0),
                velocity: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position: Vector3::new(0.0, 3.0, 0.0),
                velocity: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &tc, &tof_c, None, &propagator)
                .expect("should succeed");

        // Leg 0 post-arrival epoch should match leg 1 departure epoch
        assert_eq!(
            mission.legs[0].arrival_maneuver.epoch,
            mission.legs[1].departure_maneuver.epoch,
            "Epochs should be continuous between legs"
        );
    }

    /// `get_mission_state_at_time` returns valid states.
    #[test]
    fn get_state_at_time() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &tc, &tof_c, None, &propagator)
                .expect("should succeed");

        // Mid-mission should return a state
        let state = get_mission_state_at_time(&mission, period / 2.0);
        assert!(state.is_some(), "Should find state at mid-mission");

        // Before mission should return None
        let state = get_mission_state_at_time(&mission, -1.0);
        assert!(state.is_none(), "Should return None before mission");

        // After mission should return None
        let state = get_mission_state_at_time(&mission, period * 2.0);
        assert!(state.is_none(), "Should return None after mission");
    }

    /// Empty waypoints returns error.
    #[test]
    fn empty_waypoints_error() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();

        let result =
            plan_waypoint_mission(&departure, &[], &tc, &tof_c, None, &propagator);
        assert!(
            matches!(result, Err(MissionError::EmptyWaypoints)),
            "Empty waypoints should return EmptyWaypoints error"
        );
    }

    /// Serde roundtrip for `WaypointMission`.
    #[test]
    fn serde_roundtrip() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let (tc, tof_c) = default_configs();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &tc, &tof_c, None, &propagator)
                .expect("should succeed");

        let json = serde_json::to_string(&mission).expect("serialize should work");
        let deserialized: WaypointMission =
            serde_json::from_str(&json).expect("deserialize should work");

        assert_eq!(mission.legs.len(), deserialized.legs.len());
        assert!((mission.total_dv - deserialized.total_dv).abs() < 1e-14);
    }
}

//! Waypoint-based mission chaining.
//!
//! Chains multiple two-burn transfer legs to visit a sequence of waypoints
//! in RIC space. The arrival state of each leg becomes the departure state
//! of the next.

use hifitime::Duration;
use nalgebra::Vector3;

use crate::mission::targeting::{optimize_tof, solve_leg};
use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};
use crate::types::DepartureState;

use super::config::{MissionConfig, SafetyConfig};
use super::errors::MissionError;
use super::types::{ManeuverLeg, SafetyMetrics, Waypoint, WaypointMission};

/// Compute worst-case safety metrics across all leg trajectories.
///
/// Returns `None` if no safety config is provided **or** if all legs
/// have empty trajectories (no data to analyze). Tracks worst R/C
/// separation and worst 3D distance independently across all legs.
fn compute_worst_safety(
    legs: &[ManeuverLeg],
    safety_config: Option<&SafetyConfig>,
) -> Option<SafetyMetrics> {
    safety_config.and_then(|_| {
        let mut worst: Option<SafetyMetrics> = None;
        let mut overall_min_rc = f64::INFINITY;
        let mut overall_min_ei = f64::INFINITY;
        let mut overall_min_3d = f64::INFINITY;
        let mut rc_provenance = (0_usize, 0.0_f64, Vector3::zeros());
        let mut d3_provenance = (0_usize, 0.0_f64, Vector3::zeros());
        let mut cumulative_time = 0.0_f64;

        for (i, leg) in legs.iter().enumerate() {
            if leg.trajectory.is_empty() {
                cumulative_time += leg.tof_s;
                continue;
            }
            let Ok(m) = crate::mission::safety::analyze_trajectory_safety(&leg.trajectory) else {
                cumulative_time += leg.tof_s;
                continue;
            };
            if m.operational.min_rc_separation_km < overall_min_rc {
                overall_min_rc = m.operational.min_rc_separation_km;
                rc_provenance = (i, cumulative_time + m.operational.min_rc_elapsed_s, m.operational.min_rc_ric_position_km);
                worst = Some(m);
            }
            overall_min_ei = overall_min_ei.min(m.passive.min_ei_separation_km);
            if m.operational.min_distance_3d_km < overall_min_3d {
                overall_min_3d = m.operational.min_distance_3d_km;
                d3_provenance = (i, cumulative_time + m.operational.min_3d_elapsed_s, m.operational.min_3d_ric_position_km);
            }
            cumulative_time += leg.tof_s;
        }

        worst.map(|mut w| {
            w.operational.min_rc_separation_km = overall_min_rc;
            w.passive.min_ei_separation_km = overall_min_ei;
            w.operational.min_distance_3d_km = overall_min_3d;
            w.operational.min_rc_leg_index = rc_provenance.0;
            w.operational.min_rc_elapsed_s = rc_provenance.1;
            w.operational.min_rc_ric_position_km = rc_provenance.2;
            w.operational.min_3d_leg_index = d3_provenance.0;
            w.operational.min_3d_elapsed_s = d3_provenance.1;
            w.operational.min_3d_ric_position_km = d3_provenance.2;
            w
        })
    })
}

/// Build a `WaypointMission` from a set of legs with computed aggregates.
fn build_mission(
    legs: Vec<ManeuverLeg>,
    safety_config: Option<&SafetyConfig>,
) -> WaypointMission {
    let total_dv_km_s: f64 = legs.iter().map(|l| l.total_dv_km_s).sum();
    let total_duration_s: f64 = legs.iter().map(|l| l.tof_s).sum();
    let safety = compute_worst_safety(&legs, safety_config);
    WaypointMission {
        legs,
        total_dv_km_s,
        total_duration_s,
        safety,
        covariance: None,
    }
}

/// Plan a multi-waypoint mission by chaining two-burn transfer legs.
///
/// For each waypoint, solves a two-burn transfer from the current state
/// to the target RIC position/velocity. The post-arrival ROE of leg N
/// becomes the departure ROE of leg N+1, and the chief/epoch advance
/// with the time of flight.
///
/// # Invariants
/// - `initial.chief.a_km > 0` and `0 <= initial.chief.e < 1`
/// - `initial.chief` must be **mean** Keplerian elements, not osculating
/// - Each `Waypoint.tof_s`, if `Some`, must be > 0
/// - All epochs must be consistent (no backward time jumps)
///
/// # Errors
/// Returns `MissionError::EmptyWaypoints` if no waypoints are provided,
/// or propagates targeting/propagation errors from individual legs.
pub fn plan_waypoint_mission(
    initial: &DepartureState,
    waypoints: &[Waypoint],
    config: &MissionConfig,
    propagator: &PropagationModel,
) -> Result<WaypointMission, MissionError> {
    if waypoints.is_empty() {
        return Err(MissionError::EmptyWaypoints);
    }

    let targeting_config = &config.targeting;
    let tof_config = &config.tof;
    let safety_config = config.safety.as_ref();

    let mut legs = Vec::with_capacity(waypoints.len());
    let mut current = *initial;

    for wp in waypoints {
        let leg = if let Some(tof) = wp.tof_s {
            // Fixed TOF
            solve_leg(
                &current,
                &wp.position_ric_km,
                &wp.velocity_ric_km_s,
                tof,
                targeting_config,
                propagator,
            )?
        } else {
            // Optimize TOF
            let (_, leg) = optimize_tof(
                &current,
                &wp.position_ric_km,
                &wp.velocity_ric_km_s,
                targeting_config,
                tof_config,
                propagator,
            )?;
            leg
        };

        // Advance state for next leg: post-arrival becomes departure
        current = DepartureState {
            roe: leg.post_arrival_roe,
            chief: leg.arrival_chief_mean,
            epoch: current.epoch + Duration::from_seconds(leg.tof_s),
        };

        legs.push(leg);
    }

    Ok(build_mission(legs, safety_config))
}

/// Incrementally replan a waypoint mission from a given index.
///
/// Keeps all legs before `modified_index` unchanged and replans from
/// `modified_index` onward using `new_waypoints`. This avoids recomputing
/// expensive legs that haven't changed.
///
/// # Arguments
/// - `existing` — the previously computed mission
/// - `modified_index` — first waypoint that changed (0-indexed); legs before this are kept
/// - `new_waypoints` — full new waypoint list (must cover indices `modified_index..`)
/// - `initial` — the original departure state (used if `modified_index == 0`)
/// - Other params — same as [`plan_waypoint_mission`]
///
/// # Invariants
/// - `initial.chief` must be **mean** Keplerian elements, not osculating
/// - `modified_index <= new_waypoints.len()` and `modified_index <= existing.legs.len()`
/// - `existing` must have been produced by `plan_waypoint_mission` with the same `initial`
///
/// # Errors
/// Returns `MissionError::InvalidReplanIndex` if `modified_index > new_waypoints.len()`,
/// or propagates errors from the underlying solver.
pub fn replan_from_waypoint(
    existing: &WaypointMission,
    modified_index: usize,
    new_waypoints: &[Waypoint],
    initial: &DepartureState,
    config: &MissionConfig,
    propagator: &PropagationModel,
) -> Result<WaypointMission, MissionError> {
    // Full replan if starting from the beginning
    if modified_index == 0 {
        return plan_waypoint_mission(
            initial,
            new_waypoints,
            config,
            propagator,
        );
    }

    if modified_index > new_waypoints.len() {
        return Err(MissionError::InvalidReplanIndex {
            index: modified_index,
            num_waypoints: new_waypoints.len(),
        });
    }

    if modified_index > existing.legs.len() {
        return Err(MissionError::InvalidReplanIndex {
            index: modified_index,
            num_waypoints: existing.legs.len(),
        });
    }

    // Keep legs before the modified index
    let kept_legs: Vec<ManeuverLeg> = existing.legs[..modified_index].to_vec();

    // Remaining waypoints to plan
    let remaining_waypoints = &new_waypoints[modified_index..];

    if remaining_waypoints.is_empty() {
        // Trailing waypoints were deleted — return kept legs only
        return Ok(build_mission(kept_legs, config.safety.as_ref()));
    }

    // Reconstruct departure state from last kept leg
    let last_kept = &kept_legs[kept_legs.len() - 1];
    let replan_departure = DepartureState {
        roe: last_kept.post_arrival_roe,
        chief: last_kept.arrival_chief_mean,
        epoch: last_kept.arrival_maneuver.epoch,
    };

    // Plan remaining waypoints
    let tail_mission = plan_waypoint_mission(
        &replan_departure,
        remaining_waypoints,
        config,
        propagator,
    )?;

    // Combine kept legs with newly planned legs
    let mut all_legs = kept_legs;
    all_legs.extend(tail_mission.legs);

    Ok(build_mission(all_legs, config.safety.as_ref()))
}

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
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::mission::config::MissionConfig;
    use crate::types::QuasiNonsingularROE;
    use nalgebra::Vector3;

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

    /// Single waypoint mission.
    #[test]
    fn single_waypoint() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("single waypoint should succeed");

        assert_eq!(mission.legs.len(), 1);
        assert!(mission.total_dv_km_s > 0.0);
        assert!(mission.total_duration_s > 0.0);
    }

    /// Three-waypoint chain (use 0.75-period TOFs to avoid CW singularity).
    #[test]
    fn three_waypoint_chain() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("3-waypoint chain should succeed");

        assert_eq!(mission.legs.len(), 3);
        assert!(mission.total_dv_km_s > 0.0);
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
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // Leg 0 post-arrival epoch should match leg 1 departure epoch
        assert_eq!(
            mission.legs[0].arrival_maneuver.epoch,
            mission.legs[1].departure_maneuver.epoch,
            "Epochs should be continuous between legs"
        );
    }

    /// `get_mission_state_at_time` returns exact state via propagator.
    #[test]
    fn get_state_at_time() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
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
            (s.elapsed_s - query_t).abs() < 1e-12,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
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
            (at_start.ric.position_ric_km - first.ric.position_ric_km).norm() < 1e-12,
            "Start position should match trajectory[0]"
        );

        // At t=tof, should match last trajectory point
        let at_end = get_mission_state_at_time(&mission, period, &propagator)
            .unwrap().expect("t=tof should be valid");
        let last = mission.legs[0].trajectory.last().unwrap();
        assert!(
            (at_end.ric.position_ric_km - last.ric.position_ric_km).norm() < 1e-10,
            "End position should match trajectory.last()"
        );
    }

    /// Exact evaluation at an arbitrary mid-leg time returns exact elapsed_s.
    #[test]
    fn exact_evaluation_midpoint() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
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
            (state.elapsed_s - expected_local_t).abs() < 1e-10,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
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
            (resampled[0].ric.position_ric_km - first_orig.ric.position_ric_km).norm() < 1e-12,
            "Resampled first point should match original"
        );
    }

    /// Out-of-bounds queries return None.
    #[test]
    fn evaluate_out_of_bounds() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
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

    /// Empty waypoints returns error.
    #[test]
    fn empty_waypoints_error() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();

        let result =
            plan_waypoint_mission(&departure, &[], &config, &propagator);
        assert!(
            matches!(result, Err(MissionError::EmptyWaypoints)),
            "Empty waypoints should return EmptyWaypoints error"
        );
    }

    /// Serde roundtrip for `WaypointMission`.
    #[test]
    fn serde_roundtrip() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let json = serde_json::to_string(&mission).expect("serialize should work");
        let deserialized: WaypointMission =
            serde_json::from_str(&json).expect("deserialize should work");

        assert_eq!(mission.legs.len(), deserialized.legs.len());
        assert!((mission.total_dv_km_s - deserialized.total_dv_km_s).abs() < 1e-14);
    }

    fn three_wp_waypoints(tof: f64) -> Vec<Waypoint> {
        vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ]
    }

    /// Replan from index 0 produces the same result as a full plan.
    #[test]
    fn replan_index_zero_equals_full_plan() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let waypoints = three_wp_waypoints(period * 0.75);

        let full = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("full plan should succeed");
        let replanned = replan_from_waypoint(
            &full, 0, &waypoints, &departure, &config, &propagator,
        )
        .expect("replan from 0 should succeed");

        assert_eq!(full.legs.len(), replanned.legs.len());
        assert!(
            (full.total_dv_km_s - replanned.total_dv_km_s).abs() < 1e-14,
            "Total Δv should match"
        );
    }

    /// Replan preserves kept legs unchanged.
    #[test]
    fn replan_preserves_kept_legs() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Modify WP2 (index 2) — keep legs 0 and 1
        let mut modified_wps = waypoints.clone();
        modified_wps[2].position_ric_km = Vector3::new(1.0, 2.0, 0.0);

        let replanned = replan_from_waypoint(
            &original, 2, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // First two legs should be identical
        for i in 0..2 {
            assert!(
                (original.legs[i].total_dv_km_s - replanned.legs[i].total_dv_km_s).abs() < 1e-14,
                "Kept leg {i} Δv should be identical"
            );
            assert!(
                (original.legs[i].tof_s - replanned.legs[i].tof_s).abs() < 1e-14,
                "Kept leg {i} TOF should be identical"
            );
        }
    }

    /// Replanned leg's departure epoch matches last kept leg's arrival epoch.
    #[test]
    fn replan_state_continuity() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position_ric_km = Vector3::new(1.0, 4.0, 0.0);

        let replanned = replan_from_waypoint(
            &original, 1, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        // Leg 0's arrival epoch should match leg 1's departure epoch
        assert_eq!(
            replanned.legs[0].arrival_maneuver.epoch,
            replanned.legs[1].departure_maneuver.epoch,
            "Replan state continuity: epochs should match at boundary"
        );
    }

    /// Modify middle waypoint: leg 0 kept, legs 1-2 replanned.
    #[test]
    fn replan_middle_waypoint() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position_ric_km = Vector3::new(1.0, 4.0, 0.5);

        let replanned = replan_from_waypoint(
            &original, 1, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // Leg 0 should be identical
        assert!(
            (original.legs[0].total_dv_km_s - replanned.legs[0].total_dv_km_s).abs() < 1e-14,
            "Kept leg 0 should be identical"
        );
        // Leg 1 should differ (different target)
        assert!(
            (original.legs[1].total_dv_km_s - replanned.legs[1].total_dv_km_s).abs() > 1e-10,
            "Replanned leg 1 should differ from original"
        );
    }

    /// Second bounds check: modified_index exceeds existing legs count.
    #[test]
    fn replan_index_exceeds_existing_legs() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let waypoints = three_wp_waypoints(period * 0.75);

        // Plan a 1-waypoint mission (1 leg)
        let short_mission =
            plan_waypoint_mission(&departure, &[waypoints[0].clone()], &config, &propagator)
                .expect("should succeed");

        // Try to replan from index 2, but only 1 leg exists
        let result = replan_from_waypoint(
            &short_mission, 2, &waypoints, &departure, &config, &propagator,
        );

        assert!(
            matches!(result, Err(MissionError::InvalidReplanIndex { index: 2, num_waypoints: 1 })),
            "Should return InvalidReplanIndex when index exceeds existing legs, got {result:?}"
        );
    }

    /// Invalid replan index returns error.
    #[test]
    fn replan_invalid_index() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let waypoints = three_wp_waypoints(period * 0.75);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let result = replan_from_waypoint(
            &original, 5, &waypoints, &departure, &config, &propagator,
        );

        assert!(
            matches!(
                result,
                Err(MissionError::InvalidReplanIndex {
                    index: 5,
                    num_waypoints: 3,
                })
            ),
            "Should return InvalidReplanIndex, got {result:?}"
        );
    }

    /// Trailing deletion: replan 3-WP mission with only 2 WPs from index 2.
    #[test]
    fn replan_trailing_deletion() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Replan with only 2 waypoints, from index 2 → remaining slice is empty
        let shortened = &waypoints[..2];
        let replanned = replan_from_waypoint(
            &original, 2, shortened, &departure, &config, &propagator,
        )
        .expect("trailing deletion should succeed");

        assert_eq!(replanned.legs.len(), 2);
        // Both kept legs should be identical to original
        for i in 0..2 {
            assert!(
                (original.legs[i].total_dv_km_s - replanned.legs[i].total_dv_km_s).abs() < 1e-14,
                "Kept leg {i} should be identical"
            );
        }
    }

    /// Per-leg metadata fields are populated with sensible values.
    #[test]
    fn metadata_fields_populated() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let waypoints = vec![Waypoint {
            position_ric_km: target_pos,
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let leg = &mission.legs[0];
        assert!(leg.iterations > 0, "Should have at least 1 iteration");
        assert!(
            leg.position_error_km < config.targeting.position_tol_km,
            "Position error should be below tolerance"
        );
        // to_position_ric_km should match the target
        assert!(
            (leg.to_position_ric_km - target_pos).norm() < 1e-14,
            "to_position_ric_km should match target"
        );
        // target_velocity_ric_km_s should be zeros
        assert!(
            leg.target_velocity_ric_km_s.norm() < 1e-14,
            "target_velocity_ric_km_s should be zeros"
        );
        // from_position_ric_km: departure ROE is zero → from_position_ric_km should be near origin
        assert!(
            leg.from_position_ric_km.norm() < 1e-6,
            "from_position_ric_km should be near origin for zero ROE departure"
        );
    }

    /// Leg boundary ownership: intermediate and final boundaries are included.
    #[test]
    fn leg_boundary_ownership() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
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
        let past_end = get_mission_state_at_time(&mission, 2.0 * tof + 1e-6, &propagator).unwrap();
        assert!(past_end.is_none(), "Past mission end should return None");
    }

    /// Serde roundtrip preserves new metadata fields.
    #[test]
    fn serde_roundtrip_with_new_fields() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let json = serde_json::to_string(&mission).expect("serialize");
        let deserialized: WaypointMission = serde_json::from_str(&json).expect("deserialize");

        let leg = &deserialized.legs[0];
        assert_eq!(leg.iterations, mission.legs[0].iterations);
        assert!(
            (leg.position_error_km - mission.legs[0].position_error_km).abs() < 1e-14,
            "position_error_km should survive roundtrip"
        );
        assert!(
            (leg.to_position_ric_km - mission.legs[0].to_position_ric_km).norm() < 1e-14,
            "to_position_ric_km should survive roundtrip"
        );
        assert!(
            (leg.from_position_ric_km - mission.legs[0].from_position_ric_km).norm() < 1e-14,
            "from_position_ric_km should survive roundtrip"
        );
    }

    /// Safety provenance tracks correct leg index and cumulative time.
    #[test]
    fn safety_provenance_leg_index() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;

        let config = MissionConfig {
            targeting: crate::mission::config::TargetingConfig::default(),
            tof: crate::mission::config::TofOptConfig::default(),
            safety: Some(crate::mission::config::SafetyConfig::default()),
        };

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("mission with safety should succeed");

        let safety = mission.safety.expect("safety should be present");

        // Leg indices should be in range
        assert!(
            safety.operational.min_rc_leg_index < mission.legs.len(),
            "R/C leg index {} should be < {}",
            safety.operational.min_rc_leg_index,
            mission.legs.len(),
        );
        assert!(
            safety.operational.min_3d_leg_index < mission.legs.len(),
            "3D leg index {} should be < {}",
            safety.operational.min_3d_leg_index,
            mission.legs.len(),
        );

        // Elapsed times should be non-negative and within mission duration
        assert!(
            safety.operational.min_rc_elapsed_s >= 0.0,
            "R/C elapsed_s should be non-negative"
        );
        assert!(
            safety.operational.min_3d_elapsed_s >= 0.0,
            "3D elapsed_s should be non-negative"
        );
        assert!(
            safety.operational.min_rc_elapsed_s <= mission.total_duration_s + 1e-6,
            "R/C elapsed_s ({}) should be within mission duration ({})",
            safety.operational.min_rc_elapsed_s,
            mission.total_duration_s,
        );
        assert!(
            safety.operational.min_3d_elapsed_s <= mission.total_duration_s + 1e-6,
            "3D elapsed_s ({}) should be within mission duration ({})",
            safety.operational.min_3d_elapsed_s,
            mission.total_duration_s,
        );

        // RIC positions should be nonzero (we have nonzero waypoints)
        assert!(
            safety.operational.min_3d_ric_position_km.norm() > 0.0,
            "3D RIC position should be nonzero"
        );
    }
}

//! Waypoint-based mission chaining.
//!
//! Chains multiple two-burn transfer legs to visit a sequence of waypoints
//! in RIC space. The arrival state of each leg becomes the departure state
//! of the next.

use hifitime::Duration;

use crate::mission::targeting::{optimize_tof, solve_leg};
use crate::propagation::propagator::{PropagatedState, RelativePropagator};
use crate::types::{
    DepartureState, ManeuverLeg, MissionConfig, MissionError, SafetyConfig, SafetyMetrics,
    Waypoint, WaypointMission,
};

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
        let mut overall_min_3d = f64::INFINITY;

        for leg in legs {
            if leg.trajectory.is_empty() {
                continue;
            }
            let m = crate::mission::safety::analyze_trajectory_safety(&leg.trajectory);
            if m.min_rc_separation_km < overall_min_rc {
                overall_min_rc = m.min_rc_separation_km;
                worst = Some(m);
            }
            if m.min_distance_3d_km < overall_min_3d {
                overall_min_3d = m.min_distance_3d_km;
            }
        }

        worst.map(|mut w| {
            w.min_rc_separation_km = overall_min_rc;
            w.min_distance_3d_km = overall_min_3d;
            w
        })
    })
}

/// Build a `WaypointMission` from a set of legs with computed aggregates.
fn build_mission(
    legs: Vec<ManeuverLeg>,
    safety_config: Option<&SafetyConfig>,
) -> WaypointMission {
    let total_dv: f64 = legs.iter().map(|l| l.total_dv).sum();
    let total_duration_s: f64 = legs.iter().map(|l| l.tof_s).sum();
    let safety = compute_worst_safety(&legs, safety_config);
    WaypointMission {
        legs,
        total_dv,
        total_duration_s,
        safety,
    }
}

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
    config: &MissionConfig,
    propagator: &dyn RelativePropagator,
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
/// # Errors
/// Returns `MissionError::InvalidReplanIndex` if `modified_index > new_waypoints.len()`,
/// or propagates errors from the underlying solver.
pub fn replan_from_waypoint(
    existing: &WaypointMission,
    modified_index: usize,
    new_waypoints: &[Waypoint],
    initial: &DepartureState,
    config: &MissionConfig,
    propagator: &dyn RelativePropagator,
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
    use crate::types::{MissionConfig, QuasiNonsingularROE};
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
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
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
        let config = default_config();
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
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
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
        let config = default_config();
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
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
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
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
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
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let json = serde_json::to_string(&mission).expect("serialize should work");
        let deserialized: WaypointMission =
            serde_json::from_str(&json).expect("deserialize should work");

        assert_eq!(mission.legs.len(), deserialized.legs.len());
        assert!((mission.total_dv - deserialized.total_dv).abs() < 1e-14);
    }

    fn three_wp_waypoints(tof: f64) -> Vec<Waypoint> {
        vec![
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
        ]
    }

    /// Replan from index 0 produces the same result as a full plan.
    #[test]
    fn replan_index_zero_equals_full_plan() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
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
            (full.total_dv - replanned.total_dv).abs() < 1e-14,
            "Total Δv should match"
        );
    }

    /// Replan preserves kept legs unchanged.
    #[test]
    fn replan_preserves_kept_legs() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Modify WP2 (index 2) — keep legs 0 and 1
        let mut modified_wps = waypoints.clone();
        modified_wps[2].position = Vector3::new(1.0, 2.0, 0.0);

        let replanned = replan_from_waypoint(
            &original, 2, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // First two legs should be identical
        for i in 0..2 {
            assert!(
                (original.legs[i].total_dv - replanned.legs[i].total_dv).abs() < 1e-14,
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
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position = Vector3::new(1.0, 4.0, 0.0);

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
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position = Vector3::new(1.0, 4.0, 0.5);

        let replanned = replan_from_waypoint(
            &original, 1, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // Leg 0 should be identical
        assert!(
            (original.legs[0].total_dv - replanned.legs[0].total_dv).abs() < 1e-14,
            "Kept leg 0 should be identical"
        );
        // Leg 1 should differ (different target)
        assert!(
            (original.legs[1].total_dv - replanned.legs[1].total_dv).abs() > 1e-10,
            "Replanned leg 1 should differ from original"
        );
    }

    /// Invalid replan index returns error.
    #[test]
    fn replan_invalid_index() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
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
        let propagator = J2StmPropagator;
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
                (original.legs[i].total_dv - replanned.legs[i].total_dv).abs() < 1e-14,
                "Kept leg {i} should be identical"
            );
        }
    }

    /// Per-leg metadata fields are populated with sensible values.
    #[test]
    fn metadata_fields_populated() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let waypoints = vec![Waypoint {
            position: target_pos,
            velocity: Vector3::zeros(),
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
        // to_position should match the target
        assert!(
            (leg.to_position - target_pos).norm() < 1e-14,
            "to_position should match target"
        );
        // target_velocity should be zeros
        assert!(
            leg.target_velocity.norm() < 1e-14,
            "target_velocity should be zeros"
        );
        // from_position: departure ROE is zero → from_position should be near origin
        assert!(
            leg.from_position.norm() < 1e-6,
            "from_position should be near origin for zero ROE departure"
        );
    }

    /// Serde roundtrip preserves new metadata fields.
    #[test]
    fn serde_roundtrip_with_new_fields() {
        let departure = zero_departure();
        let propagator = J2StmPropagator;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion();

        let waypoints = vec![Waypoint {
            position: Vector3::new(0.0, 5.0, 0.0),
            velocity: Vector3::zeros(),
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
            (leg.to_position - mission.legs[0].to_position).norm() < 1e-14,
            "to_position should survive roundtrip"
        );
        assert!(
            (leg.from_position - mission.legs[0].from_position).norm() < 1e-14,
            "from_position should survive roundtrip"
        );
    }
}

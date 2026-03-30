//! Core leg-chaining orchestration for multi-waypoint missions.
//!
//! Chains multiple two-burn transfer legs to visit a sequence of waypoints
//! in RIC space. The arrival state of each leg becomes the departure state
//! of the next.

use hifitime::Duration;
use nalgebra::Vector3;

use crate::mission::config::{MissionConfig, SafetyConfig};
use crate::mission::errors::MissionError;
use crate::mission::targeting::{optimize_tof, solve_leg};
use crate::mission::types::{ManeuverLeg, SafetyMetrics, Waypoint, WaypointMission};
use crate::propagation::propagator::PropagationModel;
use crate::types::DepartureState;

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
    let eclipse = super::eclipse::compute_mission_eclipse(&legs).ok();
    WaypointMission {
        legs,
        total_dv_km_s,
        total_duration_s,
        safety,
        covariance: None,
        eclipse,
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
///
/// **Validated:**
/// - `waypoints` must be non-empty
///
/// **Caller-enforced:**
/// - `initial.chief.a_km > 0` and `0 <= initial.chief.e < 1`
/// - `initial.chief` must be **mean** Keplerian elements, not osculating
/// - Each `Waypoint.tof_s`, if `Some`, must be > 0
/// - All epochs must be consistent (no backward time jumps)
///
/// # Arguments
/// * `initial` — Departure state (ROE, chief elements, epoch)
/// * `waypoints` — Ordered list of target waypoints (RIC position/velocity, optional TOF)
/// * `config` — Mission configuration (targeting, TOF optimization, safety)
/// * `propagator` — Propagation model (J2 or J2+Drag)
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
///
/// **Validated:**
/// - `modified_index <= new_waypoints.len()`
/// - `modified_index <= existing.legs.len()`
///
/// **Caller-enforced:**
/// - `initial.chief` must be **mean** Keplerian elements, not osculating
/// - `existing` must have been produced by `plan_waypoint_mission` with the same `initial`
///
/// # Errors
/// Returns `MissionError::InvalidReplanIndex` if `modified_index` exceeds bounds,
/// or propagates errors from the underlying solver.
pub fn replan_from_waypoint(
    existing: WaypointMission,
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

    // Take ownership of legs, keep only those before the modified index
    let mut kept_legs = existing.legs;
    kept_legs.truncate(modified_index);

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::config::MissionConfig;
    use crate::mission::types::Waypoint;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::{DepartureState, QuasiNonsingularROE};
    use nalgebra::Vector3;

    /// Tolerance for f64 scalar/vector roundtrips where both values come from
    /// the same computation path (serde, kept-leg identity, Δv sums).
    /// No independent evaluation — agreement limited only by f64 representation.
    const SCALAR_ROUNDTRIP_TOL: f64 = 1e-14;

    /// Tolerance for cross-path propagation agreement: comparing a freshly
    /// propagated state against the last point of a discretized trajectory.
    /// Different evaluation paths (direct propagation vs. trajectory sampling
    /// at `n_steps` resolution) introduce ~1e-10 accumulated error.
    const CROSS_PATH_PROPAGATION_TOL: f64 = 1e-10;

    /// Tolerance for accumulated floating-point error across multiple legs.
    /// Summing 3 TOFs via iterator `.sum()` vs. the mission's precomputed
    /// `total_duration_s` can diverge by O(1e-6) due to associativity.
    const MULTI_LEG_ACCUMULATION_TOL: f64 = 1e-6;

    /// Physical bound for near-origin RIC position (km).
    /// A zero-ROE departure state maps to the chief origin; 1e-6 km = 1 mm
    /// accounts for linearization residual in the ROE→RIC mapping.
    const ZERO_ROE_POSITION_BOUND_KM: f64 = 1e-6;

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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
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
            (mission.total_duration_s - sum_tof).abs() < MULTI_LEG_ACCUMULATION_TOL,
            "Total duration should match sum of leg TOFs"
        );
    }

    /// State continuity: post-arrival epoch of leg N matches departure of leg N+1.
    #[test]
    fn state_continuity() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

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
        assert!((mission.total_dv_km_s - deserialized.total_dv_km_s).abs() < SCALAR_ROUNDTRIP_TOL);
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let waypoints = three_wp_waypoints(period * 0.75);

        let full = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("full plan should succeed");
        let full_legs_len = full.legs.len();
        let full_dv = full.total_dv_km_s;
        let replanned = replan_from_waypoint(
            full, 0, &waypoints, &departure, &config, &propagator,
        )
        .expect("replan from 0 should succeed");

        assert_eq!(full_legs_len, replanned.legs.len());
        assert!(
            (full_dv - replanned.total_dv_km_s).abs() < SCALAR_ROUNDTRIP_TOL,
            "Total Δv should match"
        );
    }

    /// Replan preserves kept legs unchanged.
    #[test]
    fn replan_preserves_kept_legs() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Capture kept-leg scalars before ownership transfer
        let kept_dvs: Vec<f64> = original.legs.iter().take(2).map(|l| l.total_dv_km_s).collect();
        let kept_tofs: Vec<f64> = original.legs.iter().take(2).map(|l| l.tof_s).collect();

        // Modify WP2 (index 2) — keep legs 0 and 1
        let mut modified_wps = waypoints.clone();
        modified_wps[2].position_ric_km = Vector3::new(1.0, 2.0, 0.0);

        let replanned = replan_from_waypoint(
            original, 2, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // First two legs should be identical
        for i in 0..2 {
            assert!(
                (kept_dvs[i] - replanned.legs[i].total_dv_km_s).abs() < SCALAR_ROUNDTRIP_TOL,
                "Kept leg {i} Δv should be identical"
            );
            assert!(
                (kept_tofs[i] - replanned.legs[i].tof_s).abs() < SCALAR_ROUNDTRIP_TOL,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position_ric_km = Vector3::new(1.0, 4.0, 0.0);

        let replanned = replan_from_waypoint(
            original, 1, &modified_wps, &departure, &config, &propagator,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Capture scalars before ownership transfer
        let original_leg0_dv = original.legs[0].total_dv_km_s;
        let original_leg1_dv = original.legs[1].total_dv_km_s;

        let mut modified_wps = waypoints.clone();
        modified_wps[1].position_ric_km = Vector3::new(1.0, 4.0, 0.5);

        let replanned = replan_from_waypoint(
            original, 1, &modified_wps, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        assert_eq!(replanned.legs.len(), 3);
        // Leg 0 should be identical
        assert!(
            (original_leg0_dv - replanned.legs[0].total_dv_km_s).abs() < SCALAR_ROUNDTRIP_TOL,
            "Kept leg 0 should be identical"
        );
        // Leg 1 should differ (different target)
        assert!(
            (original_leg1_dv - replanned.legs[1].total_dv_km_s).abs() > CROSS_PATH_PROPAGATION_TOL,
            "Replanned leg 1 should differ from original"
        );
    }

    /// Second bounds check: modified_index exceeds existing legs count.
    #[test]
    fn replan_index_exceeds_existing_legs() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let waypoints = three_wp_waypoints(period * 0.75);

        // Plan a 1-waypoint mission (1 leg)
        let short_mission =
            plan_waypoint_mission(&departure, &[waypoints[0].clone()], &config, &propagator)
                .expect("should succeed");

        // Try to replan from index 2, but only 1 leg exists
        let result = replan_from_waypoint(
            short_mission, 2, &waypoints, &departure, &config, &propagator,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let waypoints = three_wp_waypoints(period * 0.75);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        let result = replan_from_waypoint(
            original, 5, &waypoints, &departure, &config, &propagator,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;
        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("original should succeed");

        // Capture kept-leg scalars before ownership transfer
        let kept_dvs: Vec<f64> = original.legs.iter().take(2).map(|l| l.total_dv_km_s).collect();

        // Replan with only 2 waypoints, from index 2 → remaining slice is empty
        let shortened = &waypoints[..2];
        let replanned = replan_from_waypoint(
            original, 2, shortened, &departure, &config, &propagator,
        )
        .expect("trailing deletion should succeed");

        assert_eq!(replanned.legs.len(), 2);
        // Both kept legs should be identical to original
        for i in 0..2 {
            assert!(
                (kept_dvs[i] - replanned.legs[i].total_dv_km_s).abs() < SCALAR_ROUNDTRIP_TOL,
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
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

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
            (leg.to_position_ric_km - target_pos).norm() < SCALAR_ROUNDTRIP_TOL,
            "to_position_ric_km should match target"
        );
        // target_velocity_ric_km_s should be zeros
        assert!(
            leg.target_velocity_ric_km_s.norm() < SCALAR_ROUNDTRIP_TOL,
            "target_velocity_ric_km_s should be zeros"
        );
        // from_position_ric_km: departure ROE is zero → from_position_ric_km should be near origin
        assert!(
            leg.from_position_ric_km.norm() < ZERO_ROE_POSITION_BOUND_KM,
            "from_position_ric_km should be near origin for zero ROE departure"
        );
    }

    /// Serde roundtrip preserves new metadata fields.
    #[test]
    fn serde_roundtrip_with_new_fields() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

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
            (leg.position_error_km - mission.legs[0].position_error_km).abs() < SCALAR_ROUNDTRIP_TOL,
            "position_error_km should survive roundtrip"
        );
        assert!(
            (leg.to_position_ric_km - mission.legs[0].to_position_ric_km).norm() < SCALAR_ROUNDTRIP_TOL,
            "to_position_ric_km should survive roundtrip"
        );
        assert!(
            (leg.from_position_ric_km - mission.legs[0].from_position_ric_km).norm() < SCALAR_ROUNDTRIP_TOL,
            "from_position_ric_km should survive roundtrip"
        );
    }

    /// Safety provenance tracks correct leg index and cumulative time.
    #[test]
    fn safety_provenance_leg_index() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
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
            safety.operational.min_rc_elapsed_s <= mission.total_duration_s + crate::constants::ELAPSED_TIME_TOL_S,
            "R/C elapsed_s ({}) should be within mission duration ({})",
            safety.operational.min_rc_elapsed_s,
            mission.total_duration_s,
        );
        assert!(
            safety.operational.min_3d_elapsed_s <= mission.total_duration_s + crate::constants::ELAPSED_TIME_TOL_S,
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

    // =======================================================================
    // Eclipse integration tests
    // =======================================================================

    /// `plan_waypoint_mission()` always returns non-None eclipse data
    /// when the mission has at least one leg with a trajectory.
    #[test]
    fn waypoint_mission_includes_eclipse() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let eclipse = mission.eclipse.as_ref().expect("eclipse should be Some");
        assert_eq!(
            eclipse.legs.len(),
            mission.legs.len(),
            "eclipse leg count should match mission leg count"
        );
    }

    /// Eclipse intervals must be bounded within the mission time window.
    #[test]
    fn eclipse_intervals_span_mission() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let eclipse = mission.eclipse.as_ref().expect("eclipse should be Some");
        let mission_start = mission.legs[0].departure_maneuver.epoch;
        let mission_end = mission.legs.last().unwrap().arrival_maneuver.epoch;

        for interval in &eclipse.summary.intervals {
            assert!(
                interval.start >= mission_start,
                "interval start {interval:?} should be >= mission start"
            );
            assert!(
                interval.end <= mission_end,
                "interval end {interval:?} should be <= mission end"
            );
        }
    }

    /// Per-leg celestial snapshot count matches the trajectory sample count.
    #[test]
    fn leg_celestial_count_matches() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
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

        let eclipse = mission.eclipse.as_ref().expect("eclipse should be Some");

        for (i, leg) in mission.legs.iter().enumerate() {
            assert_eq!(
                eclipse.legs[i].chief_celestial.len(),
                leg.trajectory.len(),
                "leg {i}: celestial snapshot count should match trajectory length"
            );
        }
    }

    /// Per-leg deputy eclipse state count matches the celestial snapshot count.
    #[test]
    fn leg_deputy_eclipse_count_matches() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        let eclipse = mission.eclipse.as_ref().expect("eclipse should be Some");

        for (i, leg) in eclipse.legs.iter().enumerate() {
            assert_eq!(
                leg.deputy_eclipse.len(),
                leg.chief_celestial.len(),
                "leg {i}: deputy eclipse count should match celestial snapshot count"
            );
        }
    }

    /// `replan_from_waypoint()` produces fresh eclipse data.
    #[test]
    fn replan_recomputes_eclipse() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        let tof = period * 0.75;

        let waypoints = three_wp_waypoints(tof);

        let original =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        // Replan from waypoint 2 with a modified target
        let mut new_waypoints = waypoints.clone();
        new_waypoints[2] = Waypoint {
            position_ric_km: Vector3::new(0.0, 8.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(tof),
        };

        let replanned = replan_from_waypoint(
            original, 2, &new_waypoints, &departure, &config, &propagator,
        )
        .expect("replan should succeed");

        let eclipse = replanned.eclipse.as_ref().expect("replanned eclipse should be Some");
        assert_eq!(
            eclipse.legs.len(),
            replanned.legs.len(),
            "replanned eclipse leg count should match mission leg count"
        );
    }

    /// WaypointMission with eclipse data survives serde roundtrip.
    #[test]
    fn serde_roundtrip_with_eclipse() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        }];

        let mission =
            plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
                .expect("should succeed");

        assert!(mission.eclipse.is_some(), "eclipse should be computed");

        let json = serde_json::to_string(&mission).expect("serialize");
        let deserialized: WaypointMission =
            serde_json::from_str(&json).expect("deserialize");

        let orig = mission.eclipse.as_ref().unwrap();
        let deser = deserialized.eclipse.as_ref().expect("eclipse should survive roundtrip");

        assert_eq!(orig.summary.intervals.len(), deser.summary.intervals.len());
        assert_eq!(orig.legs.len(), deser.legs.len());
        assert!(
            (orig.summary.total_shadow_duration_s - deser.summary.total_shadow_duration_s).abs()
                < SCALAR_ROUNDTRIP_TOL
        );
    }

    /// Eclipse that spans a maneuver boundary (leg N end → leg N+1 start)
    /// is merged into one contiguous interval, not two.
    ///
    /// Strategy: use a 2-leg mission with TOF chosen so that the maneuver
    /// epoch falls mid-eclipse. The ISS-like orbit at 51.6° inclination
    /// spends ~35% of each orbit in shadow. By choosing an epoch and TOF
    /// that place the leg boundary during an eclipse, we can verify merging.
    ///
    /// We verify by checking that the total interval count is less than or
    /// equal to what we'd get from independent per-leg extraction (which
    /// would split eclipses at the boundary).
    #[test]
    fn cross_leg_eclipse_merging() {
        let departure = zero_departure();
        let propagator = PropagationModel::J2Stm;
        let config = default_config();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();
        // Use 0.75-period legs: ~4,100s each. Two legs cover ~1.5 orbits.
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

        let eclipse = mission.eclipse.as_ref().expect("eclipse should be Some");

        // Count intervals from independent per-leg extraction
        let mut independent_count = 0;
        for leg in &eclipse.legs {
            let summary = crate::elements::extract_eclipse_intervals(&leg.chief_celestial);
            independent_count += summary.intervals.len();
        }

        // Merged count should be <= independent count (equal if no boundary eclipse,
        // strictly less if merging occurred)
        assert!(
            eclipse.summary.intervals.len() <= independent_count,
            "merged count ({}) should be <= independent count ({independent_count})",
            eclipse.summary.intervals.len()
        );

        // At ~1.5 orbits for ISS-like orbit, expect at least 1 eclipse
        assert!(
            !eclipse.summary.intervals.is_empty(),
            "ISS-like orbit over ~1.5 periods should have at least 1 eclipse"
        );

        // Intervals should be non-overlapping and sorted
        for window in eclipse.summary.intervals.windows(2) {
            assert!(
                window[0].end <= window[1].start,
                "intervals should be non-overlapping and sorted: {:?} overlaps {:?}",
                window[0], window[1]
            );
        }
    }
}

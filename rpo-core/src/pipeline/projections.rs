//! Lean projection types for API responses.
//!
//! These types project the full mission data structures down to the
//! subset needed by the frontend. The API handler constructs these
//! directly — the CLI continues using the full `PipelineOutput`.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::mission::types::{ManeuverLeg, Maneuver, MissionPhase, SafetyMetrics};
use crate::propagation::lambert::TransferDirection;
use crate::propagation::propagator::PropagatedState;
use crate::types::roe::QuasiNonsingularROE;

/// Lean summary of a maneuver leg (no trajectory arrays, no ROE/Keplerian state).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegSummary {
    /// Departure burn.
    pub departure_maneuver: Maneuver,
    /// Arrival burn.
    pub arrival_maneuver: Maneuver,
    /// Time of flight for this leg (seconds).
    pub tof_s: f64,
    /// Total delta-v for this leg (km/s).
    pub total_dv_km_s: f64,
    /// Departure RIC position (km).
    pub from_position_ric_km: Vector3<f64>,
    /// Target RIC position (km).
    pub to_position_ric_km: Vector3<f64>,
    /// Target RIC velocity (km/s).
    pub target_velocity_ric_km_s: Vector3<f64>,
    /// Newton-Raphson iterations used.
    pub iterations: u32,
    /// Final position error after convergence (km).
    pub position_error_km: f64,
    /// Human-readable label for this leg.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

/// Lean trajectory point for 3D visualization (~56 bytes vs ~200 for `PropagatedState`).
///
/// Uses `[f64; 3]` arrays instead of `Vector3<f64>` for JSON wire efficiency,
/// matching `WaypointInput` precedent. Conversion from `Vector3` happens in
/// `propagated_to_point()`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Elapsed time from leg start (seconds).
    pub elapsed_s: f64,
    /// Position in RIC frame (km): \[radial, in-track, cross-track\].
    pub position_ric_km: [f64; 3],
    /// Velocity in RIC frame (km/s): \[radial, in-track, cross-track\].
    pub velocity_ric_km_s: [f64; 3],
}

/// Per-leg trajectory data for on-demand fetch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegTrajectory {
    /// Index of this leg in the mission.
    pub leg_index: usize,
    /// Trajectory points.
    pub points: Vec<TrajectoryPoint>,
}

/// Lean Lambert transfer summary (no full state vectors).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransferSummary {
    /// Total transfer delta-v (km/s).
    pub total_dv_km_s: f64,
    /// Transfer time-of-flight (seconds).
    pub tof_s: f64,
    /// Transfer direction (short/long way).
    pub direction: TransferDirection,
    /// Arrival epoch.
    #[serde(with = "crate::types::state::epoch_serde")]
    pub arrival_epoch: Epoch,
}

/// Lean mission plan result for API responses (~10-30 KB vs 350-500 KB).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LeanPlanResult {
    /// Classification phase.
    pub phase: MissionPhase,
    /// Lambert transfer summary (None if proximity).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer_summary: Option<TransferSummary>,
    /// Idealized perch ROE target.
    pub perch_roe: QuasiNonsingularROE,
    /// Per-leg summaries.
    pub legs: Vec<LegSummary>,
    /// Total delta-v: Lambert + waypoint legs (km/s).
    pub total_dv_km_s: f64,
    /// Total duration: Lambert + waypoint legs (seconds).
    pub total_duration_s: f64,
    /// Safety metrics (if computed).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety: Option<SafetyMetrics>,
}

// ---------------------------------------------------------------------------
// Projection methods on ManeuverLeg
// ---------------------------------------------------------------------------

impl ManeuverLeg {
    /// Project to a lean summary (no trajectory, no ROE, no chief elements).
    #[must_use]
    pub fn to_summary(&self, label: Option<String>) -> LegSummary {
        LegSummary {
            departure_maneuver: self.departure_maneuver,
            arrival_maneuver: self.arrival_maneuver,
            tof_s: self.tof_s,
            total_dv_km_s: self.total_dv_km_s,
            from_position_ric_km: self.from_position_ric_km,
            to_position_ric_km: self.to_position_ric_km,
            target_velocity_ric_km_s: self.target_velocity_ric_km_s,
            iterations: self.iterations,
            position_error_km: self.position_error_km,
            label,
        }
    }

    /// Convert full trajectory to lean trajectory points.
    #[must_use]
    pub fn to_trajectory_points(&self) -> Vec<TrajectoryPoint> {
        self.trajectory.iter().map(propagated_to_point).collect()
    }

    /// Resample trajectory to at most `max_points` lean points.
    ///
    /// Uses uniform time spacing with linear interpolation.
    /// First and last points are always preserved.
    /// If `max_points >= trajectory.len()`, returns all points converted.
    #[must_use]
    pub fn resample_trajectory(&self, max_points: u32) -> Vec<TrajectoryPoint> {
        resample_propagated(&self.trajectory, max_points)
    }
}

/// Convert a single `PropagatedState` to a `TrajectoryPoint`.
fn propagated_to_point(state: &PropagatedState) -> TrajectoryPoint {
    TrajectoryPoint {
        elapsed_s: state.elapsed_s,
        position_ric_km: [
            state.ric.position_ric_km[0],
            state.ric.position_ric_km[1],
            state.ric.position_ric_km[2],
        ],
        velocity_ric_km_s: [
            state.ric.velocity_ric_km_s[0],
            state.ric.velocity_ric_km_s[1],
            state.ric.velocity_ric_km_s[2],
        ],
    }
}

/// Minimum elapsed-time span for linear interpolation (seconds).
/// Below this threshold, interpolation degenerates and we snap to the first bracket point.
const INTERPOLATION_SPAN_TOL_S: f64 = 1.0e-12;

/// Resample a propagated trajectory to at most `max_points` lean points.
///
/// Uniform time spacing, first/last always preserved, linear interpolation.
fn resample_propagated(trajectory: &[PropagatedState], max_points: u32) -> Vec<TrajectoryPoint> {
    let max_pts = max_points as usize; // u32 → usize: always safe (usize ≥ 32 bits)

    if trajectory.is_empty() || max_points == 0 {
        return Vec::new();
    }
    if max_points == 1 {
        return vec![propagated_to_point(&trajectory[0])];
    }
    if max_pts >= trajectory.len() {
        return trajectory.iter().map(propagated_to_point).collect();
    }

    let first = &trajectory[0];
    let last = &trajectory[trajectory.len() - 1];
    let t_start = first.elapsed_s;
    let t_end = last.elapsed_s;
    let intervals = max_points - 1;
    let dt = (t_end - t_start) / f64::from(intervals);

    let mut result = Vec::with_capacity(max_pts);
    result.push(propagated_to_point(first));

    let mut search_start = 0;
    for i in 1..intervals {
        let t_target = t_start + dt * f64::from(i);

        // Find bracketing points
        while search_start + 1 < trajectory.len()
            && trajectory[search_start + 1].elapsed_s < t_target
        {
            search_start += 1;
        }

        if search_start + 1 >= trajectory.len() {
            result.push(propagated_to_point(last));
            continue;
        }

        let a = &trajectory[search_start];
        let b = &trajectory[search_start + 1];
        let span = b.elapsed_s - a.elapsed_s;
        let frac = if span.abs() < INTERPOLATION_SPAN_TOL_S {
            0.0
        } else {
            (t_target - a.elapsed_s) / span
        };

        result.push(TrajectoryPoint {
            elapsed_s: t_target,
            position_ric_km: [
                a.ric.position_ric_km[0]
                    + frac * (b.ric.position_ric_km[0] - a.ric.position_ric_km[0]),
                a.ric.position_ric_km[1]
                    + frac * (b.ric.position_ric_km[1] - a.ric.position_ric_km[1]),
                a.ric.position_ric_km[2]
                    + frac * (b.ric.position_ric_km[2] - a.ric.position_ric_km[2]),
            ],
            velocity_ric_km_s: [
                a.ric.velocity_ric_km_s[0]
                    + frac * (b.ric.velocity_ric_km_s[0] - a.ric.velocity_ric_km_s[0]),
                a.ric.velocity_ric_km_s[1]
                    + frac * (b.ric.velocity_ric_km_s[1] - a.ric.velocity_ric_km_s[1]),
                a.ric.velocity_ric_km_s[2]
                    + frac * (b.ric.velocity_ric_km_s[2] - a.ric.velocity_ric_km_s[2]),
            ],
        });
    }

    result.push(propagated_to_point(last));
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::types::MissionPhase;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::RICState;
    use hifitime::Duration;
    use nalgebra::Vector3;

    /// Position field comparison tolerance. Fields are copied directly
    /// (no arithmetic), so exact bitwise equality holds; 1e-15 is a
    /// conservative guard against any future indirection.
    const FIELD_COPY_TOL: f64 = 1e-15;

    /// Build a minimal `ManeuverLeg` with a synthetic trajectory for testing.
    fn test_maneuver_leg(n_traj_points: usize) -> ManeuverLeg {
        let epoch = test_epoch();
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();

        let trajectory: Vec<PropagatedState> = (0..n_traj_points)
            .map(|i| {
                let t = f64::from(u32::try_from(i).expect("test index fits u32")) * 100.0;
                PropagatedState {
                    epoch: epoch + Duration::from_seconds(t),
                    roe,
                    chief_mean: chief.clone(),
                    ric: RICState {
                        position_ric_km: Vector3::new(
                            t * 0.001,
                            t * 0.01,
                            t * 0.0001,
                        ),
                        velocity_ric_km_s: Vector3::new(
                            0.001,
                            0.002,
                            0.0003,
                        ),
                    },
                    elapsed_s: t,
                }
            })
            .collect();

        ManeuverLeg {
            departure_maneuver: Maneuver {
                dv_ric_km_s: Vector3::new(0.01, 0.02, 0.003),
                epoch,
            },
            arrival_maneuver: Maneuver {
                dv_ric_km_s: Vector3::new(-0.005, -0.01, -0.001),
                epoch: epoch + Duration::from_seconds(1000.0),
            },
            tof_s: 1000.0,
            total_dv_km_s: 0.05,
            post_departure_roe: roe,
            departure_chief_mean: chief.clone(),
            pre_arrival_roe: roe,
            post_arrival_roe: roe,
            arrival_chief_mean: chief,
            trajectory,
            from_position_ric_km: Vector3::new(0.0, 0.0, 0.0),
            to_position_ric_km: Vector3::new(1.0, 10.0, 0.1),
            target_velocity_ric_km_s: Vector3::new(0.001, 0.002, 0.0003),
            iterations: 5,
            position_error_km: 1.2e-6,
        }
    }

    #[test]
    fn test_to_summary_preserves_fields() {
        let leg = test_maneuver_leg(11);
        let summary = leg.to_summary(Some("test-leg".to_string()));

        assert!(
            (summary.tof_s - leg.tof_s).abs() < FIELD_COPY_TOL,
            "tof_s mismatch"
        );
        assert!(
            (summary.total_dv_km_s - leg.total_dv_km_s).abs() < FIELD_COPY_TOL,
            "total_dv_km_s mismatch"
        );
        assert!(
            (summary.from_position_ric_km - leg.from_position_ric_km).norm() < FIELD_COPY_TOL,
            "from_position_ric_km mismatch"
        );
        assert!(
            (summary.to_position_ric_km - leg.to_position_ric_km).norm() < FIELD_COPY_TOL,
            "to_position_ric_km mismatch"
        );
        assert!(
            (summary.target_velocity_ric_km_s - leg.target_velocity_ric_km_s).norm()
                < FIELD_COPY_TOL,
            "target_velocity_ric_km_s mismatch"
        );
        assert_eq!(summary.iterations, leg.iterations, "iterations mismatch");
        assert!(
            (summary.position_error_km - leg.position_error_km).abs() < FIELD_COPY_TOL,
            "position_error_km mismatch"
        );
        assert_eq!(
            summary.departure_maneuver.dv_ric_km_s,
            leg.departure_maneuver.dv_ric_km_s,
            "departure_maneuver dv mismatch"
        );
        assert_eq!(
            summary.arrival_maneuver.dv_ric_km_s,
            leg.arrival_maneuver.dv_ric_km_s,
            "arrival_maneuver dv mismatch"
        );
        assert_eq!(
            summary.label,
            Some("test-leg".to_string()),
            "label mismatch"
        );
    }

    #[test]
    fn test_to_trajectory_points() {
        let leg = test_maneuver_leg(11);
        let points = leg.to_trajectory_points();

        assert_eq!(
            points.len(),
            leg.trajectory.len(),
            "point count should match trajectory length"
        );

        // Spot-check first point
        let first = &points[0];
        assert!(
            (first.elapsed_s - leg.trajectory[0].elapsed_s).abs() < FIELD_COPY_TOL,
            "first elapsed_s mismatch"
        );
        assert!(
            (first.position_ric_km[0] - leg.trajectory[0].ric.position_ric_km[0]).abs()
                < FIELD_COPY_TOL,
            "first position R mismatch"
        );

        // Spot-check last point
        let last_idx = points.len() - 1;
        let last = &points[last_idx];
        let last_traj = &leg.trajectory[last_idx];
        assert!(
            (last.elapsed_s - last_traj.elapsed_s).abs() < FIELD_COPY_TOL,
            "last elapsed_s mismatch"
        );
        assert!(
            (last.position_ric_km[1] - last_traj.ric.position_ric_km[1]).abs() < FIELD_COPY_TOL,
            "last position I mismatch"
        );
    }

    #[test]
    fn test_resample_two_points() {
        let leg = test_maneuver_leg(11);
        let resampled = leg.resample_trajectory(2);

        assert_eq!(resampled.len(), 2, "resample(2) should yield exactly 2 points");

        // First point matches trajectory start
        assert!(
            (resampled[0].elapsed_s - leg.trajectory[0].elapsed_s).abs() < FIELD_COPY_TOL,
            "first resampled point should match trajectory start"
        );

        // Last point matches trajectory end
        let last_traj = &leg.trajectory[leg.trajectory.len() - 1];
        assert!(
            (resampled[1].elapsed_s - last_traj.elapsed_s).abs() < FIELD_COPY_TOL,
            "last resampled point should match trajectory end"
        );
    }

    #[test]
    fn test_resample_all_points() {
        let leg = test_maneuver_leg(5);
        // Request more points than available — should return all converted
        let resampled = leg.resample_trajectory(100);

        assert_eq!(
            resampled.len(),
            leg.trajectory.len(),
            "resample(N >= len) should return all points"
        );

        for (i, (r, t)) in resampled.iter().zip(leg.trajectory.iter()).enumerate() {
            assert!(
                (r.elapsed_s - t.elapsed_s).abs() < FIELD_COPY_TOL,
                "point {i} elapsed_s mismatch"
            );
        }
    }

    #[test]
    fn test_resample_empty() {
        let mut leg = test_maneuver_leg(0);
        assert!(leg.trajectory.is_empty());

        let resampled = leg.resample_trajectory(10);
        assert!(
            resampled.is_empty(),
            "empty trajectory should produce empty resample"
        );

        // Also test max_points=0 with non-empty trajectory
        leg = test_maneuver_leg(5);
        let resampled_zero = leg.resample_trajectory(0);
        assert!(
            resampled_zero.is_empty(),
            "max_points=0 should produce empty resample"
        );
    }

    #[test]
    fn test_lean_plan_result_serde_roundtrip() {
        let chief = iss_like_elements();
        let epoch = test_epoch();

        let plan = LeanPlanResult {
            phase: MissionPhase::Proximity {
                roe: QuasiNonsingularROE::default(),
                chief_elements: chief.clone(),
                deputy_elements: chief.clone(),
                separation_km: 1.0,
                delta_r_over_r: 1e-4,
            },
            transfer_summary: Some(TransferSummary {
                total_dv_km_s: 0.5,
                tof_s: 3600.0,
                direction: TransferDirection::ShortWay,
                arrival_epoch: epoch + Duration::from_seconds(3600.0),
            }),
            perch_roe: QuasiNonsingularROE {
                da: 0.001,
                dlambda: 0.002,
                dex: 0.0001,
                dey: -0.0001,
                dix: 0.0003,
                diy: 0.0002,
            },
            legs: vec![LegSummary {
                departure_maneuver: Maneuver {
                    dv_ric_km_s: Vector3::new(0.01, 0.02, 0.003),
                    epoch,
                },
                arrival_maneuver: Maneuver {
                    dv_ric_km_s: Vector3::new(-0.005, -0.01, -0.001),
                    epoch: epoch + Duration::from_seconds(1000.0),
                },
                tof_s: 1000.0,
                total_dv_km_s: 0.05,
                from_position_ric_km: Vector3::new(0.0, 0.0, 0.0),
                to_position_ric_km: Vector3::new(1.0, 10.0, 0.1),
                target_velocity_ric_km_s: Vector3::new(0.001, 0.002, 0.0003),
                iterations: 5,
                position_error_km: 1.2e-6,
                label: Some("leg-0".to_string()),
            }],
            total_dv_km_s: 0.55,
            total_duration_s: 4600.0,
            safety: None,
        };

        let json = serde_json::to_string(&plan).expect("serialize should succeed");
        let roundtrip: LeanPlanResult =
            serde_json::from_str(&json).expect("deserialize should succeed");

        assert!(
            (roundtrip.total_dv_km_s - plan.total_dv_km_s).abs() < FIELD_COPY_TOL,
            "total_dv_km_s mismatch after roundtrip"
        );
        assert!(
            (roundtrip.total_duration_s - plan.total_duration_s).abs() < FIELD_COPY_TOL,
            "total_duration_s mismatch after roundtrip"
        );
        assert_eq!(
            roundtrip.legs.len(),
            plan.legs.len(),
            "leg count mismatch after roundtrip"
        );
        assert!(
            (roundtrip.perch_roe.da - plan.perch_roe.da).abs() < FIELD_COPY_TOL,
            "perch_roe.da mismatch after roundtrip"
        );
    }

    #[test]
    fn test_trajectory_point_serde_array_format() {
        let point = TrajectoryPoint {
            elapsed_s: 42.0,
            position_ric_km: [1.0, 2.0, 3.0],
            velocity_ric_km_s: [0.1, 0.2, 0.3],
        };

        let json = serde_json::to_string(&point).expect("serialize should succeed");

        // Verify arrays serialize as JSON arrays, not objects
        let value: serde_json::Value =
            serde_json::from_str(&json).expect("parse as Value should succeed");

        assert!(
            value["position_ric_km"].is_array(),
            "position_ric_km should serialize as a JSON array, got: {}",
            value["position_ric_km"]
        );
        assert!(
            value["velocity_ric_km_s"].is_array(),
            "velocity_ric_km_s should serialize as a JSON array, got: {}",
            value["velocity_ric_km_s"]
        );

        // Verify values roundtrip
        let roundtrip: TrajectoryPoint =
            serde_json::from_str(&json).expect("deserialize should succeed");
        assert!(
            (roundtrip.elapsed_s - 42.0).abs() < FIELD_COPY_TOL,
            "elapsed_s roundtrip mismatch"
        );
        assert!(
            (roundtrip.position_ric_km[0] - 1.0).abs() < FIELD_COPY_TOL,
            "position R roundtrip mismatch"
        );
        assert!(
            (roundtrip.velocity_ric_km_s[2] - 0.3).abs() < FIELD_COPY_TOL,
            "velocity C roundtrip mismatch"
        );
    }

    #[test]
    fn test_resample_interpolation_accuracy() {
        // Trajectory has linear position: pos = (t*0.001, t*0.01, t*0.0001)
        // and constant velocity: (0.001, 0.002, 0.0003).
        // 11 points at t = 0, 100, 200, ..., 1000.
        // Resample to 3 points → t = 0, 500, 1000.
        // Midpoint (t=500) should match the linear profile exactly.
        let leg = test_maneuver_leg(11);
        let resampled = leg.resample_trajectory(3);

        assert_eq!(resampled.len(), 3);

        // First point: t=0
        assert!((resampled[0].elapsed_s).abs() < FIELD_COPY_TOL);

        // Midpoint: t=500 → position = (0.5, 5.0, 0.05)
        let mid = &resampled[1];
        assert!(
            (mid.elapsed_s - 500.0).abs() < FIELD_COPY_TOL,
            "midpoint elapsed_s: expected 500, got {}",
            mid.elapsed_s
        );
        assert!(
            (mid.position_ric_km[0] - 0.5).abs() < FIELD_COPY_TOL,
            "midpoint R: expected 0.5, got {}",
            mid.position_ric_km[0]
        );
        assert!(
            (mid.position_ric_km[1] - 5.0).abs() < FIELD_COPY_TOL,
            "midpoint I: expected 5.0, got {}",
            mid.position_ric_km[1]
        );
        assert!(
            (mid.position_ric_km[2] - 0.05).abs() < FIELD_COPY_TOL,
            "midpoint C: expected 0.05, got {}",
            mid.position_ric_km[2]
        );

        // Velocity should also interpolate to constant value
        assert!(
            (mid.velocity_ric_km_s[0] - 0.001).abs() < FIELD_COPY_TOL,
            "midpoint vR: expected 0.001, got {}",
            mid.velocity_ric_km_s[0]
        );

        // Last point: t=1000
        assert!(
            (resampled[2].elapsed_s - 1000.0).abs() < FIELD_COPY_TOL,
            "last point elapsed_s mismatch"
        );
    }
}

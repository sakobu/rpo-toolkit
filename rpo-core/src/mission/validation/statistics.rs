//! Aggregate report statistics for validation comparisons.

use nalgebra::Vector3;

use crate::propagation::propagator::PropagatedState;
use crate::types::RICState;

use crate::mission::types::ValidationPoint;

/// Find the closest analytical RIC state by elapsed time (nearest-neighbor).
///
/// Uses binary search on the time-sorted trajectory for O(log M) lookup.
/// Falls back to zero RIC if the trajectory is empty. Interpolation error
/// from nearest-neighbor lookup is ~14m for a 200-step trajectory, well below
/// expected analytical-vs-numerical divergence.
pub(super) fn find_closest_analytical_ric(trajectory: &[PropagatedState], elapsed_s: f64) -> RICState {
    if trajectory.is_empty() {
        return RICState {
            position_ric_km: Vector3::zeros(),
            velocity_ric_km_s: Vector3::zeros(),
        };
    }

    // Trajectory is time-sorted by construction; binary search for insertion point
    let idx = trajectory.partition_point(|s| s.elapsed_s < elapsed_s);

    // Compare the two candidates bracketing the insertion point
    let best = if idx == 0 {
        &trajectory[0]
    } else if idx >= trajectory.len() {
        &trajectory[trajectory.len() - 1]
    } else {
        let before = &trajectory[idx - 1];
        let after = &trajectory[idx];
        if (before.elapsed_s - elapsed_s).abs() <= (after.elapsed_s - elapsed_s).abs() {
            before
        } else {
            after
        }
    };
    best.ric.clone()
}

/// Aggregate error statistics from analytical vs numerical trajectory comparison.
pub(super) struct ReportStatistics {
    /// Maximum position error across all points (km).
    pub(super) max_position_error_km: f64,
    /// Mean position error across all points (km).
    pub(super) mean_position_error_km: f64,
    /// RMS position error across all points (km).
    pub(super) rms_position_error_km: f64,
    /// Maximum velocity error across all points (km/s).
    pub(super) max_velocity_error_km_s: f64,
}

/// Compute aggregate report statistics from per-leg validation points.
///
/// Returns zero-valued statistics for empty input.
pub(super) fn compute_report_statistics(leg_points: &[Vec<ValidationPoint>]) -> ReportStatistics {
    let mut max_pos = 0.0_f64;
    let mut max_vel = 0.0_f64;
    let mut sum_pos = 0.0_f64;
    let mut sum_pos_sq = 0.0_f64;
    let mut count = 0_u32;

    for points in leg_points {
        for p in points {
            max_pos = max_pos.max(p.position_error_km);
            max_vel = max_vel.max(p.velocity_error_km_s);
            sum_pos += p.position_error_km;
            sum_pos_sq += p.position_error_km * p.position_error_km;
            count += 1;
        }
    }

    if count == 0 {
        return ReportStatistics {
            max_position_error_km: 0.0,
            mean_position_error_km: 0.0,
            rms_position_error_km: 0.0,
            max_velocity_error_km_s: 0.0,
        };
    }

    let mean_pos = sum_pos / f64::from(count);
    let rms_pos = (sum_pos_sq / f64::from(count)).sqrt();
    ReportStatistics {
        max_position_error_km: max_pos,
        mean_position_error_km: mean_pos,
        rms_position_error_km: rms_pos,
        max_velocity_error_km_s: max_vel,
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::mission::types::ValidationPoint;
    use crate::types::RICState;

    /// Tolerance for exact-arithmetic statistics (max, mean, sum).
    /// These involve only addition/comparison of representable f64 values,
    /// so agreement to machine epsilon is expected.
    const EXACT_ARITHMETIC_TOL: f64 = 1e-15;

    /// Tolerance for RMS computation.
    /// `sqrt` introduces ~1 ULP of floating-point error beyond the exact sum,
    /// so we allow a slightly wider tolerance than exact arithmetic.
    const RMS_COMPUTATION_TOL: f64 = 1e-12;

    /// Verify `compute_report_statistics` with known error values.
    ///
    /// Three points with position errors [1.0, 3.0, 2.0] km:
    /// max=3.0, mean=2.0, rms=sqrt(14/3)≈2.160, max_vel=0.03.
    /// Also verifies empty input returns zeros.
    #[test]
    fn validation_report_statistics() {
        let make_point = |pos_err: f64, vel_err: f64| ValidationPoint {
            elapsed_s: 0.0,
            analytical_ric: RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::zeros(),
            },
            numerical_ric: RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::zeros(),
            },
            position_error_km: pos_err,
            velocity_error_km_s: vel_err,
        };

        let leg = vec![
            make_point(1.0, 0.01),
            make_point(3.0, 0.03),
            make_point(2.0, 0.02),
        ];
        let stats = super::compute_report_statistics(&[leg]);

        assert!(
            (stats.max_position_error_km - 3.0).abs() < EXACT_ARITHMETIC_TOL,
            "max_pos = {}, expected 3.0", stats.max_position_error_km
        );
        assert!(
            (stats.mean_position_error_km - 2.0).abs() < EXACT_ARITHMETIC_TOL,
            "mean_pos = {}, expected 2.0", stats.mean_position_error_km
        );
        // rms = sqrt((1 + 9 + 4) / 3) = sqrt(14/3) ≈ 2.160
        let expected_rms = (14.0_f64 / 3.0).sqrt();
        assert!(
            (stats.rms_position_error_km - expected_rms).abs() < RMS_COMPUTATION_TOL,
            "rms_pos = {}, expected {expected_rms}", stats.rms_position_error_km
        );
        assert!(
            (stats.max_velocity_error_km_s - 0.03).abs() < EXACT_ARITHMETIC_TOL,
            "max_vel = {}, expected 0.03", stats.max_velocity_error_km_s
        );

        // Empty input returns zeros
        let empty = super::compute_report_statistics(&[]);
        assert_eq!(empty.max_position_error_km, 0.0, "empty max_pos");
        assert_eq!(empty.mean_position_error_km, 0.0, "empty mean_pos");
        assert_eq!(empty.rms_position_error_km, 0.0, "empty rms_pos");
        assert_eq!(empty.max_velocity_error_km_s, 0.0, "empty max_vel");
    }
}

//! Aggregate report statistics for validation comparisons.

use nalgebra::Vector3;

use rpo_core::propagation::propagator::PropagatedState;
use rpo_core::types::RICState;

use rpo_core::mission::types::{LegValidationSummary, ValidationPoint};

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
///
/// Intermediate between per-leg [`LegValidationSummary`] and the final
/// [`ValidationReport`](rpo_core::mission::types::ValidationReport) fields.
/// Produced by [`compute_report_statistics`] and consumed when populating the report.
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

/// Derive aggregate report statistics from pre-computed per-leg summaries.
///
/// Folds per-leg max/mean/rms into global statistics in a single pass over
/// the summaries (the raw `leg_points` are not re-traversed). Returns
/// zero-valued statistics when all summaries have `num_points == 0`.
///
/// # Invariants
/// - Summaries must have consistent `mean`/`rms`/`num_points` values
///   (i.e., produced by [`compute_leg_summaries`])
pub(super) fn compute_report_statistics(summaries: &[LegValidationSummary]) -> ReportStatistics {
    let mut max_pos = 0.0_f64;
    let mut max_vel = 0.0_f64;
    let mut weighted_sum_pos = 0.0_f64;
    let mut weighted_sum_pos_sq = 0.0_f64;
    let mut total_count = 0_u32;

    for s in summaries {
        if s.num_points == 0 {
            continue;
        }
        let n = f64::from(s.num_points);
        max_pos = max_pos.max(s.max_position_error_km);
        max_vel = max_vel.max(s.max_velocity_error_km_s);
        // Recover per-leg sum and sum-of-squares from mean and rms:
        //   mean = sum / n  =>  sum = mean * n
        //   rms  = sqrt(sum_sq / n)  =>  sum_sq = rms^2 * n
        weighted_sum_pos += s.mean_position_error_km * n;
        weighted_sum_pos_sq += s.rms_position_error_km * s.rms_position_error_km * n;
        total_count += s.num_points;
    }

    if total_count == 0 {
        return ReportStatistics {
            max_position_error_km: 0.0,
            mean_position_error_km: 0.0,
            rms_position_error_km: 0.0,
            max_velocity_error_km_s: 0.0,
        };
    }
    let total = f64::from(total_count);
    ReportStatistics {
        max_position_error_km: max_pos,
        mean_position_error_km: weighted_sum_pos / total,
        rms_position_error_km: (weighted_sum_pos_sq / total).sqrt(),
        max_velocity_error_km_s: max_vel,
    }
}

/// Compute per-leg validation summaries (post-COLA points excluded).
///
/// Returns one [`LegValidationSummary`] per leg. Empty legs or legs where all
/// points are post-COLA produce zero-valued summaries.
///
/// # Invariants
/// - Each inner `Vec<ValidationPoint>` represents a single leg's trajectory samples
/// - `position_error_km` and `velocity_error_km_s` values must be non-negative
pub(super) fn compute_leg_summaries(
    leg_points: &[Vec<ValidationPoint>],
) -> Vec<LegValidationSummary> {
    leg_points.iter().map(|points| {
        let mut max_pos = 0.0_f64;
        let mut max_vel = 0.0_f64;
        let mut sum_pos = 0.0_f64;
        let mut sum_pos_sq = 0.0_f64;
        let mut count = 0_u32;
        let mut excluded = 0_u32;

        for p in points {
            if p.post_cola {
                excluded += 1;
                continue;
            }
            max_pos = max_pos.max(p.position_error_km);
            max_vel = max_vel.max(p.velocity_error_km_s);
            sum_pos += p.position_error_km;
            sum_pos_sq += p.position_error_km * p.position_error_km;
            count += 1;
        }

        if count == 0 {
            return LegValidationSummary {
                max_position_error_km: 0.0,
                mean_position_error_km: 0.0,
                rms_position_error_km: 0.0,
                max_velocity_error_km_s: 0.0,
                num_points: 0,
                num_post_cola_excluded: excluded,
            };
        }

        let n = f64::from(count);
        LegValidationSummary {
            max_position_error_km: max_pos,
            mean_position_error_km: sum_pos / n,
            rms_position_error_km: (sum_pos_sq / n).sqrt(),
            max_velocity_error_km_s: max_vel,
            num_points: count,
            num_post_cola_excluded: excluded,
        }
    }).collect()
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use rpo_core::mission::types::ValidationPoint;
    use rpo_core::types::RICState;

    /// Tolerance for exact-arithmetic statistics (max, mean, sum).
    /// These involve only addition/comparison of representable f64 values,
    /// so agreement to machine epsilon is expected.
    const EXACT_ARITHMETIC_TOL: f64 = 1e-15;

    /// Tolerance for RMS computation.
    /// `sqrt` introduces ~1 ULP of floating-point error beyond the exact sum,
    /// so we allow a slightly wider tolerance than exact arithmetic.
    const RMS_COMPUTATION_TOL: f64 = 1e-12;

    /// Verify `compute_report_statistics` derives correct aggregates from leg summaries.
    ///
    /// Two legs: leg 0 has 3 points (max=3.0, mean=2.0, rms=sqrt(14/3), `max_vel=0.03`),
    /// leg 1 has 2 points (max=2.0, mean=1.5, `max_vel=0.02`).
    /// Global: max=3.0, mean=(2.0*3+1.5*2)/5=1.8, `max_vel=0.03`.
    /// Also verifies empty input returns zeros.
    #[test]
    fn report_statistics_from_summaries() {
        use rpo_core::mission::types::LegValidationSummary;

        let summaries = vec![
            LegValidationSummary {
                max_position_error_km: 3.0,
                mean_position_error_km: 2.0,
                rms_position_error_km: (14.0_f64 / 3.0).sqrt(),
                max_velocity_error_km_s: 0.03,
                num_points: 3,
                num_post_cola_excluded: 0,
            },
            LegValidationSummary {
                max_position_error_km: 2.0,
                mean_position_error_km: 1.5,
                rms_position_error_km: (2.5_f64).sqrt(), // sqrt((1+4)/2)
                max_velocity_error_km_s: 0.02,
                num_points: 2,
                num_post_cola_excluded: 1,
            },
        ];

        let stats = super::compute_report_statistics(&summaries);

        assert!(
            (stats.max_position_error_km - 3.0).abs() < EXACT_ARITHMETIC_TOL,
            "max_pos = {}, expected 3.0", stats.max_position_error_km
        );
        // Global mean = (2.0*3 + 1.5*2) / 5 = 9.0/5 = 1.8
        assert!(
            (stats.mean_position_error_km - 1.8).abs() < EXACT_ARITHMETIC_TOL,
            "mean_pos = {}, expected 1.8", stats.mean_position_error_km
        );
        // Global rms = sqrt((14/3*3 + 2.5*2) / 5) = sqrt((14+5)/5) = sqrt(19/5)
        let expected_rms = (19.0_f64 / 5.0).sqrt();
        assert!(
            (stats.rms_position_error_km - expected_rms).abs() < RMS_COMPUTATION_TOL,
            "rms_pos = {}, expected {expected_rms}", stats.rms_position_error_km
        );
        assert!(
            (stats.max_velocity_error_km_s - 0.03).abs() < EXACT_ARITHMETIC_TOL,
            "max_vel = {}, expected 0.03", stats.max_velocity_error_km_s
        );

        // Empty input returns structural zeros (default-initialized f64, never
        // subject to arithmetic that could yield -0.0), so bit-pattern equality
        // against `+0.0` is the exact contract.
        let empty = super::compute_report_statistics(&[]);
        assert_eq!(empty.max_position_error_km.to_bits(), 0_u64, "empty max_pos");
        assert_eq!(empty.mean_position_error_km.to_bits(), 0_u64, "empty mean_pos");
        assert_eq!(empty.rms_position_error_km.to_bits(), 0_u64, "empty rms_pos");
        assert_eq!(empty.max_velocity_error_km_s.to_bits(), 0_u64, "empty max_vel");
    }

    /// Verify all-zero summaries (e.g. all post-COLA) produce zero aggregate.
    #[test]
    fn report_statistics_all_zero_points() {
        use rpo_core::mission::types::LegValidationSummary;

        let summaries = vec![LegValidationSummary {
            max_position_error_km: 0.0,
            mean_position_error_km: 0.0,
            rms_position_error_km: 0.0,
            max_velocity_error_km_s: 0.0,
            num_points: 0,
            num_post_cola_excluded: 5,
        }];

        let stats = super::compute_report_statistics(&summaries);
        // All-zero summaries: no arithmetic path can yield -0.0, so +0.0 bits.
        assert_eq!(stats.max_position_error_km.to_bits(), 0_u64);
        assert_eq!(stats.mean_position_error_km.to_bits(), 0_u64);
    }

    /// Verify `compute_leg_summaries` with mixed pre/post-COLA points across legs.
    ///
    /// Leg 0: errors [1.0, 3.0, 2.0], no post-COLA → max=3.0, mean=2.0, rms=sqrt(14/3)
    /// Leg 1: errors [1.0(pre), 2.0(pre), 10.0(post)] → max=2.0, mean=1.5, 1 excluded
    /// Leg 2: all post-COLA → `num_points=0`, 2 excluded
    #[test]
    fn leg_summaries_mixed_cola() {
        let make = |pos_err: f64, vel_err: f64, post_cola: bool| ValidationPoint {
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
            post_cola,
        };

        let legs = vec![
            // Leg 0: all pre-COLA
            vec![make(1.0, 0.01, false), make(3.0, 0.03, false), make(2.0, 0.02, false)],
            // Leg 1: mixed
            vec![make(1.0, 0.01, false), make(2.0, 0.02, false), make(10.0, 0.1, true)],
            // Leg 2: all post-COLA
            vec![make(5.0, 0.05, true), make(7.0, 0.07, true)],
        ];

        let summaries = super::compute_leg_summaries(&legs);
        assert_eq!(summaries.len(), 3);

        // Leg 0
        assert!((summaries[0].max_position_error_km - 3.0).abs() < EXACT_ARITHMETIC_TOL);
        assert!((summaries[0].mean_position_error_km - 2.0).abs() < EXACT_ARITHMETIC_TOL);
        let expected_rms_0 = (14.0_f64 / 3.0).sqrt();
        assert!((summaries[0].rms_position_error_km - expected_rms_0).abs() < RMS_COMPUTATION_TOL);
        assert!((summaries[0].max_velocity_error_km_s - 0.03).abs() < EXACT_ARITHMETIC_TOL);
        assert_eq!(summaries[0].num_points, 3);
        assert_eq!(summaries[0].num_post_cola_excluded, 0);

        // Leg 1: post-COLA excluded
        assert!((summaries[1].max_position_error_km - 2.0).abs() < EXACT_ARITHMETIC_TOL);
        assert!((summaries[1].mean_position_error_km - 1.5).abs() < EXACT_ARITHMETIC_TOL);
        assert_eq!(summaries[1].num_points, 2);
        assert_eq!(summaries[1].num_post_cola_excluded, 1);

        // Leg 2: all excluded — max starts at +0.0 and never updates.
        assert_eq!(summaries[2].max_position_error_km.to_bits(), 0_u64);
        assert_eq!(summaries[2].num_points, 0);
        assert_eq!(summaries[2].num_post_cola_excluded, 2);
    }

    /// Empty input produces empty summaries.
    #[test]
    fn leg_summaries_empty() {
        let summaries = super::compute_leg_summaries(&[]);
        assert!(summaries.is_empty());
    }

    /// A leg with no points produces a zero-valued summary.
    #[test]
    fn leg_summaries_empty_leg() {
        let summaries = super::compute_leg_summaries(&[vec![]]);
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].num_points, 0);
        assert_eq!(summaries[0].num_post_cola_excluded, 0);
        // No input means no update to `max_position_error_km`; stays at +0.0.
        assert_eq!(summaries[0].max_position_error_km.to_bits(), 0_u64);
    }
}

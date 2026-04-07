//! Analytical vs numerical eclipse validation: interval extraction, matching, and comparison.

use hifitime::Epoch;
use nalgebra::Vector3;

use rpo_core::constants::ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
use rpo_core::elements::eclipse::{compute_eclipse_state, sun_position_eci_km};
use rpo_core::types::{EclipseInterval, EclipseState, EclipseSummary};

use rpo_core::mission::types::{
    EclipseIntervalComparison, EclipseValidation, EclipseValidationPoint,
};

/// Classify the worst-case shadow state from a maximum eclipse percentage.
fn classify_shadow_state(max_pct: f64) -> EclipseState {
    if max_pct >= UMBRA_PERCENTAGE_THRESHOLD {
        EclipseState::Umbra
    } else {
        EclipseState::Penumbra {
            shadow_fraction: max_pct / 100.0,
        }
    }
}

/// Guard threshold for flat eclipse percentage segments in [`interpolate_crossing`].
/// When |pct1 - pct0| < machine epsilon, the segment is flat and interpolation
/// is undefined; fall back to the current sample epoch to avoid division by zero.
const ECLIPSE_INTERPOLATION_FLAT_GUARD: f64 = f64::EPSILON;

/// Shadow percentage at or above which the eclipse state is classified as full umbra
/// rather than penumbra. Based on physical convention: >99% shadow is effectively total.
const UMBRA_PERCENTAGE_THRESHOLD: f64 = 99.0;

/// Per-sample eclipse data collected during nyx propagation for later comparison.
pub(super) struct EclipseSample {
    /// Elapsed time since mission start (seconds).
    pub(super) elapsed_s: f64,
    /// Epoch at this sample.
    pub(super) epoch: Epoch,
    /// ANISE eclipse percentage (0-100).
    pub(super) numerical_pct: f64,
    /// Sun position in ECI J2000 (km) from ANISE.
    pub(super) anise_sun_eci_km: Vector3<f64>,
    /// Chief position in ECI J2000 (km) at this sample.
    pub(super) chief_eci_km: Vector3<f64>,
}

/// Compute eclipse intervals from per-sample ANISE eclipse data.
///
/// Walks the sample array with a sliding window, detects transitions between
/// sunlit (<1% eclipse) and shadow (>=1% eclipse) states, and records
/// contiguous shadow intervals with worst-case shadow state.
///
/// When a transition is detected between adjacent samples `(t0, pct0)` and
/// `(t1, pct1)`, linear interpolation finds the crossing epoch:
///
/// ```text
/// t_cross = t0 + (t1 - t0) * (threshold - pct0) / (pct1 - pct0)
/// ```
///
/// This reduces eclipse boundary error from +(`sample_interval`/2) to the
/// non-linearity of the shadow geometry between samples.
///
/// # Arguments
/// * `samples` -- Time-ordered [`EclipseSample`] array. Uses `epoch` and
///   `numerical_pct` fields (0.0 = fully sunlit, 100.0 = full umbra).
///
/// # Returns
/// A vector of [`EclipseInterval`] for each detected shadow interval.
/// Returns an empty vector if fewer than 2 samples are provided.
fn compute_numerical_eclipse_intervals(
    samples: &[EclipseSample],
) -> Vec<EclipseInterval> {
    if samples.len() < 2 {
        return Vec::new();
    }

    let threshold = ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
    let mut intervals = Vec::new();
    let mut in_shadow = false;
    let mut shadow_start = samples[0].epoch;
    let mut max_pct = 0.0_f64;

    // Check if first sample is already in shadow (no interpolation possible)
    let first_pct = samples[0].numerical_pct;
    if first_pct >= threshold {
        in_shadow = true;
        shadow_start = samples[0].epoch;
        max_pct = first_pct;
    }

    for i in 1..samples.len() {
        let prev_epoch = samples[i - 1].epoch;
        let prev_pct = samples[i - 1].numerical_pct;
        let curr_epoch = samples[i].epoch;
        let curr_pct = samples[i].numerical_pct;

        if curr_pct >= threshold {
            if in_shadow {
                max_pct = max_pct.max(curr_pct);
            } else {
                // Entering shadow: interpolate crossing epoch
                in_shadow = true;
                max_pct = curr_pct;
                shadow_start = interpolate_crossing(
                    prev_epoch, prev_pct, curr_epoch, curr_pct, threshold,
                );
            }
        } else if in_shadow {
            // Exiting shadow: interpolate crossing epoch
            in_shadow = false;
            let shadow_end = interpolate_crossing(
                prev_epoch, prev_pct, curr_epoch, curr_pct, threshold,
            );
            let duration_s = (shadow_end - shadow_start).to_seconds();
            intervals.push(EclipseInterval {
                start: shadow_start,
                end: shadow_end,
                duration_s,
                state: classify_shadow_state(max_pct),
            });
        }
    }

    // Close any open interval at end of samples
    if in_shadow {
        let last_epoch = samples[samples.len() - 1].epoch;
        let duration_s = (last_epoch - shadow_start).to_seconds();
        intervals.push(EclipseInterval {
            start: shadow_start,
            end: last_epoch,
            duration_s,
            state: classify_shadow_state(max_pct),
        });
    }

    intervals
}

/// Linearly interpolate the epoch at which the eclipse percentage crosses a threshold.
///
/// Given two adjacent samples `(t0, pct0)` and `(t1, pct1)` that straddle
/// `threshold`, returns the interpolated crossing epoch. Falls back to `t1`
/// if `pct1 == pct0` (flat segment, avoids division by zero).
fn interpolate_crossing(
    t0: Epoch,
    pct0: f64,
    t1: Epoch,
    pct1: f64,
    threshold: f64,
) -> Epoch {
    let dpct = pct1 - pct0;
    if dpct.abs() < ECLIPSE_INTERPOLATION_FLAT_GUARD {
        return t1;
    }
    let dt_s = (t1 - t0).to_seconds();
    let frac = (threshold - pct0) / dpct;
    // Clamp to [0, 1] to stay within the sample interval
    let frac = frac.clamp(0.0, 1.0);
    t0 + hifitime::Duration::from_seconds(frac * dt_s)
}

/// Match analytical and numerical eclipse intervals by closest start epoch.
///
/// Greedy matching: for each analytical interval, find the numerical interval
/// with the closest start epoch that has not already been matched. Unmatched
/// intervals are counted separately.
///
/// # Arguments
/// * `analytical` -- The analytical eclipse summary containing intervals to match.
/// * `numerical` -- The numerical eclipse intervals to match against.
///
/// # Returns
/// A tuple of `(comparisons, unmatched_count)` where `comparisons` contains
/// one entry per matched pair and `unmatched_count` is the number of intervals
/// that could not be paired.
fn match_eclipse_intervals(
    analytical: &EclipseSummary,
    numerical: &[EclipseInterval],
) -> (Vec<EclipseIntervalComparison>, usize) {
    let mut comparisons = Vec::new();
    let mut used = vec![false; numerical.len()];
    let mut matched_analytical = 0_usize;

    for a_interval in &analytical.intervals {
        // Find the closest unused numerical interval by start epoch
        let mut best_idx: Option<usize> = None;
        let mut best_diff = f64::INFINITY;

        for (j, n_interval) in numerical.iter().enumerate() {
            if used[j] {
                continue;
            }
            let diff = (a_interval.start - n_interval.start).to_seconds().abs();
            if diff < best_diff {
                best_diff = diff;
                best_idx = Some(j);
            }
        }

        if let Some(idx) = best_idx {
            used[idx] = true;
            matched_analytical += 1;

            let n_interval = &numerical[idx];
            let entry_error_s = (a_interval.start - n_interval.start).to_seconds();
            let exit_error_s = (a_interval.end - n_interval.end).to_seconds();
            let duration_error_s = a_interval.duration_s - n_interval.duration_s;

            comparisons.push(EclipseIntervalComparison {
                analytical_start: a_interval.start,
                numerical_start: n_interval.start,
                entry_error_s,
                analytical_end: a_interval.end,
                numerical_end: n_interval.end,
                exit_error_s,
                duration_error_s,
            });
        }
    }

    let unmatched_analytical = analytical.intervals.len() - matched_analytical;
    let unmatched_numerical = numerical.len() - used.iter().filter(|&&u| u).count();
    let unmatched_count = unmatched_analytical + unmatched_numerical;

    (comparisons, unmatched_count)
}

/// Build eclipse validation from collected ANISE samples and analytical eclipse data.
///
/// Compares per-point analytical (Meeus) vs numerical (ANISE) Sun direction and
/// eclipse state, matches eclipse intervals, and computes aggregate statistics.
///
/// Returns `None` if there are no eclipse samples.
pub(super) fn build_eclipse_validation(
    eclipse_data: &rpo_core::types::MissionEclipseData,
    eclipse_samples: &[EclipseSample],
) -> Option<EclipseValidation> {
    if eclipse_samples.is_empty() {
        return None;
    }

    // Build per-point comparison
    let mut ev_points = Vec::with_capacity(eclipse_samples.len());
    let mut sum_sun_err = 0.0_f64;
    let mut max_sun_err = 0.0_f64;

    for sample in eclipse_samples {
        // Analytical Sun position at this epoch (Meeus Ch. 25)
        let meeus_sun = sun_position_eci_km(sample.epoch);
        let meeus_sun_dir = meeus_sun.normalize();
        let anise_sun_dir = sample.anise_sun_eci_km.normalize();
        let sun_err = meeus_sun_dir
            .dot(&anise_sun_dir)
            .clamp(-1.0, 1.0)
            .acos();

        // Analytical eclipse state at nyx chief position using Meeus Sun
        let analytical_state = compute_eclipse_state(&sample.chief_eci_km, &meeus_sun);
        let analytical_pct = match analytical_state {
            EclipseState::Sunlit => 0.0,
            EclipseState::Penumbra { shadow_fraction } => shadow_fraction * 100.0,
            EclipseState::Umbra => 100.0,
        };

        let pct_err = (analytical_pct - sample.numerical_pct).abs();
        max_sun_err = max_sun_err.max(sun_err);
        sum_sun_err += sun_err;

        ev_points.push(EclipseValidationPoint {
            elapsed_s: sample.elapsed_s,
            analytical_eclipse_pct: analytical_pct,
            numerical_eclipse_pct: sample.numerical_pct,
            eclipse_pct_error: pct_err,
            sun_direction_error_rad: sun_err,
        });
    }

    let numerical_intervals = compute_numerical_eclipse_intervals(eclipse_samples);

    // Match analytical vs numerical intervals
    let (comparisons, unmatched) =
        match_eclipse_intervals(&eclipse_data.summary, &numerical_intervals);

    // Compute aggregate statistics
    let n = ev_points.len();
    let mean_sun_err = if n == 0 {
        0.0
    } else {
        sum_sun_err / f64::from(u32::try_from(n).ok()?)
    };
    let max_timing = comparisons
        .iter()
        .flat_map(|c| [c.entry_error_s.abs(), c.exit_error_s.abs()])
        .fold(0.0_f64, f64::max);
    let mean_timing = if comparisons.is_empty() {
        0.0
    } else {
        let sum: f64 = comparisons
            .iter()
            .flat_map(|c| [c.entry_error_s.abs(), c.exit_error_s.abs()])
            .sum();
        let denom = 2.0 * f64::from(u32::try_from(comparisons.len()).ok()?);
        sum / denom
    };
    let matched_count = comparisons.len();

    Some(EclipseValidation {
        points: ev_points,
        interval_comparisons: comparisons,
        max_sun_direction_error_rad: max_sun_err,
        mean_sun_direction_error_rad: mean_sun_err,
        max_timing_error_s: max_timing,
        mean_timing_error_s: mean_timing,
        analytical_interval_count: eclipse_data.summary.intervals.len(),
        numerical_interval_count: numerical_intervals.len(),
        matched_interval_count: matched_count,
        unmatched_interval_count: unmatched,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use crate::nyx_bridge;
    use rpo_core::propagation::propagator::PropagationModel;
    use rpo_core::test_helpers::{iss_like_elements, test_epoch};
    use rpo_core::types::SpacecraftConfig;

    // Named tolerance constants for eclipse validation tests

    /// Serde roundtrip precision for f64 fields. JSON serialization
    /// preserves full f64 precision; 1e-15 is at machine epsilon.
    const SERDE_ROUNDTRIP_TOL: f64 = 1e-15;

    /// Eclipse interval matching: entry/exit/duration error for
    /// exact-match test cases. 1e-10 s for identical intervals.
    const INTERVAL_EXACT_MATCH_TOL_S: f64 = 1e-10;

    /// Eclipse numerical interval duration tolerance.
    /// Sample-spaced intervals have +/-1 sample quantization;
    /// at 1s spacing, 2s covers worst case.
    const INTERVAL_DURATION_TOL_S: f64 = 2.0;

    /// Maximum expected Sun direction angular error (Meeus vs ANISE DE440s).
    /// Meeus Ch. 25 with precession correction gives ~0.005 deg at 2024 epoch.
    /// 0.02 deg (3.5e-4 rad) provides margin for nutation/perturbation residuals.
    const SUN_DIRECTION_VALIDATION_TOL_RAD: f64 = 3.5e-4;

    /// Maximum expected Moon direction angular error (Meeus vs ANISE DE440s).
    /// Meeus Ch. 47 truncated series gives ~0.007 deg with precession correction.
    /// 1.0 deg (0.0175 rad) provides generous margin for the truncated series.
    const MOON_DIRECTION_VALIDATION_TOL_RAD: f64 = 0.0175;

    /// Maximum eclipse entry/exit timing error (seconds).
    /// With linear interpolation of eclipse boundaries between samples,
    /// the error is dominated by the non-linearity of the shadow geometry
    /// rather than sample-interval quantization.
    const ECLIPSE_TIMING_VALIDATION_TOL_S: f64 = 120.0;

    /// Interpolation accuracy tolerance for eclipse boundary tests.
    /// Linear interpolation of a linear ramp should be exact to floating-point
    /// precision; 1e-9 s provides margin for epoch arithmetic rounding.
    const INTERPOLATION_ACCURACY_TOL_S: f64 = 1e-9;

    #[test]
    fn eclipse_validation_serde_roundtrip() {
        use rpo_core::mission::types::EclipseValidationPoint;

        let point = EclipseValidationPoint {
            elapsed_s: 100.0,
            analytical_eclipse_pct: 0.0,
            numerical_eclipse_pct: 0.5,
            eclipse_pct_error: 0.5,
            sun_direction_error_rad: 1e-4,
        };
        let json = serde_json::to_string(&point).unwrap();
        let recovered: EclipseValidationPoint = serde_json::from_str(&json).unwrap();
        assert!(
            (recovered.elapsed_s - 100.0).abs() < SERDE_ROUNDTRIP_TOL,
            "elapsed_s roundtrip"
        );
        assert!(
            (recovered.eclipse_pct_error - 0.5).abs() < SERDE_ROUNDTRIP_TOL,
            "eclipse_pct_error roundtrip"
        );
    }

    #[test]
    fn eclipse_validation_tolerances_are_reasonable() {
        use rpo_core::constants::ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
        // Sun direction: 0.02 deg = 3.5e-4 rad (Meeus with precession correction)
        assert!(SUN_DIRECTION_VALIDATION_TOL_RAD > 0.0);
        assert!(SUN_DIRECTION_VALIDATION_TOL_RAD < 0.01); // < 0.57 deg
        // Moon direction: 1.0 deg = 0.0175 rad (Meeus truncated ~0.5 deg)
        assert!(MOON_DIRECTION_VALIDATION_TOL_RAD > SUN_DIRECTION_VALIDATION_TOL_RAD);
        assert!(MOON_DIRECTION_VALIDATION_TOL_RAD < 0.1); // < 5.7 deg
        // Eclipse timing: <120s (dominated by sample-interval quantization)
        assert!(ECLIPSE_TIMING_VALIDATION_TOL_S > 0.0);
        assert!(ECLIPSE_TIMING_VALIDATION_TOL_S < 300.0);
        // Eclipse percentage agreement: <1% difference
        assert!(ECLIPSE_PERCENTAGE_AGREEMENT_TOL > 0.0);
        assert!(ECLIPSE_PERCENTAGE_AGREEMENT_TOL < 10.0);
    }

    // =========================================================================
    // Eclipse Interval Matching Tests
    // =========================================================================

    /// Helper to create an EclipseInterval for testing.
    fn make_eclipse_interval(start_s: f64, end_s: f64) -> rpo_core::types::EclipseInterval {
        use rpo_core::types::{EclipseInterval, EclipseState};
        let base = rpo_core::test_helpers::test_epoch();
        EclipseInterval {
            start: base + hifitime::Duration::from_seconds(start_s),
            end: base + hifitime::Duration::from_seconds(end_s),
            duration_s: end_s - start_s,
            state: EclipseState::Umbra,
        }
    }

    /// Helper to create an EclipseSummary from intervals.
    fn make_eclipse_summary(
        intervals: Vec<rpo_core::types::EclipseInterval>,
    ) -> rpo_core::types::EclipseSummary {
        let total = intervals.iter().map(|i| i.duration_s).sum::<f64>();
        rpo_core::types::EclipseSummary {
            intervals,
            total_shadow_duration_s: total,
            time_in_shadow_fraction: 0.0,
            max_shadow_duration_s: 0.0,
        }
    }

    #[test]
    fn eclipse_interval_matching_exact() {
        let intervals = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
        ];
        let summary = make_eclipse_summary(intervals.clone());

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &intervals);

        assert_eq!(comparisons.len(), 2, "should match both intervals");
        assert_eq!(unmatched, 0, "no unmatched intervals");
        for c in &comparisons {
            assert!(
                c.entry_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S,
                "entry error should be zero"
            );
            assert!(c.exit_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S, "exit error should be zero");
            assert!(
                c.duration_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S,
                "duration error should be zero"
            );
        }
    }

    #[test]
    fn eclipse_interval_matching_shifted() {
        let analytical = vec![make_eclipse_interval(100.0, 200.0)];
        let numerical = vec![make_eclipse_interval(102.0, 198.0)];
        let summary = make_eclipse_summary(analytical);

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &numerical);

        assert_eq!(comparisons.len(), 1);
        assert_eq!(unmatched, 0);
        // analytical starts 2s before numerical -> entry_error = 100 - 102 = -2.0
        assert!(
            (comparisons[0].entry_error_s - (-2.0)).abs() < INTERVAL_EXACT_MATCH_TOL_S,
            "entry_error = {}, expected -2.0",
            comparisons[0].entry_error_s
        );
        // analytical ends 2s after numerical -> exit_error = 200 - 198 = 2.0
        assert!(
            (comparisons[0].exit_error_s - 2.0).abs() < INTERVAL_EXACT_MATCH_TOL_S,
            "exit_error = {}, expected 2.0",
            comparisons[0].exit_error_s
        );
    }

    #[test]
    fn eclipse_interval_unmatched() {
        let analytical = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
            make_eclipse_interval(900.0, 1000.0),
        ];
        let numerical = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
        ];
        let summary = make_eclipse_summary(analytical);

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &numerical);

        assert_eq!(comparisons.len(), 2, "should match 2 of 3");
        assert!(unmatched > 0, "should have unmatched intervals");
    }

    #[test]
    fn eclipse_comparison_point_construction() {
        use rpo_core::mission::types::EclipseValidationPoint;

        let point = EclipseValidationPoint {
            elapsed_s: 100.0,
            analytical_eclipse_pct: 0.0,
            numerical_eclipse_pct: 50.0,
            eclipse_pct_error: 50.0,
            sun_direction_error_rad: 1e-4,
        };
        assert!((point.eclipse_pct_error - 50.0).abs() < SERDE_ROUNDTRIP_TOL);
        assert!((point.sun_direction_error_rad - 1e-4).abs() < SERDE_ROUNDTRIP_TOL);
    }

    /// Helper to build an `EclipseSample` for interval detection tests.
    /// Only `epoch` and `numerical_pct` matter; other fields are zeroed.
    fn make_interval_sample(epoch: hifitime::Epoch, pct: f64, elapsed_s: f64) -> super::EclipseSample {
        super::EclipseSample {
            elapsed_s,
            epoch,
            numerical_pct: pct,
            anise_sun_eci_km: Vector3::zeros(),
            chief_eci_km: Vector3::zeros(),
        }
    }

    #[test]
    fn compute_numerical_intervals_from_samples() {
        let base = rpo_core::test_helpers::test_epoch();
        let samples: Vec<super::EclipseSample> = (0..100)
            .map(|i| {
                let t = f64::from(i);
                let epoch = base + hifitime::Duration::from_seconds(t);
                // Eclipse from t=20 to t=40 and t=60 to t=80
                let pct = if (20.0..=40.0).contains(&t) || (60.0..=80.0).contains(&t) {
                    100.0
                } else {
                    0.0
                };
                make_interval_sample(epoch, pct, t)
            })
            .collect();

        let intervals = super::compute_numerical_eclipse_intervals(&samples);

        assert_eq!(
            intervals.len(),
            2,
            "should detect 2 eclipse intervals, got {}",
            intervals.len()
        );
        // First interval should be ~20s duration
        assert!(
            (intervals[0].duration_s - 20.0).abs() < INTERVAL_DURATION_TOL_S,
            "first interval duration = {}, expected ~20",
            intervals[0].duration_s
        );
    }

    /// Verify that eclipse boundary interpolation accurately locates the
    /// threshold crossing for a linear ramp in eclipse percentage.
    ///
    /// Constructs samples with known linear transitions:
    /// - t=0..90: sunlit (0%)
    /// - t=90..110: linear ramp 0% -> 100% (threshold 1% crossed at t~90.2)
    /// - t=110..190: full umbra (100%)
    /// - t=190..210: linear ramp 100% -> 0% (threshold 1% crossed at t~209.8)
    /// - t=210..300: sunlit (0%)
    ///
    /// With linear interpolation, the entry/exit epochs should be accurate
    /// to floating-point precision (not quantized to sample boundaries).
    #[test]
    fn eclipse_boundary_interpolation_accuracy() {
        let base = rpo_core::test_helpers::test_epoch();

        // 10s sample spacing, 31 samples from t=0 to t=300
        let samples: Vec<super::EclipseSample> = (0..=30)
            .map(|i| {
                let t = f64::from(i) * 10.0;
                let epoch = base + hifitime::Duration::from_seconds(t);
                let pct = if t <= 90.0 {
                    0.0
                } else if t <= 110.0 {
                    // Linear ramp: 0% at t=90, 100% at t=110
                    (t - 90.0) / 20.0 * 100.0
                } else if t <= 190.0 {
                    100.0
                } else if t <= 210.0 {
                    // Linear ramp: 100% at t=190, 0% at t=210
                    (210.0 - t) / 20.0 * 100.0
                } else {
                    0.0
                };
                make_interval_sample(epoch, pct, t)
            })
            .collect();

        let intervals = super::compute_numerical_eclipse_intervals(&samples);

        assert_eq!(intervals.len(), 1, "should detect 1 eclipse interval");

        // Entry: threshold (1%) crossing on 0%->50% ramp between t=90 and t=100
        // Interpolation: t_cross = 90 + 10 * (1 - 0) / (50 - 0) = 90.2
        let expected_entry_s = 90.2;
        let actual_entry_s = (intervals[0].start - base).to_seconds();
        let entry_err = (actual_entry_s - expected_entry_s).abs();
        assert!(
            entry_err < INTERPOLATION_ACCURACY_TOL_S,
            "entry at {actual_entry_s:.6}s, expected {expected_entry_s:.6}s, error {entry_err:.2e}s"
        );

        // Exit: threshold (1%) crossing on 50%->0% ramp between t=200 and t=210
        // Interpolation: t_cross = 200 + 10 * (1 - 50) / (0 - 50) = 200 + 10 * 49/50 = 209.8
        let expected_exit_s = 209.8;
        let actual_exit_s = (intervals[0].end - base).to_seconds();
        let exit_err = (actual_exit_s - expected_exit_s).abs();
        assert!(
            exit_err < INTERPOLATION_ACCURACY_TOL_S,
            "exit at {actual_exit_s:.6}s, expected {expected_exit_s:.6}s, error {exit_err:.2e}s"
        );

        // Duration should match
        let expected_duration = expected_exit_s - expected_entry_s;
        let duration_err = (intervals[0].duration_s - expected_duration).abs();
        assert!(
            duration_err < INTERPOLATION_ACCURACY_TOL_S,
            "duration {:.6}s, expected {expected_duration:.6}s, error {duration_err:.2e}s",
            intervals[0].duration_s,
        );
    }

    // =========================================================================
    // Eclipse Validation Integration Tests (require MetaAlmanac)
    // =========================================================================

    /// Full multi-waypoint mission with eclipse validation.
    /// Verifies that `validate_mission_nyx()` produces eclipse validation data
    /// and that Sun direction and timing errors are within tolerances.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_eclipse_full_mission() {
        use rpo_core::mission::waypoints::plan_waypoint_mission;
        use rpo_core::test_helpers::deputy_from_roe;
        use rpo_core::mission::config::MissionConfig;
        use rpo_core::mission::types::Waypoint;
        use rpo_core::types::{DepartureState, QuasiNonsingularROE};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.3 / a,
            dey: -0.2 / a,
            dix: 0.2 / a,
            diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period().unwrap();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.5, 3.0, 0.5),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(1.0, -2.0, 1.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Some(Vector3::zeros()),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        // Verify eclipse data was computed
        assert!(
            mission.eclipse.is_some(),
            "mission should have eclipse data"
        );

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let val_config = crate::validation::ValidationConfig {
            samples_per_leg: 50,
            chief_config: SpacecraftConfig::SERVICER_500KG,
            deputy_config: SpacecraftConfig::SERVICER_500KG,
        };

        let report = crate::validation::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            &val_config,
            &[],
            &almanac,
        )
        .expect("validation should succeed");

        // Eclipse validation should be present
        let ev = report
            .eclipse_validation
            .expect("eclipse validation should be present");

        eprintln!("Eclipse validation results:");
        eprintln!(
            "  Sun direction error: max {:.6} deg mean {:.6} deg",
            ev.max_sun_direction_error_rad.to_degrees(),
            ev.mean_sun_direction_error_rad.to_degrees(),
        );
        eprintln!(
            "  Intervals: {} analytical, {} numerical, {} matched, {} unmatched",
            ev.analytical_interval_count,
            ev.numerical_interval_count,
            ev.matched_interval_count,
            ev.unmatched_interval_count,
        );
        if !ev.interval_comparisons.is_empty() {
            eprintln!(
                "  Timing error: max {:.2} s, mean {:.2} s",
                ev.max_timing_error_s, ev.mean_timing_error_s,
            );
        }

        // Sun direction should be within Meeus accuracy
        assert!(
            ev.max_sun_direction_error_rad < SUN_DIRECTION_VALIDATION_TOL_RAD,
            "max Sun direction error = {:.6} deg (expected < {:.4} deg)",
            ev.max_sun_direction_error_rad.to_degrees(),
            SUN_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );

        // Timing error should be within tolerance (if intervals were matched)
        if !ev.interval_comparisons.is_empty() {
            assert!(
                ev.max_timing_error_s < ECLIPSE_TIMING_VALIDATION_TOL_S,
                "max timing error = {:.2} s (expected < {:.1} s)",
                ev.max_timing_error_s,
                ECLIPSE_TIMING_VALIDATION_TOL_S,
            );
        }

        // Should have some eclipse points
        assert!(
            !ev.points.is_empty(),
            "should have eclipse validation points"
        );
    }

    /// Meeus Sun position vs ANISE DE440s over 24 hours.
    /// Validates that the analytical Sun ephemeris (Meeus Ch. 25) agrees
    /// with the high-fidelity ANISE ephemeris within the stated accuracy.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_sun_direction_accuracy() {
        use rpo_core::elements::eclipse::sun_position_eci_km;
        use anise::constants::frames::{SUN_J2000, EARTH_J2000 as ANISE_EARTH_J2000};

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let base_epoch = test_epoch();

        let mut max_err = 0.0_f64;
        let mut sum_err = 0.0_f64;
        let n_samples = 864; // 24 hours at 100s intervals

        for i in 0..n_samples {
            let dt = f64::from(i) * 100.0;
            let epoch = base_epoch + hifitime::Duration::from_seconds(dt);

            // Analytical (Meeus)
            let meeus_sun = sun_position_eci_km(epoch);
            let meeus_dir = meeus_sun.normalize();

            // Numerical (ANISE DE440s)
            let sun_state = almanac
                .translate(SUN_J2000, ANISE_EARTH_J2000, epoch, None)
                .expect("Sun translate should succeed");
            let anise_dir = Vector3::new(
                sun_state.radius_km.x,
                sun_state.radius_km.y,
                sun_state.radius_km.z,
            )
            .normalize();

            let err = meeus_dir.dot(&anise_dir).clamp(-1.0, 1.0).acos();
            max_err = max_err.max(err);
            sum_err += err;
        }

        let mean_err = sum_err / f64::from(n_samples);
        eprintln!(
            "Sun direction accuracy (24h, 100s intervals):"
        );
        eprintln!("  max: {:.6} deg", max_err.to_degrees());
        eprintln!("  mean: {:.6} deg", mean_err.to_degrees());

        assert!(
            max_err < SUN_DIRECTION_VALIDATION_TOL_RAD,
            "max Sun direction error = {:.6} deg (expected < {:.4} deg)",
            max_err.to_degrees(),
            SUN_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );
    }

    /// Meeus Moon position vs ANISE DE440s over 24 hours.
    /// Validates that the analytical Moon ephemeris (Meeus Ch. 47, truncated)
    /// agrees with the high-fidelity ANISE ephemeris within the stated accuracy.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_moon_direction_accuracy() {
        use rpo_core::elements::eclipse::moon_position_eci_km;
        use anise::constants::frames::{MOON_J2000, EARTH_J2000 as ANISE_EARTH_J2000};

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let base_epoch = test_epoch();

        let mut max_err = 0.0_f64;
        let mut sum_err = 0.0_f64;
        let n_samples = 864; // 24 hours at 100s intervals

        for i in 0..n_samples {
            let dt = f64::from(i) * 100.0;
            let epoch = base_epoch + hifitime::Duration::from_seconds(dt);

            // Analytical (Meeus Ch. 47 truncated)
            let meeus_moon = moon_position_eci_km(epoch);
            let meeus_dir = meeus_moon.normalize();

            // Numerical (ANISE DE440s)
            let moon_state = almanac
                .translate(MOON_J2000, ANISE_EARTH_J2000, epoch, None)
                .expect("Moon translate should succeed");
            let anise_dir = Vector3::new(
                moon_state.radius_km.x,
                moon_state.radius_km.y,
                moon_state.radius_km.z,
            )
            .normalize();

            let err = meeus_dir.dot(&anise_dir).clamp(-1.0, 1.0).acos();
            max_err = max_err.max(err);
            sum_err += err;
        }

        let mean_err = sum_err / f64::from(n_samples);
        eprintln!(
            "Moon direction accuracy (24h, 100s intervals):"
        );
        eprintln!("  max: {:.6} deg", max_err.to_degrees());
        eprintln!("  mean: {:.6} deg", mean_err.to_degrees());

        assert!(
            max_err < MOON_DIRECTION_VALIDATION_TOL_RAD,
            "max Moon direction error = {:.6} deg (expected < {:.4} deg)",
            max_err.to_degrees(),
            MOON_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );
    }
}

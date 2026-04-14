//! Monte Carlo statistics: percentile computation, dispersion envelopes,
//! and covariance validation comparison.

use nalgebra::Vector3;

use rpo_core::constants::COVARIANCE_SIGMA_FLOOR;
use rpo_core::mission::monte_carlo::{
    CovarianceCrossCheck, DispersionEnvelope, EnsembleStatistics, PercentileStats,
};
use rpo_core::mission::monte_carlo::MonteCarloError as CoreMonteCarloError;
use rpo_core::propagation::covariance::types::MissionCovarianceReport;

use super::types::SampleTrajectorySummary;
use super::MonteCarloError;

/// Compute percentile statistics, erroring if the input has no finite values.
///
/// Wraps [`crate::statistics::compute_percentile_stats`] (which returns
/// `Option<PercentileStats>`) with a `Result` signature that maps the empty
/// case to [`CoreMonteCarloError::EmptyEnsemble`]. The distinct name avoids
/// shadowing the core function at call sites inside `monte_carlo/`.
///
/// The input slice is sorted in place; non-finite values are compacted to the end.
pub(crate) fn require_percentile_stats(
    values: &mut [f64],
) -> Result<PercentileStats, MonteCarloError> {
    crate::statistics::compute_percentile_stats(values)
        .ok_or_else(|| CoreMonteCarloError::EmptyEnsemble.into())
}

/// Compute trajectory dispersion envelope from per-sample trajectories.
///
/// For each time index, collects RIC positions across samples and computes
/// per-axis percentile statistics. Trajectories may have variable length;
/// time steps with no data are silently skipped.
///
/// # Invariants
/// - Returns empty `Vec` if `sample_trajectories` is empty or `n_steps == 0`.
/// - Each trajectory may be shorter than `n_steps`; steps beyond a
///   trajectory's length are excluded from that step's statistics.
pub(crate) fn compute_dispersion_envelope(
    summaries: &[SampleTrajectorySummary],
    n_steps: u32,
) -> Vec<DispersionEnvelope> {
    if summaries.is_empty() || n_steps == 0 {
        return Vec::new();
    }

    let mut envelopes = Vec::with_capacity(n_steps as usize + 1); // u32 → usize: always safe (usize ≥ 32 bits)

    for j in 0..=n_steps {
        let j_idx = j as usize; // u32 → usize: always safe (usize ≥ 32 bits)
        let mut radial_values: Vec<f64> = Vec::with_capacity(summaries.len());
        let mut intrack_values: Vec<f64> = Vec::with_capacity(summaries.len());
        let mut crosstrack_values: Vec<f64> = Vec::with_capacity(summaries.len());
        let mut elapsed_s = 0.0_f64;
        let mut needs_elapsed = true;

        for summary in summaries {
            if j_idx < summary.positions_ric_km.len() {
                let pos = &summary.positions_ric_km[j_idx];
                radial_values.push(pos.x);
                intrack_values.push(pos.y);
                crosstrack_values.push(pos.z);
                if needs_elapsed {
                    elapsed_s = summary.elapsed_s[j_idx];
                    needs_elapsed = false;
                }
            }
        }

        if radial_values.is_empty() {
            continue;
        }

        // These succeed because the vecs are non-empty (checked above)
        let Some(r_stats) = crate::statistics::compute_percentile_stats(&mut radial_values) else {
            continue;
        };
        let Some(i_stats) = crate::statistics::compute_percentile_stats(&mut intrack_values) else {
            continue;
        };
        let Some(c_stats) = crate::statistics::compute_percentile_stats(&mut crosstrack_values) else {
            continue;
        };

        envelopes.push(DispersionEnvelope {
            elapsed_s,
            radial_km: r_stats,
            in_track_km: i_stats,
            cross_track_km: c_stats,
        });
    }

    envelopes
}

/// Compare MC ensemble statistics against covariance predictions.
///
/// Computes per-axis sigma ratios (MC sigma / covariance sigma) and a
/// sample-based terminal 3-sigma box containment fraction. The containment
/// check uses the **nominal predicted terminal position** as center (not
/// the MC mean), so terminal bias is visible rather than hidden.
///
/// A well-calibrated covariance model in `OpenLoop` mode should produce
/// sigma ratios near 1.0 and containment near 99.7%.
///
/// # Invariants
/// - If `statistics.dispersion_envelope` is empty, sigma ratios default to 1.0.
/// - If predicted covariance sigma is below [`COVARIANCE_SIGMA_FLOOR`],
///   the corresponding sigma ratio defaults to 1.0 (avoids division by zero).
/// - If `summaries` is empty, containment defaults to 0.0.
/// - Returns `CoreMonteCarloError::TooManySamples` if summary count exceeds u32
///   (should not happen — bounded by `MonteCarloConfig::num_samples: u32`).
pub(crate) fn compute_covariance_cross_check(
    cov_report: &MissionCovarianceReport,
    statistics: &EnsembleStatistics,
    summaries: &[SampleTrajectorySummary],
) -> Result<CovarianceCrossCheck, MonteCarloError> {
    // Extract end-of-mission dispersion envelope.
    let last_env = statistics.dispersion_envelope.last();

    // Terminal covariance predictions (explicit fields, no fragile chain).
    let cov_sigma3 = cov_report.terminal_sigma3_position_ric_km;
    let nominal_center = cov_report.terminal_position_ric_km;

    // Compute sigma ratios when both MC dispersion and covariance data are available.
    let sigma_ratio_ric = if let Some(env) = last_env {
        let mc_sigma = Vector3::new(
            env.radial_km.std_dev,
            env.in_track_km.std_dev,
            env.cross_track_km.std_dev,
        );

        // Per-axis sigma ratio: MC 1-sigma / covariance 1-sigma (expect ~1.0)
        let cov_1sigma = cov_sigma3 / 3.0;
        Vector3::new(
            if cov_1sigma.x > COVARIANCE_SIGMA_FLOOR {
                mc_sigma.x / cov_1sigma.x
            } else {
                1.0
            },
            if cov_1sigma.y > COVARIANCE_SIGMA_FLOOR {
                mc_sigma.y / cov_1sigma.y
            } else {
                1.0
            },
            if cov_1sigma.z > COVARIANCE_SIGMA_FLOOR {
                mc_sigma.z / cov_1sigma.z
            } else {
                1.0
            },
        )
    } else {
        // No dispersion data — neutral default
        Vector3::new(1.0, 1.0, 1.0)
    };

    // Terminal 3-sigma box containment: count samples whose terminal RIC position
    // falls within +/-3sigma of the nominal predicted position on all 3 axes.
    let n_total = summaries.len();
    let terminal_3sigma_containment = if n_total > 0 {
        let n_within = summaries
            .iter()
            .filter_map(SampleTrajectorySummary::terminal_position_ric_km)
            .filter(|pos| {
                let d = pos - nominal_center;
                d.x.abs() <= cov_sigma3.x
                    && d.y.abs() <= cov_sigma3.y
                    && d.z.abs() <= cov_sigma3.z
            })
            .count();
        let n_total_u32 = u32::try_from(n_total)
            .map_err(|_| CoreMonteCarloError::TooManySamples { count: n_total })?;
        let n_within_u32 = u32::try_from(n_within)
            .map_err(|_| CoreMonteCarloError::TooManySamples { count: n_within })?;
        f64::from(n_within_u32) / f64::from(n_total_u32)
    } else {
        0.0
    };

    Ok(CovarianceCrossCheck {
        terminal_3sigma_containment,
        min_mahalanobis_distance: cov_report.min_mahalanobis_distance,
        sigma_ratio_ric,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::constants::TEST_F64_SQRT_ACCUMULATION_TOL;
    use rpo_core::mission::monte_carlo::Distribution;
    use super::super::sampling::sample_distribution;
    use rpo_core::propagation::covariance::types::{
        CovarianceState, LegCovarianceReport, NavigationAccuracy,
    };
    use rpo_core::types::Matrix6;
    use hifitime::Epoch;
    use nalgebra::SMatrix;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    // --- Percentile reduction fixture values ------------------------------
    // Hoisted constants for the `percentile_single_value` and
    // `percentile_filters_nan` tests. The degenerate cases are intentionally
    // chosen so min/max/p50/mean are bitwise-exact copies of inputs, letting
    // the assertions use `.to_bits()` equality instead of a tolerance.

    /// Single-value input for `percentile_single_value`.
    const SINGLE_VALUE_INPUT: f64 = 42.0;

    /// Minimum finite value in the NaN-filter fixture (= p0 after filtering).
    const NAN_FILTER_FINITE_MIN: f64 = 1.0;

    /// Median (p50) of the finite subset [1, 2, 3, 4, 5].
    const NAN_FILTER_FINITE_MEDIAN: f64 = 3.0;

    /// Maximum finite value in the NaN-filter fixture (= p100 after filtering).
    const NAN_FILTER_FINITE_MAX: f64 = 5.0;

    /// Low filler in the NaN-filter fixture (slots between min and median).
    const NAN_FILTER_FINITE_FILLER_LOW: f64 = 2.0;

    /// High filler in the NaN-filter fixture (slots between median and max).
    const NAN_FILTER_FINITE_FILLER_HIGH: f64 = 4.0;

    /// Expected mean of the finite subset [1, 2, 3, 4, 5].
    const NAN_FILTER_FINITE_MEAN: f64 = 3.0;

    /// Build a minimal `MissionCovarianceReport` with given terminal position
    /// and per-axis 3-sigma values.
    fn mock_cov_report(sigma3_ric: Vector3<f64>) -> MissionCovarianceReport {
        mock_cov_report_with_center(sigma3_ric, Vector3::new(0.0, 5.0, 0.0))
    }

    /// Build a minimal `MissionCovarianceReport` with explicit terminal position center.
    fn mock_cov_report_with_center(
        sigma3_ric: Vector3<f64>,
        terminal_pos: Vector3<f64>,
    ) -> MissionCovarianceReport {
        let state = CovarianceState {
            epoch: Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0),
            elapsed_s: 100.0,
            covariance_roe: Matrix6::zeros(),
            covariance_ric_position_km2: SMatrix::<f64, 3, 3>::zeros(),
            sigma3_position_ric_km: sigma3_ric,
            mahalanobis_distance: 5.0,
        };
        let scalar_max = sigma3_ric.x.max(sigma3_ric.y).max(sigma3_ric.z);
        MissionCovarianceReport {
            legs: vec![LegCovarianceReport {
                states: vec![state],
                max_sigma3_position_km: scalar_max,
                min_mahalanobis_distance: 5.0,
            }],
            navigation_accuracy: NavigationAccuracy::default(),
            maneuver_uncertainty: None,
            max_sigma3_position_km: scalar_max,
            min_mahalanobis_distance: 5.0,
            terminal_position_ric_km: terminal_pos,
            terminal_sigma3_position_ric_km: sigma3_ric,
        }
    }

    /// Build a minimal `EnsembleStatistics` with one dispersion envelope entry
    /// having the given per-axis standard deviations.
    fn mock_statistics(std_devs: Vector3<f64>) -> EnsembleStatistics {
        let make_stats = |std_dev: f64| PercentileStats {
            std_dev,
            ..Default::default()
        };
        EnsembleStatistics {
            total_dv_km_s: make_stats(0.0),
            min_rc_distance_km: None,
            min_3d_distance_km: None,
            min_ei_separation_km: None,
            waypoint_miss_km: vec![],
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 0.0,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![DispersionEnvelope {
                elapsed_s: 100.0,
                radial_km: make_stats(std_devs.x),
                in_track_km: make_stats(std_devs.y),
                cross_track_km: make_stats(std_devs.z),
            }],
        }
    }

    /// Tolerance for nearest-rank percentile accuracy with 1000 integer
    /// values: off-by-one in rank gives ±1, so ±2 is conservative.
    const PERCENTILE_ACCURACY_TOL: f64 = 2.0;

    /// Tolerance for covariance validation tests comparing exact rational
    /// fractions (e.g., 3/4 = 0.75) or known-value sigma ratios.
    /// f64 arithmetic is exact for these small integers, so only floating-point
    /// rounding from the division needs coverage.
    const COVARIANCE_VALIDATION_TOL: f64 = 1e-12;

    #[test]
    fn percentile_ordering() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let mut values: Vec<f64> = (0..500)
            .map(|_| {
                let dist = Distribution::Gaussian { sigma: 1.0 };
                sample_distribution(&dist, &mut rng).unwrap()
            })
            .collect();
        let stats = require_percentile_stats(&mut values).unwrap();
        assert!(stats.min <= stats.p01);
        assert!(stats.p01 <= stats.p05);
        assert!(stats.p05 <= stats.p25);
        assert!(stats.p25 <= stats.p50);
        assert!(stats.p50 <= stats.p75);
        assert!(stats.p75 <= stats.p95);
        assert!(stats.p95 <= stats.p99);
        assert!(stats.p99 <= stats.max);
    }

    #[test]
    fn percentile_known_uniform() {
        let mut values: Vec<f64> = (0..1000).map(f64::from).collect();
        let stats = require_percentile_stats(&mut values).unwrap();
        assert!(
            (stats.p50 - 499.0).abs() < PERCENTILE_ACCURACY_TOL,
            "median should be ~499, got {}",
            stats.p50
        );
        assert!(
            (stats.p95 - 949.0).abs() < PERCENTILE_ACCURACY_TOL,
            "p95 should be ~949, got {}",
            stats.p95
        );
    }

    #[test]
    fn percentile_single_value() {
        // All stats are pure copies / degenerate reductions, so bitwise
        // equality is the correct contract — no arithmetic rounding budget.
        let stats = require_percentile_stats(&mut [SINGLE_VALUE_INPUT]).unwrap();
        assert_eq!(stats.min.to_bits(), SINGLE_VALUE_INPUT.to_bits());
        assert_eq!(stats.max.to_bits(), SINGLE_VALUE_INPUT.to_bits());
        assert_eq!(stats.p50.to_bits(), SINGLE_VALUE_INPUT.to_bits());
        assert_eq!(stats.mean.to_bits(), SINGLE_VALUE_INPUT.to_bits());
        assert_eq!(stats.std_dev.to_bits(), 0.0_f64.to_bits());
    }

    /// NaN values in input are filtered before computing statistics.
    /// The finite values should produce correct percentiles.
    #[test]
    fn percentile_filters_nan() {
        // Same bitwise rationale as `percentile_single_value`: after NaN
        // filtering the surviving [1, 2, 3, 4, 5] yields exact integer stats.
        let mut values = vec![
            NAN_FILTER_FINITE_MIN,
            NAN_FILTER_FINITE_FILLER_LOW,
            f64::NAN,
            NAN_FILTER_FINITE_MEDIAN,
            f64::NAN,
            NAN_FILTER_FINITE_FILLER_HIGH,
            NAN_FILTER_FINITE_MAX,
        ];
        let stats = require_percentile_stats(&mut values).unwrap();
        assert_eq!(stats.min.to_bits(), NAN_FILTER_FINITE_MIN.to_bits());
        assert_eq!(stats.max.to_bits(), NAN_FILTER_FINITE_MAX.to_bits());
        assert_eq!(stats.p50.to_bits(), NAN_FILTER_FINITE_MEDIAN.to_bits());
        assert!(
            (stats.mean - NAN_FILTER_FINITE_MEAN).abs() < TEST_F64_SQRT_ACCUMULATION_TOL,
            "mean should be {NAN_FILTER_FINITE_MEAN}, got {}",
            stats.mean
        );
    }

    /// All-NaN input produces `EmptyEnsemble` error (no finite data to summarize).
    #[test]
    fn percentile_all_nan_is_error() {
        let mut values = vec![f64::NAN, f64::NAN, f64::NAN];
        let result = require_percentile_stats(&mut values);
        assert!(
            matches!(
                result,
                Err(MonteCarloError::Core(CoreMonteCarloError::EmptyEnsemble))
            ),
            "all-NaN input should return EmptyEnsemble, got {result:?}"
        );
    }

    /// Sigma ratios are computed per-axis: MC `std_dev` / (covariance 3-sigma / 3).
    #[test]
    fn sigma_ratio_per_axis() {
        // Covariance 3-sigma: R=3.0, I=30.0, C=0.9
        let cov = mock_cov_report(Vector3::new(3.0, 30.0, 0.9));
        // MC std_dev: R=1.0, I=10.0, C=0.3 (should give ratios of 1.0 per axis)
        let stats = mock_statistics(Vector3::new(1.0, 10.0, 0.3));

        let cv = compute_covariance_cross_check(
            &cov, &stats, &[] as &[SampleTrajectorySummary],
        ).unwrap();
        let tol = COVARIANCE_VALIDATION_TOL;
        assert!(
            (cv.sigma_ratio_ric.x - 1.0).abs() < tol,
            "radial sigma ratio should be 1.0, got {}",
            cv.sigma_ratio_ric.x
        );
        assert!(
            (cv.sigma_ratio_ric.y - 1.0).abs() < tol,
            "in-track sigma ratio should be 1.0, got {}",
            cv.sigma_ratio_ric.y
        );
        assert!(
            (cv.sigma_ratio_ric.z - 1.0).abs() < tol,
            "cross-track sigma ratio should be 1.0, got {}",
            cv.sigma_ratio_ric.z
        );
    }

    /// Terminal containment from sample-based counting. Constructs mock
    /// summaries with known terminal positions inside/outside the 3-sigma box.
    #[test]
    fn terminal_containment_sample_fraction() {
        // Center at (0, 5, 0), 3-sigma box: R=3, I=30, C=0.9
        let center = Vector3::new(0.0, 5.0, 0.0);
        let sigma3 = Vector3::new(3.0, 30.0, 0.9);
        let cov = mock_cov_report_with_center(sigma3, center);
        let stats = mock_statistics(Vector3::new(1.0, 10.0, 0.3));

        let make_summary = |pos: Vector3<f64>| SampleTrajectorySummary {
            positions_ric_km: vec![pos],
            elapsed_s: vec![100.0],
        };

        // 3 samples inside the box, 1 outside (cross-track exceeds 0.9)
        let summaries = vec![
            make_summary(center),                                    // inside (delta = 0)
            make_summary(center + Vector3::new(1.0, 10.0, 0.5)),    // inside
            make_summary(center + Vector3::new(-2.0, -20.0, -0.8)), // inside
            make_summary(center + Vector3::new(0.0, 0.0, 1.0)),     // outside (C: 1.0 > 0.9)
        ];

        let cv = compute_covariance_cross_check(&cov, &stats, &summaries).unwrap();
        // 3/4 = 0.75
        assert!(
            (cv.terminal_3sigma_containment - 0.75).abs() < COVARIANCE_VALIDATION_TOL,
            "expected 0.75, got {}",
            cv.terminal_3sigma_containment
        );
    }

    /// Empty summaries produce containment of 0.0.
    #[test]
    fn terminal_containment_empty_trajectories() {
        let cov = mock_cov_report(Vector3::new(3.0, 30.0, 0.9));
        let stats = mock_statistics(Vector3::new(0.5, 5.0, 0.1));

        let cv = compute_covariance_cross_check(
            &cov, &stats, &[] as &[SampleTrajectorySummary],
        ).unwrap();
        assert!(
            cv.terminal_3sigma_containment.abs() < COVARIANCE_VALIDATION_TOL,
            "empty summaries should give 0.0, got {}",
            cv.terminal_3sigma_containment
        );
    }

    /// Empty dispersion envelope defaults sigma ratios to 1.0.
    #[test]
    fn sigma_ratio_empty_dispersion() {
        let cov = mock_cov_report(Vector3::new(3.0, 30.0, 0.9));
        let mut stats = mock_statistics(Vector3::new(0.5, 5.0, 0.1));
        stats.dispersion_envelope.clear();

        let cv = compute_covariance_cross_check(
            &cov, &stats, &[] as &[SampleTrajectorySummary],
        ).unwrap();
        let tol = COVARIANCE_VALIDATION_TOL;
        assert!(
            (cv.sigma_ratio_ric.x - 1.0).abs() < tol
                && (cv.sigma_ratio_ric.y - 1.0).abs() < tol
                && (cv.sigma_ratio_ric.z - 1.0).abs() < tol,
            "empty dispersion should default sigma ratios to 1.0, got {:?}",
            cv.sigma_ratio_ric
        );
    }
}

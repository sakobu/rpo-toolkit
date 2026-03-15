//! Monte Carlo statistics: percentile computation, dispersion envelopes,
//! and covariance validation comparison.

use nalgebra::Vector3;

use crate::constants::COVARIANCE_SIGMA_FLOOR;
use crate::propagation::covariance::types::MissionCovarianceReport;
use crate::propagation::propagator::PropagatedState;

use super::types::{
    CovarianceValidation, DispersionEnvelope, EnsembleStatistics, PercentileStats,
};
use super::MonteCarloError;

/// Compute percentile statistics from a slice of scalar values.
///
/// Uses nearest-rank method for percentile extraction.
/// Converts the slice length to `u32` for lossless `f64::from` conversion.
/// Standard deviation uses Bessel correction (n-1 denominator) for n > 1;
/// returns 0.0 for n = 1.
///
/// # Invariants
/// - `values` must be non-empty.
/// - For n = 1, all percentiles equal the single value and `std_dev = 0`.
///
/// # Errors
/// Returns [`MonteCarloError::EmptyEnsemble`] if no finite values remain
/// after filtering NaN/Inf.
pub(crate) fn compute_percentile_stats(values: &[f64]) -> Result<PercentileStats, MonteCarloError> {
    // Filter non-finite values (NaN, Inf) that can arise from degenerate
    // nyx propagation states or frame conversions.
    let mut sorted: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if sorted.is_empty() {
        return Err(MonteCarloError::EmptyEnsemble);
    }
    // Convert length to u32 for lossless f64 conversion.
    // MC sample counts are always bounded by MonteCarloConfig.num_samples (u32),
    // so this conversion is infallible in practice.
    let n = u32::try_from(sorted.len()).unwrap_or(u32::MAX);
    let n_f = f64::from(n);

    // Compute mean/variance before sorting (order doesn't matter for summation).
    let sum: f64 = sorted.iter().sum();
    let mean = sum / n_f;
    let variance = if n > 1 {
        sorted.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / f64::from(n - 1)
    } else {
        0.0
    };

    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Nearest-rank percentile using integer arithmetic only.
    // rank = ceil(p * n / 100) computed as (p * n + 99) / 100 in u32.
    let percentile = |p_percent: u32| -> f64 {
        if n == 1 {
            return sorted[0];
        }
        let rank = (p_percent * n).div_ceil(100);
        // u32 → usize: always widening on 32-bit and 64-bit platforms.
        let idx = rank.saturating_sub(1).min(n - 1) as usize;
        sorted[idx]
    };

    Ok(PercentileStats {
        min: sorted[0],
        p01: percentile(1),
        p05: percentile(5),
        p25: percentile(25),
        p50: percentile(50),
        p75: percentile(75),
        p95: percentile(95),
        p99: percentile(99),
        max: sorted[sorted.len() - 1],
        mean,
        std_dev: variance.sqrt(),
    })
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
    sample_trajectories: &[Vec<PropagatedState>],
    n_steps: u32,
) -> Vec<DispersionEnvelope> {
    if sample_trajectories.is_empty() || n_steps == 0 {
        return Vec::new();
    }

    // u32 → usize: always widening on 32-bit and 64-bit platforms.
    let mut envelopes = Vec::with_capacity(n_steps as usize + 1);

    for j in 0..=n_steps {
        let j_idx = j as usize;
        let mut radial_values = Vec::with_capacity(sample_trajectories.len());
        let mut intrack_values = Vec::with_capacity(sample_trajectories.len());
        let mut crosstrack_values = Vec::with_capacity(sample_trajectories.len());
        let mut elapsed = 0.0_f64;
        let mut needs_elapsed = true;

        for traj in sample_trajectories {
            if j_idx < traj.len() {
                let state = &traj[j_idx];
                radial_values.push(state.ric.position_ric_km.x);
                intrack_values.push(state.ric.position_ric_km.y);
                crosstrack_values.push(state.ric.position_ric_km.z);
                if needs_elapsed {
                    elapsed = state.elapsed_s;
                    needs_elapsed = false;
                }
            }
        }

        if radial_values.is_empty() {
            continue;
        }

        // These succeed because the vecs are non-empty (checked above)
        let Ok(r_stats) = compute_percentile_stats(&radial_values) else {
            continue;
        };
        let Ok(i_stats) = compute_percentile_stats(&intrack_values) else {
            continue;
        };
        let Ok(c_stats) = compute_percentile_stats(&crosstrack_values) else {
            continue;
        };

        envelopes.push(DispersionEnvelope {
            elapsed_s: elapsed,
            radial_km: r_stats,
            in_track_km: i_stats,
            cross_track_km: c_stats,
        });
    }

    envelopes
}

/// Compare MC ensemble statistics against covariance predictions.
///
/// Computes sigma ratios (MC σ / covariance σ per RIC axis) and approximate
/// 3-sigma containment fraction. A well-calibrated covariance model should
/// produce sigma ratios near 1.0 and containment near 99.7%.
///
/// # Invariants
/// - If `statistics.dispersion_envelope` is empty, sigma ratios default to 1.0.
/// - If predicted covariance sigma is below [`COVARIANCE_SIGMA_FLOOR`],
///   sigma ratios default to 1.0 (avoids division by zero).
pub(crate) fn compute_covariance_validation(
    cov_report: &MissionCovarianceReport,
    statistics: &EnsembleStatistics,
) -> CovarianceValidation {
    // Compute sigma ratios: MC std_dev vs covariance 1-sigma per axis
    let mut sigma_ratio_r = 1.0;
    let mut sigma_ratio_i = 1.0;
    let mut sigma_ratio_c = 1.0;

    if let Some(last_env) = statistics.dispersion_envelope.last() {
        let mc_sigma_r = last_env.radial_km.std_dev;
        let mc_sigma_i = last_env.in_track_km.std_dev;
        let mc_sigma_c = last_env.cross_track_km.std_dev;

        // Covariance max 3-sigma → 1-sigma
        let cov_sigma = cov_report.max_sigma3_position_km / 3.0;
        if cov_sigma > COVARIANCE_SIGMA_FLOOR {
            sigma_ratio_r = mc_sigma_r / cov_sigma;
            sigma_ratio_i = mc_sigma_i / cov_sigma;
            sigma_ratio_c = mc_sigma_c / cov_sigma;
        }
    }

    // 3-sigma containment approximation via percentile comparison
    let cov_3sigma = cov_report.max_sigma3_position_km;
    let fraction_within_3sigma = if let Some(ref d3_stats) = statistics.min_3d_distance_km {
        if d3_stats.std_dev > 0.0 {
            if d3_stats.p99 < cov_3sigma {
                0.99
            } else if d3_stats.p95 < cov_3sigma {
                0.95
            } else {
                0.90
            }
        } else {
            1.0
        }
    } else {
        1.0
    };

    CovarianceValidation {
        fraction_within_3sigma,
        covariance_collision_prob: cov_report.max_collision_probability,
        mc_collision_prob: statistics.collision_probability,
        sigma_ratio_ric: Vector3::new(sigma_ratio_r, sigma_ratio_i, sigma_ratio_c),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::monte_carlo::sampling::sample_distribution;
    use crate::mission::monte_carlo::types::Distribution;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    /// Tolerance for nearest-rank percentile accuracy with 1000 integer
    /// values: off-by-one in rank gives ±1, so ±2 is conservative.
    const PERCENTILE_ACCURACY_TOL: f64 = 2.0;

    #[test]
    fn percentile_ordering() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let values: Vec<f64> = (0..500)
            .map(|_| {
                let dist = Distribution::Gaussian { sigma: 1.0 };
                sample_distribution(&dist, &mut rng).unwrap()
            })
            .collect();
        let stats = compute_percentile_stats(&values).unwrap();
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
        let values: Vec<f64> = (0..1000).map(f64::from).collect();
        let stats = compute_percentile_stats(&values).unwrap();
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
        let stats = compute_percentile_stats(&[42.0]).unwrap();
        assert_eq!(stats.min, 42.0);
        assert_eq!(stats.max, 42.0);
        assert_eq!(stats.p50, 42.0);
        assert_eq!(stats.mean, 42.0);
        assert_eq!(stats.std_dev, 0.0);
    }

    /// NaN values in input are filtered before computing statistics.
    /// The finite values should produce correct percentiles.
    #[test]
    fn percentile_filters_nan() {
        let values = vec![1.0, 2.0, f64::NAN, 3.0, f64::NAN, 4.0, 5.0];
        let stats = compute_percentile_stats(&values).unwrap();
        assert_eq!(stats.min, 1.0);
        assert_eq!(stats.max, 5.0);
        assert_eq!(stats.p50, 3.0);
        assert!((stats.mean - 3.0).abs() < 1e-10, "mean should be 3.0, got {}", stats.mean);
    }

    /// All-NaN input produces EmptyEnsemble error (no finite data to summarize).
    #[test]
    fn percentile_all_nan_is_error() {
        let values = vec![f64::NAN, f64::NAN, f64::NAN];
        let result = compute_percentile_stats(&values);
        assert!(
            matches!(result, Err(MonteCarloError::EmptyEnsemble)),
            "all-NaN input should return EmptyEnsemble, got {result:?}"
        );
    }
}

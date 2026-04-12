//! Shared scalar statistics primitives.
//!
//! Provides [`BasicStats`] (`min`/`max`/`mean`/`RMS`/`std_dev` from a slice) and
//! [`compute_percentile_stats`] (nearest-rank percentiles with in-place sort).
//! Both the validation and Monte Carlo modules delegate to these primitives
//! so the scalar-reduction logic is written once.

/// Scalar summary statistics computed from a slice of `f64` values.
///
/// Provides min, max, mean, RMS, standard deviation, and count in a single
/// pass. Used as a shared primitive by both `validation::statistics` and
/// `monte_carlo::statistics`.
#[derive(Debug, Clone, Copy)]
pub struct BasicStats {
    /// Minimum value.
    pub min: f64,
    /// Maximum value.
    pub max: f64,
    /// Arithmetic mean.
    pub mean: f64,
    /// Root mean square.
    pub rms: f64,
    /// Sample standard deviation (Bessel-corrected, n−1 denominator).
    /// Returns 0.0 when count = 1.
    pub std_dev: f64,
    /// Number of values in the slice.
    pub count: usize,
}

impl BasicStats {
    /// Compute summary statistics from a non-empty slice.
    ///
    /// Returns `None` if the slice is empty. Single-pass over data.
    ///
    /// Standard deviation uses the two-pass Bessel formula
    /// `(Σx² − (Σx)²/n) / (n−1)`, which can suffer catastrophic
    /// cancellation when variance is tiny relative to mean-squared.
    /// This is acceptable for the expected inputs (km-scale position
    /// errors, km/s velocity errors) where mean and spread are similar
    /// in magnitude. A Welford online algorithm would be more robust
    /// for large-offset-plus-tiny-variance data.
    #[must_use]
    pub fn from_slice(xs: &[f64]) -> Option<Self> {
        if xs.is_empty() {
            return None;
        }
        let n = xs.len();
        // u32 intermediate for lossless f64::from (sample counts never exceed u32::MAX).
        let n_u32 = u32::try_from(n).unwrap_or(u32::MAX);
        let n_f = f64::from(n_u32);

        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        let mut sum = 0.0_f64;
        let mut sum_sq = 0.0_f64;

        for &x in xs {
            min = min.min(x);
            max = max.max(x);
            sum += x;
            sum_sq += x * x;
        }

        let mean = sum / n_f;
        let rms = (sum_sq / n_f).sqrt();
        // Bessel-corrected variance: (Σx² − (Σx)²/n) / (n−1).
        // .max(0.0) guards against floating-point noise producing a tiny negative.
        let std_dev = if n > 1 {
            ((sum_sq - sum * sum / n_f) / (n_f - 1.0)).max(0.0).sqrt()
        } else {
            0.0
        };

        Some(Self { min, max, mean, rms, std_dev, count: n })
    }
}

use rpo_core::mission::monte_carlo::PercentileStats;

/// Compact finite values to the front of the slice, returning the count.
///
/// Non-finite values (NaN, Inf, −Inf) are moved to the end. The relative
/// order of finite values is preserved within the compacted prefix.
fn compact_finite(values: &mut [f64]) -> usize {
    let mut write = 0;
    for read in 0..values.len() {
        if values[read].is_finite() {
            values.swap(write, read);
            write += 1;
        }
    }
    write
}

/// Compute percentile statistics from a mutable slice of scalar values.
///
/// Non-finite values (NaN, Inf) are compacted to the end of the slice.
/// The finite portion is sorted in place. Returns `None` if no finite
/// values remain after filtering.
///
/// Uses nearest-rank method for percentile extraction. Standard deviation
/// uses Bessel correction (n−1 denominator) for n > 1; returns 0.0 for n = 1.
///
/// # Note
///
/// The caller's slice is reordered (finite values sorted, non-finite values
/// moved to the end). This is intentional: Phase 5 A-4 pre-allocates per-metric
/// buffers and passes them here for in-place processing.
#[must_use]
pub fn compute_percentile_stats(values: &mut [f64]) -> Option<PercentileStats> {
    let n_finite = compact_finite(values);
    let finite = &mut values[..n_finite];
    if finite.is_empty() {
        return None;
    }

    // Basic stats before sorting (order-independent).
    let basic = BasicStats::from_slice(finite)?;

    // Sort in-place for percentile extraction.
    finite.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Nearest-rank percentile using integer arithmetic only.
    // rank = ceil(p * n / 100) computed as div_ceil.
    // Ceil is required by the nearest-rank definition.
    let n = u32::try_from(finite.len()).unwrap_or(u32::MAX);
    let percentile = |p_percent: u32| -> f64 {
        if n == 1 {
            return finite[0];
        }
        let rank = (p_percent * n).div_ceil(100);
        let idx = usize::try_from(rank.saturating_sub(1).min(n - 1)).unwrap_or(0);
        finite[idx]
    };

    Some(PercentileStats {
        min: basic.min,
        p01: percentile(1),
        p05: percentile(5),
        p25: percentile(25),
        p50: percentile(50),
        p75: percentile(75),
        p95: percentile(95),
        p99: percentile(99),
        max: basic.max,
        mean: basic.mean,
        std_dev: basic.std_dev,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::constants::{
        TEST_F64_EXACT_ARITHMETIC_TOL, TEST_F64_SQRT_ACCUMULATION_TOL,
    };

    #[test]
    fn basic_stats_empty_returns_none() {
        assert!(BasicStats::from_slice(&[]).is_none());
    }

    #[test]
    fn basic_stats_single_value() {
        let stats = BasicStats::from_slice(&[5.0]).unwrap();
        assert_eq!(stats.count, 1);
        assert!((stats.min - 5.0).abs() < f64::EPSILON);
        assert!((stats.max - 5.0).abs() < f64::EPSILON);
        assert!((stats.mean - 5.0).abs() < f64::EPSILON);
        assert!((stats.rms - 5.0).abs() < f64::EPSILON);
        assert!((stats.std_dev).abs() < f64::EPSILON); // n=1 → 0
    }

    #[test]
    fn basic_stats_known_values() {
        // [1, 2, 3, 4, 5]: mean=3, rms=sqrt(55/5)=sqrt(11)≈3.3166
        // std_dev = sqrt(((1-3)²+(2-3)²+(3-3)²+(4-3)²+(5-3)²)/4) = sqrt(10/4) = sqrt(2.5) ≈ 1.5811
        let stats = BasicStats::from_slice(&[1.0, 2.0, 3.0, 4.0, 5.0]).unwrap();
        assert_eq!(stats.count, 5);
        assert!((stats.min - 1.0).abs() < f64::EPSILON);
        assert!((stats.max - 5.0).abs() < f64::EPSILON);
        // mean: exact integer arithmetic on 5 values → round-off only
        assert!((stats.mean - 3.0).abs() < TEST_F64_EXACT_ARITHMETIC_TOL);
        // rms/std_dev: single sqrt on a small sum → sqrt accumulation tolerance
        assert!((stats.rms - 11.0_f64.sqrt()).abs() < TEST_F64_SQRT_ACCUMULATION_TOL);
        assert!((stats.std_dev - 2.5_f64.sqrt()).abs() < TEST_F64_SQRT_ACCUMULATION_TOL);
    }

    #[test]
    fn basic_stats_identical_values() {
        let stats = BasicStats::from_slice(&[7.0, 7.0, 7.0]).unwrap();
        assert!((stats.min - 7.0).abs() < f64::EPSILON);
        assert!((stats.max - 7.0).abs() < f64::EPSILON);
        assert!((stats.mean - 7.0).abs() < f64::EPSILON);
        assert!((stats.rms - 7.0).abs() < f64::EPSILON);
        // all identical → variance is exactly 0; guard against FP noise
        assert!(stats.std_dev.abs() < TEST_F64_EXACT_ARITHMETIC_TOL);
    }

    #[test]
    fn percentile_stats_empty_returns_none() {
        assert!(super::compute_percentile_stats(&mut []).is_none());
    }

    #[test]
    fn percentile_stats_all_non_finite_returns_none() {
        let mut vals = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
        assert!(super::compute_percentile_stats(&mut vals).is_none());
    }

    #[test]
    fn percentile_stats_single_value() {
        let mut vals = [42.0];
        let ps = super::compute_percentile_stats(&mut vals).unwrap();
        assert!((ps.min - 42.0).abs() < f64::EPSILON);
        assert!((ps.max - 42.0).abs() < f64::EPSILON);
        assert!((ps.mean - 42.0).abs() < f64::EPSILON);
        assert!((ps.p50 - 42.0).abs() < f64::EPSILON);
        assert!(ps.std_dev.abs() < f64::EPSILON);
    }

    #[test]
    fn percentile_stats_known_uniform() {
        // 1..=100: p50 = 50, p95 = 95, mean = 50.5
        let mut vals: Vec<f64> = (1_u32..=100).map(f64::from).collect();
        let ps = super::compute_percentile_stats(&mut vals).unwrap();
        assert!((ps.min - 1.0).abs() < f64::EPSILON);
        assert!((ps.max - 100.0).abs() < f64::EPSILON);
        // mean of 100 integers: sum 5050 and quotient 50.5 are exactly representable
        assert!((ps.mean - 50.5).abs() < TEST_F64_EXACT_ARITHMETIC_TOL);
        assert!((ps.p50 - 50.0).abs() < f64::EPSILON);
        assert!((ps.p95 - 95.0).abs() < f64::EPSILON);
        assert!((ps.p01 - 1.0).abs() < f64::EPSILON);
        assert!((ps.p99 - 99.0).abs() < f64::EPSILON);
    }

    #[test]
    fn percentile_stats_filters_nan() {
        let mut vals = [f64::NAN, 1.0, 2.0, f64::NAN, 3.0];
        let ps = super::compute_percentile_stats(&mut vals).unwrap();
        assert!((ps.min - 1.0).abs() < f64::EPSILON);
        assert!((ps.max - 3.0).abs() < f64::EPSILON);
        // mean of [1,2,3]: exact integer arithmetic
        assert!((ps.mean - 2.0).abs() < TEST_F64_EXACT_ARITHMETIC_TOL);
    }
}

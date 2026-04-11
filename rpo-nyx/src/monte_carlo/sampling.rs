//! Monte Carlo sampling primitives: distribution sampling and maneuver dispersion.

use nalgebra::Vector3;
use rand::Rng;
use rand_distr::{Normal, Uniform as RandUniform};

use rpo_core::constants::{DV_NORM_ZERO_THRESHOLD_KM_S, RODRIGUES_AXIS_NORM_THRESHOLD};

use rpo_core::mission::monte_carlo::{Distribution, ManeuverDispersion};
use rpo_core::mission::monte_carlo::MonteCarloError as CoreMonteCarloError;
use super::MonteCarloError;

/// Draw a single value from a [`Distribution`].
///
/// # Invariants
/// - Gaussian `sigma >= 0`; zero sigma returns `0.0` deterministically.
/// - Uniform `half_width >= 0`; zero half-width returns `0.0` deterministically.
///
/// # Errors
/// Returns [`CoreMonteCarloError::NegativeSigma`] if `sigma < 0`, or
/// [`CoreMonteCarloError::NegativeHalfWidth`] if `half_width < 0`.
pub(crate) fn sample_distribution<R: Rng>(
    dist: &Distribution,
    rng: &mut R,
) -> Result<f64, MonteCarloError> {
    match *dist {
        Distribution::Gaussian { sigma } => {
            if sigma < 0.0 {
                return Err(CoreMonteCarloError::NegativeSigma { value: sigma }.into());
            }
            if sigma > 0.0 {
                let normal = Normal::new(0.0, sigma)
                    .map_err(|_| CoreMonteCarloError::NegativeSigma { value: sigma })?;
                Ok(rng.sample(normal))
            } else {
                Ok(0.0)
            }
        }
        Distribution::Uniform { half_width } => {
            if half_width < 0.0 {
                return Err(
                    CoreMonteCarloError::NegativeHalfWidth { value: half_width }.into(),
                );
            }
            if half_width > 0.0 {
                let uniform = RandUniform::new(-half_width, half_width);
                Ok(rng.sample(uniform))
            } else {
                Ok(0.0)
            }
        }
    }
}

/// Apply magnitude and pointing errors to a nominal Δv via Rodrigues' rotation.
///
/// # Invariants
/// - `dispersion.magnitude_sigma >= 0`
/// - Zero-norm `nominal_dv` (below [`DV_NORM_ZERO_THRESHOLD_KM_S`]) returns zero.
///
/// # Algorithm
/// 1. Magnitude scaling: `(1 + N(0, magnitude_sigma)) * ‖nominal‖`
/// 2. Pointing rotation: Rodrigues' formula about a random axis on the unit
///    sphere, with angle drawn from `N(0, pointing_sigma_rad)`.
///
/// # Singularity handling
/// If the random rotation axis has norm below [`RODRIGUES_AXIS_NORM_THRESHOLD`],
/// falls back to the x-axis `[1, 0, 0]` to avoid division by zero in
/// normalization. This degeneracy occurs with probability ~0 for continuous
/// distributions and does not bias the result in practice.
///
/// # Errors
/// Returns [`CoreMonteCarloError::NegativeSigma`] if `magnitude_sigma < 0`.
pub(crate) fn disperse_maneuver<R: Rng>(
    nominal_dv: &Vector3<f64>,
    dispersion: &ManeuverDispersion,
    rng: &mut R,
) -> Result<Vector3<f64>, MonteCarloError> {
    let nominal_mag = nominal_dv.norm();
    if nominal_mag < DV_NORM_ZERO_THRESHOLD_KM_S {
        return Ok(Vector3::zeros());
    }

    // Magnitude scaling
    let mag_error = sample_distribution(
        &Distribution::Gaussian {
            sigma: dispersion.magnitude_sigma,
        },
        rng,
    )?;
    let scale = 1.0 + mag_error;

    // Pointing rotation via Rodrigues' formula
    let angle = sample_distribution(
        &Distribution::Gaussian {
            sigma: dispersion.pointing_sigma_rad,
        },
        rng,
    )?;

    // Random rotation axis on unit sphere (perpendicular to nominal direction)
    let raw_axis = Vector3::new(
        rng.sample::<f64, _>(rand_distr::StandardNormal),
        rng.sample::<f64, _>(rand_distr::StandardNormal),
        rng.sample::<f64, _>(rand_distr::StandardNormal),
    );

    let axis_norm = raw_axis.norm();
    let k = if axis_norm < RODRIGUES_AXIS_NORM_THRESHOLD {
        Vector3::x()
    } else {
        raw_axis / axis_norm
    };

    // Rodrigues: v_rot = v cos(θ) + (k × v) sin(θ) + k (k · v)(1 - cos(θ))
    let unit_dv = nominal_dv / nominal_mag;
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let v_rotated =
        unit_dv * cos_a + k.cross(&unit_dv) * sin_a + k * k.dot(&unit_dv) * (1.0 - cos_a);

    Ok(scale * nominal_mag * v_rotated)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::statistics::compute_percentile_stats;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    /// Tolerance for Gaussian mean convergence: 100k samples of N(0,1)
    /// should have mean within 2% of zero with high probability.
    const GAUSSIAN_MEAN_TOL: f64 = 0.02;

    /// Tolerance for sigma scaling ratio: 10k samples give ~1% std error
    /// on estimated std dev, so 5% tolerance is conservative.
    const SIGMA_RATIO_TOL: f64 = 0.05;

    /// Tolerance for magnitude ratio in maneuver dispersion: 10k samples,
    /// 1% magnitude sigma → expect mean ratio within 2% of 1.0.
    const MAGNITUDE_RATIO_TOL: f64 = 0.02;

    /// Pointing cone bound: 4× pointing sigma covers >99.99% of
    /// N(0, σ) samples (4-sigma bound).
    const POINTING_CONE_SIGMA_BOUND: f64 = 4.0;

    /// Tolerance for pointing-only magnitude preservation: Rodrigues
    /// rotation is exact, so magnitude change should be at machine epsilon.
    const POINTING_MAGNITUDE_TOL: f64 = 1e-14;

    #[test]
    fn gaussian_mean_converges() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let n = 100_000_u32;
        let sum: f64 = (0..n)
            .map(|_| sample_distribution(&dist, &mut rng).unwrap())
            .sum();
        let mean = sum / f64::from(n);
        assert!(
            mean.abs() < GAUSSIAN_MEAN_TOL,
            "gaussian mean should be near 0, got {mean}"
        );
    }

    #[test]
    fn gaussian_sigma_scales() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let n = 10_000_u32;

        let dist_1 = Distribution::Gaussian { sigma: 1.0 };
        let samples_1: Vec<f64> = (0..n)
            .map(|_| sample_distribution(&dist_1, &mut rng).unwrap())
            .collect();
        let std_1 = compute_percentile_stats(&samples_1).unwrap().std_dev;

        let mut rng2 = ChaCha20Rng::seed_from_u64(99);
        let dist_half = Distribution::Gaussian { sigma: 0.5 };
        let samples_half: Vec<f64> = (0..n)
            .map(|_| sample_distribution(&dist_half, &mut rng2).unwrap())
            .collect();
        let std_half = compute_percentile_stats(&samples_half).unwrap().std_dev;

        let ratio = std_half / std_1;
        assert!(
            (ratio - 0.5).abs() < SIGMA_RATIO_TOL,
            "sigma ratio should be ~0.5, got {ratio}"
        );
    }

    #[test]
    fn uniform_bounded() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let dist = Distribution::Uniform { half_width: 0.5 };
        for _ in 0..10_000 {
            let v = sample_distribution(&dist, &mut rng).unwrap();
            assert!(
                (-0.5..=0.5).contains(&v),
                "uniform sample {v} out of bounds"
            );
        }
    }

    #[test]
    fn deterministic_seed() {
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let mut rng1 = ChaCha20Rng::seed_from_u64(42);
        let mut rng2 = ChaCha20Rng::seed_from_u64(42);
        for _ in 0..100 {
            let a = sample_distribution(&dist, &mut rng1).unwrap();
            let b = sample_distribution(&dist, &mut rng2).unwrap();
            // Deterministic RNG: same seed must produce bitwise-identical f64.
            assert_eq!(a.to_bits(), b.to_bits(), "same seed should produce identical values");
        }
    }

    #[test]
    fn different_seeds_differ() {
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let mut rng1 = ChaCha20Rng::seed_from_u64(42);
        let mut rng2 = ChaCha20Rng::seed_from_u64(99);
        let a = sample_distribution(&dist, &mut rng1).unwrap();
        let b = sample_distribution(&dist, &mut rng2).unwrap();
        // Distinct seeds must diverge in the first sample (bitwise).
        assert_ne!(a.to_bits(), b.to_bits(), "different seeds should produce different values");
    }

    #[test]
    fn maneuver_magnitude_mean() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.01,
            pointing_sigma_rad: 0.0,
        };
        let n = 10_000_u32;
        let sum_mag: f64 = (0..n)
            .map(|_| disperse_maneuver(&nominal, &disp, &mut rng).unwrap().norm())
            .sum();
        let mean_mag = sum_mag / f64::from(n);
        let ratio = mean_mag / nominal.norm();
        assert!(
            (ratio - 1.0).abs() < MAGNITUDE_RATIO_TOL,
            "mean magnitude ratio should be ~1.0, got {ratio}"
        );
    }

    #[test]
    fn maneuver_pointing_preserves_magnitude() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.0,
            pointing_sigma_rad: 0.01,
        };
        let nominal_mag = nominal.norm();
        for _ in 0..1000 {
            let dispersed = disperse_maneuver(&nominal, &disp, &mut rng).unwrap();
            let diff = (dispersed.norm() - nominal_mag).abs();
            assert!(
                diff < POINTING_MAGNITUDE_TOL,
                "pointing-only: magnitude changed by {diff}"
            );
        }
    }

    #[test]
    fn maneuver_zero_dv_identity() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let zero_dv = Vector3::zeros();
        let disp = ManeuverDispersion::default();
        let result = disperse_maneuver(&zero_dv, &disp, &mut rng).unwrap();
        assert_eq!(result, Vector3::zeros(), "zero Δv should return zero");
    }

    #[test]
    fn maneuver_cone_bounded() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let pointing_sigma = 0.01_f64;
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.0,
            pointing_sigma_rad: pointing_sigma,
        };
        for _ in 0..1000 {
            let dispersed = disperse_maneuver(&nominal, &disp, &mut rng).unwrap();
            let cos_angle = nominal.dot(&dispersed) / (nominal.norm() * dispersed.norm());
            let angle = cos_angle.clamp(-1.0, 1.0).acos();
            assert!(
                angle < POINTING_CONE_SIGMA_BOUND * pointing_sigma,
                "angle {angle} exceeds {POINTING_CONE_SIGMA_BOUND}*sigma bound"
            );
        }
    }
}

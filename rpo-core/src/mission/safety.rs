//! Passive safety analysis via e/i vector separation (D'Amico Sec. 2.2).
//!
//! Evaluates formation safety using the minimum radial/cross-track
//! separation derived from the relative eccentricity and inclination vectors.

use crate::propagation::propagator::PropagatedState;
use crate::types::{KeplerianElements, QuasiNonsingularROE, SafetyConfig, SafetyMetrics};

/// Analyze passive safety of a single ROE state (D'Amico Eq. 2.22).
///
/// Computes the minimum radial/cross-track separation for bounded
/// relative motion (assuming δa ≈ 0). When δa ≠ 0, the metric is
/// still a useful safety indicator but not exact.
///
/// # Arguments
/// * `roe` - Quasi-nonsingular ROE state
/// * `chief` - Chief Keplerian elements (provides semi-major axis `a`)
/// * `config` - Safety threshold configuration
#[must_use]
pub fn analyze_safety(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
    config: &SafetyConfig,
) -> SafetyMetrics {
    let sma = chief.a;

    // Relative eccentricity vector magnitude
    let ecc_mag = (roe.dex * roe.dex + roe.dey * roe.dey).sqrt();
    // Relative inclination vector magnitude
    let inc_mag = (roe.dix * roe.dix + roe.diy * roe.diy).sqrt();

    // Phase angle between e/i vectors (D'Amico Eq. 2.22 context)
    let phase_e = roe.dey.atan2(roe.dex);
    let phase_i = roe.diy.atan2(roe.dix);
    let ei_phase_raw = (phase_e - phase_i).rem_euclid(std::f64::consts::TAU);
    // Normalize to [-π, π]
    let ei_phase = if ei_phase_raw > std::f64::consts::PI {
        ei_phase_raw - std::f64::consts::TAU
    } else {
        ei_phase_raw
    };

    // Minimum radial/cross-track separation (D'Amico Eq. 2.22):
    // δr_nr^min = sqrt(2) · a · |δe × δi| / sqrt(δe² + δi²)
    // Cross product magnitude |dex*diy - dey*dix| avoids trig on the phase angle.
    let min_sep = if ecc_mag > 1e-15 && inc_mag > 1e-15 {
        let cross = (roe.dex * roe.diy - roe.dey * roe.dix).abs();
        let norm_sq = ecc_mag * ecc_mag + inc_mag * inc_mag;
        (2.0_f64).sqrt() * sma * cross / norm_sq.sqrt()
    } else {
        0.0
    };

    SafetyMetrics {
        min_radial_crosstrack_km: min_sep,
        de_magnitude: ecc_mag,
        di_magnitude: inc_mag,
        ei_phase_angle_rad: ei_phase,
        is_safe: min_sep >= config.min_separation_km,
    }
}

/// Check if e/i vector separation exceeds a minimum threshold (D'Amico Eq. 2.24).
///
/// Returns `true` if the formation is passively safe.
#[must_use]
pub fn check_ei_separation(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
    min_sep_km: f64,
) -> bool {
    let config = SafetyConfig {
        min_separation_km: min_sep_km,
    };
    analyze_safety(roe, chief, &config).is_safe
}

/// Analyze safety along a trajectory, returning the worst-case metrics.
///
/// Evaluates safety at every point in the trajectory and returns
/// the metrics corresponding to the minimum separation.
///
/// # Panics
/// Panics if the trajectory is empty.
#[must_use]
pub fn analyze_trajectory_safety(
    trajectory: &[PropagatedState],
    config: &SafetyConfig,
) -> SafetyMetrics {
    assert!(!trajectory.is_empty(), "trajectory must not be empty");

    let mut worst = analyze_safety(&trajectory[0].roe, &trajectory[0].chief_mean, config);

    for state in &trajectory[1..] {
        let metrics = analyze_safety(&state.roe, &state.chief_mean, config);
        if metrics.min_radial_crosstrack_km < worst.min_radial_crosstrack_km {
            worst = metrics;
        }
    }

    worst
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;

    /// Zero ROE should be unsafe (zero separation).
    #[test]
    fn zero_roe_unsafe() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let config = SafetyConfig::default();
        let metrics = analyze_safety(&roe, &chief, &config);

        assert!(!metrics.is_safe, "Zero ROE should be unsafe");
        assert!(
            metrics.min_radial_crosstrack_km < 1e-10,
            "Min separation should be ~0"
        );
    }

    /// Pure δe separation (no δi) should be unsafe (zero cross-track).
    #[test]
    fn pure_de_separation_unsafe() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.001,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let config = SafetyConfig::default();
        let metrics = analyze_safety(&roe, &chief, &config);

        assert!(!metrics.is_safe, "Pure δe with no δi should be unsafe");
    }

    /// Full e/i separation with perpendicular vectors should be safe.
    #[test]
    fn full_ei_separation_safe() {
        let chief = iss_like_elements();
        // δe along x, δi along y → perpendicular → maximum safety
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.001,
            dey: 0.0,
            dix: 0.0,
            diy: 0.001,
        };
        let config = SafetyConfig {
            min_separation_km: 0.1,
        };
        let metrics = analyze_safety(&roe, &chief, &config);

        assert!(
            metrics.min_radial_crosstrack_km > 0.1,
            "Perpendicular e/i vectors should give good separation: {} km",
            metrics.min_radial_crosstrack_km
        );
        assert!(metrics.is_safe, "Should be safe with perpendicular e/i");
    }

    /// Phase angle: parallel e/i vectors should give zero separation.
    #[test]
    fn parallel_ei_zero_separation() {
        let chief = iss_like_elements();
        // δe and δi both along x → parallel → zero cross product
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.001,
            dey: 0.0,
            dix: 0.001,
            diy: 0.0,
        };
        let config = SafetyConfig::default();
        let metrics = analyze_safety(&roe, &chief, &config);

        assert!(
            metrics.min_radial_crosstrack_km < 1e-10,
            "Parallel e/i vectors should give zero separation: {} km",
            metrics.min_radial_crosstrack_km
        );
        assert!(!metrics.is_safe, "Parallel e/i should be unsafe");
    }

    /// Trajectory worst-case analysis.
    #[test]
    fn trajectory_worst_case() {
        use crate::propagation::propagator::{J2StmPropagator, RelativePropagator};
        use crate::test_helpers::test_epoch;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = J2StmPropagator;
        let period = std::f64::consts::TAU / chief.mean_motion();

        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.001,
            dex: 0.001,
            dey: 0.0,
            dix: 0.0,
            diy: 0.001,
        };

        let trajectory = propagator
            .propagate_with_steps(&roe, &chief, epoch, period, 20)
            .expect("propagation should succeed");

        let config = SafetyConfig {
            min_separation_km: 0.1,
        };
        let worst = analyze_trajectory_safety(&trajectory, &config);

        // Should have valid metrics
        assert!(worst.de_magnitude > 0.0);
        assert!(worst.di_magnitude > 0.0);
    }
}

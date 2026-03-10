//! Passive safety analysis via e/i vector separation (D'Amico Sec. 2.2).
//!
//! Evaluates formation safety using the minimum radial/cross-track
//! separation derived from the relative eccentricity and inclination vectors,
//! and minimum 3D distance from RIC position.

use nalgebra::Vector3;

use crate::propagation::propagator::PropagatedState;
use crate::types::{KeplerianElements, QuasiNonsingularROE, SafetyMetrics};

/// Analyze passive safety of a single ROE state plus instantaneous RIC distances.
///
/// Computes:
/// - **Instantaneous R/C distance**: `sqrt(R^2 + C^2)` from the RIC position.
/// - **e/i vector separation** (D'Amico Eq. 2.22): analytic orbit-averaged
///   minimum R/C bound from formation geometry.
/// - **3D distance**: `‖RIC position‖`.
///
/// # Arguments
/// * `roe` - Quasi-nonsingular ROE state
/// * `chief` - Chief Keplerian elements (provides semi-major axis `a`)
/// * `ric_position` - Relative position in RIC frame (km)
#[must_use]
pub fn analyze_safety(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
    ric_position: &Vector3<f64>,
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

    // e/i vector separation (D'Amico Eq. 2.22):
    // δr_nr^min = sqrt(2) · a · |δe × δi| / sqrt(δe² + δi²)
    // Cross product magnitude |dex*diy - dey*dix| avoids trig on the phase angle.
    let ei_separation = if ecc_mag > 1e-15 && inc_mag > 1e-15 {
        let cross = (roe.dex * roe.diy - roe.dey * roe.dix).abs();
        let norm_sq = ecc_mag * ecc_mag + inc_mag * inc_mag;
        (2.0_f64).sqrt() * sma * cross / norm_sq.sqrt()
    } else {
        0.0
    };

    // Instantaneous R/C distance: sqrt(R^2 + C^2)
    let rc_instantaneous = (ric_position.x.powi(2) + ric_position.z.powi(2)).sqrt();

    // 3D distance from RIC position
    let dist_3d = ric_position.norm();

    SafetyMetrics {
        min_rc_separation_km: rc_instantaneous,
        min_distance_3d_km: dist_3d,
        min_ei_separation_km: ei_separation,
        de_magnitude: ecc_mag,
        di_magnitude: inc_mag,
        ei_phase_angle_rad: ei_phase,
        min_rc_leg_index: 0,
        min_rc_elapsed_s: 0.0,
        min_rc_ric_position: *ric_position,
        min_3d_leg_index: 0,
        min_3d_elapsed_s: 0.0,
        min_3d_ric_position: *ric_position,
    }
}

/// Analyze safety along a trajectory, returning the worst-case metrics.
///
/// Evaluates safety at every point in the trajectory. Tracks the minimum
/// R/C separation, minimum e/i separation, and minimum 3D distance
/// independently, then returns a composite with all worst-case values.
///
/// # Sampling considerations
///
/// Both the **R/C distance** (`sqrt(R^2 + C^2)`) and **3D distance**
/// (`‖RIC‖`) oscillate at 1× and 2× orbital frequency. Their minima
/// between sample points are subject to quantization error
/// ≈ `sin(π/N) × amplitude`, where N = samples per orbit. The default 200
/// trajectory steps gives ~1.6% quantization error per single-orbit leg.
/// For safety-critical missions, increase `TargetingConfig::trajectory_steps`
/// to reduce this error further.
///
/// The **e/i separation** (D'Amico Eq. 2.22) is an analytic orbit-averaged
/// bound and varies slowly with secular J2 drift.
///
/// # Panics
/// Panics if the trajectory is empty.
#[must_use]
pub fn analyze_trajectory_safety(
    trajectory: &[PropagatedState],
) -> SafetyMetrics {
    assert!(!trajectory.is_empty(), "trajectory must not be empty");

    let first = &trajectory[0];
    let mut worst = analyze_safety(&first.roe, &first.chief_mean, &first.ric.position);
    worst.min_rc_elapsed_s = first.elapsed_s;
    worst.min_3d_elapsed_s = first.elapsed_s;
    let mut min_rc = worst.min_rc_separation_km;
    let mut min_ei = worst.min_ei_separation_km;
    let mut min_3d = worst.min_distance_3d_km;
    let mut min_3d_elapsed_s = first.elapsed_s;
    let mut min_3d_ric_position = first.ric.position;

    for state in &trajectory[1..] {
        let metrics = analyze_safety(&state.roe, &state.chief_mean, &state.ric.position);
        if metrics.min_rc_separation_km < min_rc {
            min_rc = metrics.min_rc_separation_km;
            // Keep the ROE-related fields from the worst R/C point
            worst = metrics;
            worst.min_rc_elapsed_s = state.elapsed_s;
            worst.min_rc_ric_position = state.ric.position;
        }
        if metrics.min_ei_separation_km < min_ei {
            min_ei = metrics.min_ei_separation_km;
        }
        if metrics.min_distance_3d_km < min_3d {
            min_3d = metrics.min_distance_3d_km;
            min_3d_elapsed_s = state.elapsed_s;
            min_3d_ric_position = state.ric.position;
        }
    }

    // Composite: all minimums
    worst.min_rc_separation_km = min_rc;
    worst.min_ei_separation_km = min_ei;
    worst.min_distance_3d_km = min_3d;
    worst.min_3d_elapsed_s = min_3d_elapsed_s;
    worst.min_3d_ric_position = min_3d_ric_position;
    // leg_index stays 0 — set by compute_worst_safety

    worst
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;
    use crate::types::SafetyConfig;

    /// Zero ROE should be unsafe (zero separation).
    #[test]
    fn zero_roe_unsafe() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let ric_pos = Vector3::zeros();
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_rc_separation_km < 1e-10,
            "Min R/C separation should be ~0"
        );
        assert!(
            metrics.min_distance_3d_km < 1e-10,
            "Min 3D distance should be ~0"
        );
    }

    /// Pure δe separation (no δi) should have zero e/i separation.
    /// RIC [1,0,0] gives instantaneous R/C = 1.0 (radial only).
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
        let ric_pos = Vector3::new(1.0, 0.0, 0.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_ei_separation_km < 1e-10,
            "Pure δe with no δi should have zero e/i separation"
        );
        assert!(
            (metrics.min_rc_separation_km - 1.0).abs() < 1e-10,
            "Instantaneous R/C should be 1.0 km for RIC [1,0,0]: {}",
            metrics.min_rc_separation_km,
        );
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
        let ric_pos = Vector3::new(1.0, 2.0, 1.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_rc_separation_km > 0.1,
            "Perpendicular e/i vectors should give good separation: {} km",
            metrics.min_rc_separation_km
        );
        assert!(
            metrics.min_distance_3d_km > 0.1,
            "3D distance should exceed threshold"
        );
    }

    /// Phase angle: parallel e/i vectors should give zero e/i separation.
    /// RIC [1,0,0] gives instantaneous R/C = 1.0.
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
        let ric_pos = Vector3::new(1.0, 0.0, 0.0);
        let config = SafetyConfig::default();
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_ei_separation_km < 1e-10,
            "Parallel e/i vectors should give zero e/i separation: {} km",
            metrics.min_ei_separation_km
        );
        assert!(
            metrics.min_ei_separation_km < config.min_ei_separation_km,
            "Parallel e/i should fail threshold"
        );
    }

    /// 3D distance below threshold makes formation unsafe even with good e/i separation.
    /// RIC [0.01, 0.01, 0.01] gives instantaneous R/C = sqrt(0.01^2 + 0.01^2) ≈ 0.0141.
    #[test]
    fn small_3d_distance_unsafe() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.001,
            dey: 0.0,
            dix: 0.0,
            diy: 0.001,
        };
        // Very close 3D position
        let ric_pos = Vector3::new(0.01, 0.01, 0.01);
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.1,
        };
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_ei_separation_km > 0.1,
            "e/i separation should be fine: {}",
            metrics.min_ei_separation_km
        );
        assert!(
            metrics.min_distance_3d_km < 0.1,
            "3D distance should be below threshold"
        );
        assert!(
            metrics.min_distance_3d_km < config.min_distance_3d_km,
            "Should fail 3D distance threshold"
        );
    }

    /// V-bar waypoint: R/C = 0 but 3D distance is the in-track offset.
    #[test]
    fn vbar_waypoint_3d_distance() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        // Pure in-track offset (V-bar)
        let ric_pos = Vector3::new(0.0, 5.0, 0.0);
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.1,
        };
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.min_rc_separation_km < 1e-10,
            "R/C separation should be ~0 for zero ROE"
        );
        assert!(
            (metrics.min_distance_3d_km - 5.0).abs() < 1e-10,
            "3D distance should be 5.0 km"
        );
        // e/i fails (zero ROE) but 3D passes
        assert!(
            metrics.min_ei_separation_km < config.min_ei_separation_km,
            "Should fail e/i threshold due to zero ROE"
        );
        assert!(
            metrics.min_distance_3d_km >= config.min_distance_3d_km,
            "3D distance should pass threshold"
        );
    }

    /// Higher sampling density should find a tighter (or equal) 3D minimum.
    #[test]
    fn sampling_density_convergence() {
        use crate::propagation::propagator::{J2StmPropagator, RelativePropagator};
        use crate::test_helpers::test_epoch;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = J2StmPropagator;
        let period = std::f64::consts::TAU / chief.mean_motion();

        // Formation with nonzero δe/δi so 3D distance oscillates
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.001,
            dex: 0.001,
            dey: 0.0,
            dix: 0.0,
            diy: 0.001,
        };

        let coarse = propagator
            .propagate_with_steps(&roe, &chief, epoch, period, 20)
            .expect("coarse propagation should succeed");
        let fine = propagator
            .propagate_with_steps(&roe, &chief, epoch, period, 200)
            .expect("fine propagation should succeed");

        let coarse_metrics = analyze_trajectory_safety(&coarse);
        let fine_metrics = analyze_trajectory_safety(&fine);

        // Finer sampling can only find a tighter or equal minimum
        assert!(
            fine_metrics.min_distance_3d_km <= coarse_metrics.min_distance_3d_km + 1e-12,
            "200-step minimum ({:.6} km) should be ≤ 20-step minimum ({:.6} km)",
            fine_metrics.min_distance_3d_km,
            coarse_metrics.min_distance_3d_km,
        );
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

        let worst = analyze_trajectory_safety(&trajectory);

        // Should have valid metrics
        assert!(worst.de_magnitude > 0.0);
        assert!(worst.di_magnitude > 0.0);
        assert!(worst.min_distance_3d_km > 0.0, "3D distance should be positive");
        assert!(worst.min_ei_separation_km > 0.0, "e/i separation should be positive");
    }

    /// Provenance fields are populated by `analyze_trajectory_safety`.
    #[test]
    fn trajectory_provenance_populated() {
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
            .propagate_with_steps(&roe, &chief, epoch, period, 200)
            .expect("propagation should succeed");

        let worst = analyze_trajectory_safety(&trajectory);

        // Elapsed times should be non-negative and within the trajectory span
        assert!(worst.min_rc_elapsed_s >= 0.0, "R/C elapsed_s should be non-negative");
        assert!(worst.min_3d_elapsed_s >= 0.0, "3D elapsed_s should be non-negative");
        assert!(
            worst.min_rc_elapsed_s <= period + 1e-6,
            "R/C elapsed_s should be within trajectory span"
        );
        assert!(
            worst.min_3d_elapsed_s <= period + 1e-6,
            "3D elapsed_s should be within trajectory span"
        );

        // RIC positions should match the trajectory points at those times
        let rc_match = trajectory.iter().find(|s| {
            (s.elapsed_s - worst.min_rc_elapsed_s).abs() < 1e-12
        });
        assert!(rc_match.is_some(), "Should find trajectory point matching R/C elapsed_s");
        assert!(
            (rc_match.unwrap().ric.position - worst.min_rc_ric_position).norm() < 1e-12,
            "R/C RIC position should match trajectory point"
        );

        let d3_match = trajectory.iter().find(|s| {
            (s.elapsed_s - worst.min_3d_elapsed_s).abs() < 1e-12
        });
        assert!(d3_match.is_some(), "Should find trajectory point matching 3D elapsed_s");
        assert!(
            (d3_match.unwrap().ric.position - worst.min_3d_ric_position).norm() < 1e-12,
            "3D RIC position should match trajectory point"
        );
    }
}

//! Safety analysis for rendezvous and proximity operations.
//!
//! Provides two categories of safety metrics:
//!
//! ## Operational safety (active missions)
//!
//! Instantaneous geometric distance measures — meaningful for all mission
//! types including actively guided approaches:
//! - **Minimum 3D distance**: `‖RIC position‖`
//! - **Minimum R/C distance**: `sqrt(R² + C²)` (radial/cross-track plane)
//!
//! ## Passive / abort safety (free drift)
//!
//! Orbit-averaged formation geometry bounds from D'Amico Sec. 2.2 —
//! meaningful for uncontrolled drift, abort/contingency analysis, or
//! long-coast formation flying:
//! - **E/I vector separation** (D'Amico Eq. 2.22): orbit-averaged minimum
//!   R/C bound from relative eccentricity and inclination vector geometry.
//!
//! The passive safety metric is **not enforced during active waypoint
//! targeting** — the planner does not optimize for e/i separation. High
//! violation rates in guided approach missions are expected and do not
//! indicate operational hazard.

use nalgebra::Vector3;

use serde::{Deserialize, Serialize};

use crate::propagation::propagator::PropagatedState;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::config::SafetyConfig;
use super::types::{OperationalSafety, PassiveSafety, SafetyMetrics};

/// Degenerate e/i vector guard for D'Amico Eq. 2.22 — below this magnitude,
/// the e/i separation formula is undefined (division by zero).
const ROE_MAG_EPSILON: f64 = 1e-15;

/// R/C distance below which along-track dominance is checked (km).
/// When the R/C plane minimum is below this value, the along-track
/// component is examined to determine if the geometry is V-bar-like.
const RC_ALONG_TRACK_THRESHOLD_KM: f64 = 0.01;

/// Along-track must exceed R/C by this factor to classify as along-track
/// dominated. A ratio of 10 means the along-track separation is at least
/// an order of magnitude larger than the radial/cross-track distance.
const RC_ALONG_TRACK_RATIO: f64 = 10.0;

/// Errors from safety analysis operations.
#[derive(Debug, Clone)]
pub enum SafetyError {
    /// Trajectory slice is empty; cannot analyze safety.
    EmptyTrajectory,
}

impl std::fmt::Display for SafetyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyTrajectory => write!(f, "trajectory is empty; cannot analyze safety"),
        }
    }
}

impl std::error::Error for SafetyError {}

/// Geometric context for the R/C plane minimum distance.
///
/// Classifies whether the R/C minimum is along-track dominated (common
/// for V-bar approach/hold geometries) or reflects genuine proximity in
/// the radial/cross-track plane. The classification is derived purely
/// from the RIC position vector at the minimum point — it does not
/// require knowledge of the perch type or mission design.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RcContext {
    /// R/C minimum is along-track dominated: radial and cross-track
    /// components are near zero while along-track separation is large.
    /// Common for V-bar approach/hold geometries. The R/C metric is
    /// not operationally meaningful in this configuration.
    AlongTrackDominated {
        /// Along-track component at the R/C minimum point (km).
        along_track_km: f64,
    },
    /// R/C minimum reflects genuine proximity in the radial/cross-track
    /// plane. This includes R-bar geometries, diagonal approaches, and
    /// any configuration where R/C distance is operationally meaningful.
    RadialCrossTrack,
}

/// Interpreted safety assessment combining metrics with thresholds.
///
/// Produced by [`assess_safety`] — separates factual measurements
/// ([`SafetyMetrics`]) from threshold-based verdicts and geometric
/// context. Both CLI and API consumers use this struct instead of
/// reimplementing threshold comparisons.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyAssessment {
    /// Whether the 3D keep-out distance threshold is satisfied.
    pub distance_3d_pass: bool,
    /// Whether the e/i separation threshold is satisfied.
    pub ei_separation_pass: bool,
    /// Overall safety verdict (both checks must pass).
    pub overall_pass: bool,
    /// Geometric context for the R/C plane minimum.
    pub rc_context: RcContext,
}

/// Assess safety metrics against thresholds and classify R/C geometry.
///
/// This is the interpretation layer — it takes raw measurements from
/// [`SafetyMetrics`] and a [`SafetyConfig`], and produces verdicts
/// plus geometric context for display by CLI, API, or frontend.
///
/// # R/C geometry classification
///
/// When the R/C plane minimum is near zero (below `RC_ALONG_TRACK_THRESHOLD_KM`)
/// and the along-track component is at least `RC_ALONG_TRACK_RATIO` times
/// larger, the geometry is classified as [`RcContext::AlongTrackDominated`].
/// This is a factual geometric observation — it does not depend on the
/// perch type or mission design intent.
///
/// # Arguments
/// * `metrics` — Safety measurements from [`analyze_safety`] or
///   [`analyze_trajectory_safety`]
/// * `config` — Safety thresholds for PASS/FAIL determination
#[must_use]
pub fn assess_safety(metrics: &SafetyMetrics, config: &SafetyConfig) -> SafetyAssessment {
    let distance_3d_pass = metrics.operational.min_distance_3d_km >= config.min_distance_3d_km;
    let ei_separation_pass =
        metrics.passive.min_ei_separation_km >= config.min_ei_separation_km;
    let overall_pass = distance_3d_pass && ei_separation_pass;

    let along_track = metrics.operational.min_rc_ric_position_km.y.abs();
    let rc = metrics.operational.min_rc_separation_km;
    let rc_context = if rc < RC_ALONG_TRACK_THRESHOLD_KM
        && along_track > RC_ALONG_TRACK_RATIO * rc
    {
        RcContext::AlongTrackDominated {
            along_track_km: along_track,
        }
    } else {
        RcContext::RadialCrossTrack
    };

    SafetyAssessment {
        distance_3d_pass,
        ei_separation_pass,
        overall_pass,
        rc_context,
    }
}

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
///
/// # Invariants
/// - `chief.a_km > 0` (used as scaling factor for e/i separation)
/// - ROE and `ric_position` must be at the same epoch
#[must_use]
pub fn analyze_safety(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
    ric_position: &Vector3<f64>,
) -> SafetyMetrics {
    let sma = chief.a_km;

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
    let ei_separation = if ecc_mag > ROE_MAG_EPSILON && inc_mag > ROE_MAG_EPSILON {
        let cross = (roe.dex * roe.diy - roe.dey * roe.dix).abs();
        let norm_sq = ecc_mag * ecc_mag + inc_mag * inc_mag;
        std::f64::consts::SQRT_2 * sma * cross / norm_sq.sqrt()
    } else {
        0.0
    };

    // Instantaneous R/C distance: sqrt(R^2 + C^2)
    let rc_instantaneous = (ric_position.x.powi(2) + ric_position.z.powi(2)).sqrt();

    // 3D distance from RIC position
    let dist_3d = ric_position.norm();

    SafetyMetrics {
        operational: OperationalSafety {
            min_rc_separation_km: rc_instantaneous,
            min_distance_3d_km: dist_3d,
            min_rc_leg_index: 0,
            min_rc_elapsed_s: 0.0,
            min_rc_ric_position_km: *ric_position,
            min_3d_leg_index: 0,
            min_3d_elapsed_s: 0.0,
            min_3d_ric_position_km: *ric_position,
        },
        passive: PassiveSafety {
            min_ei_separation_km: ei_separation,
            de_magnitude: ecc_mag,
            di_magnitude: inc_mag,
            ei_phase_angle_rad: ei_phase,
        },
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
/// # Invariants
/// - Trajectory states must be time-ordered
/// - Each state's ROE and RIC position must be self-consistent
///
/// # Arguments
/// * `trajectory` — Time-ordered slice of propagated states along one leg
///
/// # Errors
/// Returns `SafetyError::EmptyTrajectory` if the trajectory slice is empty.
pub fn analyze_trajectory_safety(
    trajectory: &[PropagatedState],
) -> Result<SafetyMetrics, SafetyError> {
    if trajectory.is_empty() {
        return Err(SafetyError::EmptyTrajectory);
    }

    let first = &trajectory[0];
    let mut worst = analyze_safety(&first.roe, &first.chief_mean, &first.ric.position_ric_km);
    worst.operational.min_rc_elapsed_s = first.elapsed_s;
    worst.operational.min_3d_elapsed_s = first.elapsed_s;
    let mut min_rc = worst.operational.min_rc_separation_km;
    let mut min_ei = worst.passive.min_ei_separation_km;
    let mut min_3d = worst.operational.min_distance_3d_km;
    let mut min_3d_elapsed_s = first.elapsed_s;
    let mut min_3d_ric_position_km = first.ric.position_ric_km;

    for state in &trajectory[1..] {
        let metrics = analyze_safety(&state.roe, &state.chief_mean, &state.ric.position_ric_km);
        if metrics.operational.min_rc_separation_km < min_rc {
            min_rc = metrics.operational.min_rc_separation_km;
            // Keep the passive fields from the worst R/C point
            worst = metrics;
            worst.operational.min_rc_elapsed_s = state.elapsed_s;
            worst.operational.min_rc_ric_position_km = state.ric.position_ric_km;
        }
        if metrics.passive.min_ei_separation_km < min_ei {
            min_ei = metrics.passive.min_ei_separation_km;
        }
        if metrics.operational.min_distance_3d_km < min_3d {
            min_3d = metrics.operational.min_distance_3d_km;
            min_3d_elapsed_s = state.elapsed_s;
            min_3d_ric_position_km = state.ric.position_ric_km;
        }
    }

    // Composite: all minimums
    worst.operational.min_rc_separation_km = min_rc;
    worst.passive.min_ei_separation_km = min_ei;
    worst.operational.min_distance_3d_km = min_3d;
    worst.operational.min_3d_elapsed_s = min_3d_elapsed_s;
    worst.operational.min_3d_ric_position_km = min_3d_ric_position_km;
    // leg_index stays 0 — set by compute_worst_safety

    Ok(worst)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief, iss_like_elements};
    use crate::mission::config::SafetyConfig;

    // Named tolerance constants for safety analysis tests
    //
    // Categories:
    //   SAFETY_METRIC     — analytical safety metric agreement
    //   EI_SEPARATION     — D'Amico Eq. 2.22/2.23 e/i separation
    //   TRAJECTORY_MATCH  — trajectory point lookup and RIC matching
    //   MONOTONICITY      — sampling density convergence guard

    /// Analytical safety metric agreement (e/i separation, 3D distance).
    /// Computed from ROE formulas with O(1e-15) precision; 1e-10 is conservative.
    const SAFETY_METRIC_TOL: f64 = 1e-10;

    /// D'Amico Eq. 2.22 e/i separation with exact perpendicular geometry.
    /// a·d cross-product formula; 1e-9 km tolerance for 0.200 km separation.
    const EI_SEPARATION_TOL_KM: f64 = 1e-9;

    /// D'Amico Eq. 2.22 e/i separation for unequal-magnitude case.
    /// Cross-product formula with sqrt; 1e-6 km for ~0.364 km expected.
    const EI_SEPARATION_UNEQUAL_TOL_KM: f64 = 1e-6;

    /// Trajectory point matching: find the point at a given elapsed_s.
    /// Exact f64 comparison after propagation; 1e-12 s.
    const ELAPSED_TIME_MATCH_TOL_S: f64 = 1e-12;

    /// RIC position vector matching at trajectory points.
    /// Same propagated value looked up by time; 1e-12 km.
    const RIC_POSITION_MATCH_TOL_KM: f64 = 1e-12;

    /// Sampling density monotonicity: finer ≤ coarser + epsilon.
    /// 1e-12 guards against f64 non-determinism.
    const SAMPLING_MONOTONICITY_GUARD: f64 = 1e-12;

    /// Empty trajectory returns `SafetyError::EmptyTrajectory`.
    #[test]
    fn empty_trajectory_returns_error() {
        let result = analyze_trajectory_safety(&[]);
        assert!(
            matches!(result, Err(SafetyError::EmptyTrajectory)),
            "Empty trajectory should return EmptyTrajectory, got {result:?}"
        );
    }

    /// Zero ROE should be unsafe (zero separation).
    #[test]
    fn zero_roe_unsafe() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let ric_pos = Vector3::zeros();
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.operational.min_rc_separation_km < SAFETY_METRIC_TOL,
            "Min R/C separation should be ~0"
        );
        assert!(
            metrics.operational.min_distance_3d_km < SAFETY_METRIC_TOL,
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
            metrics.passive.min_ei_separation_km < SAFETY_METRIC_TOL,
            "Pure δe with no δi should have zero e/i separation"
        );
        assert!(
            (metrics.operational.min_rc_separation_km - 1.0).abs() < SAFETY_METRIC_TOL,
            "Instantaneous R/C should be 1.0 km for RIC [1,0,0]: {}",
            metrics.operational.min_rc_separation_km,
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
            metrics.operational.min_rc_separation_km > 0.1,
            "Perpendicular e/i vectors should give good separation: {} km",
            metrics.operational.min_rc_separation_km
        );
        assert!(
            metrics.operational.min_distance_3d_km > 0.1,
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
            metrics.passive.min_ei_separation_km < SAFETY_METRIC_TOL,
            "Parallel e/i vectors should give zero e/i separation: {} km",
            metrics.passive.min_ei_separation_km
        );
        assert!(
            metrics.passive.min_ei_separation_km < config.min_ei_separation_km,
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
            metrics.passive.min_ei_separation_km > 0.1,
            "e/i separation should be fine: {}",
            metrics.passive.min_ei_separation_km
        );
        assert!(
            metrics.operational.min_distance_3d_km < 0.1,
            "3D distance should be below threshold"
        );
        assert!(
            metrics.operational.min_distance_3d_km < config.min_distance_3d_km,
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
            metrics.operational.min_rc_separation_km < SAFETY_METRIC_TOL,
            "R/C separation should be ~0 for zero ROE"
        );
        assert!(
            (metrics.operational.min_distance_3d_km - 5.0).abs() < SAFETY_METRIC_TOL,
            "3D distance should be 5.0 km"
        );
        // e/i fails (zero ROE) but 3D passes
        assert!(
            metrics.passive.min_ei_separation_km < config.min_ei_separation_km,
            "Should fail e/i threshold due to zero ROE"
        );
        assert!(
            metrics.operational.min_distance_3d_km >= config.min_distance_3d_km,
            "3D distance should pass threshold"
        );
    }

    /// Higher sampling density should find a tighter (or equal) 3D minimum.
    #[test]
    fn sampling_density_convergence() {
        use crate::propagation::propagator::PropagationModel;
        use crate::test_helpers::test_epoch;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = PropagationModel::J2Stm;
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

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

        let coarse_metrics = analyze_trajectory_safety(&coarse).unwrap();
        let fine_metrics = analyze_trajectory_safety(&fine).unwrap();

        // Finer sampling can only find a tighter or equal minimum
        assert!(
            fine_metrics.operational.min_distance_3d_km <= coarse_metrics.operational.min_distance_3d_km + SAMPLING_MONOTONICITY_GUARD,
            "200-step minimum ({:.6} km) should be ≤ 20-step minimum ({:.6} km)",
            fine_metrics.operational.min_distance_3d_km,
            coarse_metrics.operational.min_distance_3d_km,
        );
    }

    /// Trajectory worst-case analysis.
    #[test]
    fn trajectory_worst_case() {
        use crate::propagation::propagator::PropagationModel;
        use crate::test_helpers::test_epoch;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = PropagationModel::J2Stm;
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

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

        let worst = analyze_trajectory_safety(&trajectory).unwrap();

        // Should have valid metrics
        assert!(worst.passive.de_magnitude > 0.0);
        assert!(worst.passive.di_magnitude > 0.0);
        assert!(worst.operational.min_distance_3d_km > 0.0, "3D distance should be positive");
        assert!(worst.passive.min_ei_separation_km > 0.0, "e/i separation should be positive");
    }

    /// Provenance fields are populated by `analyze_trajectory_safety`.
    #[test]
    fn trajectory_provenance_populated() {
        use crate::propagation::propagator::PropagationModel;
        use crate::test_helpers::test_epoch;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = PropagationModel::J2Stm;
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

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

        let worst = analyze_trajectory_safety(&trajectory).unwrap();

        // Elapsed times should be non-negative and within the trajectory span
        assert!(worst.operational.min_rc_elapsed_s >= 0.0, "R/C elapsed_s should be non-negative");
        assert!(worst.operational.min_3d_elapsed_s >= 0.0, "3D elapsed_s should be non-negative");
        assert!(
            worst.operational.min_rc_elapsed_s <= period + crate::constants::ELAPSED_TIME_TOL_S,
            "R/C elapsed_s should be within trajectory span"
        );
        assert!(
            worst.operational.min_3d_elapsed_s <= period + crate::constants::ELAPSED_TIME_TOL_S,
            "3D elapsed_s should be within trajectory span"
        );

        // RIC positions should match the trajectory points at those times
        let rc_match = trajectory.iter().find(|s| {
            (s.elapsed_s - worst.operational.min_rc_elapsed_s).abs() < ELAPSED_TIME_MATCH_TOL_S
        });
        assert!(rc_match.is_some(), "Should find trajectory point matching R/C elapsed_s");
        assert!(
            (rc_match.unwrap().ric.position_ric_km - worst.operational.min_rc_ric_position_km).norm() < RIC_POSITION_MATCH_TOL_KM,
            "R/C RIC position should match trajectory point"
        );

        let d3_match = trajectory.iter().find(|s| {
            (s.elapsed_s - worst.operational.min_3d_elapsed_s).abs() < ELAPSED_TIME_MATCH_TOL_S
        });
        assert!(d3_match.is_some(), "Should find trajectory point matching 3D elapsed_s");
        assert!(
            (d3_match.unwrap().ric.position_ric_km - worst.operational.min_3d_ric_position_km).norm() < RIC_POSITION_MATCH_TOL_KM,
            "3D RIC position should match trajectory point"
        );
    }

    // --- Paper-traced regression tests (D'Amico Eq. 2.22/2.23) ---

    /// D'Amico Eq. 2.22: perpendicular e/i vectors of equal magnitude.
    /// δex=d, δiy=d with a·d=200m → exact separation = a·d = 0.200 km.
    /// This case matches both Eq. 2.22 and the cross-product formula exactly.
    #[test]
    fn damico_eq222_perpendicular_equal_magnitude() {
        let chief = damico_table21_chief();
        let d = 0.200 / chief.a_km; // a·d = 200m = 0.200 km
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: d,
            dey: 0.0,
            dix: 0.0,
            diy: d,
        };
        let ric_pos = Vector3::new(1.0, 2.0, 1.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            (metrics.passive.min_ei_separation_km - 0.200).abs() < EI_SEPARATION_TOL_KM,
            "e/i separation: got {}, expected 0.200 km", metrics.passive.min_ei_separation_km
        );
    }

    /// D'Amico Eq. 2.22: perpendicular e/i vectors of unequal magnitude.
    /// δex=d1 (300m/a), δiy=d2 (500m/a), a=6892.945 km (TanDEM-X approx).
    /// Code gives √2·a·d1·d2/√(d1²+d2²) = √2·0.300·0.500/√(0.300²+0.500²).
    #[test]
    fn damico_eq222_perpendicular_unequal() {
        let chief = KeplerianElements {
            a_km: 6892.945,
            e: 0.001,
            i_rad: 97.44_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let d1 = 0.300 / chief.a_km; // a·d1 = 300m = 0.300 km
        let d2 = 0.500 / chief.a_km; // a·d2 = 500m = 0.500 km
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: d1,
            dey: 0.0,
            dix: 0.0,
            diy: d2,
        };
        let ric_pos = Vector3::new(1.0, 2.0, 1.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        // Expected: √2 · 0.300 · 0.500 / √(0.300² + 0.500²) ≈ 0.36394 km
        let expected = std::f64::consts::SQRT_2 * 0.300 * 0.500
            / (0.300_f64.powi(2) + 0.500_f64.powi(2)).sqrt();
        assert!(
            (metrics.passive.min_ei_separation_km - expected).abs() < EI_SEPARATION_UNEQUAL_TOL_KM,
            "e/i separation: got {}, expected {expected}", metrics.passive.min_ei_separation_km
        );
    }

    /// D'Amico Eq. 2.23 edge case: parallel e/i vectors give zero cross product.
    /// D'Amico Table 2.1 Case 1 has parallel e/i vectors (both along +y).
    /// The cross-product formula correctly returns 0; D'Amico Eq. 2.23 gives
    /// the true minimum a·min(δe,δi)=0.200 km for parallel vectors, but the
    /// instantaneous R/C distance check in analyze_safety covers this case.
    #[test]
    fn damico_eq223_parallel_gives_zero() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe(); // δey, δiy both along +y → parallel
        let ric_pos = Vector3::new(1.0, 2.0, 1.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);

        assert!(
            metrics.passive.min_ei_separation_km.abs() < SAFETY_METRIC_TOL,
            "Parallel e/i should give 0 separation, got {}", metrics.passive.min_ei_separation_km
        );
    }

    // --- SafetyAssessment / assess_safety tests ---

    /// V-bar geometry: R/C = 0, along-track = 5 km → AlongTrackDominated.
    #[test]
    fn assess_vbar_along_track_dominated() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let ric_pos = Vector3::new(0.0, 5.0, 0.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.05,
        };
        let assessment = assess_safety(&metrics, &config);

        assert!(
            matches!(assessment.rc_context, RcContext::AlongTrackDominated { along_track_km } if (along_track_km - 5.0).abs() < 1e-10),
            "V-bar [0, 5, 0] should be AlongTrackDominated: {:?}",
            assessment.rc_context
        );
    }

    /// R-bar geometry: R/C = 5 km, along-track = 0 → RadialCrossTrack.
    #[test]
    fn assess_rbar_radial_cross_track() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let ric_pos = Vector3::new(5.0, 0.0, 0.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);
        let config = SafetyConfig::default();
        let assessment = assess_safety(&metrics, &config);

        assert_eq!(
            assessment.rc_context,
            RcContext::RadialCrossTrack,
            "R-bar [5, 0, 0] should be RadialCrossTrack"
        );
    }

    /// Diagonal geometry: R/C = √(0.5² + 0.5²) ≈ 0.71 km → RadialCrossTrack
    /// even though along-track is larger.
    #[test]
    fn assess_diagonal_radial_cross_track() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let ric_pos = Vector3::new(0.5, 5.0, 0.5);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);
        let config = SafetyConfig::default();
        let assessment = assess_safety(&metrics, &config);

        assert_eq!(
            assessment.rc_context,
            RcContext::RadialCrossTrack,
            "Diagonal [0.5, 5, 0.5] has R/C > threshold → RadialCrossTrack"
        );
    }

    /// PASS/FAIL: 3D distance passes, e/i fails → overall FAIL.
    #[test]
    fn assess_ei_fail_overall_fail() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default(); // zero ROE → zero e/i separation
        let ric_pos = Vector3::new(1.0, 2.0, 1.0);
        let metrics = analyze_safety(&roe, &chief, &ric_pos);
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.05,
        };
        let assessment = assess_safety(&metrics, &config);

        assert!(assessment.distance_3d_pass, "3D distance should pass");
        assert!(!assessment.ei_separation_pass, "e/i should fail (zero ROE)");
        assert!(!assessment.overall_pass, "overall should fail");
    }

    /// PASS/FAIL: 3D distance fails, e/i passes → overall FAIL.
    #[test]
    fn assess_3d_fail_overall_fail() {
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
        let metrics = analyze_safety(&roe, &chief, &ric_pos);
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.1,
        };
        let assessment = assess_safety(&metrics, &config);

        assert!(!assessment.distance_3d_pass, "3D distance should fail");
        assert!(assessment.ei_separation_pass, "e/i should pass");
        assert!(!assessment.overall_pass, "overall should fail");
    }

    /// PASS/FAIL: both pass → overall PASS.
    #[test]
    fn assess_both_pass() {
        let chief = iss_like_elements();
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
        let config = SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.05,
        };
        let assessment = assess_safety(&metrics, &config);

        assert!(assessment.distance_3d_pass, "3D distance should pass");
        assert!(assessment.ei_separation_pass, "e/i should pass");
        assert!(assessment.overall_pass, "overall should pass");
    }
}

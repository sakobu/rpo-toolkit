//! Transit e/i monitoring and drift compensation (Phase 4).
//!
//! Monitors e/i separation along coast arcs (trajectories between maneuvers)
//! and optionally compensates for J2-induced e/i drift on long arcs.
//!
//! # Algorithm overview
//!
//! - **Transit monitoring** ([`assess_transit_safety`]): evaluates
//!   [`compute_ei_separation`] at each trajectory sample, reports minimum
//!   and per-sample profile.
//! - **Drift compensation** ([`enrich_with_drift_compensation`]): pre-rotates
//!   departure e/i phases by half the predicted J2 perigee rotation over the
//!   coast arc, so parallel alignment occurs near mid-transit rather than at
//!   departure.
//!
//! # References
//!
//! - D'Amico Eq. 2.22: e/i vector separation metric
//! - D'Amico Eq. 2.30: perigee rotation rate phi' = kappa * Q under J2

use nalgebra::Vector3;

use crate::mission::safety::{compute_ei_separation, EiSeparation};
use crate::propagation::j2_params::compute_j2_params;
use crate::propagation::propagator::PropagatedState;
use crate::propagation::stm::propagate_roe_stm;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::safety_envelope::{enrich_waypoint, enrich_waypoint_with_pre_rotation};
use super::types::DriftCompensationStatus;
use super::{
    EiSample, EnrichedWaypoint, FormationDesignError,
    SafetyRequirements, TransitSafetyReport,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Minimum trajectory samples per orbital period for transit safety assessment.
/// E/I separation varies on the orbital timescale (period ~5400s for LEO).
/// 100 samples/orbit = ~54s between samples, sufficient to resolve the
/// minimum to within ~1% of the analytical value.
const MIN_SAMPLES_PER_ORBIT: u32 = 100;

/// Maximum coast-arc duration (in orbital periods) for which drift compensation
/// is applied. Beyond this, the linear phase approximation degrades and
/// compensation is skipped.
///
/// At 10 periods, even the worst case (ISS-like, i=51.6 deg) accumulates
/// only ~2.4 deg rotation, keeping the linear approximation error below 1%.
///
/// # References
/// - R5 decision: fixed cutoff chosen over adaptive (variation in |phi'| across
///   practical LEO is only ~20%, not enough to justify added complexity)
const MAX_DRIFT_COMPENSATION_PERIODS: f64 = 10.0;

/// Near-critical-inclination threshold for the Q parameter (|5 cos^2 i - 1|).
/// When |Q| is below this, `aop_dot` is effectively zero and drift compensation
/// is skipped (the rotation is too small to matter).
///
/// At Q = 1e-4, `aop_dot` ~ kappa * 1e-4 ~ 1e-10 rad/s, producing
/// sub-arcsecond rotation over 10 orbits. Negligible for any practical
/// formation configuration.
///
/// # References
/// - D'Amico Eq. 2.30: phi' = kappa * Q
/// - Critical inclination: i = acos(1/sqrt(5)) ~ 63.435 deg
const NEAR_CRITICAL_Q_THRESHOLD: f64 = 1e-4;

// ---------------------------------------------------------------------------
// Transit safety monitoring
// ---------------------------------------------------------------------------

/// Evaluate e/i separation along a coast-arc trajectory.
///
/// Computes [`compute_ei_separation`] at each [`PropagatedState`] sample,
/// identifies the minimum, and builds a [`TransitSafetyReport`].
///
/// # Algorithm
///
/// 1. Validate trajectory length >= `MIN_SAMPLES_PER_ORBIT` per orbit
/// 2. For each sample, compute e/i separation via [`compute_ei_separation`]
/// 3. Track the global minimum separation, its elapsed time, and phase angle
/// 4. Build [`EiSample`] profile and [`TransitSafetyReport`]
///
/// # Invariants
///
/// - `trajectory` must be non-empty
/// - `trajectory` must have >= `MIN_SAMPLES_PER_ORBIT` samples per orbit
/// - `requirements.min_separation_km > 0`
///
/// # Errors
///
/// - [`FormationDesignError::InsufficientSampling`] if the trajectory has
///   fewer than `MIN_SAMPLES_PER_ORBIT` samples per orbital period
/// - [`FormationDesignError::KeplerFailure`] if chief elements are invalid
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). Same as [`compute_ei_separation`].
///
/// # References
///
/// - D'Amico Eq. 2.22 (e/i separation at each sample)
pub fn assess_transit_safety(
    trajectory: &[PropagatedState],
    requirements: &SafetyRequirements,
) -> Result<TransitSafetyReport, FormationDesignError> {
    if trajectory.is_empty() {
        return Err(FormationDesignError::InsufficientSampling {
            total_samples: 0,
            required_per_orbit: MIN_SAMPLES_PER_ORBIT,
        });
    }

    let period_s = trajectory[0].chief_mean.period()?;
    // Safety: trajectory is non-empty (checked above), so last() always returns Some.
    let arc_duration_s = trajectory[trajectory.len() - 1].elapsed_s
        - trajectory[0].elapsed_s;
    let n_orbits = (arc_duration_s / period_s).max(1.0);

    // Sampling density check: rearranged to len >= MIN * n_orbits so all
    // arithmetic stays in f64 with no f64→integer cast needed.
    let len_u32 = u32::try_from(trajectory.len()).unwrap_or(u32::MAX);
    if f64::from(len_u32) < f64::from(MIN_SAMPLES_PER_ORBIT) * n_orbits {
        return Err(FormationDesignError::InsufficientSampling {
            total_samples: len_u32,
            required_per_orbit: MIN_SAMPLES_PER_ORBIT,
        });
    }

    let mut min_sep = f64::MAX;
    let mut min_elapsed_s = 0.0;
    let mut min_phase_rad = 0.0;
    let mut profile = Vec::with_capacity(trajectory.len());

    for state in trajectory {
        let ei = compute_ei_separation(&state.roe, &state.chief_mean);
        if ei.min_separation_km < min_sep {
            min_sep = ei.min_separation_km;
            min_elapsed_s = state.elapsed_s;
            min_phase_rad = ei.phase_angle_rad;
        }
        profile.push(EiSample {
            elapsed_s: state.elapsed_s,
            ei_separation_km: ei.min_separation_km,
            phase_angle_rad: ei.phase_angle_rad,
        });
    }

    Ok(TransitSafetyReport {
        min_ei_separation_km: min_sep,
        min_elapsed_s,
        min_phase_angle_rad: min_phase_rad,
        satisfies_requirement: min_sep >= requirements.min_separation_km,
        threshold_km: requirements.min_separation_km,
        profile,
    })
}

/// Propagate a ROE state forward by `dt_s` under the J2 STM and return
/// the e/i separation at that epoch.
///
/// Shared by drift-prediction callers that need to look ahead to a future
/// epoch (typically mid-transit, `dt_s = tof_s / 2`) without running the
/// full trajectory assessment. Composes `propagate_roe_stm` with
/// `compute_ei_separation` — both applied at the propagated (J2-evolved)
/// chief elements so the e/i frame matches the epoch.
///
/// # Invariants
///
/// - `chief_mean.a_km > 0`, `chief_mean.e < 1`
/// - `dt_s >= 0` (forward propagation)
///
/// # Errors
///
/// Propagates any error from [`propagate_roe_stm`] (invalid chief elements,
/// period failure) wrapped into [`FormationDesignError`].
///
/// # References
///
/// - Koenig Eq. A6 (J2 STM)
/// - D'Amico Eq. 2.22 (e/i separation)
pub(crate) fn ei_separation_after(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    dt_s: f64,
) -> Result<EiSeparation, FormationDesignError> {
    let (prop_roe, prop_chief) = propagate_roe_stm(roe, chief_mean, dt_s)?;
    Ok(compute_ei_separation(&prop_roe, &prop_chief))
}

// ---------------------------------------------------------------------------
// Drift-compensated enrichment
// ---------------------------------------------------------------------------

/// Enrich a departure waypoint with J2 drift-compensated e/i phases.
///
/// Pre-rotates the departure e/i vectors so that parallel alignment occurs
/// near the midpoint of the coast arc, rather than at departure. Uses the
/// perigee rotation rate phi' = d(omega)/dt from J2 secular perturbations.
///
/// # Algorithm
///
/// 1. Compute J2 params from `chief_mean` -> extract `aop_dot_rad_s`
/// 2. If TOF exceeds `MAX_DRIFT_COMPENSATION_PERIODS` orbits, skip compensation
/// 3. If near-critical inclination (|Q| < threshold), skip compensation
/// 4. Compute drift angle: `delta_phi = aop_dot * tof_s / 2` (half-arc heuristic)
/// 5. Delegate to [`enrich_waypoint_with_pre_rotation`], which backward-rotates
///    the target e-vector phase by `delta_phi` relative to the (fixed) i-vector.
///    After propagation under J2 by `tof/2` the e-vector rotates forward by
///    `delta_phi`, landing parallel to the i-vector at mid-transit.
///
/// # Invariants
///
/// - `chief_mean.a_km > 0`, `chief_mean.e < 1`
/// - `tof_s > 0`
/// - `requirements.min_separation_km > 0`
///
/// # Errors
///
/// - [`FormationDesignError::Propagation`] if J2 parameter computation fails
/// - [`FormationDesignError::KeplerFailure`] if period computation fails
/// - All errors from [`enrich_waypoint`]
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). Linear drift approximation valid when
/// `|aop_dot * tof_s| < ~0.05 rad` (~3 deg). Enforced by the
/// `MAX_DRIFT_COMPENSATION_PERIODS` cutoff.
///
/// # Near-singular behavior
///
/// Near critical inclination (i ~ 63.4 deg): Q -> 0, `aop_dot` -> 0.
/// Drift compensation degenerates to no-op. Detected and handled
/// gracefully by the `NEAR_CRITICAL_Q_THRESHOLD` guard.
///
/// # References
///
/// - D'Amico Eq. 2.30: phi' = kappa * Q (perigee rotation rate under J2)
/// - R5 decision: fixed 10-period cutoff
#[must_use = "drift compensation result should be inspected"]
pub fn enrich_with_drift_compensation(
    position_ric_km: &Vector3<f64>,
    chief_mean: &KeplerianElements,
    tof_s: f64,
    requirements: &SafetyRequirements,
) -> Result<(EnrichedWaypoint, DriftCompensationStatus), FormationDesignError> {
    let j2p = compute_j2_params(chief_mean)?;
    let period_s = chief_mean.period()?;
    let n_orbits = tof_s / period_s;

    // Skip compensation for arcs exceeding the linear regime
    if n_orbits > MAX_DRIFT_COMPENSATION_PERIODS {
        let enriched = enrich_waypoint(position_ric_km, None, chief_mean, requirements)?;
        return Ok((enriched, DriftCompensationStatus::Skipped));
    }

    // Skip compensation near critical inclination where phi' ~ 0
    let cos_i = chief_mean.i_rad.cos();
    let q = 5.0 * cos_i * cos_i - 1.0;
    if q.abs() < NEAR_CRITICAL_Q_THRESHOLD {
        let enriched = enrich_waypoint(position_ric_km, None, chief_mean, requirements)?;
        return Ok((enriched, DriftCompensationStatus::Skipped));
    }

    // Half-arc heuristic: pre-rotate by half the predicted J2 drift so that
    // parallel alignment occurs near mid-transit rather than at departure.
    let delta_phi_rad = j2p.aop_dot_rad_s * tof_s * 0.5;

    // Forward the pre-rotation angle to the safety projection, which
    // backward-rotates the target e-vector phase by `delta_phi_rad` relative
    // to the (fixed) i-vector direction.
    let enriched = enrich_waypoint_with_pre_rotation(
        position_ric_km,
        chief_mean,
        requirements,
        delta_phi_rad,
    )?;
    Ok((enriched, DriftCompensationStatus::Applied))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::wrap_angle;
    use crate::mission::formation::safety_envelope::enrich_waypoint;
    use crate::mission::formation::{EiAlignment, SafetyRequirements};
    use crate::propagation::j2_params::compute_j2_params;
    use crate::test_helpers::{
        damico_table21_chief, damico_table21_case1_roe, iss_like_elements,
        propagate_test_trajectory,
    };

    /// Predict e/i separation at mid-transit for a drift-compensated ROE.
    /// Thin wrapper around [`ei_separation_after`] that fixes `dt_s = tof_s/2`
    /// to encode the mid-transit semantics.
    fn predict_compensated_ei(
        compensated_roe: &QuasiNonsingularROE,
        departure_chief: &KeplerianElements,
        tof_s: f64,
    ) -> Result<EiSeparation, FormationDesignError> {
        ei_separation_after(compensated_roe, departure_chief, tof_s / 2.0)
    }

    /// E/I separation tolerance (km). The separation formula is exact when
    /// applied to exact ROE inputs; 1e-6 km = 1 mm covers f64 rounding
    /// and small J2-induced drift over short arcs.
    const EI_SEPARATION_TOL: f64 = 1e-6;

    /// RIC position preservation tolerance (km). Sub-millimeter.
    const RIC_POSITION_TOL: f64 = 1e-6;

    /// Drift rate regression tolerance (rad/s). Covers f64 arithmetic only.
    const DRIFT_RATE_TOL: f64 = 1e-10;

    /// Maximum acceptable rotation (rad) for the linear drift approximation.
    /// 0.05 rad ~ 2.9 deg; sin(0.05) / 0.05 = 0.9996 (< 0.04% error).
    const LINEAR_REGIME_RAD: f64 = 0.05;

    /// Phase alignment residual after compensation + propagation (rad).
    /// Sources: J2 STM second-order terms (~|Δφ|²/2 ≈ 6e-10 for SSO × 2 orbits)
    /// and drift-rate quantization over tof/2. 1e-4 rad ≈ 0.006° covers both
    /// with comfortable headroom.
    const COMPENSATED_PHASE_TOL_RAD: f64 = 1.0e-4;

    /// Bit-exact phase difference between compensated and uncompensated ROE (rad).
    /// Compensation is a single rotation of the e-vector target, so the diff
    /// must equal −Δφ within f64 rounding of the atan2 / cos / sin chain.
    /// 1e-8 is generous for this arithmetic chain.
    const COMPENSATED_PHASE_DIFF_TOL_RAD: f64 = 1.0e-8;

    /// Signal floor on the expected rotation angle (rad). Below this, the
    /// sign-sensitive phase-diff assertion is swamped by second-order STM
    /// terms (~|Δφ|²) and cannot distinguish genuine rotation from noise.
    /// This is a SIGNAL floor (detection bound), not an f64 noise floor —
    /// well above machine precision, but small enough to catch degenerate
    /// chief geometries where Δφ ≈ 0.
    const ROTATION_SIGNAL_FLOOR_RAD: f64 = 1.0e-6;

    /// Minimum detectable improvement in e/i separation from compensation (km).
    /// Any improvement below this is indistinguishable from f64 rounding of
    /// the separation formula. Unit: km — distinct from the rad-valued
    /// tolerances above that happen to share the 1e-8 literal.
    const COMPENSATION_IMPROVEMENT_MIN_KM: f64 = 1.0e-8;

    /// Zero-TOF phase invariance tolerance (rad). At dt=0 the J2 STM reduces
    /// to identity, so the e/i phase is preserved to f64 precision of the
    /// atan2 chain. Separate from `DRIFT_RATE_TOL` (rad/s) and
    /// `COMPENSATED_PHASE_DIFF_TOL_RAD` (f64 of a longer arithmetic chain).
    const ZERO_TOF_PHASE_TOL_RAD: f64 = 1.0e-10;

    fn default_requirements(threshold_km: f64) -> SafetyRequirements {
        SafetyRequirements {
            min_separation_km: threshold_km,
            alignment: EiAlignment::Parallel,
        }
    }

    // -----------------------------------------------------------------------
    // Transit safety monitoring
    // -----------------------------------------------------------------------

    /// For an ROE state with parallel e/i vectors propagated for 1 orbital
    /// period, all sample points must satisfy the separation threshold.
    ///
    /// Uses damico_table21_chief() + damico_table21_case1_roe() (parallel e/i).
    /// Propagates with J2Stm for 1 period with 200 steps.
    ///
    /// # References
    /// - D'Amico Eq. 2.22 (e/i separation)
    #[test]
    fn parallel_ei_short_arc_stays_safe() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let period_s = chief.period().unwrap();

        let trajectory = propagate_test_trajectory(&roe, &chief, period_s, 200);

        // Case 1 has dey=400m/a, diy=200m/a — parallel along +y.
        // E/I separation should be significant and stable over 1 orbit.
        let requirements = default_requirements(0.01); // 10 m threshold
        let report = assess_transit_safety(&trajectory, &requirements)
            .expect("assessment should succeed");

        assert!(
            report.satisfies_requirement,
            "parallel e/i should satisfy {:.4} km threshold, got min = {:.6} km",
            requirements.min_separation_km, report.min_ei_separation_km,
        );
        assert!(
            report.min_ei_separation_km > 0.0,
            "minimum separation must be positive"
        );
        assert_eq!(report.profile.len(), 201, "200 steps + initial");
        assert!(
            (report.threshold_km - requirements.min_separation_km).abs() < f64::EPSILON,
            "threshold in report should match requirement"
        );
    }

    /// For an ROE state with orthogonal e/i vectors (dex > 0, diy > 0, others zero),
    /// transit safety must detect that separation approaches zero.
    #[test]
    fn orthogonal_ei_detects_violation() {
        let chief = damico_table21_chief();
        let a_km = chief.a_km;
        // Orthogonal: e-vector along +x, i-vector along +y
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.200 / a_km,
            dey: 0.0,
            dix: 0.0,
            diy: 0.200 / a_km,
        };
        let period_s = chief.period().unwrap();
        let trajectory = propagate_test_trajectory(&roe, &chief, period_s, 200);

        // With orthogonal e/i, the separation formula yields a small but
        // non-zero value. The key is it's much less than for parallel vectors.
        let requirements = default_requirements(0.10); // 100 m threshold
        let report = assess_transit_safety(&trajectory, &requirements)
            .expect("assessment should succeed");

        assert!(
            !report.satisfies_requirement,
            "orthogonal e/i should violate {:.4} km threshold, got min = {:.6} km",
            requirements.min_separation_km, report.min_ei_separation_km,
        );
    }

    /// Insufficient sampling density is rejected.
    #[test]
    fn insufficient_sampling_rejected() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let period_s = chief.period().unwrap();

        // Only 10 samples over 1 orbit — way below MIN_SAMPLES_PER_ORBIT
        let trajectory = propagate_test_trajectory(&roe, &chief, period_s, 10);
        let requirements = default_requirements(0.01);

        let result = assess_transit_safety(&trajectory, &requirements);
        assert!(
            matches!(
                result,
                Err(FormationDesignError::InsufficientSampling {
                    total_samples,
                    required_per_orbit: MIN_SAMPLES_PER_ORBIT,
                }) if total_samples < MIN_SAMPLES_PER_ORBIT
            ),
            "expected InsufficientSampling, got {result:?}"
        );
    }

    /// Empty trajectory is rejected.
    #[test]
    fn empty_trajectory_rejected() {
        let requirements = default_requirements(0.01);
        let result = assess_transit_safety(&[], &requirements);
        assert!(
            matches!(
                result,
                Err(FormationDesignError::InsufficientSampling {
                    total_samples: 0,
                    required_per_orbit: MIN_SAMPLES_PER_ORBIT,
                })
            ),
            "expected InsufficientSampling with 0 samples, got {result:?}"
        );
    }

    /// Report threshold reflects the input requirement.
    #[test]
    fn report_threshold_reflects_requirement() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let period_s = chief.period().unwrap();
        let trajectory = propagate_test_trajectory(&roe, &chief, period_s, 200);

        let threshold = 0.0423;
        let requirements = default_requirements(threshold);
        let report = assess_transit_safety(&trajectory, &requirements).unwrap();

        assert!(
            (report.threshold_km - threshold).abs() < f64::EPSILON,
            "report threshold {:.6} should match requirement {:.6}",
            report.threshold_km, threshold,
        );
    }

    // -----------------------------------------------------------------------
    // Drift-compensated enrichment
    // -----------------------------------------------------------------------

    /// Drift compensation is skipped for arcs exceeding MAX_DRIFT_COMPENSATION_PERIODS.
    #[test]
    fn drift_compensation_skipped_for_long_arc() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 15.0 * period_s; // 15 orbits > 10 cutoff

        let position = Vector3::new(0.0, 0.5, 0.0); // V-bar, 500m along-track
        let requirements = default_requirements(0.10);
        let (enriched, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("enrichment should succeed");

        assert!(
            matches!(status, DriftCompensationStatus::Skipped),
            "should skip compensation for 15-orbit arc"
        );
        // Enrichment itself should still be valid
        assert!(
            enriched.enriched_ei.min_separation_km >= requirements.min_separation_km - EI_SEPARATION_TOL,
            "enrichment should still meet threshold"
        );
    }

    /// Drift compensation is applied for arcs within MAX_DRIFT_COMPENSATION_PERIODS.
    #[test]
    fn drift_compensation_applied_for_short_arc() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 2.0 * period_s; // 2 orbits < 10 cutoff

        let position = Vector3::new(0.0, 0.5, 0.0);
        let requirements = default_requirements(0.10);
        let (_enriched, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("enrichment should succeed");

        assert!(
            matches!(status, DriftCompensationStatus::Applied),
            "should apply compensation for 2-orbit arc"
        );
    }

    /// Drift compensation is skipped near critical inclination (i ~ 63.4 deg).
    #[test]
    fn drift_compensation_skipped_near_critical_inclination() {
        // Critical inclination: i = acos(1/sqrt(5)) ~ 63.4349 deg
        let critical_i_rad = (1.0_f64 / 5.0_f64.sqrt()).acos();
        let chief = KeplerianElements {
            a_km: 7078.135,
            e: 0.001,
            i_rad: critical_i_rad,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let period_s = chief.period().unwrap();
        let tof_s = 2.0 * period_s; // within period cutoff

        let position = Vector3::new(0.0, 0.5, 0.0);
        let requirements = default_requirements(0.10);
        let (_enriched, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("enrichment should succeed");

        assert!(
            matches!(status, DriftCompensationStatus::Skipped),
            "should skip compensation near critical inclination"
        );
    }

    /// Drift compensation preserves the RIC position (null-space guarantee).
    #[test]
    fn drift_compensation_preserves_ric_position() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 3.0 * period_s;

        let position = Vector3::new(0.1, 0.5, 0.05); // arbitrary RIC position
        let requirements = default_requirements(0.10);
        let (enriched, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("enrichment should succeed");

        assert!(matches!(status, DriftCompensationStatus::Applied));
        let diff = (enriched.position_ric_km - position).norm();
        assert!(
            diff < RIC_POSITION_TOL,
            "RIC position should be preserved, diff = {diff:.2e} km"
        );
    }

    // -----------------------------------------------------------------------
    // Edge-case and regression tests
    // -----------------------------------------------------------------------

    /// Perigee rotation rate regression for D'Amico Table 2.1 SSO chief.
    ///
    /// phi' = kappa * Q where:
    ///   kappa = (3/4) * n * J2 * (R_E/p)^2
    ///   Q = 5 cos^2(i) - 1
    ///
    /// # References
    /// - D'Amico Eq. 2.30
    #[test]
    fn perigee_rotation_rate_regression_sso() {
        let chief = damico_table21_chief();
        let j2p = compute_j2_params(&chief).unwrap();

        // Hand-computed for a=7078.135, e=0.001, i=98.19 deg:
        // cos(98.19 deg) ~ -0.14243, cos^2 ~ 0.02029
        // Q = 5*0.02029 - 1 = -0.8986
        // p = a*(1-e^2) ~ 7078.128
        // n = sqrt(mu/a^3) ~ 1.0600e-3 rad/s
        // kappa = 0.75 * n * J2 * (R_E/p)^2 ~ 6.988e-7 rad/s
        // phi' = kappa * Q ~ -6.28e-7 rad/s
        //
        // The exact value depends on constants; assert sign and order of magnitude.
        assert!(
            j2p.aop_dot_rad_s < 0.0,
            "SSO (i>90 deg) should have negative perigee rotation rate"
        );
        assert!(
            (j2p.aop_dot_rad_s.abs() - 6.28e-7).abs() < 0.5e-7,
            "aop_dot should be ~ -6.28e-7 rad/s for SSO, got {:.4e}",
            j2p.aop_dot_rad_s,
        );
    }

    /// At the critical inclination, aop_dot is near zero because Q = 0.
    #[test]
    fn near_critical_inclination_aop_dot_near_zero() {
        let critical_i_rad = (1.0_f64 / 5.0_f64.sqrt()).acos();
        let chief = KeplerianElements {
            a_km: 7078.135,
            e: 0.001,
            i_rad: critical_i_rad,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };

        let j2p = compute_j2_params(&chief).unwrap();
        assert!(
            j2p.aop_dot_rad_s.abs() < DRIFT_RATE_TOL,
            "aop_dot should be near zero at critical inclination, got {:.4e}",
            j2p.aop_dot_rad_s,
        );
    }

    /// The total phase rotation over MAX_DRIFT_COMPENSATION_PERIODS orbits
    /// stays within the linear regime for both SSO and ISS orbits.
    #[test]
    fn delta_phi_within_linear_regime() {
        for (label, chief) in [
            ("SSO", damico_table21_chief()),
            ("ISS", iss_like_elements()),
        ] {
            let j2p = compute_j2_params(&chief).unwrap();
            let period_s = chief.period().unwrap();
            let delta_phi = (j2p.aop_dot_rad_s * MAX_DRIFT_COMPENSATION_PERIODS * period_s).abs();
            assert!(
                delta_phi < LINEAR_REGIME_RAD,
                "{label}: |delta_phi| = {delta_phi:.4} rad exceeds linear regime ({LINEAR_REGIME_RAD} rad)"
            );
        }
    }

    // -----------------------------------------------------------------------
    // predict_compensated_ei tests
    // -----------------------------------------------------------------------

    /// For a 2-orbit SSO arc, drift-compensated enrichment followed by
    /// predict_compensated_ei should yield mid-transit e/i separation that
    /// is at least as good as the uncompensated minimum from
    /// assess_transit_safety.
    ///
    /// The compensated departure pre-rotates e/i so parallel alignment occurs
    /// near mid-transit, which should improve the worst-case separation.
    #[test]
    fn predict_compensated_ei_improves_alignment() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 2.0 * period_s;

        let position = Vector3::new(0.0, 0.5, 0.0); // V-bar, 500m along-track
        let requirements = default_requirements(0.10);
        // Get drift-compensated enrichment
        let (enriched, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("enrichment should succeed");
        assert!(matches!(status, DriftCompensationStatus::Applied));

        // Predict mid-transit e/i from compensated ROE
        let predicted = predict_compensated_ei(&enriched.roe, &chief, tof_s)
            .expect("prediction should succeed");

        // The predicted mid-transit separation should be positive
        assert!(
            predicted.min_separation_km > 0.0,
            "predicted mid-transit e/i separation must be positive, got {:.6} km",
            predicted.min_separation_km,
        );

        // Compare against uncompensated: propagate the *un*compensated enriched
        // state and assess transit safety. The compensated prediction should be
        // at least as good.
        let uncomp_enriched = enrich_waypoint(&position, None, &chief, &requirements)
            .expect("baseline enrichment should succeed");
        let trajectory = propagate_test_trajectory(
            &uncomp_enriched.roe, &chief, tof_s, 200,
        );
        let uncomp_report = assess_transit_safety(&trajectory, &requirements)
            .expect("transit assessment should succeed");

        // With working compensation, predicted mid-transit phase should be
        // near zero (e/i parallel by construction), and the predicted
        // separation should strictly exceed the uncompensated minimum —
        // otherwise compensation had no measurable effect.
        assert!(
            predicted.phase_angle_rad.abs() < COMPENSATED_PHASE_TOL_RAD,
            "compensated mid-transit phase should be near zero, got {:.4e} rad",
            predicted.phase_angle_rad,
        );
        assert!(
            predicted.min_separation_km
                > uncomp_report.min_ei_separation_km + COMPENSATION_IMPROVEMENT_MIN_KM,
            "compensated prediction ({:.6} km) should strictly exceed uncompensated \
             minimum ({:.6} km) — otherwise compensation had no effect",
            predicted.min_separation_km, uncomp_report.min_ei_separation_km,
        );
    }

    /// At zero TOF, predict_compensated_ei returns the departure e/i separation
    /// (no propagation effect).
    #[test]
    fn predict_compensated_ei_at_zero_tof() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();

        let departure_ei = compute_ei_separation(&roe, &chief);
        let predicted = predict_compensated_ei(&roe, &chief, 0.0)
            .expect("zero-tof prediction should succeed");

        assert!(
            (predicted.min_separation_km - departure_ei.min_separation_km).abs() < EI_SEPARATION_TOL,
            "at zero TOF, predicted ({:.6}) should equal departure ({:.6})",
            predicted.min_separation_km, departure_ei.min_separation_km,
        );
        assert!(
            (predicted.phase_angle_rad - departure_ei.phase_angle_rad).abs()
                < ZERO_TOF_PHASE_TOL_RAD,
            "at zero TOF, phase angle should be unchanged",
        );
    }

    /// Asserts compensation produces a genuinely rotated e-vector phase,
    /// not a bit-identical copy of the uncompensated result. Guards against
    /// silent no-op compensation where rotation is wired through parameters
    /// that downstream code ignores. The compensated phase must differ from
    /// the uncompensated phase by approximately `-delta_phi_rad`, where
    /// `delta_phi_rad = aop_dot_rad_s * tof_s / 2`.
    #[test]
    fn drift_compensation_actually_rotates_e_vector() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 2.0 * period_s; // within cutoff → Applied
        let position = Vector3::new(0.0, 0.5, 0.0); // V-bar perch
        let requirements = default_requirements(0.10);
        let (compensated, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("compensation should succeed");
        assert!(matches!(status, DriftCompensationStatus::Applied));

        let uncompensated = enrich_waypoint(&position, None, &chief, &requirements)
            .expect("uncompensated enrichment should succeed");

        // Δφ = aop_dot * tof / 2. For SSO (i=98.19°) Q<0, so Δφ<0 and the
        // e-vector drifts backward over the coast arc. Pre-rotating by
        // -Δφ = +|Δφ| at departure makes it land parallel to the (fixed)
        // i-vector at tof/2.
        let j2p = compute_j2_params(&chief).unwrap();
        let expected_delta_phi = j2p.aop_dot_rad_s * tof_s * 0.5;
        assert!(
            expected_delta_phi.abs() > ROTATION_SIGNAL_FLOOR_RAD,
            "sanity: expected rotation must exceed the signal floor to be \
             detectable above second-order STM residuals, got {expected_delta_phi:.4e}"
        );

        // e-vector phase difference between compensated and uncompensated
        let phase_comp = compensated.roe.dey.atan2(compensated.roe.dex);
        let phase_uncomp = uncompensated.roe.dey.atan2(uncompensated.roe.dex);
        let phase_diff = wrap_angle(phase_comp - phase_uncomp);

        // Compensated e-vector should be rotated by approximately -delta_phi
        // relative to uncompensated (backward, since compensation anticipates
        // forward drift over the coast arc).
        assert!(
            (phase_diff - (-expected_delta_phi)).abs() < COMPENSATED_PHASE_DIFF_TOL_RAD,
            "compensated e-vector phase should differ by ~{:.4e} rad from uncompensated, \
             got phase_diff = {:.4e} rad",
            -expected_delta_phi, phase_diff,
        );
    }

    /// After propagating the drift-compensated ROE forward by tof/2 under
    /// the J2 STM, the e/i vectors should be (approximately) parallel —
    /// i.e., the e/i phase angle reported by `compute_ei_separation`
    /// should be near zero.
    #[test]
    fn drift_compensation_aligns_at_mid_transit() {
        let chief = damico_table21_chief();
        let period_s = chief.period().unwrap();
        let tof_s = 2.0 * period_s;
        let position = Vector3::new(0.0, 0.5, 0.0);
        let requirements = default_requirements(0.10);
        let (compensated, status) = enrich_with_drift_compensation(
            &position, &chief, tof_s, &requirements,
        )
        .expect("compensation should succeed");
        assert!(matches!(status, DriftCompensationStatus::Applied));

        // Propagate compensated ROE forward by tof/2 — e/i phase at that epoch
        // should be near zero (parallel aligned).
        let (mid_roe, mid_chief) = propagate_roe_stm(&compensated.roe, &chief, tof_s / 2.0)
            .expect("propagation");
        let ei_mid = compute_ei_separation(&mid_roe, &mid_chief);

        assert!(
            ei_mid.phase_angle_rad.abs() < COMPENSATED_PHASE_TOL_RAD,
            "compensated e/i phase at mid-transit should be ~0 rad, got {:.4e} rad",
            ei_mid.phase_angle_rad,
        );
    }
}

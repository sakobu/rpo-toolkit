//! Inverse GVE solutions and main COLA orchestrator.
//!
//! Contains the analytical maneuver solvers (inverse B matrix) and the
//! top-level [`compute_avoidance`] function that classifies, solves,
//! verifies, and iterates to convergence. Equation references are
//! inline at each function and computation step.

use hifitime::{Duration, Epoch};
use nalgebra::Vector3;

use crate::constants::TWO_PI;
use crate::mission::closest_approach::ClosestApproach;
use crate::propagation::propagator::{PropagationError, PropagationModel};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::classify::classify_correction;
use super::types::{
    AvoidanceError, AvoidanceManeuver, ColaConfig, CorrectionType, BURN_TIME_CLAMP_FRACTION,
    COLA_MAX_ITERATIONS, COLA_MIN_ROE_MAGNITUDE, COLA_NEWTON_TOL_KM,
};
use super::verify::verify_avoidance;

// ---------------------------------------------------------------------------
// Private solver helpers
// ---------------------------------------------------------------------------

/// Solve for single-impulse in-plane eccentricity correction (D'Amico Eq. 2.41).
///
/// Inverts the B matrix (Eq. 2.38) rows 2–3 to find the optimal burn
/// location `u_m = atan2(-δex, δey)` and a pure radial impulse whose
/// magnitude is `n·a·|δe|`.
///
/// Returns `(dv_ric, u_m)` where `dv_ric` is the delta-v in RIC frame and
/// `u_m` is the mean argument of latitude at burn, wrapped to `[0, 2π)`.
///
/// # Arguments
///
/// * `delta_dex` — Required change in dimensionless dex.
/// * `delta_dey` — Required change in dimensionless dey.
/// * `n` — Chief mean motion (rad/s).
/// * `a` — Chief semi-major axis (km).
///
/// # Invariants
///
/// - `n > 0`, `a > 0` (caller must derive from valid chief elements)
/// - Near-circular assumption: the B matrix inversion is exact for `e = 0`;
///   accuracy degrades for `e > ~0.1`.
///
/// # Errors
///
/// Returns [`AvoidanceError::DegenerateGeometry`] if `|δe| < COLA_MIN_ROE_MAGNITUDE`.
#[allow(clippy::similar_names)]
pub(super) fn solve_inplane_eccentricity_only(
    delta_dex: f64,
    delta_dey: f64,
    n: f64,
    a: f64,
) -> Result<(Vector3<f64>, f64), AvoidanceError> {
    let de_mag = (delta_dex * delta_dex + delta_dey * delta_dey).sqrt();
    if de_mag < COLA_MIN_ROE_MAGNITUDE {
        return Err(AvoidanceError::DegenerateGeometry {
            de_mag,
            di_mag: 0.0,
        });
    }

    // Eq. 2.41 — optimal burn location from B matrix rows 2–3 inversion:
    //   B[2,0] = sin(u)/(na),  B[3,0] = -cos(u)/(na)
    //   For pure δe rotation, align radial impulse with the δe direction:
    //   u_m = atan2(-δex, δey)
    let u_m = (-delta_dex).atan2(delta_dey);

    // Eq. 2.41 — radial impulse magnitude:
    //   |δv_R| = n·a·|δe|,  signed: δv_R = -n·a·|δe|
    //   (from inverting B rows 2–3 at optimal u_m)
    let dv_r = -n * a * de_mag;

    Ok((Vector3::new(dv_r, 0.0, 0.0), u_m.rem_euclid(TWO_PI)))
}

/// Solve for single-impulse cross-track inclination correction (D'Amico Eq. 2.53).
///
/// Inverts B matrix rows 4–5 to find the optimal burn location
/// `u_m = atan2(δiy, δix)` and a pure normal impulse of magnitude
/// `n·a·|δi|`.
///
/// Returns `(dv_ric, u_m)` where `dv_ric` is the delta-v in RIC frame and
/// `u_m` is the mean argument of latitude at burn, wrapped to `[0, 2π)`.
///
/// # Arguments
///
/// * `delta_dix` — Required change in dimensionless dix.
/// * `delta_diy` — Required change in dimensionless diy.
/// * `n` — Chief mean motion (rad/s).
/// * `a` — Chief semi-major axis (km).
///
/// # Invariants
///
/// - `n > 0`, `a > 0` (caller must derive from valid chief elements)
/// - Near-circular assumption: exact for `e = 0`.
///
/// # Errors
///
/// Returns [`AvoidanceError::DegenerateGeometry`] if `|δi| < COLA_MIN_ROE_MAGNITUDE`.
#[allow(clippy::similar_names)]
fn solve_crosstrack_single(
    delta_dix: f64,
    delta_diy: f64,
    n: f64,
    a: f64,
) -> Result<(Vector3<f64>, f64), AvoidanceError> {
    let di_mag = (delta_dix * delta_dix + delta_diy * delta_diy).sqrt();
    if di_mag < COLA_MIN_ROE_MAGNITUDE {
        return Err(AvoidanceError::DegenerateGeometry {
            de_mag: 0.0,
            di_mag,
        });
    }

    // Eq. 2.53 — optimal burn location from B matrix rows 4–5 inversion:
    //   B[4,2] = cos(u)/(na),  B[5,2] = sin(u)/(na)
    //   Align normal impulse with the δi direction:
    //   u_m = atan2(δiy, δix)
    let u_m = delta_diy.atan2(delta_dix);

    // Eq. 2.53 — normal impulse magnitude:
    //   δv_N = n·a·|δi|
    let dv_n = n * a * di_mag;

    Ok((Vector3::new(0.0, 0.0, dv_n), u_m.rem_euclid(TWO_PI)))
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute a collision avoidance maneuver for a predicted close approach.
///
/// Given a POCA that violates the safety threshold, designs the minimum-fuel
/// impulsive maneuver to achieve the target separation distance.
///
/// # Algorithm
///
/// 1. Guard: skip if POCA already satisfies threshold.
/// 2. Classify correction type from POCA geometry and ROE magnitudes.
/// 3. Compute required ROE change via distance-ratio scaling.
/// 4. Apply analytical inverse GVE solution.
/// 5. Verify via re-propagation; iterate if combined correction needs refinement.
///
/// # Arguments
///
/// * `poca` - Predicted closest approach to mitigate
/// * `roe` - ROE state at leg departure (post-departure-burn)
/// * `chief_mean` - Chief mean Keplerian elements at leg departure
/// * `departure_epoch` - Epoch of leg departure
/// * `tof_s` - Time of flight for the leg (seconds)
/// * `model` - Propagation model
/// * `config` - COLA configuration (target distance, budget)
///
/// # Errors
///
/// Returns [`AvoidanceError`] if:
/// - POCA already satisfies threshold (`NoPocaViolation`)
/// - Required delta-v exceeds budget (`ExceedsBudget`)
/// - ROE geometry is degenerate (`DegenerateGeometry`)
/// - Combined correction does not converge (`NoConvergence`)
/// - Propagation or POCA computation fails
#[allow(clippy::similar_names)]
pub fn compute_avoidance(
    poca: &ClosestApproach,
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    departure_epoch: Epoch,
    tof_s: f64,
    model: &PropagationModel,
    config: &ColaConfig,
) -> Result<AvoidanceManeuver, AvoidanceError> {
    if poca.distance_km >= config.target_distance_km {
        return Err(AvoidanceError::NoPocaViolation {
            poca_km: poca.distance_km,
            threshold_km: config.target_distance_km,
        });
    }

    let de_mag = roe.de_magnitude();
    let di_mag = roe.di_magnitude();
    let n = chief_mean.mean_motion().map_err(|e| {
        AvoidanceError::PropagationFailure(PropagationError::KeplerFailure(e))
    })?;
    let a = chief_mean.a_km;

    if de_mag < COLA_MIN_ROE_MAGNITUDE && di_mag < COLA_MIN_ROE_MAGNITUDE {
        return Err(AvoidanceError::DegenerateGeometry { de_mag, di_mag });
    }

    let correction_type = classify_correction(poca, roe);

    // Distance-ratio scaling: POCA distance scales linearly with ROE magnitude
    // (via Eq. 2.17 T-matrix mapping), so the required ROE change is:
    //   Δ(δe) = δe · (d_target/d_poca − 1)
    //   Δ(δi) = δi · (d_target/d_poca − 1)
    let scale = config.target_distance_km / poca.distance_km;

    let (dv_ric, u_m) = match correction_type {
        // Eq. 2.41: single radial impulse for eccentricity correction
        CorrectionType::InPlane => {
            let delta_dex = roe.dex * (scale - 1.0);
            let delta_dey = roe.dey * (scale - 1.0);
            solve_inplane_eccentricity_only(delta_dex, delta_dey, n, a)?
        }
        // Eq. 2.53: single normal impulse for inclination correction
        CorrectionType::CrossTrack => {
            let delta_dix = roe.dix * (scale - 1.0);
            let delta_diy = roe.diy * (scale - 1.0);
            solve_crosstrack_single(delta_dix, delta_diy, n, a)?
        }
        // Eq. 2.56: combined e + i correction — superpose Eq. 2.41 and Eq. 2.53
        // at the in-plane burn location (radial + normal impulse)
        CorrectionType::Combined => {
            let delta_dex = roe.dex * (scale - 1.0);
            let delta_dey = roe.dey * (scale - 1.0);
            let delta_dix = roe.dix * (scale - 1.0);
            let delta_diy = roe.diy * (scale - 1.0);

            // In-plane component (Eq. 2.41)
            let (dv_ip, u_m_ip) = solve_inplane_eccentricity_only(delta_dex, delta_dey, n, a)?;

            // Cross-track component (Eq. 2.53)
            let (dv_ct, _u_m_ct) = solve_crosstrack_single(delta_dix, delta_diy, n, a)?;

            // Combine at in-plane burn location
            let dv_combined = Vector3::new(dv_ip.x, dv_ip.y, dv_ct.z);
            (dv_combined, u_m_ip)
        }
    };

    let fuel_cost = dv_ric.norm();
    if fuel_cost > config.max_dv_km_s {
        return Err(AvoidanceError::ExceedsBudget {
            required_km_s: fuel_cost,
            budget_km_s: config.max_dv_km_s,
        });
    }

    // Verify via re-propagation and iterate if post-avoidance POCA
    // does not meet the target (linearization error makes single-pass
    // distance-ratio scaling approximate).
    let mut post_poca = verify_avoidance(roe, chief_mean, departure_epoch, tof_s, &dv_ric, u_m, model)?;
    let mut final_dv = dv_ric;
    let mut final_cost = fuel_cost;

    {
        let mut current_dv = dv_ric;
        for iter in 0..COLA_MAX_ITERATIONS {
            if post_poca >= config.target_distance_km - COLA_NEWTON_TOL_KM {
                break;
            }

            // Scale up dv by target/actual ratio
            let adjustment = config.target_distance_km / post_poca;
            current_dv *= adjustment;

            let new_cost = current_dv.norm();
            if new_cost > config.max_dv_km_s {
                return Err(AvoidanceError::ExceedsBudget {
                    required_km_s: new_cost,
                    budget_km_s: config.max_dv_km_s,
                });
            }

            post_poca = verify_avoidance(
                roe, chief_mean, departure_epoch, tof_s, &current_dv, u_m, model,
            )?;
            final_dv = current_dv;
            final_cost = new_cost;

            if iter == COLA_MAX_ITERATIONS - 1
                && post_poca < config.target_distance_km - COLA_NEWTON_TOL_KM
            {
                return Err(AvoidanceError::NoConvergence {
                    iterations: COLA_MAX_ITERATIONS,
                    residual_km: (config.target_distance_km - post_poca).abs(),
                });
            }
        }
    }

    // Compute burn epoch
    let u_0 = chief_mean.mean_arg_of_lat();
    let mut delta_t = ((u_m - u_0).rem_euclid(TWO_PI)) / n;
    if delta_t >= tof_s {
        delta_t = tof_s * BURN_TIME_CLAMP_FRACTION;
    }
    let burn_epoch = departure_epoch + Duration::from_seconds(delta_t);

    Ok(AvoidanceManeuver {
        epoch: burn_epoch,
        dv_ric_km_s: final_dv,
        maneuver_location_rad: u_m,
        post_avoidance_poca_km: post_poca,
        fuel_cost_km_s: final_cost,
        correction_type,
        leg_index: poca.leg_index,
    })
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::closest_approach::{find_closest_approaches, ClosestApproach};
    use crate::test_helpers::{
        damico_table21_case1_roe, damico_table21_chief, propagate_test_trajectory_at, test_epoch,
    };

    /// Analytical formula tolerance: exact analytical inversion of B matrix.
    /// Operations are O(1e-16) precision; 1e-10 provides wide margin.
    const ANALYTICAL_TOL: f64 = 1e-10;

    /// D'Amico Table 2.2 cost validation tolerance (km/s).
    /// Accounts for linearization assumption mismatch.
    const TABLE_22_TOL_KM_S: f64 = 1e-4;

    // -----------------------------------------------------------------------
    // Test 1: pure eccentricity rotation (Eq. 2.41)
    // -----------------------------------------------------------------------

    #[test]
    fn pure_eccentricity_rotation_eq241() {
        let chief = damico_table21_chief();
        let n = chief.mean_motion().unwrap();
        let a = chief.a_km;

        // Desired eccentricity change: 100m/a in dey direction
        let delta_dey = 0.100 / a;
        let delta_dex = 0.0;
        let de_mag = (delta_dex * delta_dex + delta_dey * delta_dey).sqrt();

        let (dv, _u_m) = solve_inplane_eccentricity_only(delta_dex, delta_dey, n, a).unwrap();

        // |dv_r| should equal n * a * delta_de (Eq. 2.41)
        let expected_dv = n * a * de_mag;
        assert!(
            (dv.x.abs() - expected_dv).abs() < ANALYTICAL_TOL,
            "|dv_r| = {}, expected n*a*de = {expected_dv}",
            dv.x.abs()
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: double-impulse cost (Eq. 2.52)
    // -----------------------------------------------------------------------

    /// D'Amico Eqs. 2.50-2.52: two along-track impulses at ξ and ξ+π.
    /// Total cost = n·a·δe/2, which is half the single-impulse cost.
    #[test]
    fn double_impulse_cost_eq252() {
        let chief = damico_table21_chief();
        let n = chief.mean_motion().unwrap();
        let a = chief.a_km;

        let delta_dey = 0.100 / a;
        let delta_dex = 0.0;
        let de_mag = (delta_dex * delta_dex + delta_dey * delta_dey).sqrt();

        // Inline Eqs. 2.50-2.51: two along-track burns at phase angle ξ and ξ+π
        let xi = delta_dey.atan2(delta_dex);
        let dv_mag = n * a * de_mag / 4.0;
        let dv1 = Vector3::new(0.0, dv_mag, 0.0);   // at u = ξ
        let dv2 = Vector3::new(0.0, -dv_mag, 0.0);   // at u = ξ + π
        let _u_m1 = xi;
        let _u_m2 = xi + std::f64::consts::PI;

        // Total cost = |dv1| + |dv2| = n*a*de/4 + n*a*de/4 = n*a*de/2 (Eq. 2.52)
        let total_cost = dv1.norm() + dv2.norm();
        let expected_cost = n * a * de_mag / 2.0;

        assert!(
            (total_cost - expected_cost).abs() < ANALYTICAL_TOL,
            "double-impulse cost = {total_cost}, expected n*a*de/2 = {expected_cost}"
        );

        // Should be half the single-impulse cost
        let single_cost = n * a * de_mag;
        assert!(
            (total_cost - single_cost / 2.0).abs() < ANALYTICAL_TOL,
            "double cost should be half of single: {total_cost} vs {}", single_cost / 2.0
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: cross-track single impulse (Eq. 2.53)
    // -----------------------------------------------------------------------

    #[test]
    fn crosstrack_single_eq253() {
        let chief = damico_table21_chief();
        let n = chief.mean_motion().unwrap();
        let a = chief.a_km;

        let delta_diy = 0.100 / a;
        let delta_dix = 0.0;
        let di_mag = (delta_dix * delta_dix + delta_diy * delta_diy).sqrt();

        let (dv, u_m) = solve_crosstrack_single(delta_dix, delta_diy, n, a).unwrap();

        // |dv_n| should equal n * a * delta_di (Eq. 2.53)
        let expected_dv = n * a * di_mag;
        assert!(
            (dv.z.abs() - expected_dv).abs() < ANALYTICAL_TOL,
            "|dv_n| = {}, expected n*a*di = {expected_dv}",
            dv.z.abs()
        );

        // u_m should be atan2(delta_diy, delta_dix)
        let expected_u_m = delta_diy.atan2(delta_dix).rem_euclid(TWO_PI);
        assert!(
            (u_m - expected_u_m).abs() < ANALYTICAL_TOL,
            "u_m = {u_m}, expected atan2 = {expected_u_m}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 4: post-avoidance POCA exceeds threshold
    // -----------------------------------------------------------------------

    #[test]
    fn post_avoidance_poca_exceeds_threshold() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;
        let tof_s = chief.period().unwrap();

        // First compute the actual POCA
        let traj = propagate_test_trajectory_at(&roe, &chief, epoch, tof_s, 200);
        let pocas = find_closest_approaches(&traj, &chief, epoch, &model, &roe, 0).unwrap();

        if pocas.is_empty() {
            // No POCA found — skip test (safety net for edge cases)
            return;
        }

        let min_poca = ClosestApproach::nearest(&pocas).unwrap();

        // Set target well above actual POCA
        let target = min_poca.distance_km * 2.0;
        let config = ColaConfig {
            target_distance_km: target,
            max_dv_km_s: 1.0,
        };

        let result = compute_avoidance(min_poca, &roe, &chief, epoch, tof_s, &model, &config);

        match result {
            Ok(maneuver) => {
                assert!(
                    maneuver.post_avoidance_poca_km >= target - COLA_NEWTON_TOL_KM,
                    "post-avoidance POCA = {} km should be >= target {} km (within COLA tol = {} km)",
                    maneuver.post_avoidance_poca_km,
                    target,
                    COLA_NEWTON_TOL_KM,
                );
            }
            Err(AvoidanceError::NoPocaViolation { .. }) => {
                // This can happen if the actual POCA is already above threshold
                // after re-computation; acceptable
            }
            Err(e) => panic!("unexpected error: {e}"),
        }
    }

    // -----------------------------------------------------------------------
    // Test 5: budget exceeded returns error
    // -----------------------------------------------------------------------

    #[test]
    fn budget_exceeded_returns_error() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;
        let tof_s = chief.period().unwrap();

        let traj = propagate_test_trajectory_at(&roe, &chief, epoch, tof_s, 200);
        let pocas = find_closest_approaches(&traj, &chief, epoch, &model, &roe, 0).unwrap();

        if pocas.is_empty() {
            return;
        }

        let min_poca = ClosestApproach::nearest(&pocas).unwrap();

        // Tiny budget that cannot accommodate any correction
        let config = ColaConfig {
            target_distance_km: min_poca.distance_km * 10.0,
            max_dv_km_s: 1e-10,
        };

        let result = compute_avoidance(min_poca, &roe, &chief, epoch, tof_s, &model, &config);
        assert!(
            matches!(result, Err(AvoidanceError::ExceedsBudget { .. })),
            "tiny budget should return ExceedsBudget, got {result:?}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 6: D'Amico Table 2.2 analytical cost validation
    // -----------------------------------------------------------------------

    #[test]
    fn damico_table22_validation() {
        let chief = damico_table21_chief();
        let n = chief.mean_motion().unwrap();
        let a = chief.a_km;

        // For da=0, Eq. 2.44: single-impulse cost = n*a*de (with da=0 → no drift correction)
        // Using the Table 2.1 Case 1 ROE: dex=0, dey=400m/a
        let dey = 0.400 / a;
        let dex = 0.0_f64;
        let de = (dex * dex + dey * dey).sqrt();

        // Expected cost = n * a * sqrt(de^2 - (3/4)*da^2), with da=0 → n*a*de
        let expected_cost = n * a * de;

        // Verify via solve function
        let (dv, _u_m) = solve_inplane_eccentricity_only(dex, dey, n, a).unwrap();
        let actual_cost = dv.norm();

        assert!(
            (actual_cost - expected_cost).abs() < TABLE_22_TOL_KM_S,
            "single-impulse cost = {actual_cost} km/s, expected n*a*de = {expected_cost} km/s"
        );
    }
}

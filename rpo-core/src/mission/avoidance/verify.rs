//! Post-avoidance verification via re-propagation.
//!
//! After an avoidance maneuver is designed analytically, this module
//! verifies the actual post-maneuver POCA by propagating the corrected
//! trajectory and finding the minimum distance. This closes the loop on
//! linearization error in the distance-ratio scaling.

use hifitime::{Duration, Epoch};
use nalgebra::Vector3;

use crate::constants::TWO_PI;
use crate::elements::gve;
use crate::mission::closest_approach::find_closest_approaches;
use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::types::{AvoidanceError, BURN_TIME_CLAMP_FRACTION, COLA_VERIFICATION_STEPS};

/// Verify post-avoidance POCA distance by re-propagation.
///
/// Propagates to the burn location, applies the maneuver via GVE
/// (D'Amico Eq. 2.38), then propagates the remainder of the leg and
/// finds the minimum distance via Brent refinement. Falls back to
/// grid-based minimum if Brent fails.
///
/// # Arguments
///
/// * `roe` — Deputy ROE state at leg departure (post-departure-burn).
/// * `chief_mean` — Chief mean Keplerian elements at leg departure.
/// * `departure_epoch` — Epoch of leg departure.
/// * `tof_s` — Time of flight for the leg (seconds).
/// * `dv_ric` — Avoidance delta-v in RIC frame (km/s).
/// * `u_m` — Mean argument of latitude at the burn location (rad).
/// * `model` — Propagation model for trajectory generation.
///
/// # Invariants
///
/// - `tof_s > 0`
/// - `chief_mean.a_km > 0` (required for mean motion computation)
/// - Burn location `u_m` is clamped to `[0, tof_s * BURN_TIME_CLAMP_FRACTION]`
///   if the computed delta-t exceeds the leg duration.
///
/// # Errors
///
/// Returns [`AvoidanceError`] if propagation or POCA computation fails.
pub(super) fn verify_avoidance(
    roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    departure_epoch: Epoch,
    tof_s: f64,
    dv_ric: &Vector3<f64>,
    u_m: f64,
    model: &PropagationModel,
) -> Result<f64, AvoidanceError> {
    let n = chief_mean.mean_motion().map_err(|e| {
        AvoidanceError::PropagationFailure(PropagationError::KeplerFailure(e))
    })?;
    let u_0 = chief_mean.mean_arg_of_lat();

    // Time to reach burn location: Δt = (u_m − u₀) mod 2π / n
    // where u₀ = ω + M is the chief mean argument of latitude at departure
    let mut delta_t = ((u_m - u_0).rem_euclid(TWO_PI)) / n;

    // Clamp if burn location wraps past leg boundary
    if delta_t >= tof_s {
        delta_t = tof_s * BURN_TIME_CLAMP_FRACTION;
    }

    // Propagate ROE from departure to burn time using the selected model
    let state_at_burn = model.propagate(roe, chief_mean, departure_epoch, delta_t)?;

    // Advance chief mean anomaly: M(t_burn) = M₀ + n·Δt (mod 2π)
    let mut chief_at_burn = *chief_mean;
    chief_at_burn.mean_anomaly_rad = (chief_mean.mean_anomaly_rad + n * delta_t).rem_euclid(TWO_PI);

    // Apply maneuver via GVE (Eq. 2.38): δα_post = δα_pre + B(u_m) · Δv
    let post_burn_roe = gve::apply_maneuver(&state_at_burn.roe, dv_ric, &chief_at_burn)?;

    // Propagate post-burn
    let remaining_tof = tof_s - delta_t;
    let burn_epoch = departure_epoch + Duration::from_seconds(delta_t);

    if remaining_tof <= 0.0 {
        // Burn at end of leg — return distance at burn point
        let dist = state_at_burn.ric.position_ric_km.norm();
        return Ok(dist);
    }

    let post_traj = model.propagate_with_steps(
        &post_burn_roe,
        &chief_at_burn,
        burn_epoch,
        remaining_tof,
        COLA_VERIFICATION_STEPS,
    )?;

    // Try refined POCA
    match find_closest_approaches(&post_traj, &chief_at_burn, burn_epoch, model, &post_burn_roe, 0)
    {
        Ok(pocas) if !pocas.is_empty() => {
            let min_dist = pocas
                .iter()
                .map(|p| p.distance_km)
                .fold(f64::INFINITY, f64::min);
            Ok(min_dist)
        }
        _ => {
            // Fallback: grid minimum from trajectory
            let min_dist = min_grid_distance(&post_traj);
            Ok(min_dist)
        }
    }
}

/// Find the minimum RIC position norm across a trajectory (grid-based fallback).
///
/// Used when Brent POCA refinement fails or returns no candidates.
/// Returns `f64::INFINITY` for an empty trajectory.
fn min_grid_distance(trajectory: &[PropagatedState]) -> f64 {
    trajectory
        .iter()
        .map(|s| s.ric.position_ric_km.norm())
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    // Tests intentionally mirror D'Amico paper symbols (δe/δi components)
    // for equation traceability; renaming to satisfy `similar_names` would
    // break the paper-to-code mapping required by CLAUDE.md Published-method
    // fidelity and Traceability rules.
    #![allow(clippy::similar_names)]

    use super::*;
    use crate::elements::gve;
    use crate::mission::avoidance::solve::solve_inplane_eccentricity_only;
    use crate::mission::closest_approach::ClosestApproach;
    use crate::test_helpers::{damico_table21_chief, damico_table21_case1_roe, test_epoch};
    use crate::types::QuasiNonsingularROE;

    /// GVE roundtrip tolerance: apply maneuver, check ΔROE matches expected.
    /// Two matrix multiplications accumulate ~O(1e-15) error; 1e-12 is conservative.
    const ROUNDTRIP_TOL: f64 = 1e-12;

    #[test]
    fn gve_roundtrip_avoidance_dv() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let n = chief.mean_motion().unwrap();
        let a = chief.a_km;

        // Compute in-plane correction
        let scale = 2.0; // double the separation
        let delta_dex = roe.dex * (scale - 1.0);
        let delta_dey = roe.dey * (scale - 1.0);

        let (dv, u_m) = solve_inplane_eccentricity_only(delta_dex, delta_dey, n, a).unwrap();

        // Build chief at burn location
        let mut chief_at_burn = chief;
        chief_at_burn.aop_rad = 0.0;
        chief_at_burn.mean_anomaly_rad = u_m;

        // Apply maneuver via GVE
        let post_roe = gve::apply_maneuver(&roe, &dv, &chief_at_burn).unwrap();
        let actual_delta = post_roe.to_vector() - roe.to_vector();

        // The eccentricity components should match expected changes
        // Note: radial impulse also changes dlambda (B[1,0] = -2/(na))
        // so we only check dex, dey here
        assert!(
            (actual_delta[2] - delta_dex).abs() < ROUNDTRIP_TOL,
            "dex change: actual = {}, expected = {delta_dex}",
            actual_delta[2]
        );
        assert!(
            (actual_delta[3] - delta_dey).abs() < ROUNDTRIP_TOL,
            "dey change: actual = {}, expected = {delta_dey}",
            actual_delta[3]
        );
    }

    #[test]
    fn degenerate_geometry_returns_error() {
        let chief = damico_table21_chief();
        let roe = QuasiNonsingularROE::default(); // all zeros
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;
        let tof_s = chief.period().unwrap();

        // Fabricate a fake close POCA
        let poca = ClosestApproach {
            epoch,
            elapsed_s: 100.0,
            distance_km: 0.05,
            position_ric_km: Vector3::new(0.03, 0.02, 0.01),
            velocity_ric_km_s: Vector3::new(0.0, 0.0, 0.0),
            leg_index: 0,
            is_global_minimum: true,
        };

        let config = super::super::types::ColaConfig {
            target_distance_km: 0.5,
            max_dv_km_s: 1.0,
        };

        let result = super::super::solve::compute_avoidance(
            &poca, &roe, &chief, epoch, tof_s, &model, &config,
        );
        assert!(
            matches!(result, Err(AvoidanceError::DegenerateGeometry { .. })),
            "zero ROE should return DegenerateGeometry, got {result:?}"
        );
    }

    #[test]
    fn newton_nonconvergence_error_path() {
        let chief = damico_table21_chief();
        let a = chief.a_km;
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.200 / a,
            dey: 0.200 / a,
            dix: 0.200 / a,
            diy: 0.200 / a,
        };
        let epoch = test_epoch();
        let model = PropagationModel::J2Stm;
        let tof_s = chief.period().unwrap();

        // Fabricate POCA requiring large correction
        let poca = ClosestApproach {
            epoch,
            elapsed_s: 100.0,
            distance_km: 0.001,
            position_ric_km: Vector3::new(0.0005, 0.0002, 0.0005),
            velocity_ric_km_s: Vector3::zeros(),
            leg_index: 0,
            is_global_minimum: true,
        };

        // Set budget that is just barely insufficient for the huge target
        let config = super::super::types::ColaConfig {
            target_distance_km: 100.0,
            max_dv_km_s: 1e-6,
        };

        let result = super::super::solve::compute_avoidance(
            &poca, &roe, &chief, epoch, tof_s, &model, &config,
        );
        assert!(
            matches!(
                result,
                Err(AvoidanceError::ExceedsBudget { .. } | AvoidanceError::NoConvergence { ..
})
            ),
            "should fail with ExceedsBudget or NoConvergence, got {result:?}"
        );
    }
}

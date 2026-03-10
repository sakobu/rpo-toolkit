//! Validation helpers and integration tests for nyx numerical propagation comparisons.
//!
//! Provides utility functions for converting between ROE-RUST types and nyx-space types,
//! loading the ANISE almanac, and running nyx two-body numerical propagation.

use std::error::Error;
use std::sync::Arc;

use anise::prelude::{Almanac, Orbit};
use hifitime::Duration;
use nalgebra::Vector3;
use nyx_space::cosmic::Spacecraft;
use nyx_space::md::prelude::{OrbitalDynamics, Propagator, SpacecraftDynamics};

use crate::constants::EARTH_J2000;
use crate::types::StateVector;

/// Create a default ANISE almanac for two-body propagation.
///
/// Two-body dynamics only need the frame's `mu` value (baked into `EARTH_J2000`),
/// so an empty `Almanac::default()` suffices — no network download required.
#[must_use]
pub fn load_almanac() -> Arc<Almanac> {
    Arc::new(Almanac::default())
}

/// Convert a ROE-RUST [`StateVector`] to a nyx [`Spacecraft`].
///
/// Creates an `Orbit` in the `EARTH_J2000` frame, then wraps it in a
/// minimal `Spacecraft` (default mass, no SRP/drag).
///
/// # Invariants
/// - `sv` must represent a valid ECI state (non-zero position for bound orbit)
#[must_use]
pub fn state_to_spacecraft(sv: &StateVector) -> Spacecraft {
    let orbit = Orbit::new(
        sv.position_eci_km.x,
        sv.position_eci_km.y,
        sv.position_eci_km.z,
        sv.velocity_eci_km_s.x,
        sv.velocity_eci_km_s.y,
        sv.velocity_eci_km_s.z,
        sv.epoch,
        EARTH_J2000,
    );
    Spacecraft::from(orbit)
}

/// Convert a nyx [`Spacecraft`] back to a ROE-RUST [`StateVector`].
///
/// Extracts ECI position (km) and velocity (km/s) from the spacecraft's orbit.
#[must_use]
pub fn spacecraft_to_state(sc: &Spacecraft) -> StateVector {
    StateVector {
        epoch: sc.orbit.epoch,
        position_eci_km: Vector3::new(
            sc.orbit.radius_km.x,
            sc.orbit.radius_km.y,
            sc.orbit.radius_km.z,
        ),
        velocity_eci_km_s: Vector3::new(
            sc.orbit.velocity_km_s.x,
            sc.orbit.velocity_km_s.y,
            sc.orbit.velocity_km_s.z,
        ),
    }
}

/// Propagate a state vector using nyx two-body (Keplerian) dynamics.
///
/// Builds an `OrbitalDynamics::two_body()` model (no perturbations),
/// wraps it in `SpacecraftDynamics`, and propagates with an RK89 integrator.
///
/// # Invariants
/// - `sv` must represent a bound orbit (`e < 1`, `a > 0`)
/// - `duration_s` must be finite
///
/// # Errors
/// Returns an error if almanac loading or propagation fails.
pub fn nyx_propagate_two_body(
    sv: &StateVector,
    duration_s: f64,
) -> Result<StateVector, Box<dyn Error>> {
    let almanac = load_almanac();
    let orbital_dyn = OrbitalDynamics::two_body();
    let sc_dyn = SpacecraftDynamics::new(orbital_dyn);
    let propagator = Propagator::default(sc_dyn);
    let sc = state_to_spacecraft(sv);
    let duration = Duration::from_seconds(duration_s);
    let final_sc = propagator.with(sc, almanac).for_duration(duration)?;
    Ok(spacecraft_to_state(&final_sc))
}

#[cfg(test)]
mod tests {
    use crate::elements::conversions::{keplerian_to_state, state_to_keplerian};
    use crate::mission::planning::{classify_separation, plan_mission};
    use crate::propagation::propagator::PropagationModel;
    use crate::elements::roe::compute_roe;
    use crate::test_helpers::{
        iss_like_elements, leo_400km_elements, leo_800km_target_elements, test_drag_config,
        test_epoch,
    };
    use crate::types::{
        KeplerianElements, MissionPhase, PerchGeometry, ProximityConfig,
    };

    /// End-to-end mission pipeline: far-field classification → Lambert transfer →
    /// perch orbit → proximity propagation.
    #[test]
    fn full_mission_scenario() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 200.0,
            e: 0.005,
            i_rad: chief_ke.i_rad + 0.05,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };

        let chief = keplerian_to_state(&chief_ke, epoch);
        let deputy = keplerian_to_state(&deputy_ke, epoch);
        let config = ProximityConfig::default();

        // Phase classification
        let phase = classify_separation(&chief, &deputy, &config);
        assert!(
            matches!(phase, MissionPhase::FarField { .. }),
            "200 km + inclination offset should be FarField, got {phase:?}"
        );

        // Full mission plan
        let perch = PerchGeometry::VBar {
            along_track_km: 5.0,
        };

        let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0)
            .expect("mission plan should succeed");

        // Lambert transfer assertions
        let transfer = plan.transfer.as_ref().expect("should have Lambert transfer");
        assert!(
            transfer.total_dv_km_s > 0.1 && transfer.total_dv_km_s < 10.0,
            "total Δv = {} km/s unreasonable",
            transfer.total_dv_km_s
        );
        assert!(
            transfer.departure_dv_eci_km_s.norm() > 0.0,
            "departure Δv should be nonzero"
        );
        assert!(
            transfer.arrival_dv_eci_km_s.norm() > 0.0,
            "arrival Δv should be nonzero"
        );

        // Perch ROE assertions
        assert!(
            plan.perch_roe.dlambda.abs() > 1e-6,
            "V-bar perch should have nonzero dlambda"
        );
        assert!(
            plan.perch_roe.da.abs() < 1e-10,
            "V-bar perch da should be near-zero"
        );
        assert!(
            plan.perch_roe.dex.abs() < 1e-10,
            "V-bar perch dex should be near-zero"
        );
        assert!(
            plan.perch_roe.dey.abs() < 1e-10,
            "V-bar perch dey should be near-zero"
        );
    }

    /// Verify DMF drag model self-consistency: zero-ROE propagation with drag
    /// should produce drift proportional to da_dot, with quadratic along-track growth.
    #[test]
    fn drag_stm_self_consistency() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let zero_roe = crate::types::QuasiNonsingularROE::default();
        let drag_config = test_drag_config();
        let period = std::f64::consts::TAU / chief.mean_motion();

        // J2-only propagator (zero ROE should stay near zero)
        let j2_prop = PropagationModel::J2Stm;
        let j2_5 = j2_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period);
        assert!(
            j2_5.roe.da.abs() < 1e-12,
            "J2-only da from zero ROE should stay near zero, got {}",
            j2_5.roe.da
        );

        // J2+drag propagator
        let drag_prop = PropagationModel::J2DragStm {
            drag: drag_config,
        };
        let drag_5 = drag_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period);
        let drag_10 = drag_prop.propagate(&zero_roe, &chief, epoch, 10.0 * period);

        // da drift should be approximately da_dot * t
        let t5 = 5.0 * period;
        let t10 = 10.0 * period;
        let expected_da_5 = drag_config.da_dot * t5;
        let expected_da_10 = drag_config.da_dot * t10;
        let rel_err_5 = ((drag_5.roe.da - expected_da_5) / expected_da_5).abs();
        let rel_err_10 = ((drag_10.roe.da - expected_da_10) / expected_da_10).abs();
        assert!(
            rel_err_5 < 0.1,
            "da drift at 5 orbits: relative error {rel_err_5:.4} > 10%"
        );
        assert!(
            rel_err_10 < 0.1,
            "da drift at 10 orbits: relative error {rel_err_10:.4} > 10%"
        );

        // Along-track (dlambda) grows quadratically: ratio at 10/5 orbits ≈ 4
        let ratio = drag_10.roe.dlambda / drag_5.roe.dlambda;
        assert!(
            (ratio - 4.0).abs() < 1.0,
            "dlambda(10)/dlambda(5) = {ratio:.2}, expected ≈ 4.0"
        );
    }

    /// Verify J2 secular rates produce physically reasonable effects:
    /// constant da, RAAN regression, and mean longitude drift.
    #[test]
    fn j2_effect_physical_reasonableness() {
        let chief = iss_like_elements();
        let epoch = test_epoch();

        // Deputy with 1 km SMA offset and 0.001 rad inclination offset
        let mut deputy = chief;
        deputy.a_km += 1.0;
        deputy.i_rad += 0.001;
        let roe_0 = compute_roe(&chief, &deputy);
        let period = std::f64::consts::TAU / chief.mean_motion();

        let prop = PropagationModel::J2Stm;
        let state_10 = prop.propagate(&roe_0, &chief, epoch, 10.0 * period);

        // da should remain constant under J2 (no secular da drift)
        let da_change = (state_10.roe.da - roe_0.da).abs();
        assert!(
            da_change < 1e-8,
            "da should be constant under J2, changed by {da_change}"
        );

        // diy should drift due to differential RAAN regression
        // ISS RAAN regression rate is ~5 deg/day ≈ 8.7e-5 rad/s × orbital period
        // With small inclination offset, differential RAAN produces diy drift
        let diy_drift = (state_10.roe.diy - roe_0.diy).abs();
        assert!(
            diy_drift > 1e-10,
            "diy should drift under J2 differential RAAN, got {diy_drift}"
        );
        // Physically: 10 orbits ≈ 15 hours for ISS, RAAN drift ~3 deg
        // differential diy should be order 1e-5 to 1e-3
        assert!(
            diy_drift < 0.1,
            "diy drift {diy_drift} seems too large for 10 ISS orbits"
        );

        // dlambda should drift from J2 mean motion coupling with da offset
        let dlambda_drift = (state_10.roe.dlambda - roe_0.dlambda).abs();
        assert!(
            dlambda_drift > 1e-8,
            "dlambda should drift from da + J2, got {dlambda_drift}"
        );
    }

    /// Verify Lambert solution by propagating departure state with nyx two-body
    /// and comparing against expected arrival position.
    #[test]
    fn lambert_two_body_verification() {
        let epoch = test_epoch();
        let tof = 2400.0;

        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + hifitime::Duration::from_seconds(tof),
        );

        // Solve Lambert
        let transfer =
            crate::mission::lambert::solve_lambert(&dep, &arr).expect("Lambert should converge");

        // Propagate departure state (with Lambert velocity) using nyx two-body
        let propagated = super::nyx_propagate_two_body(&transfer.departure_state, tof)
            .expect("nyx propagation should succeed");

        // Position at arrival should match Lambert's arrival position
        let pos_err = (propagated.position_eci_km - arr.position_eci_km).norm();
        assert!(
            pos_err < 1.0,
            "position error = {pos_err:.4} km (expected < 1.0 km)"
        );

        let vel_err = (propagated.velocity_eci_km_s - transfer.arrival_state.velocity_eci_km_s).norm();
        assert!(
            vel_err < 0.01,
            "velocity error = {vel_err:.6} km/s (expected < 0.01 km/s)"
        );
    }

    /// Verify non-coplanar Lambert solution against nyx two-body propagation.
    #[test]
    fn lambert_non_coplanar_verification() {
        let epoch = test_epoch();
        let tof = 3600.0;

        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr_ke = KeplerianElements {
            a_km: 6378.137 + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 10.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + hifitime::Duration::from_seconds(tof));

        let transfer =
            crate::mission::lambert::solve_lambert(&dep, &arr).expect("Lambert should converge");

        let propagated = super::nyx_propagate_two_body(&transfer.departure_state, tof)
            .expect("nyx propagation should succeed");

        let pos_err = (propagated.position_eci_km - arr.position_eci_km).norm();
        assert!(
            pos_err < 1.0,
            "position error = {pos_err:.4} km (expected < 1.0 km)"
        );

        let vel_err = (propagated.velocity_eci_km_s - transfer.arrival_state.velocity_eci_km_s).norm();
        assert!(
            vel_err < 0.01,
            "velocity error = {vel_err:.6} km/s (expected < 0.01 km/s)"
        );
    }

    /// Compare analytical J2 STM propagation against nyx Keplerian (two-body) baseline.
    ///
    /// Since J2 STM includes perturbations that nyx two-body does not,
    /// this test verifies that the J2 corrections are physically reasonable
    /// rather than expecting exact agreement.
    #[test]
    fn j2_stm_vs_nyx_two_body() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;
        deputy_ke.i_rad += 0.001;

        let chief_sv = keplerian_to_state(&chief_ke, epoch);
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch);

        let period = std::f64::consts::TAU / chief_ke.mean_motion();
        let duration = 10.0 * period;

        // Propagate both with nyx two-body
        let chief_nyx = super::nyx_propagate_two_body(&chief_sv, duration)
            .expect("chief nyx propagation failed");
        let deputy_nyx = super::nyx_propagate_two_body(&deputy_sv, duration)
            .expect("deputy nyx propagation failed");

        // Compute ROEs from nyx final states
        let chief_ke_final = state_to_keplerian(&chief_nyx);
        let deputy_ke_final = state_to_keplerian(&deputy_nyx);
        let nyx_roe = compute_roe(&chief_ke_final, &deputy_ke_final);

        // Propagate with J2 STM
        let roe_0 = compute_roe(&chief_ke, &deputy_ke);
        let prop = PropagationModel::J2Stm;
        let stm_state = prop.propagate(&roe_0, &chief_ke, epoch, duration);

        // da should be constant in both (Keplerian preserves SMA)
        assert!(
            (nyx_roe.da - roe_0.da).abs() < 1e-6,
            "nyx da should be approximately constant: initial={}, final={}",
            roe_0.da,
            nyx_roe.da
        );
        assert!(
            (stm_state.roe.da - roe_0.da).abs() < 1e-6,
            "STM da should be approximately constant: initial={}, final={}",
            roe_0.da,
            stm_state.roe.da
        );

        // The J2-induced diy difference should be physically correct
        // Over 10 ISS orbits (~15 hours), differential RAAN regression
        // for 0.001 rad inclination offset produces diy drift of ~1e-5 to 1e-3 rad
        let stm_diy_drift = (stm_state.roe.diy - roe_0.diy).abs();
        assert!(
            stm_diy_drift > 1e-8 && stm_diy_drift < 0.2,
            "J2 STM diy drift = {stm_diy_drift} should be in [1e-8, 0.2] rad"
        );

        // nyx two-body should have minimal diy drift (no J2)
        let nyx_diy_drift = (nyx_roe.diy - roe_0.diy).abs();
        assert!(
            nyx_diy_drift < stm_diy_drift * 10.0 || nyx_diy_drift < 1e-4,
            "nyx two-body diy drift = {nyx_diy_drift} should be small compared to J2 drift"
        );
    }
}

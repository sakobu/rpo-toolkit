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

    // =========================================================================
    // RK4 J2 Numerical Integrator (test-only, independent truth source)
    // =========================================================================

    use nalgebra::Vector3;
    use crate::constants::{J2, MU_EARTH, R_EARTH};
    use crate::types::StateVector;

    /// ECI acceleration: two-body + J2 zonal harmonic.
    #[allow(clippy::doc_markdown)]
    fn j2_acceleration(pos: &Vector3<f64>) -> Vector3<f64> {
        let r2 = pos.dot(pos);
        let r = r2.sqrt();
        let r3 = r * r2;
        let r5 = r3 * r2;
        let z2 = pos.z * pos.z;
        let z2_over_r2 = z2 / r2;

        // Two-body
        let a_2body = -MU_EARTH / r3 * pos;

        // J2 perturbation
        let j2_coeff = -1.5 * J2 * MU_EARTH * R_EARTH * R_EARTH / r5;
        let a_j2 = Vector3::new(
            j2_coeff * pos.x * (1.0 - 5.0 * z2_over_r2),
            j2_coeff * pos.y * (1.0 - 5.0 * z2_over_r2),
            j2_coeff * pos.z * (3.0 - 5.0 * z2_over_r2),
        );

        a_2body + a_j2
    }

    /// Propagate an ECI state forward using RK4 with J2 gravity.
    ///
    /// Fixed time step `dt` (seconds). State = [pos; vel] in ECI.
    /// This is a test-only utility providing an independent numerical truth source
    /// with no shared code paths with the analytical STM.
    fn rk4_j2_propagate(sv: &StateVector, duration_s: f64, dt: f64) -> StateVector {
        debug_assert!(duration_s >= 0.0 && dt > 0.0, "duration and dt must be positive");
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_steps = (duration_s / dt).ceil() as u32;
        let actual_dt = duration_s / f64::from(n_steps);

        let mut pos = sv.position_eci_km;
        let mut vel = sv.velocity_eci_km_s;

        for _ in 0..n_steps {
            // RK4 for d(pos)/dt = vel, d(vel)/dt = accel(pos)
            let k1v = j2_acceleration(&pos);
            let k1r = vel;

            let pos2 = pos + 0.5 * actual_dt * k1r;
            let vel2 = vel + 0.5 * actual_dt * k1v;
            let k2v = j2_acceleration(&pos2);
            let k2r = vel2;

            let pos3 = pos + 0.5 * actual_dt * k2r;
            let vel3 = vel + 0.5 * actual_dt * k2v;
            let k3v = j2_acceleration(&pos3);
            let k3r = vel3;

            let pos4 = pos + actual_dt * k3r;
            let vel4 = vel + actual_dt * k3v;
            let k4v = j2_acceleration(&pos4);
            let k4r = vel4;

            pos += actual_dt / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
            vel += actual_dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        }

        StateVector {
            epoch: sv.epoch + hifitime::Duration::from_seconds(duration_s),
            position_eci_km: pos,
            velocity_eci_km_s: vel,
        }
    }

    /// Self-validate the RK4 J2 integrator: specific orbital energy should be
    /// approximately conserved (slowly varying due to J2's non-conservative-like effects
    /// on osculating elements, but not blowing up).
    #[test]
    fn rk4_j2_integrator_self_validation() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

        let period = chief_ke.period();
        let sv_1orbit = rk4_j2_propagate(&sv, period, 10.0);

        // Specific energy: E = v²/2 - μ/r
        let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv.position_eci_km.norm();
        let energy_1 = sv_1orbit.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv_1orbit.position_eci_km.norm();

        // J2 causes osculating energy to oscillate (short-period terms) but not drift secularly.
        // Over 1 orbit, relative change should be < 1e-5 (J2 oscillation amplitude).
        let rel_energy_change = ((energy_1 - energy_0) / energy_0).abs();
        assert!(
            rel_energy_change < 1e-5,
            "Energy change over 1 orbit = {rel_energy_change:.2e} (should be < 1e-5)"
        );

        // Position magnitude should remain roughly the same (bound orbit)
        let r0 = sv.position_eci_km.norm();
        let r1 = sv_1orbit.position_eci_km.norm();
        assert!(
            (r1 - r0).abs() / r0 < 0.01,
            "Radius changed by {:.4}% over 1 orbit — integrator may be unstable",
            (r1 - r0).abs() / r0 * 100.0
        );
    }

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

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        // Phase classification
        let phase = classify_separation(&chief, &deputy, &config).unwrap();
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
    /// should produce drift proportional to `da_dot`, with quadratic along-track growth.
    #[test]
    fn drag_stm_self_consistency() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let zero_roe = crate::types::QuasiNonsingularROE::default();
        let drag_config = test_drag_config();
        let period = std::f64::consts::TAU / chief.mean_motion();

        // J2-only propagator (zero ROE should stay near zero)
        let j2_prop = PropagationModel::J2Stm;
        let j2_5 = j2_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period).unwrap();
        assert!(
            j2_5.roe.da.abs() < 1e-12,
            "J2-only da from zero ROE should stay near zero, got {}",
            j2_5.roe.da
        );

        // J2+drag propagator
        let drag_prop = PropagationModel::J2DragStm {
            drag: drag_config,
        };
        let drag_5 = drag_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period).unwrap();
        let drag_10 = drag_prop.propagate(&zero_roe, &chief, epoch, 10.0 * period).unwrap();

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
        let roe_0 = compute_roe(&chief, &deputy).unwrap();
        let period = std::f64::consts::TAU / chief.mean_motion();

        let prop = PropagationModel::J2Stm;
        let state_10 = prop.propagate(&roe_0, &chief, epoch, 10.0 * period).unwrap();

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

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + hifitime::Duration::from_seconds(tof),
        ).unwrap();

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

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr_ke = KeplerianElements {
            a_km: 6378.137 + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 10.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + hifitime::Duration::from_seconds(tof)).unwrap();

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

    // =========================================================================
    // Phase 2.1–2.3: J2 STM vs. RK4 J2 Numerical Integrator Comparisons
    // =========================================================================

    /// Compare STM-propagated ROE against RK4 J2 numerical integration.
    ///
    /// Returns physical errors `a * |ROE_stm - ROE_numerical|` in meters per element.
    fn compare_stm_vs_rk4(
        chief: &KeplerianElements,
        roe: &crate::types::QuasiNonsingularROE,
        n_orbits: f64,
    ) -> [f64; 6] {
        use crate::test_helpers::deputy_from_roe;
        use crate::propagation::stm::propagate_roe_stm;

        let epoch = test_epoch();
        let deputy = deputy_from_roe(chief, roe);
        let period = chief.period();
        let duration = n_orbits * period;

        // Convert to ECI
        let chief_sv = keplerian_to_state(chief, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy, epoch).unwrap();

        // RK4 J2 numerical propagation (dt=10s for all cases)
        let chief_final = rk4_j2_propagate(&chief_sv, duration, 10.0);
        let deputy_final = rk4_j2_propagate(&deputy_sv, duration, 10.0);

        // Convert back to Keplerian and compute numerical ROE
        let chief_ke_final = state_to_keplerian(&chief_final).unwrap();
        let deputy_ke_final = state_to_keplerian(&deputy_final).unwrap();
        let numerical_roe = compute_roe(&chief_ke_final, &deputy_ke_final).unwrap();

        // STM propagation
        let (stm_roe, _) = propagate_roe_stm(roe, chief, duration).unwrap();

        // Compute errors in meters: a * |ROE_stm - ROE_numerical|
        let a = chief.a_km * 1000.0; // convert to meters
        [
            a * (stm_roe.da - numerical_roe.da).abs(),
            a * (crate::elements::roe::wrap_angle(stm_roe.dlambda - numerical_roe.dlambda)).abs(),
            a * (stm_roe.dex - numerical_roe.dex).abs(),
            a * (stm_roe.dey - numerical_roe.dey).abs(),
            a * (stm_roe.dix - numerical_roe.dix).abs(),
            a * (stm_roe.diy - numerical_roe.diy).abs(),
        ]
    }

    /// Koenig Table 4 Case 1: J2 STM accuracy over 10 orbits vs. RK4 J2 numerical integrator.
    ///
    /// Chief: a=6812 km, e=0.005, i=30°. ROE: 200m physical separation in each component.
    /// Koenig Table 4 reports QNS J2 STM errors vs. a 20×20 gravity model:
    ///   aδa=38.5m, aδλ=1808.8m, aδex'=13.5m, aδey'=11.3m, aδix=0.9m, aδiy=2.5m.
    /// Our comparison is J2 STM vs. J2-only RK4, so errors should be dominated by
    /// linearization rather than unmodeled harmonics, yielding much smaller values.
    /// We use 10× Table 4 as generous bounds.
    #[test]
    fn koenig_table4_j2_stm_accuracy_case1() {
        use crate::test_helpers::{koenig_table2_case1, koenig_table2_case1_roe};

        let chief = koenig_table2_case1();
        let roe = koenig_table2_case1_roe();
        let errors = compare_stm_vs_rk4(&chief, &roe, 10.0);

        // Bounds: 10× Koenig Table 4 QNS J2 values (Harris-Priester)
        assert!(errors[0] < 385.0,   "aδa  = {:.1}m (bound 385m)", errors[0]);
        assert!(errors[1] < 18088.0, "aδλ  = {:.1}m (bound 18088m)", errors[1]);
        assert!(errors[2] < 135.0,   "aδex = {:.1}m (bound 135m)", errors[2]);
        assert!(errors[3] < 113.0,   "aδey = {:.1}m (bound 113m)", errors[3]);
        assert!(errors[4] < 9.0,     "aδix = {:.1}m (bound 9m)", errors[4]);
        assert!(errors[5] < 25.0,    "aδiy = {:.1}m (bound 25m)", errors[5]);

        // Print actual errors for diagnostic purposes
        eprintln!("Case 1 STM vs RK4 J2 errors (m): da={:.2}, dλ={:.2}, dex={:.2}, dey={:.2}, dix={:.2}, diy={:.2}",
            errors[0], errors[1], errors[2], errors[3], errors[4], errors[5]);
    }

    /// Koenig Table 2 Cases 2-3: verify RK4 J2 integrator correctly propagates
    /// eccentric orbits (energy conservation, orbit stability).
    ///
    /// Direct ROE comparison between the mean-element STM and osculating-element
    /// numerical integration is not meaningful for eccentric orbits (e=0.2, 0.5)
    /// without osculating-to-mean averaging (Koenig Fig. 4). Instead, we verify:
    /// 1. The RK4 integrator maintains energy conservation for eccentric orbits
    /// 2. The orbit remains bound after 10 orbits
    /// 3. The STM preserves δa and eccentricity vector magnitude (structural)
    #[test]
    fn rk4_j2_eccentric_orbit_stability() {
        use crate::test_helpers::{koenig_table2_case2, koenig_table2_case3};

        for (label, chief) in &[
            ("Case 2 (e=0.2)", koenig_table2_case2()),
            ("Case 3 (e=0.5)", koenig_table2_case3()),
        ] {
            let epoch = test_epoch();
            let sv = keplerian_to_state(chief, epoch).unwrap();
            let duration = 10.0 * chief.period();

            let sv_final = rk4_j2_propagate(&sv, duration, 10.0);

            // Energy should be approximately conserved
            let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
                - MU_EARTH / sv.position_eci_km.norm();
            let energy_f = sv_final.velocity_eci_km_s.norm_squared() / 2.0
                - MU_EARTH / sv_final.position_eci_km.norm();
            let rel_energy = ((energy_f - energy_0) / energy_0).abs();
            assert!(
                rel_energy < 1e-5,
                "{label}: energy change = {rel_energy:.2e} over 10 orbits (should be < 1e-5)"
            );

            // Orbit should remain bound (can convert back to Keplerian)
            let ke_final = state_to_keplerian(&sv_final).unwrap();
            assert!(
                ke_final.e < 1.0 && ke_final.a_km > 0.0,
                "{label}: orbit became unbound (a={}, e={})", ke_final.a_km, ke_final.e
            );

            // SMA should be approximately conserved (J2 doesn't cause secular SMA drift)
            let rel_sma = ((ke_final.a_km - chief.a_km) / chief.a_km).abs();
            assert!(
                rel_sma < 0.001,
                "{label}: SMA changed by {:.4}% over 10 orbits", rel_sma * 100.0
            );
        }
    }

    /// Koenig Table 2 Cases 2-3: verify STM structural properties for eccentric orbits.
    ///
    /// The STM preserves δa exactly and rotates the eccentricity vector by ω̇τ.
    /// Over 10 orbits, verify:
    /// 1. δa is exactly conserved (row 0 of STM)
    /// 2. δix is exactly conserved (row 4 of STM)
    /// 3. Eccentricity vector magnitude |δe| is approximately conserved (rotation)
    /// 4. Propagation completes without error
    #[test]
    fn stm_structural_properties_eccentric() {
        use crate::test_helpers::{
            koenig_table2_case2, koenig_table2_case2_roe,
            koenig_table2_case3, koenig_table2_case3_roe,
        };
        use crate::propagation::stm::propagate_roe_stm;

        let cases: [(&str, KeplerianElements, crate::types::QuasiNonsingularROE); 2] = [
            ("Case 2 (e=0.2)", koenig_table2_case2(), koenig_table2_case2_roe()),
            ("Case 3 (e=0.5)", koenig_table2_case3(), koenig_table2_case3_roe()),
        ];

        for (label, chief, roe) in &cases {
            let duration = 10.0 * chief.period();
            let (roe_prop, _) = propagate_roe_stm(roe, chief, duration).unwrap();

            // δa must be exactly conserved
            assert!(
                (roe_prop.da - roe.da).abs() < 1e-8,
                "{label}: δa not conserved: {} → {}", roe.da, roe_prop.da
            );

            // δix must be exactly conserved
            assert!(
                (roe_prop.dix - roe.dix).abs() < 1e-8,
                "{label}: δix not conserved: {} → {}", roe.dix, roe_prop.dix
            );

            // |δe| approximately conserved (rotation, not growth)
            let de_0 = (roe.dex.powi(2) + roe.dey.powi(2)).sqrt();
            let de_f = (roe_prop.dex.powi(2) + roe_prop.dey.powi(2)).sqrt();
            if de_0 > 1e-10 {
                let rel_de = (de_f - de_0).abs() / de_0;
                assert!(
                    rel_de < 0.05,
                    "{label}: |δe| changed by {:.2}% over 10 orbits", rel_de * 100.0
                );
            }
        }
    }

    /// D'Amico ROE→RIC trajectory accuracy: compare analytical ROE→RIC mapping
    /// against numerical ECI→RIC conversion over 1 orbit.
    ///
    /// Chief = D'Amico Table 2.1, ROE = Case 1. Propagate both chief and deputy
    /// numerically with RK4 J2 for 1 orbit, sampling every ~30s. At each sample,
    /// compare "true" RIC (from ECI difference via DCM) against predicted RIC
    /// (from STM-propagated ROE via `roe_to_ric`). RMS position error should be < 50m.
    #[test]
    fn damico_roe_to_ric_trajectory_accuracy() {
        use crate::elements::frames::eci_to_ric_relative;
        use crate::elements::ric::roe_to_ric;
        use crate::propagation::stm::propagate_roe_stm;
        use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief, deputy_from_roe};

        let epoch = test_epoch();
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let deputy = deputy_from_roe(&chief, &roe);

        let chief_sv = keplerian_to_state(&chief, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy, epoch).unwrap();

        let period = chief.period();
        let dt_sample = 30.0; // sample every 30 seconds
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_samples = (period / dt_sample).floor() as u32;

        let mut sum_sq_err = 0.0;
        let mut count = 0;

        for i in 0..=n_samples {
            let t = f64::from(i) * dt_sample;

            // "True" RIC from numerical ECI propagation
            let chief_t = rk4_j2_propagate(&chief_sv, t, 10.0);
            let deputy_t = rk4_j2_propagate(&deputy_sv, t, 10.0);
            let ric_true = eci_to_ric_relative(&chief_t, &deputy_t);

            // Predicted RIC from STM-propagated ROE
            let (roe_t, chief_mean_t) = propagate_roe_stm(&roe, &chief, t).unwrap();
            let ric_pred = roe_to_ric(&roe_t, &chief_mean_t).unwrap();

            let pos_err = (ric_true.position_ric_km - ric_pred.position_ric_km).norm();
            sum_sq_err += pos_err * pos_err;
            count += 1;
        }

        let rms_position_m = (sum_sq_err / f64::from(count)).sqrt() * 1000.0;

        // D'Amico Fig. 2.8 shows <3m for two-body. J2 adds secular drift over
        // 1 orbit plus modeling differences (mean vs osculating). 50m is generous
        // enough to catch bugs without false positives.
        assert!(
            rms_position_m < 50.0,
            "RMS RIC position error = {rms_position_m:.2}m (expected < 50m)"
        );

        eprintln!(
            "ROE→RIC trajectory RMS position error: {rms_position_m:.2}m over {count} samples"
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

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = std::f64::consts::TAU / chief_ke.mean_motion();
        let duration = 10.0 * period;

        // Propagate both with nyx two-body
        let chief_nyx = super::nyx_propagate_two_body(&chief_sv, duration)
            .expect("chief nyx propagation failed");
        let deputy_nyx = super::nyx_propagate_two_body(&deputy_sv, duration)
            .expect("deputy nyx propagation failed");

        // Compute ROEs from nyx final states
        let chief_ke_final = state_to_keplerian(&chief_nyx).unwrap();
        let deputy_ke_final = state_to_keplerian(&deputy_nyx).unwrap();
        let nyx_roe = compute_roe(&chief_ke_final, &deputy_ke_final).unwrap();

        // Propagate with J2 STM
        let roe_0 = compute_roe(&chief_ke, &deputy_ke).unwrap();
        let prop = PropagationModel::J2Stm;
        let stm_state = prop.propagate(&roe_0, &chief_ke, epoch, duration).unwrap();

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

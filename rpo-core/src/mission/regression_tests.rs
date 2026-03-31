//! Cross-cutting integration and regression tests.
//!
//! These tests validate the interaction between multiple modules (STM, Lambert,
//! nyx bridge, mission planning) and compare analytical propagation against
//! independent numerical truth sources (RK4 J2 integrator, nyx two-body/full-physics).
//! Regression data traces to Koenig Tables 2–4 and D'Amico Table 2.1.

use nyx_space::md::prelude::{OrbitalDynamics, SpacecraftDynamics};

use crate::constants::MU_EARTH;
use crate::elements::eci_ric_dcm::eci_to_ric_relative;
use crate::elements::keplerian_conversions::{keplerian_to_state, state_to_keplerian};
use crate::elements::roe::compute_roe;
use crate::elements::roe_to_ric::roe_to_ric;
use crate::mission::closest_approach::find_closest_approaches;
use crate::mission::config::ProximityConfig;
use crate::mission::planning::{classify_separation, plan_mission};
use crate::mission::safety::analyze_trajectory_safety;
use crate::mission::types::{MissionPhase, PerchGeometry};
use crate::propagation::lambert::LambertConfig;
use crate::propagation::nyx_bridge;
use crate::propagation::propagator::PropagationModel;
use crate::propagation::stm::propagate_roe_stm;
use crate::test_helpers::{
    damico_table21_case1_roe, damico_table21_chief, deputy_from_roe, iss_like_elements,
    koenig_table2_case1, koenig_table2_case1_roe, koenig_table2_case2, koenig_table2_case2_roe,
    koenig_table2_case3, koenig_table2_case3_roe, leo_400km_elements, leo_800km_target_elements,
    rk4_j2_propagate, test_drag_config, test_epoch, DMF_RATE_NONZERO_LOWER_BOUND,
    DMF_RATE_UPPER_BOUND,
};
use crate::types::{KeplerianElements, QuasiNonsingularROE, SpacecraftConfig, StateVector};

// =========================================================================
// Named tolerance constants
// =========================================================================
//
// Categories:
//   ENERGY_*     — orbital energy conservation bounds
//   ROE_*        — ROE component conservation / drift bounds
//   PERCH_*      — perch orbit ROE structural bounds
//   LAMBERT_*    — Lambert solver + nyx cross-validation position/velocity bounds
//   STM_*        — STM vs. numerical integrator accuracy bounds (Koenig Table 4)
//   DRAG_*       — drag STM self-consistency bounds
//   DMF_*        — DMF rate extraction bounds
//   ORBIT_*      — orbit stability bounds

/// Relative energy conservation tolerance over N orbits with J2 perturbation.
/// J2 causes osculating energy to oscillate via short-period terms but not drift
/// secularly; the amplitude of the short-period oscillation is O(J2) ~ 1e-6.
/// We use 1e-5 to allow margin above the oscillation floor.
const ENERGY_J2_RELATIVE_TOL: f64 = 1e-5;

/// Relative energy conservation tolerance for full-physics dynamics (drag + SRP).
/// Drag and SRP cause real secular energy dissipation, so this is looser than
/// the J2-only bound. One orbit of ISS-like drag dissipates O(1e-5) relative
/// energy; 1e-4 provides a 10x margin.
const ENERGY_FULL_PHYSICS_RELATIVE_TOL: f64 = 1e-4;

/// Radius relative change tolerance over 1 orbit.
/// A bound orbit should return to approximately the same radius after one period.
/// J2 short-period oscillations are O(1e-3) relative; 1% covers the largest
/// short-period radial oscillation for LEO eccentricities.
const ORBIT_RADIUS_RELATIVE_TOL: f64 = 0.01;

/// SMA relative conservation tolerance over 10 orbits.
/// J2 does not cause secular SMA drift; osculating SMA oscillates by O(J2*a)
/// which is < 0.1% for LEO. We use 0.001 (0.1%) as the bound.
const ORBIT_SMA_RELATIVE_TOL: f64 = 0.001;

/// ROE da/dix secular conservation tolerance.
/// The J2 STM preserves da and dix exactly (rows 0 and 4 of the STM are
/// identity for these components). Nonzero change comes only from f64
/// arithmetic in the STM evaluation; 1e-8 covers accumulated roundoff
/// over 10-orbit propagation of ~200 m physical separations.
const ROE_SECULAR_CONSERVATION_TOL: f64 = 1e-8;

/// Near-zero ROE threshold for structural zeros.
/// V-bar perch orbits have identically zero da, dex, dey by construction.
/// Residual nonzero values arise from floating-point representation of the
/// perch geometry; 1e-10 corresponds to < 0.001 mm physical error.
const PERCH_ROE_ZERO_BOUND: f64 = 1e-10;

/// Lower bound for perch ROE nonzero components.
/// A V-bar perch at 5 km along-track must have nonzero dlambda;
/// 1e-6 corresponds to ~7 mm physical offset — well below any meaningful perch.
const PERCH_ROE_NONZERO_LOWER_BOUND: f64 = 1e-6;

/// Cross-method ROE agreement tolerance.
/// When comparing J2 STM against nyx two-body (which lacks J2), da should
/// remain approximately constant in both. The ~1e-6 threshold accounts for
/// osculating-to-mean element differences in the two-body baseline and
/// numerical truncation across the nyx boundary.
const ROE_CROSS_METHOD_DA_TOL: f64 = 1e-6;

/// J2 da secular identity tolerance.
/// Under J2-only dynamics, da is preserved identically (the STM has Φ(0,0)=1
/// with no coupling). Starting from zero ROE, da should remain at machine
/// precision level. 1e-12 is ~10x f64 epsilon for the working range.
const ROE_DA_IDENTITY_TOL: f64 = 1e-12;

/// Lower bound for physically meaningful diy drift under J2.
/// Differential RAAN regression for a 0.001 rad inclination offset over 10
/// ISS orbits (~15 hours) produces diy drift of O(1e-5). We use 1e-10 as
/// a permissive floor to confirm the drift is nonzero.
const ROE_DIY_DRIFT_LOWER_BOUND: f64 = 1e-10;

/// Physical upper bound for diy drift over 10 ISS orbits (J2 STM vs nyx).
/// RAAN regression rate is ~5 deg/day; over 15 hours with 0.001 rad
/// inclination offset, differential diy should not exceed ~0.01 rad.
/// 0.2 rad is a generous physical ceiling for cross-method comparison.
const ROE_DIY_DRIFT_UPPER_BOUND: f64 = 0.2;

/// Physical reasonableness upper bound for diy drift in 10 ISS orbits.
/// Used for J2-only propagation sanity check. With 0.001 rad inclination
/// offset, differential RAAN produces diy drift of O(1e-5 to 1e-3).
/// 0.1 rad is an order-of-magnitude ceiling.
const ROE_DIY_PHYSICAL_REASONABLENESS_BOUND: f64 = 0.1;

/// Lower bound for dlambda drift from J2 coupling with da.
/// A 1 km SMA offset over 10 orbits produces secular dlambda drift via
/// n-dot coupling. The drift is O(1e-4); 1e-8 as a floor just confirms
/// the effect is nonzero.
const ROE_DLAMBDA_DRIFT_LOWER_BOUND: f64 = 1e-8;

/// Nyx two-body diy drift tolerance.
/// Without J2, nyx two-body should produce minimal diy drift. We allow 1e-4
/// to cover osculating-element mapping noise at the nyx boundary.
const ROE_NYX_TWO_BODY_DIY_TOL: f64 = 1e-4;

/// Lambert + nyx two-body position verification tolerance (km).
/// After propagating a Lambert solution with nyx two-body dynamics,
/// final position should match the target within 1 km. The residual comes
/// from Lambert solver convergence tolerance and nyx integrator truncation.
const LAMBERT_POSITION_TOL_KM: f64 = 1.0;

/// Lambert + nyx two-body velocity verification tolerance (km/s).
/// Velocity agreement after Lambert + nyx propagation. 0.01 km/s = 10 m/s
/// accounts for integration truncation and the Lambert Δv residual.
const LAMBERT_VELOCITY_TOL_KM_S: f64 = 0.01;

/// Koenig Table 4 Case 1 error bounds (meters).
/// 10x the published QNS J2 STM errors vs. Harris-Priester gravity model.
/// Our comparison is J2 STM vs. J2-only RK4, so errors are dominated by
/// linearization rather than unmodeled harmonics; 10x provides generous margin.
const KOENIG_T4C1_DA_BOUND_M: f64 = 385.0;
const KOENIG_T4C1_DLAMBDA_BOUND_M: f64 = 18088.0;
const KOENIG_T4C1_DEX_BOUND_M: f64 = 135.0;
const KOENIG_T4C1_DEY_BOUND_M: f64 = 113.0;
const KOENIG_T4C1_DIX_BOUND_M: f64 = 9.0;
const KOENIG_T4C1_DIY_BOUND_M: f64 = 25.0;

/// Eccentricity vector magnitude conservation tolerance.
/// The STM rotates the eccentricity vector by ω̇τ without growth.
/// 5% relative change over 10 orbits accommodates linearization and
/// higher-order J2 coupling not captured by the first-order STM.
const ROE_ECC_VECTOR_RELATIVE_TOL: f64 = 0.05;

/// ROE→RIC trajectory RMS position error bound (meters).
/// D'Amico Fig. 2.8 shows <3m for two-body. J2 adds secular drift over
/// 1 orbit plus mean-vs-osculating modeling differences. 50m is generous
/// enough to catch bugs without false positives.
const ROE_TO_RIC_RMS_POSITION_BOUND_M: f64 = 50.0;

/// Drag STM relative error tolerance for da drift linearity.
/// da drift should track da_dot * t to within 10%. The residual comes from
/// quadratic and coupling terms in the 9×9 STM that are not purely linear.
const DRAG_DA_DRIFT_RELATIVE_TOL: f64 = 0.1;

/// Quadratic dlambda ratio tolerance.
/// With constant da_dot, dlambda grows quadratically: ratio at 10/5 orbits ≈ 4.
/// Tolerance of 1.0 allows ~25% deviation from the ideal ratio,
/// covering higher-order STM terms and J2 coupling.
const DRAG_DLAMBDA_QUADRATIC_RATIO_TOL: f64 = 1.0;

/// ISS-like orbit radius bounds (km).
/// ISS altitude is ~400 km; radius is ~6778 km. These bounds cover
/// periapsis/apoapsis variation for nearby LEO orbits after full-physics propagation.
const ORBIT_RADIUS_LOWER_KM: f64 = 6000.0;
const ORBIT_RADIUS_UPPER_KM: f64 = 7500.0;

/// ISS-like orbit velocity bounds (km/s).
/// Circular velocity at ~400 km LEO is ~7.67 km/s. These bounds cover
/// the full range including eccentric and perturbed orbits.
const ORBIT_VELOCITY_LOWER_KM_S: f64 = 6.5;
const ORBIT_VELOCITY_UPPER_KM_S: f64 = 8.5;

/// Lambert transfer total Δv reasonableness bounds (km/s).
/// A 200 km + inclination offset transfer should require a physically
/// meaningful Δv (> 0.1 km/s) but not an escape-class impulse (< 10 km/s).
const LAMBERT_DV_LOWER_KM_S: f64 = 0.1;
const LAMBERT_DV_UPPER_KM_S: f64 = 10.0;

// =========================================================================
// RK4 J2 Integrator Self-Validation
// =========================================================================

/// Self-validate the RK4 J2 integrator: specific orbital energy should be
/// approximately conserved (slowly varying due to J2's non-conservative-like effects
/// on osculating elements, but not blowing up).
#[test]
fn rk4_j2_integrator_self_validation() {
    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

    let period = chief_ke.period().unwrap();
    let sv_1orbit = rk4_j2_propagate(&sv, period, 10.0);

    // Specific energy: E = v²/2 - μ/r
    let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
        - MU_EARTH / sv.position_eci_km.norm();
    let energy_1 = sv_1orbit.velocity_eci_km_s.norm_squared() / 2.0
        - MU_EARTH / sv_1orbit.position_eci_km.norm();

    // J2 causes osculating energy to oscillate (short-period terms) but not drift secularly.
    // Over 1 orbit, relative change should be < ENERGY_J2_RELATIVE_TOL (J2 oscillation amplitude).
    let rel_energy_change = ((energy_1 - energy_0) / energy_0).abs();
    assert!(
        rel_energy_change < ENERGY_J2_RELATIVE_TOL,
        "Energy change over 1 orbit = {rel_energy_change:.2e} (should be < {ENERGY_J2_RELATIVE_TOL})"
    );

    // Position magnitude should remain roughly the same (bound orbit)
    let r0 = sv.position_eci_km.norm();
    let r1 = sv_1orbit.position_eci_km.norm();
    assert!(
        (r1 - r0).abs() / r0 < ORBIT_RADIUS_RELATIVE_TOL,
        "Radius changed by {:.4}% over 1 orbit — integrator may be unstable",
        (r1 - r0).abs() / r0 * 100.0
    );
}

// =========================================================================
// End-to-End Mission Pipeline
// =========================================================================

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

    let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0, &LambertConfig::default())
        .expect("mission plan should succeed");

    // Lambert transfer assertions
    let transfer = plan.transfer.as_ref().expect("should have Lambert transfer");
    assert!(
        transfer.total_dv_km_s > LAMBERT_DV_LOWER_KM_S && transfer.total_dv_km_s < LAMBERT_DV_UPPER_KM_S,
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
        plan.perch_roe.dlambda.abs() > PERCH_ROE_NONZERO_LOWER_BOUND,
        "V-bar perch should have nonzero dlambda"
    );
    assert!(
        plan.perch_roe.da.abs() < PERCH_ROE_ZERO_BOUND,
        "V-bar perch da should be near-zero"
    );
    assert!(
        plan.perch_roe.dex.abs() < PERCH_ROE_ZERO_BOUND,
        "V-bar perch dex should be near-zero"
    );
    assert!(
        plan.perch_roe.dey.abs() < PERCH_ROE_ZERO_BOUND,
        "V-bar perch dey should be near-zero"
    );
}

// =========================================================================
// Drag & J2 STM Self-Consistency
// =========================================================================

/// Verify DMF drag model self-consistency: zero-ROE propagation with drag
/// should produce drift proportional to `da_dot`, with quadratic along-track growth.
#[test]
fn drag_stm_self_consistency() {
    let chief = iss_like_elements();
    let epoch = test_epoch();
    let zero_roe = QuasiNonsingularROE::default();
    let drag_config = test_drag_config();
    let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

    // J2-only propagator (zero ROE should stay near zero)
    let j2_prop = PropagationModel::J2Stm;
    let j2_5 = j2_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period).unwrap();
    assert!(
        j2_5.roe.da.abs() < ROE_DA_IDENTITY_TOL,
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
        rel_err_5 < DRAG_DA_DRIFT_RELATIVE_TOL,
        "da drift at 5 orbits: relative error {rel_err_5:.4} > 10%"
    );
    assert!(
        rel_err_10 < DRAG_DA_DRIFT_RELATIVE_TOL,
        "da drift at 10 orbits: relative error {rel_err_10:.4} > 10%"
    );

    // Along-track (dlambda) grows quadratically: ratio at 10/5 orbits ≈ 4
    let ratio = drag_10.roe.dlambda / drag_5.roe.dlambda;
    assert!(
        (ratio - 4.0).abs() < DRAG_DLAMBDA_QUADRATIC_RATIO_TOL,
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
    let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

    let prop = PropagationModel::J2Stm;
    let state_10 = prop.propagate(&roe_0, &chief, epoch, 10.0 * period).unwrap();

    // da should remain constant under J2 (no secular da drift)
    let da_change = (state_10.roe.da - roe_0.da).abs();
    assert!(
        da_change < ROE_SECULAR_CONSERVATION_TOL,
        "da should be constant under J2, changed by {da_change}"
    );

    // diy should drift due to differential RAAN regression
    // ISS RAAN regression rate is ~5 deg/day ≈ 8.7e-5 rad/s × orbital period
    // With small inclination offset, differential RAAN produces diy drift
    let diy_drift = (state_10.roe.diy - roe_0.diy).abs();
    assert!(
        diy_drift > ROE_DIY_DRIFT_LOWER_BOUND,
        "diy should drift under J2 differential RAAN, got {diy_drift}"
    );
    // Physically: 10 orbits ≈ 15 hours for ISS, RAAN drift ~3 deg
    // differential diy should be order 1e-5 to 1e-3
    assert!(
        diy_drift < ROE_DIY_PHYSICAL_REASONABLENESS_BOUND,
        "diy drift {diy_drift} seems too large for 10 ISS orbits"
    );

    // dlambda should drift from J2 mean motion coupling with da offset
    let dlambda_drift = (state_10.roe.dlambda - roe_0.dlambda).abs();
    assert!(
        dlambda_drift > ROE_DLAMBDA_DRIFT_LOWER_BOUND,
        "dlambda should drift from da + J2, got {dlambda_drift}"
    );
}

// =========================================================================
// Lambert Solver Verification
// =========================================================================

/// Solve Lambert, propagate with nyx two-body, and assert position/velocity agreement.
fn verify_lambert_against_nyx(dep: &StateVector, arr: &StateVector) {
    let transfer = crate::propagation::lambert::solve_lambert(dep, arr)
        .expect("Lambert should converge");

    let tof = (arr.epoch - dep.epoch).to_seconds();
    let almanac = nyx_bridge::load_default_almanac();
    let dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
    let results = nyx_bridge::nyx_propagate_segment(
        &transfer.departure_state, tof, 0,
        &SpacecraftConfig::SERVICER_500KG, dynamics, &almanac,
    ).expect("nyx propagation should succeed");
    let propagated = results.last().unwrap().state.clone();

    let pos_err = (propagated.position_eci_km - arr.position_eci_km).norm();
    assert!(
        pos_err < LAMBERT_POSITION_TOL_KM,
        "position error = {pos_err:.4} km (expected < {LAMBERT_POSITION_TOL_KM} km)"
    );

    let vel_err = (propagated.velocity_eci_km_s - transfer.arrival_state.velocity_eci_km_s).norm();
    assert!(
        vel_err < LAMBERT_VELOCITY_TOL_KM_S,
        "velocity error = {vel_err:.6} km/s (expected < {LAMBERT_VELOCITY_TOL_KM_S} km/s)"
    );
}

/// Verify coplanar Lambert solution (LEO 400 km → 800 km) against nyx two-body.
#[test]
fn lambert_two_body_verification() {
    let epoch = test_epoch();
    let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
    let arr = keplerian_to_state(
        &leo_800km_target_elements(),
        epoch + hifitime::Duration::from_seconds(2400.0),
    ).unwrap();

    verify_lambert_against_nyx(&dep, &arr);
}

/// Verify non-coplanar Lambert solution (RAAN offset) against nyx two-body.
#[test]
fn lambert_non_coplanar_verification() {
    let epoch = test_epoch();
    let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
    let arr_ke = KeplerianElements {
        a_km: 6378.137 + 500.0,
        e: 0.001,
        i_rad: 51.6_f64.to_radians(),
        raan_rad: 10.0_f64.to_radians(),
        aop_rad: 0.0,
        mean_anomaly_rad: 2.0,
    };
    let arr = keplerian_to_state(&arr_ke, epoch + hifitime::Duration::from_seconds(3600.0)).unwrap();

    verify_lambert_against_nyx(&dep, &arr);
}

// =========================================================================
// J2 STM vs. RK4 J2 Numerical Integrator Comparisons
// =========================================================================

/// Compare STM-propagated ROE against RK4 J2 numerical integration.
///
/// Returns physical errors `a * |ROE_stm - ROE_numerical|` in meters per element.
fn compare_stm_vs_rk4(
    chief: &KeplerianElements,
    roe: &QuasiNonsingularROE,
    n_orbits: f64,
) -> [f64; 6] {
    let epoch = test_epoch();
    let deputy = deputy_from_roe(chief, roe);
    let period = chief.period().unwrap();
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
        a * (crate::elements::wrap_angle(stm_roe.dlambda - numerical_roe.dlambda)).abs(),
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
    let chief = koenig_table2_case1();
    let roe = koenig_table2_case1_roe();
    let errors = compare_stm_vs_rk4(&chief, &roe, 10.0);

    // Bounds: 10× Koenig Table 4 QNS J2 values (Harris-Priester)
    assert!(errors[0] < KOENIG_T4C1_DA_BOUND_M,      "aδa  = {:.1}m (bound {KOENIG_T4C1_DA_BOUND_M}m)", errors[0]);
    assert!(errors[1] < KOENIG_T4C1_DLAMBDA_BOUND_M,  "aδλ  = {:.1}m (bound {KOENIG_T4C1_DLAMBDA_BOUND_M}m)", errors[1]);
    assert!(errors[2] < KOENIG_T4C1_DEX_BOUND_M,      "aδex = {:.1}m (bound {KOENIG_T4C1_DEX_BOUND_M}m)", errors[2]);
    assert!(errors[3] < KOENIG_T4C1_DEY_BOUND_M,      "aδey = {:.1}m (bound {KOENIG_T4C1_DEY_BOUND_M}m)", errors[3]);
    assert!(errors[4] < KOENIG_T4C1_DIX_BOUND_M,      "aδix = {:.1}m (bound {KOENIG_T4C1_DIX_BOUND_M}m)", errors[4]);
    assert!(errors[5] < KOENIG_T4C1_DIY_BOUND_M,      "aδiy = {:.1}m (bound {KOENIG_T4C1_DIY_BOUND_M}m)", errors[5]);

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
    for (label, chief) in &[
        ("Case 2 (e=0.2)", koenig_table2_case2()),
        ("Case 3 (e=0.5)", koenig_table2_case3()),
    ] {
        let epoch = test_epoch();
        let sv = keplerian_to_state(chief, epoch).unwrap();
        let duration = 10.0 * chief.period().unwrap();

        let sv_final = rk4_j2_propagate(&sv, duration, 10.0);

        // Energy should be approximately conserved
        let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv.position_eci_km.norm();
        let energy_f = sv_final.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv_final.position_eci_km.norm();
        let rel_energy = ((energy_f - energy_0) / energy_0).abs();
        assert!(
            rel_energy < ENERGY_J2_RELATIVE_TOL,
            "{label}: energy change = {rel_energy:.2e} over 10 orbits (should be < {ENERGY_J2_RELATIVE_TOL})"
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
            rel_sma < ORBIT_SMA_RELATIVE_TOL,
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
    let cases: [(&str, KeplerianElements, QuasiNonsingularROE); 2] = [
        ("Case 2 (e=0.2)", koenig_table2_case2(), koenig_table2_case2_roe()),
        ("Case 3 (e=0.5)", koenig_table2_case3(), koenig_table2_case3_roe()),
    ];

    for (label, chief, roe) in &cases {
        let duration = 10.0 * chief.period().unwrap();
        let (roe_prop, _) = propagate_roe_stm(roe, chief, duration).unwrap();

        // δa must be exactly conserved
        assert!(
            (roe_prop.da - roe.da).abs() < ROE_SECULAR_CONSERVATION_TOL,
            "{label}: δa not conserved: {} → {}", roe.da, roe_prop.da
        );

        // δix must be exactly conserved
        assert!(
            (roe_prop.dix - roe.dix).abs() < ROE_SECULAR_CONSERVATION_TOL,
            "{label}: δix not conserved: {} → {}", roe.dix, roe_prop.dix
        );

        // |δe| approximately conserved (rotation, not growth)
        let de_0 = roe.de_magnitude();
        let de_f = roe_prop.de_magnitude();
        if de_0 > PERCH_ROE_ZERO_BOUND {
            let rel_de = (de_f - de_0).abs() / de_0;
            assert!(
                rel_de < ROE_ECC_VECTOR_RELATIVE_TOL,
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
    let epoch = test_epoch();
    let chief = damico_table21_chief();
    let roe = damico_table21_case1_roe();
    let deputy = deputy_from_roe(&chief, &roe);

    let chief_sv = keplerian_to_state(&chief, epoch).unwrap();
    let deputy_sv = keplerian_to_state(&deputy, epoch).unwrap();

    let period = chief.period().unwrap();
    let dt_sample = 30.0; // sample every 30 seconds

    let mut sum_sq_err = 0.0;
    let mut count = 0_u32;
    let mut t = 0.0;

    while t <= period {
        // "True" RIC from numerical ECI propagation
        let chief_t = rk4_j2_propagate(&chief_sv, t, 10.0);
        let deputy_t = rk4_j2_propagate(&deputy_sv, t, 10.0);
        let ric_true = eci_to_ric_relative(&chief_t, &deputy_t).unwrap();

        // Predicted RIC from STM-propagated ROE
        let (roe_t, chief_mean_t) = propagate_roe_stm(&roe, &chief, t).unwrap();
        let ric_pred = roe_to_ric(&roe_t, &chief_mean_t).unwrap();

        let pos_err = (ric_true.position_ric_km - ric_pred.position_ric_km).norm();
        sum_sq_err += pos_err * pos_err;
        count += 1;
        t += dt_sample;
    }

    let rms_position_m = (sum_sq_err / f64::from(count)).sqrt() * 1000.0;

    // D'Amico Fig. 2.8 shows <3m for two-body. J2 adds secular drift over
    // 1 orbit plus modeling differences (mean vs osculating). 50m is generous
    // enough to catch bugs without false positives.
    assert!(
        rms_position_m < ROE_TO_RIC_RMS_POSITION_BOUND_M,
        "RMS RIC position error = {rms_position_m:.2}m (expected < {ROE_TO_RIC_RMS_POSITION_BOUND_M}m)"
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

    let period = std::f64::consts::TAU / chief_ke.mean_motion().unwrap();
    let duration = 10.0 * period;

    // Propagate both with nyx two-body
    let almanac = nyx_bridge::load_default_almanac();
    let chief_dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
    let chief_results = nyx_bridge::nyx_propagate_segment(
        &chief_sv, duration, 0,
        &SpacecraftConfig::SERVICER_500KG, chief_dynamics, &almanac,
    ).expect("chief nyx propagation failed");
    let chief_nyx = chief_results.last().unwrap().state.clone();

    let deputy_dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
    let deputy_results = nyx_bridge::nyx_propagate_segment(
        &deputy_sv, duration, 0,
        &SpacecraftConfig::SERVICER_500KG, deputy_dynamics, &almanac,
    ).expect("deputy nyx propagation failed");
    let deputy_nyx = deputy_results.last().unwrap().state.clone();

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
        (nyx_roe.da - roe_0.da).abs() < ROE_CROSS_METHOD_DA_TOL,
        "nyx da should be approximately constant: initial={}, final={}",
        roe_0.da,
        nyx_roe.da
    );
    assert!(
        (stm_state.roe.da - roe_0.da).abs() < ROE_CROSS_METHOD_DA_TOL,
        "STM da should be approximately constant: initial={}, final={}",
        roe_0.da,
        stm_state.roe.da
    );

    // The J2-induced diy difference should be physically correct
    // Over 10 ISS orbits (~15 hours), differential RAAN regression
    // for 0.001 rad inclination offset produces diy drift of ~1e-5 to 1e-3 rad
    let stm_diy_drift = (stm_state.roe.diy - roe_0.diy).abs();
    assert!(
        stm_diy_drift > ROE_SECULAR_CONSERVATION_TOL && stm_diy_drift < ROE_DIY_DRIFT_UPPER_BOUND,
        "J2 STM diy drift = {stm_diy_drift} should be in [{ROE_SECULAR_CONSERVATION_TOL}, {ROE_DIY_DRIFT_UPPER_BOUND}] rad"
    );

    // nyx two-body should have minimal diy drift (no J2)
    let nyx_diy_drift = (nyx_roe.diy - roe_0.diy).abs();
    assert!(
        nyx_diy_drift < stm_diy_drift * 10.0 || nyx_diy_drift < ROE_NYX_TWO_BODY_DIY_TOL,
        "nyx two-body diy drift = {nyx_diy_drift} should be small compared to J2 drift"
    );
}

// =========================================================================
// Full-Physics Dynamics Stack Tests
// =========================================================================

#[test]
#[ignore] // Requires MetaAlmanac (network on first run)
fn full_almanac_loads() {
    let almanac =
        nyx_bridge::load_full_almanac().expect("MetaAlmanac::latest() should succeed");
    // Verify Earth frame data is available
    let earth = almanac
        .frame_info(anise::constants::frames::IAU_EARTH_FRAME)
        .expect("IAU_EARTH frame should be available");
    assert!(
        earth.mu_km3_s2().unwrap() > 0.0,
        "Earth mu should be positive"
    );
}

#[test]
#[ignore] // Requires MetaAlmanac (network on first run)
fn full_physics_propagate_one_orbit() {
    let almanac =
        nyx_bridge::load_full_almanac().expect("full almanac should load");
    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let sv = keplerian_to_state(&chief_ke, epoch).unwrap();
    let period = chief_ke.period().unwrap();

    let dynamics = nyx_bridge::build_full_physics_dynamics(&almanac)
        .expect("full physics dynamics should build");
    let results = nyx_bridge::nyx_propagate_segment(
        &sv,
        period,
        0,
        &SpacecraftConfig::SERVICER_500KG,
        dynamics,
        &almanac,
    )
    .expect("full physics propagation should succeed");

    let final_state = &results.last().unwrap().state;
    let r = final_state.position_eci_km.norm();
    let v = final_state.velocity_eci_km_s.norm();

    // ISS-like orbit: ~6800 km altitude, ~7.5 km/s
    assert!(
        r > ORBIT_RADIUS_LOWER_KM && r < ORBIT_RADIUS_UPPER_KM,
        "final radius = {r:.1} km (expected {ORBIT_RADIUS_LOWER_KM}-{ORBIT_RADIUS_UPPER_KM})"
    );
    assert!(
        v > ORBIT_VELOCITY_LOWER_KM_S && v < ORBIT_VELOCITY_UPPER_KM_S,
        "final velocity = {v:.4} km/s (expected {ORBIT_VELOCITY_LOWER_KM_S}-{ORBIT_VELOCITY_UPPER_KM_S})"
    );

    // Energy should be approximately conserved
    let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
        - MU_EARTH / sv.position_eci_km.norm();
    let energy_f = final_state.velocity_eci_km_s.norm_squared() / 2.0
        - MU_EARTH / final_state.position_eci_km.norm();
    let rel_energy = ((energy_f - energy_0) / energy_0).abs();
    // Full physics (drag, SRP) will cause some energy change, but not catastrophic
    assert!(
        rel_energy < ENERGY_FULL_PHYSICS_RELATIVE_TOL,
        "energy change = {rel_energy:.2e} (expected < {ENERGY_FULL_PHYSICS_RELATIVE_TOL})"
    );
}

// =========================================================================
// DMF Rate Extraction Tests
// =========================================================================

/// Two spacecraft with different ballistic coefficients should produce
/// nonzero differential drag rates. Chief: 500 kg / 1.0 m² (default),
/// Deputy: 200 kg / 2.0 m² (higher B* = Cd·A/m).
#[test]
#[ignore] // Requires MetaAlmanac (network on first run)
fn extract_dmf_rates_nonzero() {
    let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

    // Deputy on same orbit — differential drag comes from different B*
    let chief_config = SpacecraftConfig::SERVICER_500KG;
    let deputy_config = SpacecraftConfig {
        dry_mass_kg: 200.0,
        drag_area_m2: 2.0,
        ..SpacecraftConfig::SERVICER_500KG
    };

    let drag = nyx_bridge::extract_dmf_rates(
        &sv, &sv, &chief_config, &deputy_config, &almanac,
    )
    .expect("DMF extraction should succeed");

    // da_dot should be nonzero: higher B* deputy decays faster
    assert!(
        drag.da_dot.abs() > DMF_RATE_NONZERO_LOWER_BOUND,
        "da_dot should be nonzero for different B*, got {:.2e}",
        drag.da_dot
    );

    // da_dot should be negative: deputy has higher drag, so relative SMA decreases
    assert!(
        drag.da_dot < 0.0,
        "da_dot should be negative (deputy decays faster), got {:.2e}",
        drag.da_dot
    );

    // Physical reasonableness: for ISS-like orbit, differential da_dot
    // should be order 1e-12 to 1e-9 (dimensionless, per second)
    assert!(
        drag.da_dot.abs() < DMF_RATE_UPPER_BOUND,
        "da_dot = {:.2e} seems unreasonably large",
        drag.da_dot
    );

    eprintln!(
        "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
        drag.da_dot, drag.dex_dot, drag.dey_dot
    );
}

/// Identical spacecraft configs should produce zero (or sub-threshold) drag rates.
#[test]
#[ignore] // Requires MetaAlmanac (network on first run)
fn extract_dmf_rates_identical() {
    let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

    let config = SpacecraftConfig::SERVICER_500KG;
    let drag = nyx_bridge::extract_dmf_rates(
        &sv, &sv, &config, &config, &almanac,
    )
    .expect("DMF extraction should succeed");

    // Identical B* → zero differential drag
    assert!(
        drag.da_dot.abs() < f64::EPSILON,
        "da_dot should be zero for identical configs, got {:.2e}",
        drag.da_dot
    );
    assert!(
        drag.dex_dot.abs() < f64::EPSILON,
        "dex_dot should be zero for identical configs, got {:.2e}",
        drag.dex_dot
    );
    assert!(
        drag.dey_dot.abs() < f64::EPSILON,
        "dey_dot should be zero for identical configs, got {:.2e}",
        drag.dey_dot
    );
}

/// POCA invariant sweep: refined distance ≤ grid-sampled distance.
///
/// Validates across all paper-traced orbits (Koenig Cases 1–3, D'Amico Table 2.1).
/// Diverging legs (no POCA) are skipped — the invariant only applies when a
/// range-rate sign change exists.
#[test]
fn poca_refined_leq_grid_sampled_sweep() {
    let epoch = test_epoch();
    let cases: &[(&str, KeplerianElements, QuasiNonsingularROE)] = &[
        ("Koenig Case 1", koenig_table2_case1(), koenig_table2_case1_roe()),
        ("Koenig Case 2", koenig_table2_case2(), koenig_table2_case2_roe()),
        ("Koenig Case 3", koenig_table2_case3(), koenig_table2_case3_roe()),
        ("D'Amico Table 2.1", damico_table21_chief(), damico_table21_case1_roe()),
    ];

    for (label, chief, roe) in cases {
        let period_s = chief.period().expect("period should succeed");
        let traj = PropagationModel::J2Stm
            .propagate_with_steps(roe, chief, epoch, period_s, 200)
            .unwrap_or_else(|e| panic!("{label}: propagation failed: {e}"));

        let grid_min_km = analyze_trajectory_safety(&traj)
            .unwrap_or_else(|e| panic!("{label}: safety analysis failed: {e}"))
            .operational
            .min_distance_3d_km;

        let pocas = find_closest_approaches(&traj, chief, epoch, &PropagationModel::J2Stm, roe, 0)
            .unwrap_or_else(|e| panic!("{label}: POCA failed: {e}"));

        if pocas.is_empty() {
            // Diverging leg — no bracket found, invariant not applicable
            continue;
        }

        assert!(
            pocas[0].distance_km <= grid_min_km + f64::EPSILON,
            "{label}: refined POCA {:.6} km > grid-sampled {grid_min_km:.6} km",
            pocas[0].distance_km,
        );
    }
}

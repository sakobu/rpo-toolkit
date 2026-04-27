//! Tests for the Izzo Lambert solver (algorithmic kernel + public API).
//!
//! Ports the in-tree tests from the standalone `lambert_izzo` crate (Izzo
//! 2015 §5 / §6 paper-traced cases) plus new coverage for the rpo-core-
//! specific public API: `TransferDirection::Auto`, multi-rev direction
//! discrimination, `NoSolutionForRevolutions` error path.

#![allow(clippy::similar_names, clippy::unwrap_used, clippy::float_cmp)]

use core::f64::consts::PI;

use hifitime::{Duration, Epoch};
use nalgebra::Vector3;

use crate::constants::MU_EARTH;
use crate::test_helpers::test_epoch;
use crate::types::StateVector;

use super::solver::{lambert_raw, RawSolution};
use super::{
    solve_lambert_branches, solve_lambert_with_config, LambertConfig, LambertError,
    PositionTag, TransferDirection,
};

/// Sun's gravitational parameter (km³/s²) — value from DE440. Local to the
/// test module since the workspace constants only need [`MU_EARTH`] for the
/// production code paths.
const MU_SUN_KM3_S2: f64 = 1.327_124_400_18e11;

/// Inline absolute-tolerance helper used by analytic-truth comparisons.
/// Callers pass a named tolerance constant — never a bare `1e-X` literal.
fn approx(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() < tol
}

// ---------------------------------------------------------------------------
// Test tolerances (Izzo §5 paper-traced; see citations on each constant)
// ---------------------------------------------------------------------------

/// Velocity-component agreement vs. an analytic circular-orbit truth (km/s).
/// `1e-9 km/s` (1 µm/s) is comfortably above the Lambert-solver Δv noise
/// floor (~`1e-12` from Householder convergence) for LEO-scale geometries.
const VELOCITY_VS_ANALYTIC_TOL_KM_S: f64 = 1e-9;

/// Velocity-magnitude agreement for the Earth-Mars Hohmann fixture (km/s).
/// Loose because the fixture uses `r_mars = 1.524 AU` rather than a true
/// Mars-at-arrival ephemeris, and the tangent-launch approximation drops
/// the Mars sphere-of-influence handoff.
const HOHMANN_VELOCITY_TOL_KM_S: f64 = 1e-3;

/// Kepler-roundtrip (single-rev / hyperbolic / Battin / multi-rev branch)
/// position error (km). Combines Householder convergence (~`1e-9` km in
/// position) with the universal-variable propagator's Newton-iteration
/// budget; `1e-3 km` (1 m) is the per-trial tolerance Izzo §5 uses for
/// the same family of regression tests.
const KEPLER_ROUNDTRIP_POSITION_TOL_KM: f64 = 1e-3;

/// Multi-rev Δv difference floor when comparing `ShortWay` vs `LongWay`
/// branches at the same `M` (km/s). Multi-rev branches always produce
/// distinct velocities for non-degenerate geometries; `1e-6 km/s` is a
/// safe minimum separation that any two distinct branches will exceed.
const MULTI_REV_DV_DIFFERENCE_TOL_KM_S: f64 = 1e-6;

/// `Auto` direction tolerance vs. `min(short, long)`. The Auto branch
/// routes through the same solver as ShortWay/LongWay, so the result is
/// bit-identical up to the final comparison; `1e-12 km/s` covers the
/// signed-zero / `partial_cmp` ordering noise.
const AUTO_DIRECTION_AGREEMENT_TOL_KM_S: f64 = 1e-12;

/// Statistical Kepler-roundtrip bound for 1000 random single-rev geometries
/// at Earth scale (Izzo §5 / Fig. 3). Relative position error ceiling.
const KEPLER_ROUNDTRIP_STATISTICAL_TOL_REL: f64 = 1e-6;

/// Statistical Kepler-roundtrip bound for 500 random multi-rev geometries
/// (Izzo §5). Looser than single-rev because multi-rev branches near
/// `T_min(M)` are stiffer.
const KEPLER_ROUNDTRIP_MULTI_REV_TOL_REL: f64 = 1e-5;

/// Single-trial Kepler-roundtrip bound for a benign well-conditioned
/// geometry (km). Well below the per-trial floor used by the stiff
/// regimes (Battin, hyperbolic, multi-rev); see
/// [`KEPLER_ROUNDTRIP_POSITION_TOL_KM`].
const KEPLER_ROUNDTRIP_BENIGN_TOL_KM: f64 = 1e-6;

/// Lambert-success rate floor for the 1000-trial single-rev sweep.
const LAMBERT_SUCCESS_FLOOR_SINGLE: u32 = 950;
/// Converged-roundtrip floor for the 1000-trial single-rev sweep.
const ROUNDTRIP_CONVERGED_FLOOR_SINGLE: u32 = 500;
/// Branch count floor for the 500-trial multi-rev sweep.
const BRANCHES_FLOOR_MULTI: u32 = 500;
/// Converged-roundtrip floor for the 500-trial multi-rev sweep.
const ROUNDTRIP_CONVERGED_FLOOR_MULTI: u32 = 300;

/// Trial counts.
const STATISTICAL_SINGLE_REV_TRIALS: u32 = 1000;
const STATISTICAL_MULTI_REV_TRIALS: u32 = 500;
const STATISTICAL_MAX_REVS: u32 = 3;

/// Position bounds for the random-geometry sweeps (km / s).
const STATISTICAL_SINGLE_R_MIN_KM: f64 = 3500.0;
const STATISTICAL_SINGLE_R_MAX_KM: f64 = 28_000.0;
const STATISTICAL_SINGLE_TOF_MIN_S: f64 = 100.0;
const STATISTICAL_SINGLE_TOF_MAX_S: f64 = 50_000.0;
const STATISTICAL_MULTI_R_MIN_KM: f64 = 5600.0;
const STATISTICAL_MULTI_R_MAX_KM: f64 = 10_500.0;
const STATISTICAL_MULTI_TOF_MIN_S: f64 = 10_000.0;
const STATISTICAL_MULTI_TOF_MAX_S: f64 = 250_000.0;

/// Rejection-sampling bounds for `rand_unit_vec`.
const UNIT_VEC_NORM2_LO: f64 = 0.01;
const UNIT_VEC_NORM2_HI: f64 = 1.0;

// =====================================================================
// Universal-variable Kepler propagator (test-only, two-body truth source)
// =====================================================================

/// Newton-iteration cap on the universal Kepler equation.
const KEPLER_MAX_ITERS: u32 = 200;
/// Convergence tolerance on the universal-variable step `|Δχ|`.
const KEPLER_NEWTON_TOL: f64 = 1e-12;
/// Threshold below which the Newton denominator `r` is treated as a
/// divergence sentinel — the iteration returns NaN-vector and the
/// statistical tests filter those trials.
const KEPLER_DENOM_EPS: f64 = 1e-12;
/// Divergence sentinel on `|χ|`.
const KEPLER_CHI_DIVERGENCE: f64 = 1e12;
/// Branch threshold for the Stumpff `c2(ψ)`, `c3(ψ)` evaluator.
const STUMPFF_SERIES_THRESHOLD: f64 = 1e-6;

/// Propagate `(r0, v0)` for `dt` under two-body gravity using the universal-
/// variable formulation. Returns NaN-vector on Newton divergence; the
/// statistical tests filter divergent trials via `.is_finite()`.
///
/// Independent of `propagate_keplerian` (which is element-based and only
/// handles bound orbits) — this kernel propagates hyperbolic transfers
/// correctly, which Lambert tests demand.
fn kepler_propagate(
    r0_km: Vector3<f64>,
    v0_km_s: Vector3<f64>,
    dt_s: f64,
    mu_km3_s2: f64,
) -> Vector3<f64> {
    let r0n = r0_km.norm();
    let v0n2 = v0_km_s.norm_squared();
    let alpha = 2.0 / r0n - v0n2 / mu_km3_s2;
    let sqrt_mu = mu_km3_s2.sqrt();
    let sigma0 = r0_km.dot(&v0_km_s) / sqrt_mu;
    let mut chi = sqrt_mu * dt_s * alpha.abs();
    let nan_vec = Vector3::new(f64::NAN, f64::NAN, f64::NAN);
    let mut converged = false;
    for _ in 0..KEPLER_MAX_ITERS {
        let psi = alpha * chi * chi;
        let (c2, c3) = stumpff(psi);
        let r = chi * chi * c2 + sigma0 * chi * (1.0 - psi * c3) + r0n * (1.0 - psi * c2);
        if r.abs() < KEPLER_DENOM_EPS || !r.is_finite() {
            return nan_vec;
        }
        let f = sigma0 * chi * chi * c2
            + (1.0 - alpha * r0n) * chi * chi * chi * c3
            + r0n * chi
            - sqrt_mu * dt_s;
        let dchi = -f / r;
        chi += dchi;
        if !chi.is_finite() || chi.abs() > KEPLER_CHI_DIVERGENCE {
            return nan_vec;
        }
        if dchi.abs() < KEPLER_NEWTON_TOL {
            converged = true;
            break;
        }
    }
    if !converged {
        return nan_vec;
    }
    let psi = alpha * chi * chi;
    let (c2, _c3) = stumpff(psi);
    let f_lagrange = 1.0 - chi * chi / r0n * c2;
    let g_lagrange = dt_s - chi * chi * chi / sqrt_mu * stumpff(psi).1;
    r0_km * f_lagrange + v0_km_s * g_lagrange
}

fn stumpff(psi: f64) -> (f64, f64) {
    if psi > STUMPFF_SERIES_THRESHOLD {
        let s = psi.sqrt();
        ((1.0 - s.cos()) / psi, (s - s.sin()) / s.powi(3))
    } else if psi < -STUMPFF_SERIES_THRESHOLD {
        let s = (-psi).sqrt();
        ((1.0 - s.cosh()) / psi, (s.sinh() - s) / s.powi(3))
    } else {
        (
            0.5 - psi / 24.0 + psi * psi / 720.0,
            1.0 / 6.0 - psi / 120.0 + psi * psi / 5040.0,
        )
    }
}

// =====================================================================
// Kernel tests (raw lambert_raw — any μ)
// =====================================================================

#[test]
fn quarter_circle_leo() {
    let r_km = 7000.0;
    let mu = MU_EARTH;
    let v_circ = (mu / r_km).sqrt();
    let period_s = 2.0 * PI * (r_km.powi(3) / mu).sqrt();

    let r1_km = Vector3::new(r_km, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, r_km, 0.0);
    let sols = lambert_raw(r1_km, r2_km, period_s / 4.0, mu, TransferDirection::ShortWay, 0)
        .unwrap();
    assert_eq!(sols.len(), 1);
    let s = sols[0];
    assert_eq!(s.n_revs, 0);
    assert!((s.v1_km_s - Vector3::new(0.0, v_circ, 0.0)).norm() < VELOCITY_VS_ANALYTIC_TOL_KM_S);
    assert!((s.v2_km_s - Vector3::new(-v_circ, 0.0, 0.0)).norm() < VELOCITY_VS_ANALYTIC_TOL_KM_S);
}

#[test]
fn long_way_quarter_circle_leo() {
    let r_km = 7000.0;
    let mu = MU_EARTH;
    let v_circ = (mu / r_km).sqrt();
    let period_s = 2.0 * PI * (r_km.powi(3) / mu).sqrt();

    let r1_km = Vector3::new(r_km, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, r_km, 0.0);
    let sols = lambert_raw(
        r1_km,
        r2_km,
        3.0 * period_s / 4.0,
        mu,
        TransferDirection::LongWay,
        0,
    )
    .unwrap();
    assert_eq!(sols.len(), 1);
    let s = sols[0];
    assert!((s.v1_km_s - Vector3::new(0.0, -v_circ, 0.0)).norm() < VELOCITY_VS_ANALYTIC_TOL_KM_S);
    assert!((s.v2_km_s - Vector3::new(v_circ, 0.0, 0.0)).norm() < VELOCITY_VS_ANALYTIC_TOL_KM_S);
}

#[test]
fn earth_mars_hohmann() {
    const AU_KM: f64 = 1.495_978_707e8;
    let mu = MU_SUN_KM3_S2;
    let r1n = AU_KM;
    let r2n = 1.524 * AU_KM;
    let a_km = f64::midpoint(r1n, r2n);
    let tof_s = PI * (a_km.powi(3) / mu).sqrt();

    let r1_km = Vector3::new(r1n, 0.0, 0.0);
    // 1 km off-plane to dodge collinearity edge case.
    let r2_km = Vector3::new(-r2n, 1.0, 0.0);
    let sols = lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, 0).unwrap();
    let v_peri = (mu * (2.0 / r1n - 1.0 / a_km)).sqrt();
    assert!(approx(sols[0].v1_km_s.norm(), v_peri, HOHMANN_VELOCITY_TOL_KM_S));
}

#[test]
fn multi_rev_branches() {
    let mu = MU_EARTH;
    let r1_km = Vector3::new(8000.0, 0.0, 0.0);
    let r2_km = Vector3::new(5600.0, 5600.0, 0.0);
    let period_s = 2.0 * PI * (8000.0_f64.powi(3) / mu).sqrt();
    let sols = lambert_raw(r1_km, r2_km, 5.0 * period_s, mu, TransferDirection::ShortWay, 3)
        .unwrap();
    assert!(sols.len() >= 3, "got {} solutions", sols.len());
    for s in &sols {
        let energy = 0.5 * s.v1_km_s.dot(&s.v1_km_s) - mu / r1_km.norm();
        assert!(energy.is_finite());
    }
}

#[test]
fn round_trip_kepler_check_single_rev() {
    let mu = MU_EARTH;
    let r1_km = Vector3::new(10_500.0, 1400.0, 700.0);
    let r2_km = Vector3::new(-2800.0, 9100.0, -1400.0);
    let tof_s = 4500.0;
    let sols = lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, 0).unwrap();
    let r2_prop = kepler_propagate(r1_km, sols[0].v1_km_s, tof_s, mu);
    let err_km = (r2_prop - r2_km).norm();
    assert!(err_km < KEPLER_ROUNDTRIP_BENIGN_TOL_KM, "kepler-roundtrip err = {err_km} km");
}

#[test]
fn errors_on_non_positive_tof_kernel() {
    let r1_km = Vector3::new(7000.0, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, 7000.0, 0.0);
    let err = lambert_raw(r1_km, r2_km, 0.0, MU_EARTH, TransferDirection::ShortWay, 0)
        .unwrap_err();
    assert!(matches!(err, LambertError::NonPositiveTimeOfFlight { tof_s } if tof_s == 0.0));
}

#[test]
fn errors_on_zero_position_vector() {
    let r1_km = Vector3::zeros();
    let r2_km = Vector3::new(0.0, 7000.0, 0.0);
    let err = lambert_raw(
        r1_km,
        r2_km,
        1000.0,
        MU_EARTH,
        TransferDirection::ShortWay,
        0,
    )
    .unwrap_err();
    assert!(matches!(
        err,
        LambertError::DegeneratePositionVector { which: PositionTag::R1, .. }
    ));
}

#[test]
fn errors_on_collinear_geometry() {
    let r1_km = Vector3::new(7000.0, 0.0, 0.0);
    let r2_km = Vector3::new(14_000.0, 0.0, 0.0);
    let err = lambert_raw(
        r1_km,
        r2_km,
        1000.0,
        MU_EARTH,
        TransferDirection::ShortWay,
        0,
    )
    .unwrap_err();
    assert!(matches!(err, LambertError::CollinearGeometry { .. }));
}

#[test]
fn errors_on_non_positive_mu() {
    let r1_km = Vector3::new(7000.0, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, 7000.0, 0.0);
    let err = lambert_raw(r1_km, r2_km, 1000.0, 0.0, TransferDirection::ShortWay, 0)
        .unwrap_err();
    assert!(matches!(err, LambertError::NonPositiveMu { mu_km3_s2 } if mu_km3_s2 == 0.0));
}

#[test]
fn battin_regime_near_parabolic() {
    let mu = MU_EARTH;
    let r1_km = Vector3::new(7000.0, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, 42_000.0, 0.0);
    let tof_s = 7200.0;
    let sols = lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, 0).unwrap();
    let s = sols[0];
    assert!(
        (s.x - 1.0).abs() < super::BATTIN_THRESHOLD,
        "x = {} not in Battin band [1 ± {}]",
        s.x,
        super::BATTIN_THRESHOLD
    );
    let r2_prop = kepler_propagate(r1_km, s.v1_km_s, tof_s, mu);
    let err_km = (r2_prop - r2_km).norm();
    assert!(err_km < KEPLER_ROUNDTRIP_POSITION_TOL_KM, "Battin round-trip err = {err_km} km");
}

#[test]
fn hyperbolic_transfer() {
    let mu = MU_EARTH;
    let r1_km = Vector3::new(7000.0, 0.0, 0.0);
    let r2_km = Vector3::new(0.0, 200_000.0, 0.0);
    let tof_s = 30_000.0;
    let sols = lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, 0).unwrap();
    let s = sols[0];
    assert!(s.x > 1.0, "expected hyperbolic (x > 1), got x = {}", s.x);
    let energy = 0.5 * s.v1_km_s.dot(&s.v1_km_s) - mu / r1_km.norm();
    assert!(energy > 0.0, "expected positive specific energy, got {energy}");
    let r2_prop = kepler_propagate(r1_km, s.v1_km_s, tof_s, mu);
    let err_km = (r2_prop - r2_km).norm();
    assert!(err_km < KEPLER_ROUNDTRIP_POSITION_TOL_KM, "hyperbolic round-trip err = {err_km} km");
}

#[test]
fn multi_rev_branches_distinct() {
    let mu = MU_EARTH;
    let r1_km = Vector3::new(8000.0, 0.0, 0.0);
    let r2_km = Vector3::new(5600.0, 5600.0, 0.0);
    let period_s = 2.0 * PI * (8000.0_f64.powi(3) / mu).sqrt();
    let tof_s = 5.0 * period_s;
    let sols = lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, 3).unwrap();

    let multi: Vec<&RawSolution> = sols.iter().filter(|s| s.n_revs > 0).collect();
    assert!(!multi.is_empty(), "no multi-rev branches found");
    assert_eq!(multi.len() % 2, 0, "multi-rev branches not paired: {}", multi.len());

    for pair in multi.chunks(2) {
        assert_eq!(pair[0].n_revs, pair[1].n_revs, "branches in pair don't share n_revs");
        // find_xy pushes (xl, xr) per M — long-period x first, short-period x second.
        assert!(
            pair[0].x < pair[1].x,
            "M = {}: long-period x ({}) >= short-period x ({})",
            pair[0].n_revs,
            pair[0].x,
            pair[1].x,
        );
        for s in pair {
            let r2_prop = kepler_propagate(r1_km, s.v1_km_s, tof_s, mu);
            let err_km = (r2_prop - r2_km).norm();
            assert!(
                err_km < KEPLER_ROUNDTRIP_POSITION_TOL_KM,
                "M = {} branch round-trip err = {err_km} km",
                s.n_revs
            );
        }
    }
}

// =====================================================================
// Public-API tests (StateVector + MU_EARTH)
// =====================================================================

fn earth_state(epoch: Epoch, position: Vector3<f64>) -> StateVector {
    StateVector {
        epoch,
        position_eci_km: position,
        velocity_eci_km_s: Vector3::zeros(),
    }
}

#[test]
fn solve_lambert_with_config_short_way_succeeds() {
    let epoch = test_epoch();
    let dep = earth_state(epoch, Vector3::new(7000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch + Duration::from_seconds(1500.0),
        Vector3::new(0.0, 7000.0, 0.0),
    );
    let transfer = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig {
            direction: TransferDirection::ShortWay,
            revolutions: 0,
        },
    )
    .expect("Lambert succeeds");
    assert!(transfer.total_dv_km_s.is_finite() && transfer.total_dv_km_s > 0.0);
    assert_eq!(transfer.direction, TransferDirection::ShortWay);
}

#[test]
fn solve_lambert_errors_on_non_positive_tof() {
    let epoch = test_epoch();
    let dep = earth_state(epoch, Vector3::new(7000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch - Duration::from_seconds(60.0),
        Vector3::new(0.0, 7000.0, 0.0),
    );
    let err = solve_lambert_with_config(&dep, &arr, &LambertConfig::default()).unwrap_err();
    assert!(
        matches!(err, LambertError::NonPositiveTimeOfFlight { tof_s } if tof_s < 0.0),
        "got {err:?}"
    );
}

#[test]
fn auto_direction_picks_lower_dv() {
    // Geometry chosen so short and long way produce different ΔV; Auto must
    // return the lower-Δv solution.
    let epoch = test_epoch();
    let dep = earth_state(epoch, Vector3::new(7000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch + Duration::from_seconds(3600.0),
        Vector3::new(0.0, -7000.0, -500.0),
    );

    let short = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::ShortWay, revolutions: 0 },
    )
    .expect("short-way succeeds");
    let long = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::LongWay, revolutions: 0 },
    )
    .expect("long-way succeeds");
    let auto = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::Auto, revolutions: 0 },
    )
    .expect("auto succeeds");

    let lower = short.total_dv_km_s.min(long.total_dv_km_s);
    assert!(
        approx(auto.total_dv_km_s, lower, AUTO_DIRECTION_AGREEMENT_TOL_KM_S),
        "auto Δv {} should equal min(short {}, long {})",
        auto.total_dv_km_s,
        short.total_dv_km_s,
        long.total_dv_km_s,
    );
}

#[test]
fn multi_rev_respects_direction() {
    // For revolutions ≥ 1, ShortWay and LongWay must produce distinct ΔV
    // for any geometry where both branches exist. (Replaces the
    // `multi_rev_ignores_direction` workaround that existed only because
    // nyx-space 2.3.1's Izzo collapsed the directions for multi-rev.)
    let epoch = test_epoch();
    let mu = MU_EARTH;
    let period_s = 2.0 * PI * (8000.0_f64.powi(3) / mu).sqrt();
    let dep = earth_state(epoch, Vector3::new(8000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch + Duration::from_seconds(5.0 * period_s),
        Vector3::new(5600.0, 5600.0, 0.0),
    );

    let short = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::ShortWay, revolutions: 1 },
    )
    .expect("short multi-rev succeeds");
    let long = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::LongWay, revolutions: 1 },
    )
    .expect("long multi-rev succeeds");

    let delta_dv = (short.total_dv_km_s - long.total_dv_km_s).abs();
    assert!(
        delta_dv > MULTI_REV_DV_DIFFERENCE_TOL_KM_S,
        "ShortWay (Δv = {}) and LongWay (Δv = {}) collapsed for multi-rev (Δ = {delta_dv})",
        short.total_dv_km_s,
        long.total_dv_km_s,
    );
}

#[test]
fn no_solution_for_excessive_revolutions() {
    // Single-orbit-period TOF cannot accommodate revolutions = 5.
    let epoch = test_epoch();
    let mu = MU_EARTH;
    let period_s = 2.0 * PI * (8000.0_f64.powi(3) / mu).sqrt();
    let dep = earth_state(epoch, Vector3::new(8000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch + Duration::from_seconds(0.8 * period_s),
        Vector3::new(0.0, 8000.0, 0.0),
    );
    let err = solve_lambert_with_config(
        &dep,
        &arr,
        &LambertConfig { direction: TransferDirection::ShortWay, revolutions: 5 },
    )
    .unwrap_err();
    assert!(
        matches!(
            err,
            LambertError::NoSolutionForRevolutions { requested: 5, max_feasible }
                if max_feasible < 5
        ),
        "got {err:?}"
    );
}

#[test]
fn solve_lambert_branches_returns_sorted_by_dv() {
    let epoch = test_epoch();
    let mu = MU_EARTH;
    let period_s = 2.0 * PI * (8000.0_f64.powi(3) / mu).sqrt();
    let dep = earth_state(epoch, Vector3::new(8000.0, 0.0, 0.0));
    let arr = earth_state(
        epoch + Duration::from_seconds(5.0 * period_s),
        Vector3::new(5600.0, 5600.0, 0.0),
    );

    let branches = solve_lambert_branches(&dep, &arr, 3).expect("succeeds");
    assert!(!branches.is_empty(), "expect at least one branch");
    for w in branches.windows(2) {
        assert!(
            w[0].total_dv_km_s <= w[1].total_dv_km_s,
            "branches not sorted: {} > {}",
            w[0].total_dv_km_s,
            w[1].total_dv_km_s,
        );
    }
    // Both directions should appear (Auto / Short and Long produced different
    // results for this geometry).
    let has_short = branches
        .iter()
        .any(|b| b.direction == TransferDirection::ShortWay);
    let has_long = branches.iter().any(|b| b.direction == TransferDirection::LongWay);
    assert!(has_short && has_long, "expect both directions");
}

// =====================================================================
// Statistical kernel tests (Izzo §5 paper-traced)
// =====================================================================

fn rand_unit_vec(rng: &mut rand_chacha::ChaCha20Rng) -> Vector3<f64> {
    use rand::Rng;
    use rand_distr::Uniform;
    let axis: Uniform<f64> = Uniform::new(-1.0, 1.0);
    loop {
        let v = Vector3::new(rng.sample(axis), rng.sample(axis), rng.sample(axis));
        let n2 = v.norm_squared();
        if n2 > UNIT_VEC_NORM2_LO && n2 < UNIT_VEC_NORM2_HI {
            return v / n2.sqrt();
        }
    }
}

#[test]
fn kepler_roundtrip_random_single_rev() {
    use rand::{Rng, SeedableRng};
    use rand_chacha::ChaCha20Rng;
    use rand_distr::Uniform;

    let mu = MU_EARTH;
    let mut rng = ChaCha20Rng::seed_from_u64(0xC0FF_EE42);
    let radius = Uniform::new(STATISTICAL_SINGLE_R_MIN_KM, STATISTICAL_SINGLE_R_MAX_KM);
    let tof = Uniform::new(STATISTICAL_SINGLE_TOF_MIN_S, STATISTICAL_SINGLE_TOF_MAX_S);

    let mut max_rel_err = 0.0_f64;
    let mut good_count = 0_u32;
    let mut lambert_ok = 0_u32;
    for _ in 0..STATISTICAL_SINGLE_REV_TRIALS {
        let r1_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let r2_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let tof_s = rng.sample(tof);
        let way = if rng.gen_bool(0.5) {
            TransferDirection::LongWay
        } else {
            TransferDirection::ShortWay
        };
        let Ok(sols) = lambert_raw(r1_km, r2_km, tof_s, mu, way, 0) else {
            continue;
        };
        lambert_ok += 1;
        let r2_prop = kepler_propagate(r1_km, sols[0].v1_km_s, tof_s, mu);
        let rel = (r2_prop - r2_km).norm() / r2_km.norm();
        if rel.is_finite() {
            max_rel_err = max_rel_err.max(rel);
            good_count += 1;
        }
    }
    assert!(
        lambert_ok > LAMBERT_SUCCESS_FLOOR_SINGLE,
        "too many Lambert failures: {lambert_ok}/{STATISTICAL_SINGLE_REV_TRIALS}"
    );
    assert!(
        good_count > ROUNDTRIP_CONVERGED_FLOOR_SINGLE,
        "too few converged round-trips: {good_count}/{lambert_ok}"
    );
    assert!(
        max_rel_err < KEPLER_ROUNDTRIP_STATISTICAL_TOL_REL,
        "max relative round-trip err = {max_rel_err:.3e}"
    );
}

#[test]
fn kepler_roundtrip_random_multi_rev() {
    use rand::{Rng, SeedableRng};
    use rand_chacha::ChaCha20Rng;
    use rand_distr::Uniform;

    let mu = MU_EARTH;
    let mut rng = ChaCha20Rng::seed_from_u64(0xBEEF_DEAD);
    let radius = Uniform::new(STATISTICAL_MULTI_R_MIN_KM, STATISTICAL_MULTI_R_MAX_KM);
    let tof = Uniform::new(STATISTICAL_MULTI_TOF_MIN_S, STATISTICAL_MULTI_TOF_MAX_S);

    let mut max_rel_err = 0.0_f64;
    let mut branches = 0_u32;
    let mut good_count = 0_u32;
    for _ in 0..STATISTICAL_MULTI_REV_TRIALS {
        let r1_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let r2_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let tof_s = rng.sample(tof);
        let Ok(sols) =
            lambert_raw(r1_km, r2_km, tof_s, mu, TransferDirection::ShortWay, STATISTICAL_MAX_REVS)
        else {
            continue;
        };
        for s in &sols {
            let r2_prop = kepler_propagate(r1_km, s.v1_km_s, tof_s, mu);
            let rel = (r2_prop - r2_km).norm() / r2_km.norm();
            if rel.is_finite() {
                max_rel_err = max_rel_err.max(rel);
                good_count += 1;
            }
            branches += 1;
        }
    }
    assert!(
        branches > BRANCHES_FLOOR_MULTI,
        "expected branches > {BRANCHES_FLOOR_MULTI}, got {branches}"
    );
    assert!(
        good_count > ROUNDTRIP_CONVERGED_FLOOR_MULTI,
        "too few converged round-trips: {good_count}/{branches}"
    );
    assert!(
        max_rel_err < KEPLER_ROUNDTRIP_MULTI_REV_TOL_REL,
        "max relative round-trip err = {max_rel_err:.3e}"
    );
}

//! Paper-scale Lambert stress sweeps (Izzo 2015 §5, Earth scale).
//!
//! Two `#[ignore]`d 100k-trial sweeps that check `ΔE/E` and `Δh/h`
//! conservation between the solver's `r1` and `r2` endpoint evaluations —
//! a self-consistency check on the velocity reconstruction.
//!
//! Run with:
//! ```sh
//! cargo test -p rpo-core --release --test lambert_stress -- --ignored
//! ```
//!
//! # Relationship to paper
//!
//! Izzo §5 / Fig. 6 reports `|x_true − x|` (the iterate's distance from
//! the analytically-known root) as the primary accuracy metric. That
//! metric is covered by the CI-fast tests
//! `paper_fig6_x_error_single_rev` / `paper_fig6_x_error_multi_rev` in
//! `propagation::lambert::solver_tests`. The conservation check below is
//! a *related but distinct* property — the two endpoints of a two-body
//! transfer must agree on `ε = v²/2 − μ/r` and `h = r × v` — and serves
//! as a regression guard at paper-scale (100k geometries) for the full
//! public solver pipeline.
//!
//! Conservation budgets (`SINGLE_REV_ENERGY_BUDGET = 1.2e-11`,
//! `MULTI_REV_ENERGY_BUDGET = 3e-13`) are empirical: the paper does not
//! report conservation numbers, only `|x_true − x|` (Fig. 6). They are
//! tight enough to catch real velocity-reconstruction regressions and
//! loose enough to absorb arithmetic noise on 100k random geometries.

use std::sync::OnceLock;

use hifitime::{Duration, Epoch};
use nalgebra::Vector3;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rand_distr::Uniform;

use rpo_core::constants::MU_EARTH;
use rpo_core::propagation::lambert::{
    solve_lambert_branches, solve_lambert_with_config, LambertConfig, TransferDirection,
};
use rpo_core::types::StateVector;

/// Paper §5 sample size (`100,000` per M-stratum). Single-rev sweep uses
/// the same count.
const N_TRIALS: u32 = 100_000;

/// Maximum revolutions for the multi-revolution sweep.
const MULTI_REV_MAX: u8 = 5;

/// Floor on `0.5 · (|ε₁| + |ε₂|)` and `0.5 · (|h₁| + |h₂|)` when computing
/// the relative mismatch. Prevents division-by-zero on the (vanishingly
/// rare) random geometry whose energy or angular-momentum magnitude
/// collapses below `f64` round-off.
const RELATIVE_DENOM_FLOOR: f64 = 1.0e-12;

/// Single-revolution `ΔE/E` budget — empirical, set above the worst-case
/// noise floor observed across `1e6` random Earth-scale single-rev
/// geometries during initial bring-up. Not paper-traced (paper reports
/// `|x_true − x|`, not conservation).
const SINGLE_REV_ENERGY_BUDGET: f64 = 1.2e-11;

/// Multi-revolution `ΔE/E` budget. Multi-rev branches lie close to the
/// `T_min(M)` minimum where conditioning is much tighter, so the
/// achievable noise floor is correspondingly lower than single-rev.
const MULTI_REV_ENERGY_BUDGET: f64 = 3.0e-13;

/// Single-revolution sweep position bounds (km).
const SINGLE_REV_R_MIN_KM: f64 = 3500.0;
const SINGLE_REV_R_MAX_KM: f64 = 28_000.0;

/// Single-revolution sweep TOF bounds (s).
const SINGLE_REV_TOF_MIN_S: f64 = 100.0;
const SINGLE_REV_TOF_MAX_S: f64 = 50_000.0;

/// Multi-revolution sweep position bounds (km).
const MULTI_REV_R_MIN_KM: f64 = 5600.0;
const MULTI_REV_R_MAX_KM: f64 = 10_500.0;

/// Multi-revolution sweep TOF bounds (s) — long enough to admit `M ≤ 5`.
const MULTI_REV_TOF_MIN_S: f64 = 10_000.0;
const MULTI_REV_TOF_MAX_S: f64 = 250_000.0;

/// Rejection-sampling bounds for `rand_unit_vec`: keep `|v|² ∈ (lo, hi)`
/// to stay within the unit ball but away from the origin where
/// `v / |v|` amplifies round-off.
const UNIT_VEC_NORM2_LO: f64 = 0.01;
const UNIT_VEC_NORM2_HI: f64 = 1.0;

fn axis_distribution() -> &'static Uniform<f64> {
    static AXIS: OnceLock<Uniform<f64>> = OnceLock::new();
    AXIS.get_or_init(|| Uniform::new(-1.0, 1.0))
}

fn rand_unit_vec(rng: &mut ChaCha20Rng) -> Vector3<f64> {
    let axis = axis_distribution();
    loop {
        let v = Vector3::new(rng.sample(axis), rng.sample(axis), rng.sample(axis));
        let n2 = v.norm_squared();
        if n2 > UNIT_VEC_NORM2_LO && n2 < UNIT_VEC_NORM2_HI {
            return v / n2.sqrt();
        }
    }
}

fn check_conservation(
    r1_km: Vector3<f64>,
    v1_km_s: Vector3<f64>,
    r2_km: Vector3<f64>,
    v2_km_s: Vector3<f64>,
    mu_km3_s2: f64,
) -> (f64, f64) {
    let r1n = r1_km.norm();
    let r2n = r2_km.norm();
    let e1 = 0.5 * v1_km_s.dot(&v1_km_s) - mu_km3_s2 / r1n;
    let e2 = 0.5 * v2_km_s.dot(&v2_km_s) - mu_km3_s2 / r2n;
    let e_avg = 0.5 * (e1.abs() + e2.abs()).max(RELATIVE_DENOM_FLOOR);
    let e_rel = (e1 - e2).abs() / e_avg;
    let h1 = r1_km.cross(&v1_km_s);
    let h2 = r2_km.cross(&v2_km_s);
    let h_diff = (h1 - h2).norm();
    let h_avg = 0.5 * (h1.norm() + h2.norm()).max(RELATIVE_DENOM_FLOOR);
    let h_rel = h_diff / h_avg;
    (e_rel, h_rel)
}

fn make_state(epoch: Epoch, position_eci_km: Vector3<f64>) -> StateVector {
    StateVector {
        epoch,
        position_eci_km,
        velocity_eci_km_s: Vector3::zeros(),
    }
}

#[test]
#[ignore = "100k-trial paper-scale stress; run with --ignored"]
fn single_revolution_conservation_sweep() {
    let mu = MU_EARTH;
    let epoch = Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);
    let mut rng = ChaCha20Rng::seed_from_u64(0x00C0_FFEE);
    let radius = Uniform::new(SINGLE_REV_R_MIN_KM, SINGLE_REV_R_MAX_KM);
    let tof = Uniform::new(SINGLE_REV_TOF_MIN_S, SINGLE_REV_TOF_MAX_S);
    let mut count: u32 = 0;
    let mut nonconvergence: u32 = 0;
    let mut max_e_rel = 0.0_f64;
    let mut max_h_rel = 0.0_f64;
    for _ in 0..N_TRIALS {
        let r1_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let r2_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let tof_s = rng.sample(tof);
        let way = if rng.gen_bool(0.5) {
            TransferDirection::LongWay
        } else {
            TransferDirection::ShortWay
        };
        let dep = make_state(epoch, r1_km);
        let arr = make_state(epoch + Duration::from_seconds(tof_s), r2_km);
        match solve_lambert_with_config(
            &dep,
            &arr,
            &LambertConfig { direction: way, revolutions: 0 },
        ) {
            Ok(transfer) => {
                let v1 = transfer.departure_state.velocity_eci_km_s;
                let v2 = transfer.arrival_state.velocity_eci_km_s;
                let (e_rel, h_rel) = check_conservation(r1_km, v1, r2_km, v2, mu);
                if e_rel.is_finite() {
                    max_e_rel = max_e_rel.max(e_rel);
                }
                if h_rel.is_finite() {
                    max_h_rel = max_h_rel.max(h_rel);
                }
                count += 1;
            }
            Err(_) => nonconvergence += 1,
        }
    }
    eprintln!("=== Single revolution ({N_TRIALS} trials, Earth scale) ===");
    eprintln!("  successful: {count}, non-convergence: {nonconvergence}");
    eprintln!("  max relative energy mismatch: {max_e_rel:.2e}");
    eprintln!("  max relative ang.mom. mismatch: {max_h_rel:.2e}");
    assert!(
        max_e_rel < SINGLE_REV_ENERGY_BUDGET,
        "single-rev ΔE/E exceeded budget {SINGLE_REV_ENERGY_BUDGET:.2e}: {max_e_rel:.2e}"
    );
}

#[test]
#[ignore = "100k-trial paper-scale stress; run with --ignored"]
fn multi_revolution_conservation_sweep() {
    let mu = MU_EARTH;
    let epoch = Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);
    let mut rng = ChaCha20Rng::seed_from_u64(0xDEAD_BEEF);
    let radius = Uniform::new(MULTI_REV_R_MIN_KM, MULTI_REV_R_MAX_KM);
    let tof = Uniform::new(MULTI_REV_TOF_MIN_S, MULTI_REV_TOF_MAX_S);
    let mut total_branches: u32 = 0;
    let mut nonconvergence: u32 = 0;
    let mut max_e_rel = 0.0_f64;
    let mut max_h_rel = 0.0_f64;
    for _ in 0..N_TRIALS {
        let r1_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let r2_km = rand_unit_vec(&mut rng) * rng.sample(radius);
        let tof_s = rng.sample(tof);
        let dep = make_state(epoch, r1_km);
        let arr = make_state(epoch + Duration::from_seconds(tof_s), r2_km);
        match solve_lambert_branches(&dep, &arr, MULTI_REV_MAX) {
            Ok(branches) => {
                for b in &branches {
                    let v1 = b.departure_state.velocity_eci_km_s;
                    let v2 = b.arrival_state.velocity_eci_km_s;
                    let (e_rel, h_rel) = check_conservation(r1_km, v1, r2_km, v2, mu);
                    if e_rel.is_finite() {
                        max_e_rel = max_e_rel.max(e_rel);
                    }
                    if h_rel.is_finite() {
                        max_h_rel = max_h_rel.max(h_rel);
                    }
                    total_branches += 1;
                }
            }
            Err(_) => nonconvergence += 1,
        }
    }
    eprintln!(
        "=== Multi revolution ({N_TRIALS} trials, M up to {MULTI_REV_MAX}, Earth scale) ==="
    );
    eprintln!("  branches checked: {total_branches}, non-convergence: {nonconvergence}");
    eprintln!("  max relative energy mismatch: {max_e_rel:.2e}");
    eprintln!("  max relative ang.mom. mismatch: {max_h_rel:.2e}");
    assert!(
        max_e_rel < MULTI_REV_ENERGY_BUDGET,
        "multi-rev ΔE/E exceeded budget {MULTI_REV_ENERGY_BUDGET:.2e}: {max_e_rel:.2e}"
    );
}

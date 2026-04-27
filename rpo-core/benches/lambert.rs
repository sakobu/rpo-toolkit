//! Criterion benches for the Izzo (2015) Lambert solver.
//!
//! Two halves:
//!
//! 1. **Throughput** (`lambert_throughput` group): per-call timings for the
//!    public solver API at canonical Earth-scale geometries — short-way,
//!    long-way, `Auto`, multi-rev branches.
//!
//! 2. **Paper §5 / Fig. 6 reproduction** (`lambert_paper_metrics` group):
//!    sweeps `N` random `(λ, x_true, M)` and prints the distribution of
//!    `|x_true − x|` and Householder iteration counts to stderr at bench
//!    startup. The criterion measurement on each function still times a
//!    representative call, so regressions in per-call cost still surface;
//!    the printed stats track Izzo's claims (avg 2.1 / 3.3 iters, max
//!    `~7`; max `|x_true − x|` `~1e-11` single, `~1e-8` multi).
//!
//! Run with:
//! ```sh
//! cargo bench -p rpo-core --bench lambert
//! cargo bench -p rpo-core --bench lambert -- lambert_throughput
//! cargo bench -p rpo-core --bench lambert -- lambert_paper_metrics
//! ```

use std::sync::OnceLock;

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use hifitime::{Duration, Epoch};
use nalgebra::Vector3;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rand_distr::Uniform;

use rpo_core::constants::MU_EARTH;
use rpo_core::propagation::lambert::{
    kernel::{solve_x_multi_rev_branches, solve_x_single_rev, x_to_tof, MultiRevBranches},
    solve_lambert_branches, solve_lambert_with_config, LambertConfig, TransferDirection,
};
use rpo_core::types::StateVector;

// =====================================================================
// Sweep sizes
// =====================================================================

/// Single-rev sweep size for the paper §5 / Fig. 6 reproduction. Smaller
/// than the paper's `1e6` to keep bench startup interactive (~1 s).
/// Sufficient to detect order-of-magnitude regressions in the published
/// `~1e-11` accuracy / `2.1`-iter bounds.
const PAPER_TRIALS_SINGLE: u32 = 10_000;

/// Multi-rev sweep size. Smaller because each trial may converge two
/// branches and thus has ~2× the work of single-rev.
const PAPER_TRIALS_MULTI: u32 = 5_000;

// =====================================================================
// Sampling bounds (Izzo §5)
// =====================================================================

const PAPER_LAMBDA_MIN: f64 = -0.999;
const PAPER_LAMBDA_MAX: f64 = 0.999;
const PAPER_X_TRUE_SINGLE_MIN: f64 = -0.99;
const PAPER_X_TRUE_SINGLE_MAX: f64 = 3.0;
const PAPER_X_TRUE_MULTI_MIN: f64 = -0.999;
const PAPER_X_TRUE_MULTI_MAX: f64 = 0.999;
const PAPER_M_CHOICES: [u32; 3] = [1, 2, 3];

// =====================================================================
// Throughput-bench fixtures (Earth scale)
// =====================================================================

const TYPICAL_TOF_S: f64 = 1500.0;
const TYPICAL_R_KM: f64 = 7000.0;
const MULTI_REV_R_KM: f64 = 8000.0;

fn typical_epoch() -> &'static Epoch {
    static EPOCH: OnceLock<Epoch> = OnceLock::new();
    EPOCH.get_or_init(|| Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0))
}

fn typical_short_pair() -> (StateVector, StateVector) {
    let epoch = *typical_epoch();
    let dep = StateVector {
        epoch,
        position_eci_km: Vector3::new(TYPICAL_R_KM, 0.0, 0.0),
        velocity_eci_km_s: Vector3::zeros(),
    };
    let arr = StateVector {
        epoch: epoch + Duration::from_seconds(TYPICAL_TOF_S),
        position_eci_km: Vector3::new(0.0, TYPICAL_R_KM, 0.0),
        velocity_eci_km_s: Vector3::zeros(),
    };
    (dep, arr)
}

fn multi_rev_pair() -> (StateVector, StateVector) {
    let epoch = *typical_epoch();
    let mu = MU_EARTH;
    let period_s = 2.0 * core::f64::consts::PI * (MULTI_REV_R_KM.powi(3) / mu).sqrt();
    let dep = StateVector {
        epoch,
        position_eci_km: Vector3::new(MULTI_REV_R_KM, 0.0, 0.0),
        velocity_eci_km_s: Vector3::zeros(),
    };
    let arr = StateVector {
        epoch: epoch + Duration::from_seconds(5.0 * period_s),
        position_eci_km: Vector3::new(5600.0, 5600.0, 0.0),
        velocity_eci_km_s: Vector3::zeros(),
    };
    (dep, arr)
}

// =====================================================================
// Throughput group
// =====================================================================

fn bench_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("lambert_throughput");

    let (dep_short, arr_short) = typical_short_pair();
    let cfg_short = LambertConfig {
        direction: TransferDirection::ShortWay,
        revolutions: 0,
    };
    let cfg_long = LambertConfig {
        direction: TransferDirection::LongWay,
        revolutions: 0,
    };
    let cfg_auto = LambertConfig {
        direction: TransferDirection::Auto,
        revolutions: 0,
    };

    group.bench_function("short_way_single_rev", |b| {
        b.iter(|| {
            solve_lambert_with_config(black_box(&dep_short), black_box(&arr_short), black_box(&cfg_short))
        });
    });

    group.bench_function("long_way_single_rev", |b| {
        b.iter(|| {
            solve_lambert_with_config(black_box(&dep_short), black_box(&arr_short), black_box(&cfg_long))
        });
    });

    group.bench_function("auto_single_rev", |b| {
        b.iter(|| {
            solve_lambert_with_config(black_box(&dep_short), black_box(&arr_short), black_box(&cfg_auto))
        });
    });

    let (dep_multi, arr_multi) = multi_rev_pair();
    group.bench_function("branches_M3", |b| {
        b.iter(|| solve_lambert_branches(black_box(&dep_multi), black_box(&arr_multi), black_box(3)));
    });
    group.bench_function("branches_M5", |b| {
        b.iter(|| solve_lambert_branches(black_box(&dep_multi), black_box(&arr_multi), black_box(5)));
    });

    // Kernel-level: pure Householder iteration without geometry construction.
    group.bench_function("kernel_solve_x_single_rev", |b| {
        b.iter(|| solve_x_single_rev(black_box(0.5), black_box(2.0)));
    });

    group.finish();
}

// =====================================================================
// Paper §5 / Fig. 6 metrics group
// =====================================================================

#[derive(Default)]
struct SweepStats {
    converged: u32,
    max_x_err: f64,
    sum_x_err: f64,
    max_iters: u32,
    sum_iters: u64,
}

impl SweepStats {
    fn record(&mut self, x_err: f64, iters: u32) {
        if !x_err.is_finite() {
            return;
        }
        self.converged += 1;
        self.max_x_err = self.max_x_err.max(x_err);
        self.sum_x_err += x_err;
        self.max_iters = self.max_iters.max(iters);
        self.sum_iters += u64::from(iters);
    }

    fn print(&self, label: &str, total: u32) {
        let mean_err = if self.converged > 0 {
            self.sum_x_err / f64::from(self.converged)
        } else {
            f64::NAN
        };
        let mean_iters = if self.converged > 0 {
            self.sum_iters as f64 / f64::from(self.converged)
        } else {
            f64::NAN
        };
        eprintln!(
            "  {label}: converged {c}/{total}, |x_true-x| max {mx:.2e} mean {me:.2e}, \
             Householder iters mean {mi:.2} max {mxi}",
            c = self.converged,
            mx = self.max_x_err,
            me = mean_err,
            mi = mean_iters,
            mxi = self.max_iters,
        );
    }
}

fn run_single_rev_sweep(n: u32) -> SweepStats {
    let mut rng = ChaCha20Rng::seed_from_u64(0x5EED_F161);
    let lambda_dist = Uniform::new(PAPER_LAMBDA_MIN, PAPER_LAMBDA_MAX);
    let x_true_dist = Uniform::new(PAPER_X_TRUE_SINGLE_MIN, PAPER_X_TRUE_SINGLE_MAX);
    let mut stats = SweepStats::default();
    for _ in 0..n {
        let lambda = rng.sample(lambda_dist);
        let x_true = rng.sample(x_true_dist);
        let big_t = x_to_tof(x_true, lambda, 0);
        if !big_t.is_finite() || big_t <= 0.0 {
            continue;
        }
        if let Ok(root) = solve_x_single_rev(lambda, big_t) {
            stats.record((x_true - root.x).abs(), root.iterations);
        }
    }
    stats
}

fn run_multi_rev_sweep(n: u32) -> SweepStats {
    let mut rng = ChaCha20Rng::seed_from_u64(0x5EED_F162);
    let lambda_dist = Uniform::new(PAPER_LAMBDA_MIN, PAPER_LAMBDA_MAX);
    let x_true_dist = Uniform::new(PAPER_X_TRUE_MULTI_MIN, PAPER_X_TRUE_MULTI_MAX);
    let mut stats = SweepStats::default();
    for _ in 0..n {
        let lambda = rng.sample(lambda_dist);
        let x_true = rng.sample(x_true_dist);
        let m = PAPER_M_CHOICES[rng.gen_range(0..PAPER_M_CHOICES.len())];
        let big_t = x_to_tof(x_true, lambda, m);
        if !big_t.is_finite() || big_t <= 0.0 {
            continue;
        }
        let MultiRevBranches { long_period, short_period } =
            solve_x_multi_rev_branches(lambda, big_t, m);
        // Take the branch closer to x_true (the side of x_min that x_true lies on).
        let candidate = match (long_period, short_period) {
            (Some(l), Some(r)) => {
                if (l.x - x_true).abs() <= (r.x - x_true).abs() {
                    Some(l)
                } else {
                    Some(r)
                }
            }
            (Some(p), None) | (None, Some(p)) => Some(p),
            (None, None) => None,
        };
        if let Some(root) = candidate {
            stats.record((x_true - root.x).abs(), root.iterations);
        }
    }
    stats
}

fn bench_paper_metrics(c: &mut Criterion) {
    eprintln!("\n=== Izzo §5 / Fig. 6 paper-metric sweep ===");
    eprintln!(
        "Izzo claims: single-rev mean iter ~2.1 max ~7 / max |x_true-x| ~1e-11; \
         multi-rev mean iter ~3.3 / max |x_true-x| ~1e-8"
    );

    let single = run_single_rev_sweep(PAPER_TRIALS_SINGLE);
    single.print("single-rev", PAPER_TRIALS_SINGLE);

    let multi = run_multi_rev_sweep(PAPER_TRIALS_MULTI);
    multi.print("multi-rev ", PAPER_TRIALS_MULTI);
    eprintln!();

    // Criterion measurement: time one representative kernel call so
    // per-call regressions still surface in the bench output.
    let mut group = c.benchmark_group("lambert_paper_metrics");
    group.bench_function("solve_x_single_rev_typical", |b| {
        b.iter(|| solve_x_single_rev(black_box(0.5), black_box(2.0)));
    });
    group.bench_function("solve_x_multi_rev_typical_M2", |b| {
        // λ=0.5, x_true=0.3, M=2 → big_t computed once, then solve.
        let big_t = x_to_tof(0.3, 0.5, 2);
        b.iter(|| solve_x_multi_rev_branches(black_box(0.5), black_box(big_t), black_box(2)));
    });
    group.finish();
}

criterion_group!(benches, bench_throughput, bench_paper_metrics);
criterion_main!(benches);

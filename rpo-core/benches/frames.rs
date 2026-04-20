//! Criterion benchmarks for ECI↔ECEF frame transforms.
//!
//! Purpose: pin the amortization win of [`EciEcefTransform::at`] + N
//! [`forward_state`](EciEcefTransform::forward_state) calls over N per-call
//! invocations of the [`eci_to_ecef_state_km`] free function. The per-call
//! form recomputes the Earth Rotation Angle polynomial + sin/cos on every
//! invocation; the cached form does it once and amortizes across the batch.
//!
//! Workload sizes (100 / 1_000 / 10_000) match realistic consumers:
//! the ground-track view samples trajectories at ~200 points per tick,
//! and the contact-windows grid sweep hits `N_samples × N_stations`.
//!
//! Benchmark files cannot use `test_helpers` (not compiled with `cfg(test)`),
//! so fixtures are inline (same convention as `analytical.rs`).

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use hifitime::Epoch;
use nalgebra::Vector3;

use rpo_core::elements::eci_ecef::{eci_to_ecef_state_km, EciEcefTransform};
use rpo_core::types::StateVector;

/// Reference epoch for the batch: a 2024-01-01T00:00 UTC sample used
/// throughout the workspace. The absolute epoch is immaterial — the ERA
/// computation is what's being measured.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0)
}

/// Generate `n` distinct-looking ECI states sharing a single epoch.
///
/// Offsets the baseline LEO position by 1 m per sample along X so that
/// consecutive states are observably different — guards against the
/// optimizer collapsing the batch into a single matrix-vector op.
fn sample_states(n: usize) -> Vec<StateVector> {
    let ep = epoch();
    (0..n)
        .map(|i| StateVector {
            epoch: ep,
            position_eci_km: Vector3::new(7000.0 + (i as f64) * 0.001, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, 7.5, 0.0),
        })
        .collect()
}

/// Per-call free function: each iteration re-derives the ERA + sin/cos.
fn bench_eci_to_ecef_per_call(c: &mut Criterion) {
    let mut group = c.benchmark_group("eci_to_ecef_per_call_free_fn");
    for &n in &[100usize, 1_000, 10_000] {
        let states = sample_states(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &states, |b, states| {
            b.iter(|| {
                for s in states {
                    let _ = eci_to_ecef_state_km(s);
                }
            });
        });
    }
    group.finish();
}

/// Cached transform: ERA computed once, then N cheap matrix-vector applications.
fn bench_eci_to_ecef_cached_transform(c: &mut Criterion) {
    let mut group = c.benchmark_group("eci_to_ecef_cached_transform");
    for &n in &[100usize, 1_000, 10_000] {
        let states = sample_states(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &states, |b, states| {
            b.iter(|| {
                let xform = EciEcefTransform::at(epoch());
                for s in states {
                    let _ = xform.forward_state(s);
                }
            });
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_eci_to_ecef_per_call,
    bench_eci_to_ecef_cached_transform,
);
criterion_main!(benches);

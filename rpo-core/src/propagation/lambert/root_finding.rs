//! Householder + Halley iteration on `T(x, λ, M) = T_target`.
//!
//! Drives the search from the analytic initial guesses (Izzo Eqs. 30, 31)
//! to converged `(x, y)` pairs for every reachable revolution count.

use core::f64::consts::PI;

use super::geometry::Geometry;
use super::tof::{compute_y, tof_derivatives, x_to_tof};
use super::{
    LambertError, HALLEY_MAX_ITERS, HALLEY_TOL, HOUSEHOLDER_DENOM_EPS, HOUSEHOLDER_MAX_ITERS,
    HOUSEHOLDER_TOL_MULTI, HOUSEHOLDER_TOL_SINGLE,
};

/// One converged Householder root for a given branch.
#[derive(Debug, Clone, Copy)]
pub(super) struct Root {
    /// Lancaster–Blanchard `x` (Izzo's free parameter, dimensionless).
    pub x: f64,
    /// Companion variable `y = sqrt(1 − λ²(1 − x²))` (Izzo Eq. 7).
    pub y: f64,
    /// Branch revolution count: `0` = single-rev, `≥ 1` = multi-rev.
    pub n_revs: u32,
}

/// One Householder convergence record (Izzo Eq. 32).
///
/// Returned by every kernel entry point that drives the Householder
/// iteration so paper-fidelity benchmarks (Izzo §5 / Fig. 6) can track
/// `x` and the iteration count taken to reach the tolerance.
#[derive(Debug, Clone, Copy)]
pub struct HouseholderRoot {
    /// Converged Lancaster–Blanchard `x` (Izzo's free parameter, dimensionless).
    pub x: f64,
    /// Iterations taken to reach `HOUSEHOLDER_TOL_*`.
    pub iterations: u32,
}

/// Both Householder branches for a multi-rev `T(x, λ, M) = big_t` problem.
///
/// `None` indicates the corresponding branch failed to converge — the
/// kernel benches treat per-branch failure as a partial answer rather
/// than an error, since the other branch may still be usable.
#[derive(Debug, Clone, Copy)]
pub struct MultiRevBranches {
    /// Long-period asymptote (Izzo Eq. 31, `x_left` initial guess).
    pub long_period: Option<HouseholderRoot>,
    /// Short-period asymptote (Izzo Eq. 31, `x_right` initial guess).
    pub short_period: Option<HouseholderRoot>,
}

/// Find the single-revolution root only. Always exists for valid geometry.
///
/// Used by the dominant call shape (`solve_lambert` / direct transfers) to
/// avoid the `Vec` allocation that the multi-rev path requires.
///
/// # Errors
///
/// Propagates [`LambertError::NoConvergence`] /
/// [`LambertError::SingularDenominator`] from the Householder iteration.
pub(super) fn find_xy_single(geom: &Geometry) -> Result<Root, LambertError> {
    let lambda = geom.lambda;
    let big_t = geom.big_t;
    debug_assert!(lambda.abs() < 1.0);
    debug_assert!(big_t > 0.0);

    let t00 = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
    let t1 = (2.0 / 3.0) * (1.0 - lambda * lambda * lambda);
    let x0 = initial_guess_single_rev(big_t, t00, t1, lambda);
    let x = householder(x0, big_t, lambda, 0)?;
    let y = compute_y(x, lambda);
    Ok(Root { x, y, n_revs: 0 })
}

/// Find every reachable `(x, y)` root for the given geometry and revolution
/// budget.
///
/// Returns the single-rev root first, followed by `(long-period,
/// short-period)` pairs for each `M` from `1` up to `min(max_revs, ⌊T/π⌋)`.
///
/// # Errors
///
/// Propagates [`LambertError::NoConvergence`] /
/// [`LambertError::SingularDenominator`] from the underlying Householder
/// or Halley iterations.
#[allow(clippy::similar_names)]
pub(super) fn find_xy_branches(
    geom: &Geometry,
    max_revs: u32,
) -> Result<Vec<Root>, LambertError> {
    let single = find_xy_single(geom)?;
    let cap = 1 + 2 * (max_revs as usize); // u32 → usize: always safe (usize ≥ 32 bits)
    let mut out = Vec::with_capacity(cap);
    out.push(single);

    let lambda = geom.lambda;
    let big_t = geom.big_t;
    let t00 = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();

    // Multi-rev branches. Iterate upward from M = 1; stop at the first
    // infeasible branch. T_min(M) is monotonically increasing in M, so once
    // a branch fails, all higher branches also fail. The Halley T_min check
    // only fires on the boundary branch where big_t lies below the analytic
    // minimum T(x=0, M) = t00 + M·π — at most one Halley call per call.
    for m in 1..=max_revs {
        let m_pi = f64::from(m) * PI;

        // Quick reject: T_min(M) ≥ M·π always, so big_t < M·π ⇒ infeasible.
        if big_t < m_pi {
            break;
        }

        // Boundary regime: big_t below the analytic minimum at x = 0.
        // Confirm numerical T_min(M) ≤ big_t; otherwise drop the branch
        // (and all higher M, which have higher T_min).
        if big_t < t00 + m_pi {
            let (_x_min, t_min) = halley_t_min(lambda, m)?;
            if t_min > big_t {
                break;
            }
        }

        let (x0l, x0r) = initial_guess_multi_rev(big_t, m);
        let xl = householder(x0l, big_t, lambda, m)?;
        let yl = compute_y(xl, lambda);
        let xr = householder(x0r, big_t, lambda, m)?;
        let yr = compute_y(xr, lambda);
        out.push(Root { x: xl, y: yl, n_revs: m });
        out.push(Root { x: xr, y: yr, n_revs: m });
    }
    Ok(out)
}

/// Single-rev initial guess (Izzo Eq. 30 + the derivative-matched
/// hyperbolic starter just after).
fn initial_guess_single_rev(big_t: f64, t0: f64, t1: f64, lambda: f64) -> f64 {
    if big_t >= t0 {
        // T ≥ T0: high-energy / slow-transfer side.
        (t0 / big_t).powf(2.0 / 3.0) - 1.0
    } else if big_t <= t1 {
        // T ≤ T1: hyperbolic side; derivative-matched starter.
        2.5 * t1 * (t1 - big_t) / (big_t * (1.0 - lambda.powi(5))) + 1.0
    } else {
        // T1 < T < T0: log-linear interpolation in (ξ, τ) plane.
        // Paper Eq. 30 displays the middle branch as `(T0/T)^(log2(T1/T0)) − 1`,
        // which fails the boundary x = 1 at T = T1 (a known typo). The textual
        // derivation just before Eq. 30 (page 12) gives the correct form below;
        // PyKEP uses the same formulation.
        (t0 / big_t).powf(2_f64.ln() / (t0 / t1).ln()) - 1.0
    }
}

/// Multi-rev initial guesses (Izzo Eq. 31). Returns `(left, right)` —
/// long-period and short-period asymptotes respectively.
#[allow(clippy::similar_names)]
fn initial_guess_multi_rev(big_t: f64, m: u32) -> (f64, f64) {
    let m_pi = f64::from(m) * PI;
    let kl = ((m_pi + PI) / (8.0 * big_t)).powf(2.0 / 3.0);
    let x0l = (kl - 1.0) / (kl + 1.0);
    let kr = ((8.0 * big_t) / m_pi).powf(2.0 / 3.0);
    let x0r = (kr - 1.0) / (kr + 1.0);
    (x0l, x0r)
}

/// Householder's 3rd-order iteration for `T(x) − T_target = 0` (Izzo Eq. 32).
fn householder(x: f64, big_t: f64, lambda: f64, m: u32) -> Result<f64, LambertError> {
    householder_counted(x, big_t, lambda, m).map(|root| root.x)
}

/// Householder iteration that also returns the iteration count to
/// convergence. Used by paper §5 / Fig. 6 telemetry benchmarks; production
/// callers go through [`householder`] which discards the count. Izzo §5
/// claims mean 2.1 (single-rev) / 3.3 (multi-rev) and max ~7 across `1e6`
/// random geometries — the benchmarks verify our implementation tracks
/// those numbers.
///
/// # Errors
///
/// - [`LambertError::SingularDenominator`] when the Householder denominator
///   collapses (algebraic singularity at multi-rev `T_min` boundary).
/// - [`LambertError::NoConvergence`] when [`HOUSEHOLDER_MAX_ITERS`] iters
///   pass without `|Δx| < HOUSEHOLDER_TOL_*`.
#[allow(clippy::similar_names)]
pub fn householder_counted(
    mut x: f64,
    big_t: f64,
    lambda: f64,
    m: u32,
) -> Result<HouseholderRoot, LambertError> {
    let tol = if m == 0 {
        HOUSEHOLDER_TOL_SINGLE
    } else {
        HOUSEHOLDER_TOL_MULTI
    };
    let mut last_step = f64::INFINITY;
    for i in 0..HOUSEHOLDER_MAX_ITERS {
        let tof = x_to_tof(x, lambda, m);
        let (dt, ddt, dddt) = tof_derivatives(x, lambda, tof);
        let f = tof - big_t;
        let denom = dt * (dt * dt - f * ddt) + dddt * f * f / 6.0;
        if denom.abs() < HOUSEHOLDER_DENOM_EPS {
            return Err(LambertError::SingularDenominator { n_revs: m });
        }
        let delta = f * (dt * dt - 0.5 * f * ddt) / denom;
        x -= delta;
        last_step = delta.abs();
        if last_step < tol {
            return Ok(HouseholderRoot { x, iterations: i + 1 });
        }
    }
    Err(LambertError::NoConvergence {
        iterations: HOUSEHOLDER_MAX_ITERS,
        last_step,
        n_revs: m,
    })
}

/// Solve `T(x, λ, 0) = big_t` for the single-rev `x` directly, without
/// constructing a full [`Geometry`]. Returns `(x, iter_count)` so paper-
/// fidelity benchmarks can track Izzo §5 / Fig. 6 metrics directly.
///
/// # Errors
///
/// Propagates the [`householder_counted`] failure modes
/// ([`LambertError::SingularDenominator`], [`LambertError::NoConvergence`]).
pub fn solve_x_single_rev(
    lambda: f64,
    big_t: f64,
) -> Result<HouseholderRoot, LambertError> {
    let t00 = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
    let t1 = (2.0 / 3.0) * (1.0 - lambda * lambda * lambda);
    let x0 = initial_guess_single_rev(big_t, t00, t1, lambda);
    householder_counted(x0, big_t, lambda, 0)
}

/// Solve `T(x, λ, M) = big_t` for both multi-rev branches.
///
/// Each branch field is `Some(root)` if Householder converged on that
/// asymptote, `None` otherwise.
#[allow(clippy::similar_names)]
#[must_use]
pub fn solve_x_multi_rev_branches(
    lambda: f64,
    big_t: f64,
    m: u32,
) -> MultiRevBranches {
    let (x0l, x0r) = initial_guess_multi_rev(big_t, m);
    let long_period = householder_counted(x0l, big_t, lambda, m).ok();
    let short_period = householder_counted(x0r, big_t, lambda, m).ok();
    MultiRevBranches { long_period, short_period }
}

/// Locate the minimum of `T(x)` along the M-revolution branch starting at
/// `x = 0` via Halley iteration on `dT/dx = 0`. Returns `(x_min, T_min)`.
///
/// `T_min(M)` is the feasibility gate for branch `M`: a multi-rev branch
/// admits a solution iff `big_t ≥ T_min(M)`. A singular denominator at
/// any iterate is a hard algebraic failure — propagate as
/// [`LambertError::SingularDenominator`] rather than silently exit with
/// a stale `T_min` (which could mis-classify an infeasible branch as
/// feasible or vice versa).
#[allow(clippy::similar_names)]
fn halley_t_min(lambda: f64, m: u32) -> Result<(f64, f64), LambertError> {
    let mut x = 0.0;
    for _ in 0..HALLEY_MAX_ITERS {
        let tof = x_to_tof(x, lambda, m);
        let (dt, ddt, dddt) = tof_derivatives(x, lambda, tof);
        let denom = 2.0 * ddt * ddt - dt * dddt;
        if denom.abs() < HOUSEHOLDER_DENOM_EPS {
            return Err(LambertError::SingularDenominator { n_revs: m });
        }
        let delta = 2.0 * dt * ddt / denom;
        x -= delta;
        if delta.abs() < HALLEY_TOL {
            break;
        }
    }
    let t_min = x_to_tof(x, lambda, m);
    Ok((x, t_min))
}

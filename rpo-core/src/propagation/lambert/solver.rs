//! Public Lambert solver entry points: `solve_lambert`, `solve_lambert_with_config`,
//! `solve_lambert_branches`.
//!
//! Bridges the workspace's [`StateVector`] inputs to the algorithmic kernels
//! in [`super::geometry`], [`super::tof`], and [`super::root_finding`].

use nalgebra::Vector3;

use crate::constants::MU_EARTH;
use crate::types::StateVector;

use super::geometry::Geometry;
use super::root_finding::{find_xy_branches, find_xy_single, Root};
use super::{
    LambertConfig, LambertError, LambertTransfer, TransferDirection, TransferFeasibility,
};

/// Solve Lambert's problem between two ECI states using the default
/// [`LambertConfig`] (single-revolution, [`TransferDirection::Auto`]).
///
/// Mathematically frame-invariant — the algorithm only requires that
/// `departure.position_eci_km`, `arrival.position_eci_km` are in the same
/// inertial frame. ECI is the workspace convention.
///
/// # Validity
///
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - `|departure.position_eci_km|`, `|arrival.position_eci_km|` strictly
///   above [`super::LAMBERT_DEGENERATE_NORM_KM`]
/// - Transfer angle `θ ∉ {0, π}` (collinear `r1`, `r2` are rejected)
///
/// # Singularities
///
/// - `θ → 0` or `θ → π`: surfaces as [`LambertError::CollinearGeometry`].
/// - Near-parabolic regime (`x → 1`): handled internally by the Battin
///   hypergeometric branch; no caller-visible singularity.
/// - Multi-rev `T_min(M)`: requesting a rev count above the maximum
///   feasible for the geometry returns
///   [`LambertError::NoSolutionForRevolutions`] with the highest rev count
///   that does have a solution.
///
/// # Errors
///
/// Returns [`LambertError`] on any input-validation or solver-convergence failure.
pub fn solve_lambert(
    departure: &StateVector,
    arrival: &StateVector,
) -> Result<LambertTransfer, LambertError> {
    solve_lambert_with_config(departure, arrival, &LambertConfig::default())
}

/// Solve Lambert's problem with explicit configuration.
///
/// For `config.revolutions == 0`, returns the single-revolution transfer for
/// the requested direction. For `config.revolutions == N >= 1`, returns the
/// long-period (low-energy) branch of the M=N multi-revolution family.
///
/// `TransferDirection::Auto` runs the solver in both `ShortWay` and `LongWay`
/// configurations and returns whichever produced the lower total Δv. If only
/// one direction yields a solution, that one is returned. If both fail, the
/// short-way error is returned (canonical default; the dual-failure case is
/// rare in practice — re-issue with explicit `ShortWay`/`LongWay` to inspect
/// the per-direction diagnostic).
///
/// # Validity
///
/// - `arrival.epoch > departure.epoch`
/// - Position vectors non-degenerate, non-collinear
/// - For `revolutions >= 1`, `tof_s` must satisfy
///   `tof_s >= T_min(M, λ)`; otherwise [`LambertError::NoSolutionForRevolutions`]
///   is returned, naming the maximum feasible rev count.
///
/// # Errors
///
/// Returns [`LambertError`] on input validation, solver-convergence, or
/// rev-count infeasibility.
pub fn solve_lambert_with_config(
    departure: &StateVector,
    arrival: &StateVector,
    config: &LambertConfig,
) -> Result<LambertTransfer, LambertError> {
    let tof_s = (arrival.epoch - departure.epoch).to_seconds();

    match config.direction {
        TransferDirection::Auto => {
            let (geom_short, geom_long) = Geometry::pair_from_inputs(
                departure.position_eci_km,
                arrival.position_eci_km,
                tof_s,
                MU_EARTH,
            )?;
            let short = solve_with_geometry(
                &geom_short,
                departure,
                arrival,
                tof_s,
                TransferDirection::ShortWay,
                config.revolutions,
            );
            let long = solve_with_geometry(
                &geom_long,
                departure,
                arrival,
                tof_s,
                TransferDirection::LongWay,
                config.revolutions,
            );
            match (short, long) {
                (Ok(s), Ok(l)) => Ok(if s.total_dv_km_s <= l.total_dv_km_s { s } else { l }),
                (Ok(s), Err(_)) => Ok(s),
                (Err(_), Ok(l)) => Ok(l),
                (Err(e), Err(_)) => Err(e),
            }
        }
        dir => {
            let geom = Geometry::from_inputs(
                departure.position_eci_km,
                arrival.position_eci_km,
                tof_s,
                MU_EARTH,
                dir,
            )?;
            solve_with_geometry(&geom, departure, arrival, tof_s, dir, config.revolutions)
        }
    }
}

/// Return every Lambert branch up to `max_revs` revolutions, sorted by total
/// Δv ascending.
///
/// Computes both short-way and long-way families and concatenates them. For
/// `max_revs == 0` this is two solutions (short + long single-rev). For
/// `max_revs == M`, up to `2 · (1 + 2M)` branches when every multi-rev branch
/// is feasible — fewer when `tof_s` falls below `T_min(m, λ)` for some `m`.
///
/// # Errors
///
/// Returns [`LambertError`] only when *both* directions fail; partial
/// results are sorted and returned otherwise.
pub fn solve_lambert_branches(
    departure: &StateVector,
    arrival: &StateVector,
    max_revs: u8,
) -> Result<Vec<LambertTransfer>, LambertError> {
    let tof_s = (arrival.epoch - departure.epoch).to_seconds();
    let (geom_short, geom_long) = Geometry::pair_from_inputs(
        departure.position_eci_km,
        arrival.position_eci_km,
        tof_s,
        MU_EARTH,
    )?;

    let short = all_branches_with_geometry(
        &geom_short,
        departure,
        arrival,
        tof_s,
        TransferDirection::ShortWay,
        max_revs,
    );
    let long = all_branches_with_geometry(
        &geom_long,
        departure,
        arrival,
        tof_s,
        TransferDirection::LongWay,
        max_revs,
    );

    let mut combined: Vec<LambertTransfer> = match (short, long) {
        (Ok(mut s), Ok(mut l)) => {
            s.append(&mut l);
            s
        }
        (Ok(s), Err(_)) => s,
        (Err(_), Ok(l)) => l,
        (Err(e), Err(_)) => return Err(e),
    };

    if combined.len() > 1 {
        combined.sort_by(|a, b| {
            a.total_dv_km_s
                .partial_cmp(&b.total_dv_km_s)
                .unwrap_or(core::cmp::Ordering::Equal)
        });
    }
    Ok(combined)
}

/// Solve a single direction (Short or Long) given a pre-built geometry, and
/// pick the branch matching `revolutions`.
fn solve_with_geometry(
    geom: &Geometry,
    departure: &StateVector,
    arrival: &StateVector,
    tof_s: f64,
    direction: TransferDirection,
    revolutions: u8,
) -> Result<LambertTransfer, LambertError> {
    let root = if revolutions == 0 {
        find_xy_single(geom)?
    } else {
        let roots = find_xy_branches(geom, u32::from(revolutions))?;
        match select_branch(&roots, revolutions) {
            BranchSelection::Found(r) => r,
            BranchSelection::Missing { max_feasible } => {
                return Err(LambertError::NoSolutionForRevolutions {
                    requested: revolutions,
                    max_feasible,
                });
            }
        }
    };

    let (v1_eci_km_s, v2_eci_km_s) = velocities_from_root(geom, root.x, root.y);
    build_transfer(
        departure,
        arrival,
        v1_eci_km_s,
        v2_eci_km_s,
        tof_s,
        direction,
    )
}

/// Compute every reachable branch in one direction, returning a `Vec<LambertTransfer>`.
fn all_branches_with_geometry(
    geom: &Geometry,
    departure: &StateVector,
    arrival: &StateVector,
    tof_s: f64,
    direction: TransferDirection,
    max_revs: u8,
) -> Result<Vec<LambertTransfer>, LambertError> {
    let roots = find_xy_branches(geom, u32::from(max_revs))?;
    roots
        .into_iter()
        .map(|r| {
            let (v1, v2) = velocities_from_root(geom, r.x, r.y);
            build_transfer(departure, arrival, v1, v2, tof_s, direction)
        })
        .collect()
}

enum BranchSelection {
    Found(Root),
    Missing { max_feasible: u8 },
}

/// Find the requested-rev root in `roots`. If absent, return the highest
/// rev count that does appear (or `0` if `roots` is empty, which is
/// unreachable since `find_xy_branches` always pushes the single-rev root).
fn select_branch(roots: &[Root], revolutions: u8) -> BranchSelection {
    let target = u32::from(revolutions);
    if let Some(r) = roots.iter().find(|r| r.n_revs == target) {
        return BranchSelection::Found(*r);
    }
    let max_feasible_u32 = roots.iter().map(|r| r.n_revs).max().unwrap_or(0);
    // `find_xy_branches(geom, target)` only emits roots with `n_revs ≤ target`,
    // so `max_feasible_u32 ≤ u32::from(revolutions)` and the `try_from` is
    // never wider than `u8`. The `unwrap_or(revolutions)` saturates to the
    // requested count rather than `u8::MAX`, preserving the invariant
    // `max_feasible <= requested` if upstream code ever loosens that bound.
    debug_assert!(max_feasible_u32 <= target);
    let max_feasible = u8::try_from(max_feasible_u32).unwrap_or(revolutions);
    BranchSelection::Missing { max_feasible }
}

/// Velocity reconstruction at `r1` and `r2` from the converged `(x, y)`
/// (Izzo Eq. 23; the radial/tangential decomposition spelled out in §3 of
/// the paper, just before Eq. 24).
///
/// Builds the radial and tangential velocity components in the basis
/// `(ir, it)`, then assembles the ECI-frame velocity vectors.
#[allow(clippy::similar_names)]
fn velocities_from_root(geom: &Geometry, x: f64, y: f64) -> (Vector3<f64>, Vector3<f64>) {
    let lambda_y_minus_x = geom.lambda * y - x;
    let lambda_y_plus_x = geom.lambda * y + x;
    let tangential_num = geom.gamma * geom.sigma * (y + geom.lambda * x);

    let v_r1 = geom.gamma * (lambda_y_minus_x - geom.rho * lambda_y_plus_x) / geom.r1n;
    let v_r2 = -geom.gamma * (lambda_y_minus_x + geom.rho * lambda_y_plus_x) / geom.r2n;
    let v_t1 = tangential_num / geom.r1n;
    let v_t2 = tangential_num / geom.r2n;

    let v1 = geom.ir1 * v_r1 + geom.it1 * v_t1;
    let v2 = geom.ir2 * v_r2 + geom.it2 * v_t2;
    (v1, v2)
}

/// One raw Lambert solution: velocities, branch index, and the converged
/// Lancaster–Blanchard `x`. Test-only diagnostic shape.
#[cfg(test)]
#[derive(Debug, Clone, Copy)]
pub(super) struct RawSolution {
    /// Velocity at `r1` (km/s, same frame as inputs).
    pub v1_km_s: Vector3<f64>,
    /// Velocity at `r2` (km/s, same frame as inputs).
    pub v2_km_s: Vector3<f64>,
    /// Revolution count: `0` = single-rev, `≥ 1` = multi-rev.
    pub n_revs: u32,
    /// Converged free parameter `x` (Izzo §3, dimensionless). Useful for
    /// branch identification and regime checks (Battin / Lancaster /
    /// Lagrange / hyperbolic).
    pub x: f64,
}

/// Test-only kernel access: take raw `(r1, r2, tof, mu)` and return all
/// branches without wrapping into [`LambertTransfer`]. Lets the in-tree
/// test suite exercise the algorithm at any μ (Earth, Sun, …) without
/// forcing a `mu_km3_s2` parameter onto the public API.
#[cfg(test)]
pub(super) fn lambert_raw(
    r1_km: Vector3<f64>,
    r2_km: Vector3<f64>,
    tof_s: f64,
    mu_km3_s2: f64,
    direction: TransferDirection,
    max_revs: u32,
) -> Result<Vec<RawSolution>, LambertError> {
    let geom = Geometry::from_inputs(r1_km, r2_km, tof_s, mu_km3_s2, direction)?;
    let roots = find_xy_branches(&geom, max_revs)?;
    Ok(roots
        .into_iter()
        .map(|r| {
            let (v1, v2) = velocities_from_root(&geom, r.x, r.y);
            RawSolution { v1_km_s: v1, v2_km_s: v2, n_revs: r.n_revs, x: r.x }
        })
        .collect())
}

/// Wrap solver outputs into the public [`LambertTransfer`] type with Δv,
/// feasibility, and specific orbital energy populated.
///
/// The geometry validation in [`Geometry::from_inputs`] guarantees
/// `|r1| ≥ LAMBERT_DEGENERATE_NORM_KM`, so the
/// [`TransferFeasibility::from_state`] call below never errors on the
/// solver path; the `?` propagation is for the (test-only) callers that
/// might pass synthetic inputs.
fn build_transfer(
    departure: &StateVector,
    arrival: &StateVector,
    v1_eci_km_s: Vector3<f64>,
    v2_eci_km_s: Vector3<f64>,
    tof_s: f64,
    direction: TransferDirection,
) -> Result<LambertTransfer, LambertError> {
    let departure_dv = v1_eci_km_s - departure.velocity_eci_km_s;
    let arrival_dv = arrival.velocity_eci_km_s - v2_eci_km_s;
    let total_dv = departure_dv.norm() + arrival_dv.norm();
    // Specific orbital energy ε = v²/2 − μ/r is conserved on a two-body
    // conic, so `r1` and `r2` evaluations agree up to round-off; we sample
    // at `r1` for a single, well-defined value. Sign convention:
    // ε < 0 → elliptic, ε = 0 → parabolic, ε > 0 → hyperbolic. For
    // hyperbolic transfers `v∞² = 2·ε`; sub-parabolic transfers have no
    // physical `v∞` and consumers should branch on `ε ≥ 0` before
    // interpreting it as `C₃`.
    let r1_km_norm = departure.position_eci_km.norm();
    let specific_energy_km2_s2 = 0.5 * v1_eci_km_s.norm_squared() - MU_EARTH / r1_km_norm;
    let feasibility = TransferFeasibility::from_state(departure.position_eci_km, v1_eci_km_s)?;

    Ok(LambertTransfer {
        departure_state: StateVector {
            epoch: departure.epoch,
            position_eci_km: departure.position_eci_km,
            velocity_eci_km_s: v1_eci_km_s,
        },
        arrival_state: StateVector {
            epoch: arrival.epoch,
            position_eci_km: arrival.position_eci_km,
            velocity_eci_km_s: v2_eci_km_s,
        },
        departure_dv_eci_km_s: departure_dv,
        arrival_dv_eci_km_s: arrival_dv,
        total_dv_km_s: total_dv,
        tof_s,
        specific_energy_km2_s2,
        direction,
        feasibility,
    })
}

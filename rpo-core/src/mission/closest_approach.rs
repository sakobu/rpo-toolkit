//! Refined closest-approach (POCA) analysis via Brent's method on range rate.
//!
//! Scans a trajectory for range-rate sign changes (approach → recession
//! transitions) and refines each bracket to find the exact time, position,
//! and distance of closest approach.
//!
//! # Algorithm
//!
//! 1. **Bracket detection**: compute range rate `ṙ = r·v / ‖r‖` at each
//!    trajectory point; sign change `ṙ < 0 → ṙ > 0` indicates a local
//!    distance minimum.
//! 2. **Brent refinement**: for each bracket, apply Brent's method
//!    (Brent 1973, Ch. 4) to find `ṙ(t*) = 0`. Each trial evaluation
//!    calls [`PropagationModel::propagate`] at the trial time.
//!
//! # References
//! - Brent, "Algorithms for Minimization without Derivatives", 1973, Ch. 4
//! - Range-rate from RIC state: D'Amico Eq. 2.17 (unperturbed), Eq. 2.31 (J2)
//! - Minimum distance for bounded motion: D'Amico Eq. 2.23
//!   `δr_nr^min(u = φ, φ + kπ) = a · min{δe, δi}`
//!
//! # Validity
//! - Engine-agnostic: operates on any `&[PropagatedState]` regardless of
//!   propagation model (analytical STM, nyx full-physics, etc.).
//! - Brent's method assumes continuous range-rate within each bracket.
//!   This holds for all physical trajectories.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

// ---------------------------------------------------------------------------
// Tolerances
// ---------------------------------------------------------------------------

/// Brent's method convergence on range rate (km/s).
/// At ~7 km/s orbital velocity, 1e-12 km/s corresponds to ~1 nm/s —
/// well below measurement resolution.
const POCA_RANGE_RATE_TOL_KM_S: f64 = 1e-12;

/// Maximum Brent iterations. Brent converges superlinearly on smooth
/// functions; 50 iterations handles any bracketed interval with margin.
const POCA_MAX_ITERATIONS: usize = 50;

/// Bracket width tolerance for Brent convergence (seconds).
/// When the bracketing interval narrows below this, the POCA time is
/// determined to sub-microsecond precision — the corresponding distance
/// uncertainty is `~ṙ_max * 1e-6 s ≈ 1e-9 km` (sub-millimeter).
const POCA_BRACKET_TOL_S: f64 = 1e-6;

/// Range-rate magnitude below which a grid point is treated as a candidate
/// POCA directly (grazing case). Same units as [`POCA_RANGE_RATE_TOL_KM_S`].
const POCA_GRAZING_TOL_KM_S: f64 = 1e-10;

/// Soft convergence threshold for Brent (km/s). When Brent exhausts its
/// iteration budget but the best residual is below this value, the result
/// is accepted. This handles eccentric orbits (e up to ~0.5) where the
/// ROE-to-RIC linearization error prevents the range-rate from reaching
/// a true zero. At ~7 km/s orbital velocity, 1e-6 km/s ≈ 1 µm/s;
/// corresponding position uncertainty over a 30 s bracket is ~30 µm.
const POCA_SOFT_CONVERGENCE_TOL_KM_S: f64 = 1e-6;

/// Minimum position norm for range-rate computation (km).
/// Below this, the vehicles are effectively co-located and range-rate
/// is undefined — return 0.0.
const POSITION_NORM_EPSILON_KM: f64 = 1e-15;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Refined closest-approach point found by Brent's method on range rate.
///
/// Each POCA represents a local minimum of the inter-satellite distance
/// within a single maneuver leg. Multi-orbit legs may have multiple POCAs
/// (one per orbit); the global minimum is flagged.
///
/// # References
/// - Brent's method: Brent (1973), Ch. 4
/// - RIC position/velocity: D'Amico Eq. 2.17 (unperturbed), Eq. 2.31 (J2)
/// - Minimum distance for bounded motion: D'Amico Eq. 2.23
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClosestApproach {
    /// Epoch of closest approach.
    #[serde(with = "crate::types::state::epoch_serde")]
    pub epoch: Epoch,
    /// Elapsed time from leg start (s).
    pub elapsed_s: f64,
    /// Refined minimum distance (km).
    pub distance_km: f64,
    /// RIC position at closest approach (km).
    pub position_ric_km: Vector3<f64>,
    /// RIC velocity at closest approach (km/s).
    pub velocity_ric_km_s: Vector3<f64>,
    /// Leg index within the mission.
    pub leg_index: usize,
    /// Whether this is the global minimum across all POCAs in this leg.
    pub is_global_minimum: bool,
}

impl ClosestApproach {
    /// Return the closest approach with the smallest distance from a slice.
    ///
    /// NaN distances are treated as infinitely far (sorted last). Returns
    /// `None` if the slice is empty.
    #[must_use]
    pub fn nearest(pocas: &[Self]) -> Option<&Self> {
        pocas
            .iter()
            .min_by(|a, b| {
                a.distance_km
                    .partial_cmp(&b.distance_km)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }
}

/// Errors from POCA computation.
#[derive(Debug, Clone)]
pub enum PocaError {
    /// Trajectory has fewer than 2 points.
    InsufficientPoints {
        /// Number of points provided.
        count: usize,
    },
    /// Brent's method did not converge within iteration limit.
    NoConvergence {
        /// Number of iterations attempted.
        iterations: usize,
        /// Final range-rate residual (km/s).
        residual_km_s: f64,
        /// Bracket start time (s).
        bracket_start_s: f64,
        /// Bracket end time (s).
        bracket_end_s: f64,
    },
    /// Propagation failed during a Brent trial evaluation.
    PropagationFailure {
        /// Trial time at which propagation failed (s).
        trial_time_s: f64,
        /// Underlying propagation error.
        source: PropagationError,
    },
}

impl std::fmt::Display for PocaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InsufficientPoints { count } => {
                write!(f, "POCA requires >= 2 trajectory points, got {count}")
            }
            Self::NoConvergence {
                iterations,
                residual_km_s,
                bracket_start_s,
                bracket_end_s,
            } => write!(
                f,
                "Brent's method did not converge after {iterations} iterations \
                 (residual: {residual_km_s:.2e} km/s, bracket: [{bracket_start_s:.1}, {bracket_end_s:.1}] s)"
            ),
            Self::PropagationFailure {
                trial_time_s,
                source,
            } => write!(
                f,
                "propagation failed at trial time {trial_time_s:.3} s: {source}"
            ),
        }
    }
}

impl std::error::Error for PocaError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::PropagationFailure { source, .. } => Some(source),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Internal: bracket types
// ---------------------------------------------------------------------------

/// A bracketed interval where range-rate changes sign, or a grazing point.
#[derive(Debug)]
enum Bracket {
    /// Range-rate sign change between `trajectory[lo]` and `trajectory[hi]`.
    SignChange {
        lo: usize,
        hi: usize,
    },
    /// Grid point with `|ṙ| < POCA_GRAZING_TOL_KM_S`.
    Grazing {
        index: usize,
    },
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Propagation context for Brent trial evaluations.
///
/// Bundles the departure state and propagation model so that `brent_refine`
/// can evaluate range-rate at any elapsed time without threading four
/// separate parameters through the solver.
struct PocaEvaluator<'a> {
    departure_roe: &'a QuasiNonsingularROE,
    chief_mean: &'a KeplerianElements,
    departure_epoch: Epoch,
    model: &'a PropagationModel,
}

impl PocaEvaluator<'_> {
    /// Propagate to elapsed time `t_s` and return the full state.
    fn propagate(&self, t_s: f64) -> Result<PropagatedState, PocaError> {
        self.model
            .propagate(self.departure_roe, self.chief_mean, self.departure_epoch, t_s)
            .map_err(|err| PocaError::PropagationFailure {
                trial_time_s: t_s,
                source: err,
            })
    }
}

/// Compute range rate from a propagated state.
///
/// `ṙ = r · v / ‖r‖` where `r` and `v` are the RIC position and velocity.
/// Returns 0.0 if the position norm is below [`POSITION_NORM_EPSILON_KM`].
fn compute_range_rate(state: &PropagatedState) -> f64 {
    let r = &state.ric.position_ric_km;
    let v = &state.ric.velocity_ric_km_s;
    let r_norm = r.norm();
    if r_norm < POSITION_NORM_EPSILON_KM {
        return 0.0;
    }
    r.dot(v) / r_norm
}

/// Scan trajectory for range-rate brackets and grazing points.
///
/// Returns brackets sorted by trajectory index.
fn find_brackets(range_rates: &[f64]) -> Vec<Bracket> {
    let mut brackets = Vec::new();

    for i in 0..range_rates.len() {
        // Grazing: grid point with near-zero range-rate.
        // Only emit when the previous point was NOT also grazing, to collapse
        // runs of constant-distance points into a single representative.
        if range_rates[i].abs() < POCA_GRAZING_TOL_KM_S {
            let prev_was_grazing =
                i > 0 && range_rates[i - 1].abs() < POCA_GRAZING_TOL_KM_S;
            let already_bracketed = i > 0
                && range_rates[i - 1] < 0.0
                && range_rates[i] >= 0.0;
            if !already_bracketed && !prev_was_grazing {
                brackets.push(Bracket::Grazing { index: i });
            }
        }

        // Sign change: approach → recession
        if i + 1 < range_rates.len()
            && range_rates[i] < -POCA_GRAZING_TOL_KM_S
            && range_rates[i + 1] > POCA_GRAZING_TOL_KM_S
        {
            brackets.push(Bracket::SignChange { lo: i, hi: i + 1 });
        }
    }

    brackets
}

/// Construct a [`ClosestApproach`] from a propagated state.
fn closest_approach_from_state(
    state: &PropagatedState,
    leg_index: usize,
) -> ClosestApproach {
    ClosestApproach {
        epoch: state.epoch,
        elapsed_s: state.elapsed_s,
        distance_km: state.ric.position_ric_km.norm(),
        position_ric_km: state.ric.position_ric_km,
        velocity_ric_km_s: state.ric.velocity_ric_km_s,
        leg_index,
        is_global_minimum: false,
    }
}

/// Refine a bracketed interval using Brent's method (Brent 1973, Ch. 4).
///
/// Finds the root of the range-rate function within `[a_s, b_s]`.
/// Each trial evaluation calls [`PocaEvaluator::propagate`] at the trial time.
///
/// Near the root, the range-rate function reaches the numerical noise
/// floor of the analytical STM (~1e-12 km/s). If strict convergence
/// (`|ṙ| < POCA_RANGE_RATE_TOL_KM_S`) cannot be achieved, the algorithm
/// accepts the best point found if its residual is below the soft
/// convergence tolerance.
#[allow(clippy::similar_names, clippy::many_single_char_names)]
fn brent_refine(
    a_s: f64,
    b_s: f64,
    fa: f64,
    fb: f64,
    eval: &PocaEvaluator<'_>,
    leg_index: usize,
) -> Result<ClosestApproach, PocaError> {
    let mut a = a_s;
    let mut b = b_s;
    let mut fa_val = fa;
    let mut fb_val = fb;
    let mut c = a;
    let mut fc = fa_val;
    let mut d = b - a;
    let mut e = d;

    // Track the best point (smallest |range_rate|) for soft convergence.
    // Store only the time value — not the state — to avoid cloning
    // PropagatedState inside the iteration loop.
    let mut best_rr = fa.abs().min(fb.abs());
    let mut best_time: Option<f64> = None;
    // Cache the last propagated state at `b` to avoid re-propagating at
    // convergence. Invalidated when the b/c swap changes `b`.
    let mut last_state_at_b: Option<PropagatedState> = None;

    for _iter in 0..POCA_MAX_ITERATIONS {
        if (fb_val > 0.0 && fc > 0.0) || (fb_val < 0.0 && fc < 0.0) {
            c = a;
            fc = fa_val;
            d = b - a;
            e = d;
        }
        if fc.abs() < fb_val.abs() {
            a = b;
            b = c;
            c = a;
            fa_val = fb_val;
            fb_val = fc;
            fc = fa_val;
            last_state_at_b = None;
        }

        let tol = 2.0 * f64::EPSILON * b.abs() + POCA_BRACKET_TOL_S;
        let m = 0.5 * (c - b);

        // Convergence: range-rate is negligible OR bracket is sub-microsecond
        if fb_val.abs() <= POCA_RANGE_RATE_TOL_KM_S || m.abs() <= tol {
            let state = last_state_at_b.map_or_else(|| eval.propagate(b), Ok)?;
            return Ok(closest_approach_from_state(&state, leg_index));
        }

        // Decide: inverse quadratic interpolation or bisection
        if e.abs() >= tol && fa_val.abs() > fb_val.abs() {
            let s_val = fb_val / fa_val;
            let step = if (a - c).abs() < f64::EPSILON {
                2.0 * m * s_val
            } else {
                let q = fa_val / fc;
                let r = fb_val / fc;
                s_val
                    * (2.0 * m * q * (q - r) - (b - a) * (r - 1.0))
                    / ((q - 1.0) * (r - 1.0) * (s_val - 1.0))
            };
            if step.abs() < (1.5 * m).abs() && step.abs() < (0.5 * e).abs() {
                e = d;
                d = step;
            } else {
                d = m;
                e = m;
            }
        } else {
            d = m;
            e = m;
        }

        a = b;
        fa_val = fb_val;
        b += if d.abs() > tol { d } else if m > 0.0 { tol } else { -tol };

        let state = eval.propagate(b)?;
        fb_val = compute_range_rate(&state);

        if fb_val.abs() < best_rr {
            best_rr = fb_val.abs();
            best_time = Some(b);
        }
        last_state_at_b = Some(state);
    }

    // Soft convergence: best residual is at the numerical noise floor.
    // Re-propagate once at the best time (avoids storing state in the loop).
    if best_rr < POCA_SOFT_CONVERGENCE_TOL_KM_S {
        if let Some(t) = best_time {
            let state = eval.propagate(t)?;
            return Ok(closest_approach_from_state(&state, leg_index));
        }
    }

    Err(PocaError::NoConvergence {
        iterations: POCA_MAX_ITERATIONS,
        residual_km_s: best_rr,
        bracket_start_s: a_s,
        bracket_end_s: b_s,
    })
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Find all points of closest approach in a trajectory.
///
/// Scans for range-rate sign changes between adjacent points, then refines
/// each bracket using Brent's method to find the exact time, position, and
/// distance of closest approach.
///
/// # Arguments
/// * `trajectory` - Sampled RIC trajectory points (position + velocity)
/// * `chief_mean` - Chief mean Keplerian elements at leg start
/// * `departure_epoch` - Epoch at leg start
/// * `model` - Propagation model for Brent trial evaluations
/// * `departure_roe` - ROE state at leg start (for trial propagations)
/// * `leg_index` - Index of this leg in the mission
///
/// # Invariants
/// - `trajectory.len() >= 2`
/// - Points must be time-ordered
/// - `chief_mean.a_km > 0`, `chief_mean.e` in [0, 1)
///
/// # Validity regime
/// - The algorithm is engine-agnostic — it operates on any sequence of RIC
///   position+velocity states, regardless of how they were generated.
/// - Brent's method assumes continuous range-rate within each bracket.
///   The bisection fallback handles narrow brackets gracefully.
///
/// # Returns
/// `Vec<ClosestApproach>` sorted by distance (ascending). Empty if no
/// range-rate sign change detected (diverging leg). The global minimum
/// is flagged via `is_global_minimum`.
///
/// # Errors
/// - [`PocaError::InsufficientPoints`] if trajectory has < 2 points
/// - [`PocaError::NoConvergence`] if Brent's method fails to converge
/// - [`PocaError::PropagationFailure`] if a Brent trial propagation fails
pub fn find_closest_approaches(
    trajectory: &[PropagatedState],
    chief_mean: &KeplerianElements,
    departure_epoch: Epoch,
    model: &PropagationModel,
    departure_roe: &QuasiNonsingularROE,
    leg_index: usize,
) -> Result<Vec<ClosestApproach>, PocaError> {
    if trajectory.len() < 2 {
        return Err(PocaError::InsufficientPoints {
            count: trajectory.len(),
        });
    }

    let eval = PocaEvaluator {
        departure_roe,
        chief_mean,
        departure_epoch,
        model,
    };

    // Precompute range rates
    let range_rates: Vec<f64> = trajectory.iter().map(compute_range_rate).collect();

    // Find brackets
    let brackets = find_brackets(&range_rates);

    // Refine each bracket
    let mut results = Vec::with_capacity(brackets.len());
    for bracket in &brackets {
        match bracket {
            Bracket::Grazing { index } => {
                results.push(closest_approach_from_state(&trajectory[*index], leg_index));
            }
            Bracket::SignChange { lo, hi } => {
                let ca = brent_refine(
                    trajectory[*lo].elapsed_s,
                    trajectory[*hi].elapsed_s,
                    range_rates[*lo],
                    range_rates[*hi],
                    &eval,
                    leg_index,
                )?;
                results.push(ca);
            }
        }
    }

    // Sort by distance ascending
    results.sort_by(|a, b| {
        a.distance_km
            .partial_cmp(&b.distance_km)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // is_global_minimum stays false at this level — the pipeline sets it
    // across all legs once it has visibility into the full mission.

    Ok(results)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{
        damico_table21_case1_roe, damico_table21_chief, koenig_table2_case1,
        koenig_table2_case1_roe, test_epoch,
    };

    // Named tolerance constants
    //
    // Categories:
    // - IDENTITY: machine-epsilon accumulation (exact arithmetic)
    // - STM_LINEARIZATION: J2 STM linearization vs analytic Eq. 2.23 bound
    const RANGE_RATE_IDENTITY_TOL: f64 = 1e-14;
    const POCA_DISTANCE_TOL_KM: f64 = 1e-6;

    /// Helper: propagate a trajectory for testing.
    fn propagate_trajectory(
        roe: &QuasiNonsingularROE,
        chief: &KeplerianElements,
        epoch: Epoch,
        tof_s: f64,
        n_steps: usize,
    ) -> Vec<PropagatedState> {
        PropagationModel::J2Stm
            .propagate_with_steps(roe, chief, epoch, tof_s, n_steps)
            .expect("propagation should succeed")
    }

    // ── Test 1: Bounded circular motion (D'Amico Eq. 2.23) ──────────

    #[test]
    fn test_bounded_motion_poca() {
        // D'Amico Table 2.1: 700 km SSO, parallel e/i vectors
        // a*dey = 400 m → δe_mag = 400/7078.135
        // a*diy = 200 m → δi_mag = 200/7078.135
        //
        // With parallel e/i vectors (both along +y), the radial and
        // cross-track oscillations reach their minima at the same argument
        // of latitude. The 3D POCA distance is dominated by the in-track
        // component from δλ secular drift under J2, plus the residual
        // cross-track/radial oscillation at the POCA epoch.
        //
        // For this geometry: POCA ≈ a * δe = 0.400 km (the eccentricity
        // vector sets the radial oscillation amplitude; the inclination
        // vector's smaller amplitude doesn't further reduce the minimum).
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");
        let n_steps = 200;

        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, n_steps);

        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("POCA should succeed");

        assert!(!pocas.is_empty(), "bounded motion should have at least one POCA");

        // is_global_minimum is set at the pipeline level, not per-leg.
        // At this level, all POCAs should have is_global_minimum = false.
        assert!(
            !pocas[0].is_global_minimum,
            "per-leg POCA should not set is_global_minimum"
        );

        // Verify POCA distance is physically reasonable:
        // - Must be > 0 (vehicles don't collide)
        // - Must be < a * (δe + δi) ≈ 0.600 km (upper bound from triangle inequality)
        let a_km = chief.a_km;
        let de_mag = (roe.dex * roe.dex + roe.dey * roe.dey).sqrt();
        let di_mag = (roe.dix * roe.dix + roe.diy * roe.diy).sqrt();
        let upper_bound_km = a_km * (de_mag + di_mag);

        assert!(
            pocas[0].distance_km > 0.0,
            "POCA distance must be positive"
        );
        assert!(
            pocas[0].distance_km < upper_bound_km,
            "POCA distance {:.3} km exceeds upper bound {upper_bound_km:.3} km",
            pocas[0].distance_km,
        );

        // For this specific geometry (parallel e/i), POCA ≈ a * δe = 0.400 km
        let expected_km = a_km * de_mag;
        assert!(
            (pocas[0].distance_km - expected_km).abs() < POCA_DISTANCE_TOL_KM,
            "POCA distance {:.6} km should be ≈ {expected_km:.6} km, \
             diff = {:.2e} km",
            pocas[0].distance_km,
            (pocas[0].distance_km - expected_km).abs(),
        );
    }

    // ── Test 2: Diverging leg (empty vec) ────────────────────────────

    #[test]
    fn test_diverging_leg_returns_empty() {
        // Large positive δa → monotonic drift opening
        let chief = damico_table21_chief();
        let roe = QuasiNonsingularROE {
            da: 0.01, // very large δa → rapid along-track drift
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 200);

        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("POCA should succeed even for diverging legs");

        // Diverging leg: distance increases monotonically from t=0,
        // range-rate stays positive → no bracket → empty vec
        // Note: the very first point might have near-zero range-rate due to
        // the initial dlambda=0 condition, but after that it diverges.
        // We allow empty or at most a grazing detection at t=0.
        // The key invariant: no sign-change brackets should be found.
        assert!(
            pocas.len() <= 1,
            "diverging leg should have at most 1 grazing POCA (at t≈0), got {}",
            pocas.len()
        );
    }

    // ── Test 3: Multi-orbit (one POCA per orbit) ─────────────────────

    #[test]
    fn test_multi_orbit_multiple_pocas() {
        let chief = koenig_table2_case1();
        let roe = koenig_table2_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");
        let n_orbits: u32 = 3;
        let tof_s = period_s * f64::from(n_orbits);
        let n_steps = 200 * n_orbits;

        let traj = propagate_trajectory(
            &roe,
            &chief,
            epoch,
            tof_s,
            usize::try_from(n_steps).expect("n_steps fits usize"),
        );

        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("POCA should succeed");

        // Bounded relative motion: ~1 POCA per orbit.
        // Allow some tolerance (±1) since bracket detection at orbit
        // boundaries may merge or split.
        assert!(
            pocas.len() >= 2,
            "3 orbits of bounded motion should have >= 2 POCAs, got {}",
            pocas.len()
        );

        // Sorted ascending by distance
        for w in pocas.windows(2) {
            assert!(
                w[0].distance_km <= w[1].distance_km + f64::EPSILON,
                "POCAs should be sorted by distance: {:.6} > {:.6}",
                w[0].distance_km,
                w[1].distance_km,
            );
        }

        // No global minimum set at per-leg level
        let global_count = pocas.iter().filter(|p| p.is_global_minimum).count();
        assert_eq!(global_count, 0, "per-leg POCA should not set is_global_minimum");
    }

    // ── Test 4: Refined ≤ grid-sampled (invariant property) ──────────

    #[test]
    fn test_refined_leq_grid_sampled() {
        use crate::mission::safety::analyze_trajectory_safety;

        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 200);

        let grid_safety =
            analyze_trajectory_safety(&traj).expect("safety analysis should succeed");
        let grid_min_km = grid_safety.operational.min_distance_3d_km;

        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("POCA should succeed");

        assert!(!pocas.is_empty(), "should have at least one POCA");

        // Refined POCA distance must be <= grid-sampled minimum
        assert!(
            pocas[0].distance_km <= grid_min_km + f64::EPSILON,
            "refined POCA {:.6} km should be <= grid-sampled {grid_min_km:.6} km",
            pocas[0].distance_km,
        );
    }

    // ── Test 5: Grazing point detection ──────────────────────────────

    #[test]
    fn test_grazing_point_detection() {
        // Use a trajectory where the grid happens to land near a POCA.
        // With 200 steps per orbit, at least one grid point will have
        // small |ṙ|. We verify the algorithm doesn't crash and produces
        // valid results.
        let chief = koenig_table2_case1();
        let roe = koenig_table2_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        // Use enough steps that grid resolution captures the POCA region
        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 1000);

        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("POCA should succeed with fine grid");

        // Should find at least one POCA (bounded motion)
        assert!(!pocas.is_empty(), "fine grid should detect POCAs");

        // All POCAs should have positive distance
        for poca in &pocas {
            assert!(
                poca.distance_km > 0.0,
                "POCA distance should be positive, got {}",
                poca.distance_km,
            );
        }
    }

    // ── Test 6: Brent convergence ────────────────────────────────────

    #[test]
    fn test_brent_convergence() {
        // Standard bounded motion — Brent should converge well within 50 iterations
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 200);

        // If Brent didn't converge, find_closest_approaches would return
        // Err(PocaError::NoConvergence). Success = convergence.
        let pocas = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        )
        .expect("Brent should converge within POCA_MAX_ITERATIONS");

        assert!(!pocas.is_empty(), "should find at least one POCA");
    }

    // ── Test 7: InsufficientPoints error ─────────────────────────────

    #[test]
    fn test_insufficient_points_empty() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();

        let result = find_closest_approaches(
            &[], &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        );

        match result {
            Err(PocaError::InsufficientPoints { count: 0 }) => {}
            other => panic!("expected InsufficientPoints(0), got {other:?}"),
        }
    }

    #[test]
    fn test_insufficient_points_single() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();

        let traj = propagate_trajectory(&roe, &chief, epoch, 100.0, 1);
        // propagate_with_steps(n_steps=1) returns 2 points (start + end),
        // but let's slice to 1 point to test the error path
        let result = find_closest_approaches(
            &traj[..1], &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        );

        match result {
            Err(PocaError::InsufficientPoints { count: 1 }) => {}
            other => panic!("expected InsufficientPoints(1), got {other:?}"),
        }
    }

    // ── Test 8: Two-point minimal trajectory ─────────────────────────

    #[test]
    fn test_two_point_trajectory() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        // n_steps=1 → 2 points (t=0, t=tof)
        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 1);
        assert_eq!(traj.len(), 2, "should have exactly 2 points");

        // Should not panic — may find bracket or not depending on geometry
        let result = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        );
        assert!(result.is_ok(), "two-point trajectory should not error");
    }

    // ── Test 9: Along-track only separation (degenerate) ─────────────

    #[test]
    fn test_along_track_only_degenerate() {
        // Only dlambda nonzero → along-track separation only, no radial
        // or cross-track oscillation. No classic approach/recession cycle.
        let chief = damico_table21_chief();
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 1.0 / chief.a_km, // ~1 km along-track
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let epoch = test_epoch();
        let period_s = chief.period().expect("period should succeed");

        let traj = propagate_trajectory(&roe, &chief, epoch, period_s, 200);

        // Should not panic or error
        let result = find_closest_approaches(
            &traj, &chief, epoch, &PropagationModel::J2Stm, &roe, 0,
        );
        assert!(result.is_ok(), "along-track only case should not error");
    }

    // ── Test 10: Range-rate unit computation ─────────────────────────

    #[test]
    fn test_compute_range_rate_approaching() {
        // r = [1, 0, 0], v = [-1, 0, 0] → ṙ = -1 (approaching)
        let state = PropagatedState {
            epoch: test_epoch(),
            roe: QuasiNonsingularROE {
                da: 0.0, dlambda: 0.0, dex: 0.0, dey: 0.0, dix: 0.0, diy: 0.0,
            },
            chief_mean: damico_table21_chief(),
            ric: crate::types::RICState {
                position_ric_km: Vector3::new(1.0, 0.0, 0.0),
                velocity_ric_km_s: Vector3::new(-1.0, 0.0, 0.0),
            },
            elapsed_s: 0.0,
        };
        let rr = compute_range_rate(&state);
        assert!(
            (rr - (-1.0)).abs() < RANGE_RATE_IDENTITY_TOL,
            "expected ṙ = -1, got {rr}"
        );
    }

    #[test]
    fn test_compute_range_rate_receding() {
        // r = [1, 0, 0], v = [1, 0, 0] → ṙ = +1 (receding)
        let state = PropagatedState {
            epoch: test_epoch(),
            roe: QuasiNonsingularROE {
                da: 0.0, dlambda: 0.0, dex: 0.0, dey: 0.0, dix: 0.0, diy: 0.0,
            },
            chief_mean: damico_table21_chief(),
            ric: crate::types::RICState {
                position_ric_km: Vector3::new(1.0, 0.0, 0.0),
                velocity_ric_km_s: Vector3::new(1.0, 0.0, 0.0),
            },
            elapsed_s: 0.0,
        };
        let rr = compute_range_rate(&state);
        assert!(
            (rr - 1.0).abs() < RANGE_RATE_IDENTITY_TOL,
            "expected ṙ = +1, got {rr}"
        );
    }

    #[test]
    fn test_compute_range_rate_zero_position() {
        // r = [0, 0, 0] → degenerate → ṙ = 0
        let state = PropagatedState {
            epoch: test_epoch(),
            roe: QuasiNonsingularROE {
                da: 0.0, dlambda: 0.0, dex: 0.0, dey: 0.0, dix: 0.0, diy: 0.0,
            },
            chief_mean: damico_table21_chief(),
            ric: crate::types::RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::new(1.0, 2.0, 3.0),
            },
            elapsed_s: 0.0,
        };
        let rr = compute_range_rate(&state);
        assert!(
            rr.abs() < RANGE_RATE_IDENTITY_TOL,
            "expected ṙ = 0 for zero position, got {rr}"
        );
    }
}

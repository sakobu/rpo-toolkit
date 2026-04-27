//! Lambert transfer solver: in-tree Izzo implementation + transfer types.
//!
//! The solver dispatches across three TOF regimes (Battin near-parabolic /
//! Lancaster–Blanchard mid-band / Lagrange elsewhere) and uses Householder
//! iteration for root-finding, with a Halley-iterated `T_min` check on
//! multi-revolution branches.
//!
//! [`TransferFeasibility`] is carried on the success type rather than as a
//! [`LambertError`] variant because the UI renders the arc + Δv numbers for
//! sub-surface transfers — the user needs to see what their inputs produced
//! to diagnose the geometry.
//!
//! # Validity
//!
//! - `arrival.epoch > departure.epoch` (positive time of flight).
//! - `|r1|`, `|r2|` ≥ [`LAMBERT_DEGENERATE_NORM_KM`] (`1e-15 km`).
//! - Transfer angle `θ ∉ {0, π}`: `|r1 × r2| / (|r1| |r2|)` ≥
//!   [`COLLINEARITY_TOL`] (`1e-15`).
//! - For `revolutions ≥ 1`, `tof_s ≥ T_min(M, λ)`. The solver returns
//!   [`LambertError::NoSolutionForRevolutions`] with the maximum feasible
//!   `M` when this is violated.
//!
//! # Singularities
//!
//! - `θ → 0` / `θ → π` (radial transfers, opposing-radial transfers): the
//!   transfer plane is undefined; rejected with
//!   [`LambertError::CollinearGeometry`].
//! - Near-parabolic regime (`x → 1`, equivalently `tof_s → T_parabolic`):
//!   the Lancaster `1/(1−x²)` factor amplifies round-off; handled by the
//!   Battin hypergeometric branch (Izzo Eq. 20) for `|x − 1| ≤
//!   BATTIN_THRESHOLD`.
//! - Multi-rev `T_min(M)` boundary: the Householder denominator
//!   `dt·(dt² − f·ddt) + dddt·f²/6` collapses as `dt → 0`; surfaced as
//!   [`LambertError::SingularDenominator`] at the
//!   [`HOUSEHOLDER_DENOM_EPS`] threshold rather than silently returning
//!   garbage.
//!
//! # Reference
//!
//! Izzo, D. *Revisiting Lambert's Problem*. Celestial Mechanics and Dynamical
//! Astronomy 121.1 (2015): 1–15. arXiv:1403.2705. Eq. 7 (`y(x, λ)`),
//! Eq. 9 / 18 / 20 (Lagrange / Lancaster–Blanchard / Battin TOF regimes),
//! Eq. 22 (`dT/dx` derivatives), Algorithm 1 (velocity reconstruction),
//! Eq. 30 / 31 (initial guesses), Eq. 32 (Householder update).
//! Inline `Eq. N` references in the source point to that paper.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::constants::{MIN_PERIAPSIS_ALTITUDE_KM, MU_EARTH, R_EARTH};
use crate::types::StateVector;

mod geometry;
mod root_finding;
mod solver;
mod tof;

pub use solver::{solve_lambert, solve_lambert_branches, solve_lambert_with_config};

/// Kernel access for benchmarks (and any future tooling that needs to
/// drive the solver below the public API). Items here are unstable —
/// they expose the Izzo paper's internal `(λ, T, M) → x` formulation
/// and are intended for paper-fidelity telemetry, not application code.
/// Hidden from rustdoc.
#[doc(hidden)]
pub mod kernel {
    pub use super::root_finding::{
        householder_counted, solve_x_multi_rev_branches, solve_x_single_rev,
        HouseholderRoot, MultiRevBranches,
    };
    pub use super::tof::x_to_tof;
}

// =====================================================================
// Named tolerances (Izzo 2015 §5; rpo-core "Tolerance Policy")
// =====================================================================

/// Minimum norm for a valid Lambert position vector (km). Below this, the
/// position is treated as degenerate and the chord/lambda construction is
/// undefined. `1e-15 km` (≈ 1 fm) is `f64` noise at any reasonable scale.
///
/// Distinct from [`crate::constants::MIN_POSITION_NORM_KM`] (`1e-10 km`,
/// ≈ 0.1 mm) — the workspace-wide zero-vector guard for Cartesian↔Keplerian
/// conversion. The Lambert kernel needs the looser `f64`-noise threshold so
/// it can accept any geometry the conversion path admits without spurious
/// rejection in the chord-construction step.
pub const LAMBERT_DEGENERATE_NORM_KM: f64 = 1.0e-15;

/// Threshold below which `r1` and `r2` are treated as collinear (transfer
/// plane undefined). Compares the unitless ratio `|r1 × r2| / (|r1| · |r2|)`,
/// so the check has meaning regardless of input scale. The geometric
/// separation check the deleted `LAMBERT_MIN_SEPARATION_KM` provided is
/// subsumed: a coincident `r1 == r2` produces `|r1 × r2| = 0` and trips
/// this guard; a near-coincident pair with a non-zero in-plane separation
/// (e.g. `r2 = r1 + 1 m`) is mathematically a well-posed parabolic Lambert
/// problem and is correctly handled by the Battin near-parabolic regime.
pub const COLLINEARITY_TOL: f64 = 1.0e-15;

/// Round-off tolerance for arguments that are mathematically non-negative
/// but can dip slightly below zero due to `f64` cancellation.
///
/// Applied to `y² = 1 − λ²(1 − x²)` (Izzo Eq. 7), `λ² = 1 − c/s`, and
/// `σ² = 1 − ρ²`. Each is provably `≥ 0` by triangle inequality / Eq. 7;
/// `f64` round-off in the parabolic / `|λ| → 1` regimes can produce
/// arguments down to `~1 ULP × 1 ≈ 2.2e-16`. `1e-12` provides a four-order
/// safety margin while still catching arguments that are meaningfully
/// negative (an algorithmic bug, not round-off).
pub const LAMBERT_ROUNDOFF_TOL: f64 = 1.0e-12;

/// Householder `|Δx|` tolerance for the single-revolution branch.
///
/// Izzo §5: "1e-5 is sufficient." Looser than the multi-rev tolerance
/// because the single-rev branch is well-conditioned; further refinement
/// does not improve the velocity reconstruction beyond `~1e-9` ΔE/E.
pub const HOUSEHOLDER_TOL_SINGLE: f64 = 1.0e-5;

/// Householder `|Δx|` tolerance for multi-revolution branches (`M ≥ 1`).
///
/// Tighter than single-rev because multi-rev branches lie close to the
/// `T_min` minimum where `dT/dx → 0` and the iteration is more sensitive.
pub const HOUSEHOLDER_TOL_MULTI: f64 = 1.0e-8;

/// Halley iteration tolerance for the `T_min` search on multi-rev branches.
///
/// Tighter than the Householder tolerances because `T_min` decides whether
/// a given `M` branch admits a solution at all — a noisy `T_min` could
/// silently drop or admit branches.
pub const HALLEY_TOL: f64 = 1.0e-13;

/// Householder safety cap. Izzo Fig. 3 reports convergence in `≤ 7` iters
/// over `1e6` random geometries; `15` is a comfortable margin.
pub const HOUSEHOLDER_MAX_ITERS: u32 = 15;

/// Halley safety cap. Cubic convergence reaches `f64` precision well under
/// this limit in normal use.
pub const HALLEY_MAX_ITERS: u32 = 12;

/// Threshold below which the Householder / Halley denominator is treated
/// as algebraically zero (singular).
///
/// The Householder denominator is `dt·(dt² − f·ddt) + dddt·f²/6`; at
/// convergence (`f → 0`) it reduces to `dt³` which is `O(1)` in Izzo's
/// non-dimensional `T`-space. `1e-30` flags only true mathematical
/// singularities (`dt → 0` at multi-rev `T_min`); normal iteration values
/// are well above this floor.
pub const HOUSEHOLDER_DENOM_EPS: f64 = 1.0e-30;

/// `|x − 1| ≤ this` → use Battin's hypergeometric formulation (Izzo Eq. 20).
///
/// Below `0.01`, the Lancaster form's `1/(1−x²)` factor amplifies round-off
/// to `~1e-6` of the true TOF; Battin's series remains exact. Izzo §5 does
/// not prescribe a numeric value but identifies the regime boundary near
/// `x = 1`; `PyKEP` uses the same `0.01` threshold (the de-facto reference
/// implementation).
pub const BATTIN_THRESHOLD: f64 = 0.01;

/// `BATTIN_THRESHOLD < |x − 1| ≤ this` → use Lancaster–Blanchard (Izzo Eq. 18).
///
/// Above `0.2`, the Lagrange form's trigonometric reductions are stable
/// enough that the Lancaster path's added complexity is unnecessary. Izzo
/// §5 does not prescribe a numeric value; `PyKEP` uses the same `0.2`
/// threshold.
pub const LAGRANGE_THRESHOLD: f64 = 0.2;

/// Convergence tolerance for the direct `2F1(3, 1; 5/2; z)` series sum
/// used by the Battin TOF formulation.
pub const HYPERGEOMETRIC_2F1_TOL: f64 = 1.0e-11;

/// Maximum terms for the `2F1` series sum before bailing out.
pub const HYPERGEOMETRIC_2F1_MAX_TERMS: u32 = 1000;

/// Bounded clamp-then-sqrt for arguments that are mathematically `≥ 0` but
/// can dip slightly negative due to `f64` round-off (Izzo Eq. 7,
/// `λ² = 1 − c/s`, `σ² = 1 − ρ²`).
///
/// In debug builds, panics if the argument is below `-LAMBERT_ROUNDOFF_TOL`
/// — that range indicates an algorithmic bug, not arithmetic round-off.
/// Release builds clamp silently to preserve hot-loop performance; the
/// upstream input validation in [`geometry::Geometry::from_inputs`] prevents
/// any caller-controlled path from triggering the bug case.
#[inline]
pub(crate) fn safe_sqrt_nonneg(arg: f64) -> f64 {
    debug_assert!(
        arg > -LAMBERT_ROUNDOFF_TOL,
        "safe_sqrt_nonneg: argument {arg:.3e} below -LAMBERT_ROUNDOFF_TOL ({:.3e}); not round-off",
        -LAMBERT_ROUNDOFF_TOL
    );
    arg.max(0.0).sqrt()
}

// =====================================================================
// Public domain types
// =====================================================================

/// Transfer direction for Lambert solutions.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferDirection {
    /// Compute both short-way and long-way; return the lower-Δv result.
    #[default]
    Auto,
    /// Short-way transfer: transfer angle `θ ≤ π`.
    ShortWay,
    /// Long-way transfer: transfer angle `θ > π`.
    LongWay,
}

impl std::fmt::Display for TransferDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Auto => write!(f, "Auto"),
            Self::ShortWay => write!(f, "Short-way (θ ≤ π)"),
            Self::LongWay => write!(f, "Long-way (θ > π)"),
        }
    }
}

/// Configuration for Lambert solver behavior.
///
/// # Default
///
/// `direction = TransferDirection::Auto`, `revolutions = 0` — the safest
/// single-revolution search. The `Auto` direction runs the solver in both
/// `ShortWay` and `LongWay` and returns the lower-Δv result.
///
/// # Validity
///
/// - `revolutions ≤ 5` in practice. The type accepts `u8::MAX` but the
///   solver allocates `1 + 2·M` root entries; values above `~10` carry
///   no physical meaning for orbital transfers and waste memory. Multi-rev
///   branches above `M = 5` are exotic outside interplanetary mission
///   design.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LambertConfig {
    /// Transfer direction selection. See [`TransferDirection`].
    pub direction: TransferDirection,
    /// Number of complete revolutions: `0` for a direct transfer,
    /// `N ≥ 1` for the `M = N` long-period branch.
    pub revolutions: u8,
}

/// Identifies which of the two Lambert position vectors triggered a
/// degenerate-norm error.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PositionTag {
    /// Departure position `r1`.
    R1,
    /// Arrival position `r2`.
    R2,
}

impl std::fmt::Display for PositionTag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::R1 => write!(f, "r1"),
            Self::R2 => write!(f, "r2"),
        }
    }
}

/// Physical feasibility classification for a Lambert transfer's conic.
///
/// Lambert's problem produces a conic that connects two ECI states in the
/// requested time of flight; mathematically any such conic is a "valid"
/// solution, but for proximity-ops planning the conic is only useful if its
/// periapsis stays above Earth's surface. This enum surfaces that
/// distinction without making it a solver error.
///
/// Tagged with `kind` for ergonomic discrimination in TypeScript:
/// `feasibility.kind === 'feasible'` / `'sub_surface'`.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TransferFeasibility {
    /// Conic's periapsis is at or above
    /// [`crate::constants::R_EARTH`] + [`crate::constants::MIN_PERIAPSIS_ALTITUDE_KM`].
    Feasible {
        /// Altitude of periapsis above the mean equatorial radius (km).
        periapsis_altitude_km: f64,
    },
    /// Conic's periapsis is below the feasibility threshold — the transfer
    /// arc passes through (or grazes) the atmosphere/surface and is
    /// non-physical.
    SubSurface {
        /// Altitude of periapsis above the mean equatorial radius (km).
        periapsis_altitude_km: f64,
    },
}

impl TransferFeasibility {
    /// Compute feasibility from a transfer-orbit state.
    ///
    /// Inputs must be in an inertial frame; ECI is the workspace convention.
    /// For Lambert transfers, pass the solver-returned velocity at `r`
    /// (`v1_vec` for departure, `v2_vec` for arrival) — not the natural
    /// orbit velocity, which would describe a different conic.
    ///
    /// # Validity
    ///
    /// Every conic is acceptable — elliptic, parabolic, hyperbolic, and
    /// rectilinear (`r × v = 0`). Rectilinear orbits report
    /// `periapsis_altitude_km = -R_EARTH` (radial impact), which is
    /// correctly classified as [`Self::SubSurface`].
    ///
    /// # Singularities
    ///
    /// - `r → 0`: rejected with [`LambertError::DegeneratePositionVector`].
    ///   Below [`LAMBERT_DEGENERATE_NORM_KM`] the e-vector denominator
    ///   amplifies `f64` round-off into a non-physical periapsis radius.
    ///
    /// # Errors
    ///
    /// Returns [`LambertError::DegeneratePositionVector`] when `|r|` is
    /// below [`LAMBERT_DEGENERATE_NORM_KM`].
    pub fn from_state(
        r_eci_km: Vector3<f64>,
        v_eci_km_s: Vector3<f64>,
    ) -> Result<Self, LambertError> {
        // Inline e-vector + p computation rather than delegating to
        // `state_to_keplerian`: that function returns `Err(UnboundOrbit)` for
        // `e ≥ 1`, but hyperbolic Lambert transfers are valid (short TOF +
        // far targets) and `r_p = p / (1 + e)` stays finite and correct on
        // every conic — elliptic, parabolic, and hyperbolic.
        let r = r_eci_km.norm();
        if r < LAMBERT_DEGENERATE_NORM_KM {
            return Err(LambertError::DegeneratePositionVector {
                which: PositionTag::R1,
                norm_km: r,
            });
        }
        let h_vec = r_eci_km.cross(&v_eci_km_s);
        let h = h_vec.norm();
        let v_sq = v_eci_km_s.norm_squared();
        let e_vec = ((v_sq - MU_EARTH / r) * r_eci_km
            - r_eci_km.dot(&v_eci_km_s) * v_eci_km_s)
            / MU_EARTH;
        let e = e_vec.norm();
        let p = (h * h) / MU_EARTH;
        let periapsis_radius_km = p / (1.0 + e);
        let periapsis_altitude_km = periapsis_radius_km - R_EARTH;
        Ok(if periapsis_altitude_km >= MIN_PERIAPSIS_ALTITUDE_KM {
            Self::Feasible { periapsis_altitude_km }
        } else {
            Self::SubSurface { periapsis_altitude_km }
        })
    }

    /// Periapsis altitude (km above mean equatorial radius), regardless of
    /// feasibility variant. Negative when sub-surface.
    #[must_use]
    pub fn periapsis_altitude_km(&self) -> f64 {
        match self {
            Self::Feasible { periapsis_altitude_km }
            | Self::SubSurface { periapsis_altitude_km } => *periapsis_altitude_km,
        }
    }

    /// `true` when the conic's periapsis is below the feasibility threshold.
    #[must_use]
    pub fn is_sub_surface(&self) -> bool {
        matches!(self, Self::SubSurface { .. })
    }
}

/// A solved Lambert transfer between two ECI states.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LambertTransfer {
    /// Departure state (original deputy position, computed departure velocity).
    pub departure_state: StateVector,
    /// Arrival state (target position, computed arrival velocity).
    pub arrival_state: StateVector,
    /// Required Δv at departure (km/s, ECI).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub departure_dv_eci_km_s: Vector3<f64>,
    /// Required Δv at arrival (km/s, ECI).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub arrival_dv_eci_km_s: Vector3<f64>,
    /// Total Δv magnitude (km/s).
    pub total_dv_km_s: f64,
    /// Time of flight (seconds).
    pub tof_s: f64,
    /// Specific orbital energy `ε = v²/2 − μ/r` of the transfer conic
    /// (km²/s²). Sign convention: `ε < 0` elliptic, `ε = 0` parabolic,
    /// `ε > 0` hyperbolic. For hyperbolic transfers, the characteristic
    /// energy is `C₃ = v∞² = 2·ε`; consumers should branch on `ε ≥ 0`
    /// before interpreting `2·ε` as `C₃` (sub-parabolic conics have no
    /// physical `v∞`).
    pub specific_energy_km2_s2: f64,
    /// Transfer direction used.
    pub direction: TransferDirection,
    /// Physical feasibility of the transfer conic.
    pub feasibility: TransferFeasibility,
}

impl LambertTransfer {
    /// Generate a dense ECI trajectory along the Lambert transfer arc.
    ///
    /// The arc is a two-body Keplerian orbit defined by `departure_state`.
    /// Produces `n_steps + 1` states from departure to arrival (inclusive
    /// endpoints). The canonical caller passes
    /// `n_steps = LAMBERT_ARC_SAMPLES − 1` so the output length is exactly
    /// [`crate::constants::LAMBERT_ARC_SAMPLES`].
    ///
    /// # Errors
    ///
    /// Returns
    /// [`crate::elements::keplerian_conversions::ConversionError::UnboundOrbit`]
    /// when the transfer conic is hyperbolic (`e ≥ 1`,
    /// `specific_energy_km2_s2 ≥ 0`). The current `propagate_keplerian`
    /// kernel handles bound conics only; hyperbolic Lambert solutions are
    /// valid and reachable (short TOF + far targets), so callers that
    /// densify arbitrary transfers must surface this error rather than
    /// silently substituting an empty arc. Other variants are unreachable
    /// for any state produced by the Lambert solver.
    pub fn densify_arc(
        &self,
        n_steps: u32,
    ) -> Result<Vec<StateVector>, crate::elements::keplerian_conversions::ConversionError> {
        let ke = crate::elements::state_to_keplerian(&self.departure_state)?;
        crate::propagation::keplerian::propagate_keplerian_from_elements(
            &ke,
            self.departure_state.epoch,
            self.tof_s,
            n_steps,
        )
    }
}

/// Errors from the Lambert solver.
///
/// All variants carry structured numeric fields per the workspace `CLAUDE.md`
/// error-handling policy ("structured fields, not stringly-typed messages").
#[derive(Debug, Clone, thiserror::Error)]
pub enum LambertError {
    /// Time of flight must be strictly positive.
    #[error("non-positive time of flight — tof = {tof_s:.6} s")]
    NonPositiveTimeOfFlight {
        /// The non-positive TOF value (seconds).
        tof_s: f64,
    },

    /// Gravitational parameter must be strictly positive.
    #[error("non-positive gravitational parameter — μ = {mu_km3_s2:.6e} km³/s²")]
    NonPositiveMu {
        /// The non-positive μ value (km³/s²).
        mu_km3_s2: f64,
    },

    /// One position vector has near-zero norm; geometry undefined.
    #[error("degenerate position vector {which} — norm = {norm_km:.3e} km")]
    DegeneratePositionVector {
        /// `r1` (departure) or `r2` (arrival).
        which: PositionTag,
        /// Norm of the offending vector (km).
        norm_km: f64,
    },

    /// `r1` and `r2` are collinear; the transfer plane is undefined.
    #[error("collinear position vectors — |r1 × r2| / (|r1| |r2|) = {sin_angle:.3e}")]
    CollinearGeometry {
        /// Sine of the transfer angle (unitless).
        sin_angle: f64,
    },

    /// Householder iteration did not reach the configured tolerance.
    #[error(
        "Householder did not converge after {iterations} iters \
         (last |Δx| = {last_step:.3e}, branch M = {n_revs})"
    )]
    NoConvergence {
        /// Iterations performed before giving up.
        iterations: u32,
        /// Magnitude of the last `|Δx|` step (Izzo's `x`-space, unitless).
        last_step: f64,
        /// Branch index: `0` = single-rev, `≥ 1` = multi-rev.
        n_revs: u32,
    },

    /// Householder denominator collapsed to zero — algebraic singularity,
    /// distinct from slow iterative convergence.
    #[error("Householder denominator vanished on branch M = {n_revs}")]
    SingularDenominator {
        /// Branch index where the singularity occurred.
        n_revs: u32,
    },

    /// The requested revolution count exceeds what the geometry/TOF admit.
    #[error(
        "no Lambert solution at {requested} revolutions — \
         max feasible for this geometry/TOF is {max_feasible}"
    )]
    NoSolutionForRevolutions {
        /// The rev count the caller asked for.
        requested: u8,
        /// Highest rev count that does have a solution (may be 0).
        max_feasible: u8,
    },
}

// =====================================================================
// Tests
// =====================================================================

#[cfg(test)]
mod feasibility_tests {
    use super::{TransferFeasibility, MIN_PERIAPSIS_ALTITUDE_KM, MU_EARTH, R_EARTH};
    use crate::test_helpers::ellipse_state_at_apogee;
    use nalgebra::Vector3;

    /// Tolerance on derived periapsis altitude (km). 1e-6 km (1 mm) is
    /// comfortably above arithmetic noise and well below any physically
    /// meaningful threshold.
    const PERIAPSIS_ALT_TOL_KM: f64 = 1.0e-6;

    #[test]
    fn circular_leo_is_feasible() {
        let r_km = Vector3::new(R_EARTH + 500.0, 0.0, 0.0);
        let v_km_s = Vector3::new(0.0, (MU_EARTH / r_km.norm()).sqrt(), 0.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s).expect("valid state");
        assert!(matches!(f, TransferFeasibility::Feasible { .. }), "got {f:?}");
        assert!((f.periapsis_altitude_km() - 500.0).abs() < PERIAPSIS_ALT_TOL_KM);
        assert!(!f.is_sub_surface());
    }

    #[test]
    fn elliptical_with_perigee_below_threshold_is_sub_surface() {
        let (r_km, v_km_s) =
            ellipse_state_at_apogee(R_EARTH + 50.0, R_EARTH + 1000.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s).expect("valid state");
        assert!(f.is_sub_surface(), "expected SubSurface, got {f:?}");
        assert!(
            (f.periapsis_altitude_km() - 50.0).abs() < PERIAPSIS_ALT_TOL_KM,
            "periapsis altitude = {} km",
            f.periapsis_altitude_km()
        );
    }

    #[test]
    fn perigee_just_above_threshold_is_feasible() {
        let (r_km, v_km_s) = ellipse_state_at_apogee(
            R_EARTH + MIN_PERIAPSIS_ALTITUDE_KM + 1.0,
            R_EARTH + 1000.0,
        );
        let f = TransferFeasibility::from_state(r_km, v_km_s).expect("valid state");
        assert!(matches!(f, TransferFeasibility::Feasible { .. }), "got {f:?}");
    }

    #[test]
    fn surface_intersecting_orbit_is_sub_surface() {
        let (r_km, v_km_s) =
            ellipse_state_at_apogee(R_EARTH - 200.0, R_EARTH + 1500.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s).expect("valid state");
        assert!(f.is_sub_surface());
        assert!(f.periapsis_altitude_km() < 0.0);
    }

    #[test]
    fn serde_tag_round_trips_as_kind() {
        let f = TransferFeasibility::Feasible { periapsis_altitude_km: 500.0 };
        let json = serde_json::to_string(&f).unwrap();
        assert!(json.contains("\"kind\":\"feasible\""), "got {json}");
        let f_round: TransferFeasibility = serde_json::from_str(&json).unwrap();
        assert_eq!(f, f_round);

        let f = TransferFeasibility::SubSurface { periapsis_altitude_km: -123.4 };
        let json = serde_json::to_string(&f).unwrap();
        assert!(json.contains("\"kind\":\"sub_surface\""), "got {json}");
        let f_round: TransferFeasibility = serde_json::from_str(&json).unwrap();
        assert_eq!(f, f_round);
    }
}

#[cfg(test)]
mod solver_tests;

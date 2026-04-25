//! Lambert transfer types and error definitions.
//!
//! The solver functions (`solve_lambert`, `solve_lambert_with_config`) that
//! depend on nyx-space live in `rpo-nyx`. This module retains the domain
//! types used by both the analytical engine and the nyx-backed solver.
//!
//! [`TransferFeasibility`] is carried on the success type rather than as a
//! [`LambertError`] variant because the UI renders the arc + Δv numbers for
//! sub-surface transfers — the user needs to see what their inputs produced
//! to diagnose the geometry. Solver-level errors (no convergence, identical
//! positions, …) still return `Err`. The frontend gates ACCEPT TRANSFER on
//! `feasibility.kind === 'sub_surface'`; the analytical engine never enforces
//! this — feasibility is a UX concern, not a math concern.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::constants::{MIN_PERIAPSIS_ALTITUDE_KM, MU_EARTH, R_EARTH};
use crate::types::StateVector;

/// Transfer direction for Lambert solutions.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferDirection {
    /// Automatically determine short or long way based on geometry.
    ///
    /// nyx-space 2.3.1's `TransferKind::Auto` dispatch has a typo in
    /// `direction_of_motion`: `r_init[1].atan2(r_final[1])` — the second
    /// argument should reference `r_final[0]`. Until the upstream fix
    /// lands, the `Default` impl selects `ShortWay` so
    /// `LambertConfig::default()` does not silently exercise the buggy
    /// path. See `docs/nyx-lambert-bug-report.md` for the full diagnosis
    /// and the tracking notes on when this default can be re-flipped.
    Auto,
    /// Short-way (prograde) transfer: transfer angle < 180°.
    #[default]
    ShortWay,
    /// Long-way (retrograde) transfer: transfer angle > 180°.
    LongWay,
}

impl std::fmt::Display for TransferDirection {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Auto => write!(f, "Auto"),
            Self::ShortWay => write!(f, "Short-way (prograde)"),
            Self::LongWay => write!(f, "Long-way (retrograde)"),
        }
    }
}

/// Configuration for Lambert solver behavior.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LambertConfig {
    /// Transfer direction selection.
    pub direction: TransferDirection,
    /// Number of complete revolutions (0 = direct transfer).
    pub revolutions: u8,
}

/// Physical feasibility classification for a Lambert transfer's conic.
///
/// Lambert's problem produces a conic that connects two ECI states in the
/// requested time of flight; mathematically any such conic is a "valid"
/// solution, but for proximity-ops planning the conic is only useful if its
/// periapsis stays above Earth's surface. This enum surfaces that
/// distinction without making it a solver error — the transfer object is
/// still well-formed, but a caller (or UI) can refuse to act on a
/// `SubSurface` result.
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
    /// non-physical. The altitude is signed (negative if below the
    /// reference radius).
    SubSurface {
        /// Altitude of periapsis above the mean equatorial radius (km).
        periapsis_altitude_km: f64,
    },
}

impl TransferFeasibility {
    /// Compute feasibility from a transfer-orbit state.
    ///
    /// Inputs must be in an inertial frame; ECI is the workspace convention
    /// (every caller in this codebase passes ECI).
    ///
    /// For Lambert transfers, pass the solver-returned velocity at `r`
    /// (`v1_vec` for departure, `v2_vec` for arrival) — not the natural
    /// orbit velocity, which would describe a different conic. At a single
    /// position, two velocities are physically meaningful (pre-burn = the
    /// deputy's natural orbit, post-burn = the transfer arc); the math
    /// here computes the conic implied by `(r, v)` regardless, but only
    /// the post-burn case answers the feasibility question.
    ///
    /// # Invariants
    /// - `r_eci_km.norm()` ≥ [`crate::constants::MIN_POSITION_NORM_KM`]
    /// - `(r_eci_km × v_eci_km_s).norm()` ≥
    ///   [`crate::constants::MIN_ANGULAR_MOMENTUM_NORM_KM2_S`]
    ///   (non-rectilinear orbit; the conic is undefined otherwise)
    ///
    /// `from_state` is currently called only by `build_transfer` after
    /// `validate_inputs` has rejected degenerate geometries; callers
    /// outside that path are responsible for honoring the invariants.
    /// No runtime assertion is performed (per the
    /// `debug_assert!`-as-API-validation prohibition in `CLAUDE.md`).
    #[must_use]
    pub fn from_state(r_eci_km: Vector3<f64>, v_eci_km_s: Vector3<f64>) -> Self {
        // Inline e-vector + p computation rather than delegating to
        // `state_to_keplerian`: that function returns `Err(UnboundOrbit)` for
        // `e ≥ 1`, but hyperbolic Lambert transfers are valid (short TOF +
        // far targets) and `r_p = p / (1 + e)` stays finite and correct on
        // every conic — elliptic, parabolic, and hyperbolic. The elliptic-
        // specific `a · (1 - e)` form is also unsuitable: catastrophic
        // cancellation as `e → 1`.
        let h_vec = r_eci_km.cross(&v_eci_km_s);
        let h = h_vec.norm();
        let r = r_eci_km.norm();
        let v_sq = v_eci_km_s.norm_squared();
        let e_vec = ((v_sq - MU_EARTH / r) * r_eci_km
            - r_eci_km.dot(&v_eci_km_s) * v_eci_km_s)
            / MU_EARTH;
        let e = e_vec.norm();
        let p = (h * h) / MU_EARTH;
        let periapsis_radius_km = p / (1.0 + e);
        let periapsis_altitude_km = periapsis_radius_km - R_EARTH;
        if periapsis_altitude_km >= MIN_PERIAPSIS_ALTITUDE_KM {
            Self::Feasible { periapsis_altitude_km }
        } else {
            Self::SubSurface { periapsis_altitude_km }
        }
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
    /// Characteristic energy C3 = v∞² (km²/s²). Always populated by Izzo.
    pub c3_km2_s2: f64,
    /// Transfer direction used.
    pub direction: TransferDirection,
    /// Physical feasibility of the transfer conic — `SubSurface` indicates
    /// the conic's periapsis dips below
    /// [`crate::constants::MIN_PERIAPSIS_ALTITUDE_KM`] above Earth's
    /// equatorial radius. Computed at solver-build time from the departure
    /// state plus the solver-returned velocity. Independent of `direction`
    /// and `revolutions`.
    pub feasibility: TransferFeasibility,
}

impl LambertTransfer {
    /// Generate a dense ECI trajectory along the Lambert transfer arc.
    ///
    /// The arc is a two-body Keplerian orbit defined by `departure_state`.
    /// Produces `n_steps + 1` states from departure to arrival.
    ///
    /// # Invariants
    /// - `n_steps > 0` (delegates to `propagate_keplerian`; see its invariants)
    ///
    /// # Errors
    /// Returns `ConversionError` if the departure state cannot be converted
    /// to Keplerian elements (should not happen for valid Lambert solutions).
    pub fn densify_arc(&self, n_steps: u32) -> Result<Vec<StateVector>, crate::elements::keplerian_conversions::ConversionError> {
        crate::propagation::keplerian::propagate_keplerian(&self.departure_state, self.tof_s, n_steps)
    }
}

/// Errors from the Lambert solver.
#[derive(Debug, Clone, thiserror::Error)]
pub enum LambertError {
    /// Time of flight must be positive.
    #[error("non-positive time of flight — tof = {tof_s:.6} s")]
    NonPositiveTimeOfFlight {
        /// The non-positive TOF value (seconds).
        tof_s: f64,
    },
    /// Departure and arrival positions are too close.
    #[error("identical positions — separation = {separation_km:.6e} km")]
    IdenticalPositions {
        /// The separation distance (km).
        separation_km: f64,
    },
    /// Invalid input from nyx-space (opaque upstream error).
    ///
    /// The nyx-space Lambert API does not expose structured error data (iteration count,
    /// residual, etc.), so this variant carries the formatted upstream message string.
    /// If nyx ever surfaces structured errors, migrate to dedicated fields here.
    #[error("invalid input — {details}")]
    InvalidInput {
        /// Formatted upstream error message.
        details: String,
    },
    /// Lambert solver (Gooding or Izzo) failed to converge (opaque upstream error).
    ///
    /// nyx-space's Lambert solvers do not expose iteration count or residual
    /// values in their error types, so structured fields cannot be populated.
    /// The formatted message is the best available diagnostic from the
    /// upstream crate. This is the only `String`-shaped field in
    /// [`LambertError`] and must stay until nyx-space publishes a structured
    /// solver error (or this workspace forks the nyx Lambert tools to own
    /// the error type). Every other variant carries its diagnostic as a
    /// typed numeric field, per the error-handling policy in the workspace
    /// `CLAUDE.md`.
    #[error("Lambert solver convergence failure — {details}")]
    SolverConvergenceFailure {
        /// Formatted upstream error message.
        details: String,
    },
}

#[cfg(test)]
mod feasibility_tests {
    use super::{TransferFeasibility, MIN_PERIAPSIS_ALTITUDE_KM, MU_EARTH, R_EARTH};
    use crate::test_helpers::ellipse_state_at_apogee;
    use nalgebra::Vector3;

    /// Tolerance on derived periapsis altitude (km). The derivation is a
    /// handful of f64 operations on km-scale quantities; ~1e-6 km (1 mm)
    /// is comfortably above arithmetic noise and well below any
    /// physically meaningful threshold.
    const PERIAPSIS_ALT_TOL_KM: f64 = 1.0e-6;

    #[test]
    fn circular_leo_is_feasible() {
        // 500 km altitude circular orbit: r = R_EARTH + 500, v = sqrt(μ/r),
        // periapsis altitude = 500 km. Above the 200 km feasibility
        // threshold (`MIN_PERIAPSIS_ALTITUDE_KM`).
        let r_km = Vector3::new(R_EARTH + 500.0, 0.0, 0.0);
        let v_km_s = Vector3::new(0.0, (MU_EARTH / r_km.norm()).sqrt(), 0.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s);
        assert!(matches!(f, TransferFeasibility::Feasible { .. }), "got {f:?}");
        assert!((f.periapsis_altitude_km() - 500.0).abs() < PERIAPSIS_ALT_TOL_KM);
        assert!(!f.is_sub_surface());
    }

    #[test]
    fn elliptical_with_perigee_below_threshold_is_sub_surface() {
        // Ellipse with perigee at 50 km altitude (below the 200 km
        // feasibility threshold, `MIN_PERIAPSIS_ALTITUDE_KM`), apogee at
        // 1000 km altitude. State placed at apogee with the vis-viva
        // apogee speed; expected periapsis altitude is 50 km.
        let (r_km, v_km_s) =
            ellipse_state_at_apogee(R_EARTH + 50.0, R_EARTH + 1000.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s);
        assert!(f.is_sub_surface(), "expected SubSurface, got {f:?}");
        assert!(
            (f.periapsis_altitude_km() - 50.0).abs() < PERIAPSIS_ALT_TOL_KM,
            "periapsis altitude = {} km",
            f.periapsis_altitude_km()
        );
    }

    #[test]
    fn perigee_just_above_threshold_is_feasible() {
        // 1 km above MIN_PERIAPSIS_ALTITUDE_KM. The classifier uses `>=`,
        // so anything strictly above the threshold should classify as
        // feasible. (Exactly at the threshold is f64-fragile because the
        // conic round-trip introduces ~1e-13 km of noise; 1 km of
        // clearance is comfortably above that.)
        let (r_km, v_km_s) = ellipse_state_at_apogee(
            R_EARTH + MIN_PERIAPSIS_ALTITUDE_KM + 1.0,
            R_EARTH + 1000.0,
        );
        let f = TransferFeasibility::from_state(r_km, v_km_s);
        assert!(matches!(f, TransferFeasibility::Feasible { .. }), "got {f:?}");
    }

    #[test]
    fn surface_intersecting_orbit_is_sub_surface() {
        // Perigee below the surface (negative altitude): sub-surface
        // classification, with the altitude reported as a negative number.
        let (r_km, v_km_s) =
            ellipse_state_at_apogee(R_EARTH - 200.0, R_EARTH + 1500.0);
        let f = TransferFeasibility::from_state(r_km, v_km_s);
        assert!(f.is_sub_surface());
        assert!(f.periapsis_altitude_km() < 0.0);
    }

    #[test]
    fn serde_tag_round_trips_as_kind() {
        // The frontend reads `feasibility.kind === 'feasible' / 'sub_surface'`.
        // Lock the tag in serialization AND verify the deserialization round
        // trip recovers an equal value (PartialEq is derived on the enum).
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

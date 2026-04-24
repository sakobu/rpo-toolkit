//! Lambert transfer types and error definitions.
//!
//! The solver functions (`solve_lambert`, `solve_lambert_with_config`) that
//! depend on nyx-space live in `rpo-nyx`. This module retains the domain
//! types used by both the analytical engine and the nyx-backed solver.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

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

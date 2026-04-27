//! WASM bindings for the in-tree Izzo Lambert solver.
//!
//! Replaces the previous WebSocket round-trip to `rpo-api`'s
//! `compute_transfer` route. The browser now solves Lambert locally —
//! microseconds in JS, no network — by calling these bindings directly.

use wasm_bindgen::prelude::*;

use rpo_core::propagation::lambert::{
    LambertConfig, LambertTransfer,
};
use rpo_core::types::StateVector;

use crate::error::WasmError;

/// Solve Lambert's problem between two ECI states using the default
/// configuration (single-revolution, [`rpo_core::propagation::lambert::TransferDirection::Auto`]).
///
/// Mathematically frame-invariant under any inertial frame; ECI is the
/// workspace convention.
///
/// # Errors
///
/// Returns [`WasmError`] with [`crate::error::WasmErrorCode::Lambert`] if
/// the inputs violate the solver invariants (non-positive TOF, degenerate
/// or collinear positions, no convergence, etc.).
#[wasm_bindgen]
pub fn solve_lambert(
    departure: StateVector,
    arrival: StateVector,
) -> Result<LambertTransfer, WasmError> {
    rpo_core::propagation::lambert::solve_lambert(&departure, &arrival)
        .map_err(WasmError::from)
}

/// Solve Lambert's problem with explicit configuration (direction +
/// revolution count).
///
/// For `config.revolutions == 0`, returns the single-revolution transfer
/// in the requested direction. For `config.revolutions == N >= 1`,
/// returns the long-period (low-energy) branch of the M=N family.
///
/// # Errors
///
/// Returns [`WasmError`] with [`crate::error::WasmErrorCode::Lambert`] on
/// any solver failure (input validation, convergence, or infeasible
/// rev count).
#[wasm_bindgen]
pub fn solve_lambert_with_config(
    departure: StateVector,
    arrival: StateVector,
    config: LambertConfig,
) -> Result<LambertTransfer, WasmError> {
    rpo_core::propagation::lambert::solve_lambert_with_config(&departure, &arrival, &config)
        .map_err(WasmError::from)
}

/// Solve every Lambert branch up to `max_revs` revolutions, sorted by
/// total Δv ascending.
///
/// Computes both short-way and long-way branches and concatenates them.
/// Up to `2 * (1 + 2 * max_revs)` branches when every multi-rev branch is
/// feasible — fewer when `tof_s` falls below the M-rev minimum.
///
/// Designed for "browse transfer options" UX where the operator picks a
/// branch interactively. Intentionally separate from
/// [`solve_lambert_with_config`] so the single-solution path stays
/// branch-free.
///
/// # Errors
///
/// Returns [`WasmError`] only when both directions fail; partial results
/// are sorted and returned otherwise.
#[wasm_bindgen]
pub fn solve_lambert_branches(
    departure: StateVector,
    arrival: StateVector,
    max_revs: u8,
) -> Result<Vec<LambertTransfer>, WasmError> {
    rpo_core::propagation::lambert::solve_lambert_branches(&departure, &arrival, max_revs)
        .map_err(WasmError::from)
}

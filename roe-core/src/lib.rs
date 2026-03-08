//! Core astrodynamics library for relative orbital element (ROE) computation.
//!
//! Provides Keplerian element conversions, quasi-nonsingular ROE computation
//! (D'Amico / Koenig formulation), J2-perturbed propagation via the Koenig STM,
//! J2+drag propagation via the DMF STM (Koenig Sec. VIII),
//! and linearized ROE-to-RIC frame mapping.
//!
//! ## Module pipeline
//! `types` → `constants` → `elements` → `roe` → `j2_params` → `stm` → `drag_stm` → `propagator` → `frames`

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

pub mod constants;
pub mod drag_stm;
pub mod elements;
pub mod frames;
pub mod j2_params;
pub mod propagator;
pub mod roe;
pub mod stm;
pub mod types;

pub use drag_stm::{compute_j2_drag_stm, propagate_roe_j2_drag};
pub use elements::{keplerian_to_state, state_to_keplerian};
pub use frames::roe_to_ric;
pub use j2_params::{compute_j2_params, J2Params};
pub use propagator::{
    J2DragStmPropagator, J2StmPropagator, PropagatedState, PropagationError, RelativePropagator,
};
pub use roe::compute_roe;
pub use stm::{compute_stm, compute_stm_with_params, propagate_roe_stm};
pub use types::{DragConfig, KeplerianElements, QuasiNonsingularROE, RICState, StateVector};

#[cfg(test)]
mod test_helpers;

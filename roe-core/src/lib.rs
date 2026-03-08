//! Core astrodynamics library for relative orbital element (ROE) computation.
//!
//! Provides Keplerian element conversions, quasi-nonsingular ROE computation
//! (D'Amico / Koenig formulation), J2-perturbed propagation via the Koenig STM,
//! and linearized ROE-to-RIC frame mapping.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

pub mod constants;
pub mod elements;
pub mod frames;
pub mod j2_params;
pub mod propagator;
pub mod roe;
pub mod stm;
pub mod types;

pub use elements::{keplerian_to_state, state_to_keplerian};
pub use frames::roe_to_ric;
pub use j2_params::{compute_j2_params, J2Params};
pub use propagator::{J2StmPropagator, PropagatedState, PropagationError, RelativePropagator};
pub use roe::compute_roe;
pub use stm::{compute_stm, compute_stm_with_params, propagate_roe_stm};
pub use types::{KeplerianElements, QuasiNonsingularROE, RICState, StateVector};

#[cfg(test)]
mod test_helpers;

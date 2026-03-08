//! Core astrodynamics library for relative orbital element (ROE) computation.
//!
//! Provides Keplerian element conversions, quasi-nonsingular ROE computation
//! (D'Amico / Koenig formulation), and linearized ROE-to-RIC frame mapping.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::must_use_candidate)]

pub mod constants;
pub mod elements;
pub mod frames;
pub mod roe;
pub mod types;

pub use elements::{keplerian_to_state, state_to_keplerian};
pub use frames::roe_to_ric;
pub use roe::compute_roe;
pub use types::{KeplerianElements, QuasiNonsingularROE, RICState, StateVector};

#[cfg(test)]
mod test_helpers;

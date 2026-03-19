//! # rpo-core -- Analytical RPO Mission Planning
//!
//! Astrodynamics library for rendezvous and proximity operations (RPO) in
//! low Earth orbit.  Implements quasi-nonsingular relative orbital elements
//! (D'Amico / Koenig formulation), J2-perturbed propagation via closed-form
//! STMs, impulsive maneuver targeting, passive safety analysis, and
//! multi-waypoint mission planning.
//!
//! ## Quick start
//!
//! ```rust,no_run
//! use rpo_core::prelude::*;
//! ```
//!
//! The [`prelude`] re-exports the essentials for a typical mission planning
//! workflow: state types, mission configuration, and the primary planning
//! functions.
//!
//! ## Modules
//!
//! | Module | Purpose |
//! |--------|---------|
//! | [`types`] | Domain vocabulary: state vectors, Keplerian elements, ROEs, eclipse types |
//! | [`elements`] | Static geometry: ECI/Keplerian/ROE/RIC conversions, frame transforms, eclipse |
//! | [`propagation`] | Trajectory computation: J2 and J2+drag STMs, Lambert solver, covariance, nyx bridge |
//! | [`mission`] | Mission orchestration: planning, targeting, safety, validation, Monte Carlo |
//! | [`constants`] | Physical constants and named tolerances |
//!
//! ## Entry points
//!
//! Most workflows start with one of these functions:
//!
//! - [`mission::plan_waypoint_mission`] — multi-waypoint proximity operations
//! - [`mission::plan_mission`] — classify + Lambert transfer for far-field
//! - [`mission::classify_separation`] — proximity vs. far-field classification
//! - [`mission::validate_mission_nyx`] — nyx full-physics validation
//! - [`mission::run_monte_carlo`] — full-physics Monte Carlo ensemble
//! - [`propagation::solve_lambert`] — Lambert transfer solver

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

pub mod constants;
pub mod elements;
pub mod mission;
pub mod prelude;
pub mod propagation;
pub mod types;

#[cfg(test)]
mod test_helpers;

#[cfg(test)]
mod copy_trait_tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    fn assert_copy<T: Copy>() {}

    #[test]
    fn test_vector3_is_copy() {
        assert_copy::<Vector3<f64>>();
    }

    #[test]
    fn test_epoch_is_copy() {
        assert_copy::<Epoch>();
    }
}

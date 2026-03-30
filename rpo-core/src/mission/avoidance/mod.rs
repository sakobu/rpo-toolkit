//! Collision avoidance (COLA) maneuver computation via inverse GVE.
//!
//! Implements collision avoidance maneuver design using the inverse of the
//! Gauss Variational Equation (GVE) B matrix (D'Amico Eqs. 2.38-2.56).

pub(crate) mod types;
mod classify;
pub(crate) mod solve;
mod verify;

pub use types::{AvoidanceError, AvoidanceManeuver, ColaConfig, CorrectionType};
pub use solve::compute_avoidance;
pub(crate) use types::BURN_TIME_CLAMP_FRACTION;

//! Collision avoidance (COLA) maneuver computation via inverse GVE.
//!
//! Implements collision avoidance maneuver design using the inverse of the
//! Gauss Variational Equation (GVE) B matrix (D'Amico Eqs. 2.38-2.56).

mod classify;
pub(crate) mod solve;
pub(crate) mod types;
mod verify;

pub use solve::compute_avoidance;
pub use types::{AvoidanceError, AvoidanceManeuver, ColaConfig, CorrectionType};
pub(crate) use types::BURN_TIME_CLAMP_FRACTION;

//! J2 and J2+drag propagation via closed-form STMs.

pub mod covariance;
pub mod drag_stm;
pub mod j2_params;
pub mod keplerian;
pub mod propagator;
pub mod stm;

pub use covariance::propagate_mission_covariance;
pub use drag_stm::{compute_j2_drag_stm, propagate_roe_j2_drag};
pub use j2_params::{compute_j2_params, J2Params};
pub use keplerian::propagate_keplerian;
pub use propagator::{PropagatedState, PropagationError, PropagationModel};
pub use stm::{compute_stm, compute_stm_with_params, propagate_roe_stm};

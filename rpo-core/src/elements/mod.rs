//! Element conversions, ROE computation, frame mapping, and GVE.

pub mod keplerian_conversions;
pub mod eci_ric_dcm;
pub mod gve;
pub mod roe_to_ric;
pub mod roe;

pub use keplerian_conversions::{keplerian_to_state, state_to_keplerian, ConversionError};
pub use eci_ric_dcm::{
    eci_to_ric_dcm, eci_to_ric_dv, eci_to_ric_relative, ric_to_eci_dv, ric_to_eci_position,
    ric_to_eci_state,
};
pub use gve::{apply_maneuver, compute_b_matrix};
pub use roe_to_ric::{
    compute_t_matrix, compute_t_position, compute_t_velocity, ric_position_to_roe, roe_to_ric,
    RicError,
};
pub use roe::compute_roe;

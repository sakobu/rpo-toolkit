//! Full-physics validation via nyx-space: trajectory comparison, statistics, and eclipse cross-check.

mod errors;
mod eclipse;
mod statistics;
mod trajectory;

pub use errors::ValidationError;
pub use trajectory::{
    validate_leg_nyx, validate_mission_nyx, ColaBurn, ColaValidationInput,
    LegValidationOutput, ValidationConfig, convert_cola_to_burns,
};

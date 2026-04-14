//! Full-physics validation via nyx-space: trajectory comparison, statistics, and eclipse cross-check.

mod eclipse;
mod errors;
mod statistics;
#[cfg(test)]
mod test_scenario;
mod trajectory;

pub use errors::ValidationError;
pub use trajectory::{
    validate_mission_nyx, ColaBurn, ColaValidationInput, ValidationConfig, ValidationPipelineCtx,
};
pub(crate) use trajectory::convert_cola_to_burns;

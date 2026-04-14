//! Full-physics validation orchestrator: COLA injection, per-leg comparison,
//! and mission-level pipeline assembly.

mod cola;
mod leg;
mod pipeline;

pub use cola::{ColaBurn, ColaValidationInput};
pub use pipeline::{validate_mission_nyx, ValidationConfig, ValidationPipelineCtx};

pub(crate) use cola::convert_cola_to_burns;

#[cfg(test)]
pub(in crate::validation) use leg::LegPropagationCtx;

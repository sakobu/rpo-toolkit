//! Server-side pipeline: nyx-dependent mission orchestration.
//!
//! Most of the pipeline (Lambert + classification + perch geometry +
//! enrichment + arc densification) is now WASM-eligible and lives in
//! [`rpo_core::pipeline`]. This module keeps only the pieces that genuinely
//! require full physics — currently just [`compute_validation_burns`], which
//! converts COLA assessments into nyx validation burns.
//!
//! ## Re-exports for backwards compatibility
//!
//! `compute_transfer`, `execute_mission`, and `plan_mission` are re-exported
//! from `rpo_core::pipeline` so existing callers (`rpo-cli`, `rpo-api`)
//! continue compiling without import retargeting; the actual implementations
//! live in `rpo_core` and run on the in-tree Izzo solver.

pub mod errors;

pub use errors::PipelineError;

pub use rpo_core::pipeline::{compute_transfer, execute_mission, plan_mission};

use rpo_core::mission::config::SafetyConfig;
use rpo_core::mission::avoidance::ColaConfig;
use rpo_core::pipeline::{compute_safety_analysis, SafetyAnalysis};

use crate::validation::convert_cola_to_burns;

/// Compute safety analysis and derive COLA burns for validation injection.
///
/// Combines [`compute_safety_analysis`] and [`convert_cola_to_burns`] into a
/// single call, ensuring COLA burns are always derived consistently from the
/// safety analysis. Used by both CLI and API validate handlers. Stays in
/// `rpo-nyx` because [`convert_cola_to_burns`] depends on the nyx full-physics
/// validation surface.
///
/// # Errors
/// Returns [`crate::validation::ValidationError`] if a COLA burn epoch is out
/// of bounds.
pub fn compute_validation_burns(
    wp_mission: &rpo_core::mission::types::WaypointMission,
    safety: Option<&SafetyConfig>,
    cola: Option<&ColaConfig>,
    propagator: &rpo_core::propagation::propagator::PropagationModel,
) -> Result<(SafetyAnalysis, Vec<crate::validation::ColaBurn>), crate::validation::ValidationError>
{
    let analysis = compute_safety_analysis(wp_mission, safety, cola, propagator);
    let burns = convert_cola_to_burns(analysis.cola.as_deref(), wp_mission)?;
    Ok((analysis, burns))
}

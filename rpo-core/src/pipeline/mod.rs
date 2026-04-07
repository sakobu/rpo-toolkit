//! Pipeline module: shared mission orchestration for CLI, API, and WASM.
//!
//! Owns the canonical input/output types and the mission planning entry
//! points. Entry points ([`execute_mission_from_transfer`],
//! [`replan_from_transfer`]) accept a pre-computed [`TransferResult`].
//! Server-only wrappers (`compute_transfer`, `execute_mission`, `replan_mission`)
//! that require nyx-space live in `rpo-nyx`.
//!
//! ## DAG position
//!
//! `constants → types → elements → propagation → mission → pipeline`
//!
//! The pipeline module composes — it does not modify any upstream module.

pub mod convert;
pub mod errors;
pub mod execute;
pub mod projections;
pub mod types;

pub use convert::{resolve_propagator, to_propagation_model, to_waypoints};
pub use errors::PipelineError;
pub use execute::{
    accept_waypoint_enrichment, apply_perch_enrichment, build_lean_plan_result, build_output,
    BuildOutputCtx, compute_formation_report,
    compute_free_drift_analysis, compute_free_drift_poca, compute_mission_covariance,
    compute_poca_analysis, compute_safety_analysis, execute_mission_from_transfer,
    plan_waypoints_from_transfer, replan_from_transfer, suggest_enrichment,
    suggest_enrichment_from_parts,
};
pub use projections::{
    LeanPlanResult, LegSummary, LegTrajectory, TrajectoryPoint, TransferSummary,
    propagated_to_point, resample_propagated,
};
pub use types::{
    EnrichmentSuggestion, PipelineInput, PipelineOutput, PlanVariant, PropagatorChoice,
    SafetyAnalysis, SpacecraftChoice, TransferResult, WaypointInput,
};

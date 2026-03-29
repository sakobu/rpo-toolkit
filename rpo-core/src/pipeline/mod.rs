//! Pipeline module: shared mission orchestration for CLI and API.
//!
//! Owns the canonical input/output types and the `execute_mission()` /
//! `compute_transfer()` / `replan_mission()` entry points that compose
//! the core planning primitives into a complete mission pipeline.
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
    build_output, compute_free_drift_analysis, compute_free_drift_poca,
    compute_mission_covariance, compute_poca_analysis, compute_transfer, execute_mission,
    plan_waypoints_from_transfer, replan_mission,
};
pub use projections::{
    LeanPlanResult, LegSummary, LegTrajectory, TrajectoryPoint, TransferSummary,
    propagated_to_point, resample_propagated,
};
pub use types::{
    PipelineInput, PipelineOutput, PropagatorChoice, SpacecraftChoice, TransferResult, WaypointInput,
};

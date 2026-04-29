//! Pipeline module: shared mission orchestration for CLI, API, and WASM.
//!
//! Owns the canonical input/output types and the mission planning entry
//! points. Two flavors:
//!
//! - **Transfer-driven** ([`compute_transfer`], [`execute_mission`],
//!   [`plan_mission`]) — start from raw chief/deputy states and run Lambert
//!   in-tree (Izzo via [`crate::propagation::lambert`]).
//! - **Pre-computed transfer** ([`execute_mission_from_transfer`],
//!   [`replan_from_transfer`]) — accept a [`TransferResult`] from a prior
//!   [`compute_transfer`] call.
//!
//! All functions are WASM-eligible; nothing here depends on nyx-space.
//!
//! ## DAG position
//!
//! `constants → types → elements → propagation → mission → pipeline`

pub mod convert;
pub mod errors;
pub mod execute;
pub mod projections;
pub mod transfer;
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
pub use transfer::{
    compute_transfer, compute_transfer_with_enrichment, execute_mission, plan_mission,
    ArcDensificationError,
};
pub use crate::mission::formation::EnrichmentSuggestion;
pub use types::{
    default_perch, MissionInput, PipelineInput, PipelineOutput,
    PlanVariant, PropagatorChoice, SafetyAnalysis, SpacecraftChoice,
    TransferComputationInput, TransferResult, WaypointInput, DEFAULT_LAMBERT_TOF_S,
    DEFAULT_PERCH_ALONG_TRACK_KM,
};

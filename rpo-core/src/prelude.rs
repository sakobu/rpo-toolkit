//! Convenience re-exports for common mission planning workflows.
//!
//! ```rust,no_run
//! use rpo_core::prelude::*;
//! ```
//!
//! Imports the minimum set of types and functions needed to classify
//! two spacecraft states, plan a waypoint mission, and read back results.
//! For Monte Carlo, covariance, and eclipse analysis, import
//! from the relevant module directly.
//!
//! Entry points (`execute_mission_from_transfer`, `replan_from_transfer`)
//! accept a pre-computed `TransferResult`. Server-only wrappers
//! (`execute_mission`, `replan_mission`, `plan_mission`) that require
//! nyx-space live in `rpo-nyx`.

// Core domain types
pub use crate::types::{
    DepartureState, KeplerianElements, QuasiNonsingularROE, SpacecraftConfig, StateVector,
};

// Mission planning
pub use crate::mission::{
    classify_separation, plan_waypoint_mission, MissionConfig, MissionPhase, Waypoint,
    WaypointMission,
};

// Propagation model selection
pub use crate::propagation::PropagationModel;

// Pipeline
pub use crate::pipeline::{
    execute_mission_from_transfer, replan_from_transfer, PipelineInput, PipelineOutput,
    PropagatorChoice, TransferResult, WaypointInput,
};

//! Convenience re-exports for common mission planning workflows.
//!
//! ```rust,no_run
//! use rpo_core::prelude::*;
//! ```
//!
//! Imports the minimum set of types and functions needed to classify
//! two spacecraft states, plan a waypoint mission, and read back results.
//! For validation, Monte Carlo, covariance, and eclipse analysis, import
//! from the relevant module directly.

// Core domain types
pub use crate::types::{
    DepartureState, KeplerianElements, QuasiNonsingularROE, SpacecraftConfig, StateVector,
};

// Mission planning
pub use crate::mission::{
    classify_separation, plan_mission, plan_waypoint_mission, MissionConfig, MissionPhase, Waypoint,
    WaypointMission,
};

// Propagation model selection
pub use crate::propagation::PropagationModel;

// Pipeline
pub use crate::pipeline::{
    execute_mission, replan_mission, PipelineInput, PipelineOutput, PropagatorChoice, WaypointInput,
};

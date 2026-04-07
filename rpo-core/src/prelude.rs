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
//!
//! Some functions (`plan_mission`, `execute_mission`, `replan_mission`)
//! require the `"server"` feature (enabled by default). They are not
//! available when building with `--no-default-features` (e.g., for WASM).
//! WASM-eligible entry points (`execute_mission_from_transfer`,
//! `replan_from_transfer`) are always available.

// Core domain types
pub use crate::types::{
    DepartureState, KeplerianElements, QuasiNonsingularROE, SpacecraftConfig, StateVector,
};

// Mission planning
pub use crate::mission::{
    classify_separation, plan_waypoint_mission, MissionConfig, MissionPhase, Waypoint,
    WaypointMission,
};
#[cfg(feature = "server")]
pub use crate::mission::plan_mission;

// Propagation model selection
pub use crate::propagation::PropagationModel;

// Pipeline
pub use crate::pipeline::{
    execute_mission_from_transfer, replan_from_transfer, PipelineInput, PipelineOutput,
    PropagatorChoice, TransferResult, WaypointInput,
};
#[cfg(feature = "server")]
pub use crate::pipeline::{execute_mission, replan_mission};

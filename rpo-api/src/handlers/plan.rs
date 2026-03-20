//! Mission planning handlers: plan, move waypoint, update config.
//!
//! These use the shared pipeline from rpo-core. All are pure functions
//! that return immediately (microseconds–100ms).

use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::mission::types::WaypointMission;
use rpo_core::pipeline::{execute_mission, replan_mission};

use crate::error::ApiError;
use crate::protocol::{MissionDefinition, MissionResultPayload};

/// Plan a full mission: classify → Lambert → waypoints → safety → eclipse.
///
/// Pure function — runs in microseconds to ~100ms. Optional covariance
/// propagation (when `navigation_accuracy` is provided) adds a few milliseconds.
///
/// # Errors
/// Returns [`ApiError`] if classification, Lambert, or waypoint planning fails.
pub fn handle_plan(
    def: &MissionDefinition,
    _almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    let output = execute_mission(def)?;
    Ok(output)
}

/// Replan from a moved waypoint (keeps earlier legs).
///
/// When `cached_mission` is provided, legs before `modified_index` are reused
/// without re-solving. When `None`, falls back to planning the full mission first.
///
/// # Errors
/// Returns [`ApiError`] if planning or replanning fails.
pub fn handle_move_waypoint(
    def: &MissionDefinition,
    modified_index: usize,
    cached_mission: Option<&WaypointMission>,
    _almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    let output = replan_mission(def, modified_index, cached_mission)?;
    Ok(output)
}

/// Re-solve all waypoints with new config/propagator.
///
/// Identical to `handle_plan` — the "update" semantic is a frontend optimization
/// hint (the server always recomputes from scratch).
///
/// # Errors
/// Returns [`ApiError`] if planning fails.
pub fn handle_update_config(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    handle_plan(def, almanac)
}

//! Mission planning handlers: plan, move waypoint, update config.
//!
//! These mirror the CLI pipeline: classify → Lambert → perch → waypoints.
//! All are pure functions that return immediately (microseconds–100ms).

use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::mission::{
    compute_transfer_eclipse, plan_mission, plan_waypoint_mission, replan_from_waypoint,
    PerchGeometry,
};
use rpo_core::types::DepartureState;

use crate::convert::{to_propagation_model, to_waypoints};
use crate::error::ApiError;
use crate::protocol::{MissionDefinition, MissionResultPayload};

/// Intermediate result from classification + Lambert + perch computation.
struct TransferResult {
    plan: rpo_core::mission::MissionPlan,
    arrival_epoch: hifitime::Epoch,
    lambert_dv_km_s: f64,
}

/// Classify separation, solve Lambert if far-field, compute perch handoff states.
fn compute_transfer(def: &MissionDefinition) -> Result<TransferResult, ApiError> {
    let perch = def
        .perch
        .clone()
        .unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 });
    let proximity = def.proximity.unwrap_or_default();
    let lambert_tof_s = def.lambert_tof_s.unwrap_or(3600.0);
    let lambert_cfg = def.lambert_config.clone().unwrap_or_default();

    let plan = plan_mission(
        &def.chief,
        &def.deputy,
        &perch,
        &proximity,
        lambert_tof_s,
        &lambert_cfg,
    )?;

    let lambert_dv_km_s = plan.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);
    let arrival_epoch =
        def.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);

    Ok(TransferResult {
        plan,
        arrival_epoch,
        lambert_dv_km_s,
    })
}

/// Build a full `MissionResultPayload` from a planned mission.
fn build_result(
    def: &MissionDefinition,
    transfer: &TransferResult,
    wp_mission: rpo_core::mission::WaypointMission,
    auto_drag_config: Option<rpo_core::propagation::DragConfig>,
) -> MissionResultPayload {
    let transfer_eclipse = transfer.plan.transfer.as_ref().and_then(|t| {
        compute_transfer_eclipse(t, &def.chief, 200).ok()
    });

    let total_dv_km_s = transfer.lambert_dv_km_s + wp_mission.total_dv_km_s;

    MissionResultPayload {
        phase: transfer.plan.phase.clone(),
        transfer: transfer.plan.transfer.clone(),
        transfer_eclipse,
        mission: wp_mission,
        total_dv_km_s,
        auto_drag_config,
        covariance: None,
    }
}

/// Plan a full mission: classify → Lambert → waypoints → safety → eclipse.
///
/// Pure function — runs in microseconds to ~100ms.
pub fn handle_plan(
    def: &MissionDefinition,
    _almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    let transfer = compute_transfer(def)?;
    let propagator = to_propagation_model(&def.propagator);
    let waypoints = to_waypoints(&def.waypoints);

    let departure = DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    };

    let wp_mission =
        plan_waypoint_mission(&departure, &waypoints, &def.config, &propagator)?;

    Ok(build_result(def, &transfer, wp_mission, None))
}

/// Replan from a moved waypoint (keeps earlier legs).
///
/// Pure function — runs in microseconds.
pub fn handle_move_waypoint(
    def: &MissionDefinition,
    modified_index: usize,
    _almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    let transfer = compute_transfer(def)?;
    let propagator = to_propagation_model(&def.propagator);
    let waypoints = to_waypoints(&def.waypoints);

    let departure = DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    };

    // First plan the full mission to get the existing legs
    let full_mission =
        plan_waypoint_mission(&departure, &waypoints, &def.config, &propagator)?;

    // Then replan from the modified index
    let wp_mission = replan_from_waypoint(
        &full_mission,
        modified_index,
        &waypoints,
        &departure,
        &def.config,
        &propagator,
    )?;

    Ok(build_result(def, &transfer, wp_mission, None))
}

/// Re-solve all waypoints with new config/propagator.
///
/// Identical to `handle_plan` — the "update" semantic is a frontend optimization
/// hint (the server always recomputes from scratch).
pub fn handle_update_config(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
) -> Result<MissionResultPayload, ApiError> {
    handle_plan(def, almanac)
}

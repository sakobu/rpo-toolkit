//! Shared planning logic for background handlers (validate, mc).
//!
//! Extracts the duplicated phases 1–3 (plan mission → compute perch states →
//! resolve propagator → plan waypoints) into a single helper.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::mission::{
    plan_mission, plan_waypoint_mission, MissionPlan, PerchGeometry, ProximityConfig,
    WaypointMission,
};
use rpo_core::propagation::{propagate_keplerian, LambertConfig, PropagationModel};
use rpo_core::types::{SpacecraftConfig, StateVector};

use crate::convert::{resolve_propagator, to_propagation_model, to_waypoints};
use crate::error::ApiError;
use crate::protocol::MissionDefinition;

/// Resolved mission defaults from a `MissionDefinition`.
///
/// Centralizes the default values for perch, proximity, Lambert TOF, and
/// Lambert config so they stay consistent across plan and background handlers.
pub(crate) struct MissionDefaults {
    /// Perch geometry (defaults to V-bar at 5 km).
    pub perch: PerchGeometry,
    /// Proximity thresholds (defaults to `ProximityConfig::default()`).
    pub proximity: ProximityConfig,
    /// Lambert time-of-flight in seconds (defaults to 3600).
    pub lambert_tof_s: f64,
    /// Lambert solver config (defaults to `LambertConfig::default()`).
    pub lambert_cfg: LambertConfig,
}

impl MissionDefaults {
    /// Resolve mission defaults from a wire-format `MissionDefinition`.
    pub(crate) fn from_definition(def: &MissionDefinition) -> Self {
        Self {
            perch: def
                .perch
                .clone()
                .unwrap_or(PerchGeometry::VBar { along_track_km: 5.0 }),
            proximity: def.proximity.unwrap_or_default(),
            lambert_tof_s: def.lambert_tof_s.unwrap_or(3600.0),
            lambert_cfg: def.lambert_config.clone().unwrap_or_default(),
        }
    }
}

/// Result of the shared planning prefix (phases 1–3).
///
/// Contains everything the background handlers need after planning:
/// perch ECI states, resolved propagator, and the planned waypoint mission.
pub(crate) struct PlannedMission {
    /// Chief ECI state at Lambert arrival (or original if proximity).
    pub perch_chief: StateVector,
    /// Deputy ECI state at perch (or original if proximity).
    pub perch_deputy: StateVector,
    /// Resolved propagation model (J2 or J2+drag after auto-drag).
    pub propagator: PropagationModel,
    /// The underlying mission plan (phase, transfer, `perch_roe`, `chief_at_arrival`).
    pub plan: MissionPlan,
    /// Planned waypoint mission.
    pub wp_mission: WaypointMission,
}

/// Shared planning prefix for background handlers.
///
/// Executes the phases that are identical in validate and mc:
/// 1. Resolve perch defaults, call `plan_mission()`, compute perch ECI states
/// 2. Check cancellation
/// 3. Resolve propagator (with optional auto-drag)
/// 4. Convert waypoints, build `DepartureState`, call `plan_waypoint_mission()`
///
/// `chief_config`/`deputy_config` are passed in because mc requires them
/// (errors if missing) while validate defaults them.
///
/// # Errors
/// Returns [`ApiError`] if planning, propagation, or propagator resolution fails,
/// or if the operation is cancelled.
pub(crate) fn plan_and_prepare(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
    cancel: &Arc<AtomicBool>,
    auto_drag: bool,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
) -> Result<PlannedMission, ApiError> {
    // Phase 1: Plan the mission
    let defaults = MissionDefaults::from_definition(def);

    let plan = plan_mission(
        &def.chief,
        &def.deputy,
        &defaults.perch,
        &defaults.proximity,
        defaults.lambert_tof_s,
        &defaults.lambert_cfg,
    )?;

    let arrival_epoch =
        def.chief.epoch + hifitime::Duration::from_seconds(defaults.lambert_tof_s);

    let (perch_chief, perch_deputy) = if let Some(ref transfer) = plan.transfer {
        let chief_traj = propagate_keplerian(&def.chief, defaults.lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or(ApiError::InvalidInput(
                crate::error::InvalidInputError::EmptyTrajectory,
            ))?
            .clone();
        let deputy_at_perch = StateVector {
            epoch: arrival_epoch,
            position_eci_km: transfer.arrival_state.position_eci_km,
            velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
                + transfer.arrival_dv_eci_km_s,
        };
        (chief_at_arrival, deputy_at_perch)
    } else {
        (def.chief.clone(), def.deputy.clone())
    };

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 2: Resolve propagator
    let (propagator, _derived_drag) = resolve_propagator(
        auto_drag,
        &perch_chief,
        &perch_deputy,
        chief_config,
        deputy_config,
        almanac,
        to_propagation_model(&def.propagator),
    )?;

    // Phase 3: Plan waypoints
    let waypoints = to_waypoints(&def.waypoints);
    let departure = rpo_core::types::DepartureState {
        roe: plan.perch_roe,
        chief: plan.chief_at_arrival,
        epoch: arrival_epoch,
    };
    let wp_mission =
        plan_waypoint_mission(&departure, &waypoints, &def.config, &propagator)?;

    Ok(PlannedMission {
        perch_chief,
        perch_deputy,
        propagator,
        plan,
        wp_mission,
    })
}

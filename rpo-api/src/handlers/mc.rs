//! Monte Carlo handler: full-physics ensemble analysis with progress polling.
//!
//! Designed to be called from `spawn_blocking`. Uses `MonteCarloControl` for
//! cooperative cancellation and progress reporting via atomic counters.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{
    plan_mission, plan_waypoint_mission, propagate_mission_covariance, run_monte_carlo,
    MonteCarloControl, MonteCarloInput, MonteCarloReport,
};
use rpo_core::propagation::{propagate_keplerian, ric_accuracy_to_roe_covariance};
use rpo_core::types::{DepartureState, StateVector};

use crate::convert::{resolve_propagator, to_propagation_model, to_waypoints};
use crate::error::ApiError;
use crate::handlers::validate::ProgressUpdate;
use crate::protocol::MissionDefinition;

/// Default covariance sample count per leg.
const DEFAULT_COVARIANCE_SAMPLES: usize =
    rpo_core::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;

/// Run full-physics Monte Carlo with progress polling and cancellation.
///
/// Plans the mission first, optionally runs covariance propagation,
/// then executes the MC ensemble. Progress is reported via `MonteCarloControl`
/// atomic counters, polled by the WebSocket handler on a timer.
#[allow(clippy::too_many_lines)]
pub fn handle_mc(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
    progress_tx: mpsc::Sender<ProgressUpdate>,
    cancel: Arc<AtomicBool>,
    auto_drag: bool,
) -> Result<MonteCarloReport, ApiError> {
    // Validate required inputs
    let chief_config = def
        .chief_config
        .ok_or_else(|| ApiError::InvalidMessage("chief_config required for MC".into()))?;
    let deputy_config = def
        .deputy_config
        .ok_or_else(|| ApiError::InvalidMessage("deputy_config required for MC".into()))?;
    let mc_config = def
        .monte_carlo
        .as_ref()
        .ok_or_else(|| ApiError::InvalidMessage("monte_carlo config required for MC".into()))?;

    // Phase 1: Plan the mission
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "mc".into(),
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    });

    let perch = def
        .perch
        .clone()
        .unwrap_or(rpo_core::mission::PerchGeometry::VBar { along_track_km: 5.0 });
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

    let arrival_epoch =
        def.chief.epoch + hifitime::Duration::from_seconds(lambert_tof_s);

    let (perch_chief, perch_deputy) = if let Some(ref transfer) = plan.transfer {
        let chief_traj = propagate_keplerian(&def.chief, lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or_else(|| ApiError::InvalidMessage("empty chief trajectory".into()))?
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
        &chief_config,
        &deputy_config,
        almanac,
        to_propagation_model(&def.propagator),
    )?;

    // Phase 3: Plan waypoints
    let waypoints = to_waypoints(&def.waypoints);
    let departure = DepartureState {
        roe: plan.perch_roe,
        chief: plan.chief_at_arrival,
        epoch: arrival_epoch,
    };
    let mission_config = def.config.clone();
    let wp_mission =
        plan_waypoint_mission(&departure, &waypoints, &mission_config, &propagator)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 4: Optional covariance propagation
    let covariance_report = if let Some(ref nav) = def.navigation_accuracy {
        let initial_p = ric_accuracy_to_roe_covariance(nav, &plan.chief_at_arrival)?;
        let report = propagate_mission_covariance(
            &wp_mission,
            &initial_p,
            nav,
            def.maneuver_uncertainty.as_ref(),
            &propagator,
            DEFAULT_COVARIANCE_SAMPLES,
        )?;
        Some(report)
    } else {
        None
    };

    // Phase 5: Run Monte Carlo with control hooks
    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "mc".into(),
        detail: Some(format!("Running {} MC samples...", mc_config.num_samples)),
        fraction: Some(0.1),
    });

    let mc_control = MonteCarloControl {
        progress: Arc::new(AtomicU32::new(0)),
        cancel: cancel.clone(),
    };

    let mc_input = MonteCarloInput {
        nominal_mission: &wp_mission,
        initial_chief: &perch_chief,
        initial_deputy: &perch_deputy,
        config: mc_config,
        mission_config: &mission_config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &propagator,
        almanac,
        covariance_report: covariance_report.as_ref(),
        control: Some(&mc_control),
    };

    let report = run_monte_carlo(&mc_input)?;

    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "mc".into(),
        detail: Some("Monte Carlo complete".into()),
        fraction: Some(1.0),
    });

    Ok(report)
}

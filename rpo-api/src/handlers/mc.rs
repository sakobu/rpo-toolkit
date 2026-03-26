//! Monte Carlo handler: full-physics ensemble analysis with progress polling.
//!
//! Designed to be called from `spawn_blocking`. Uses `MonteCarloControl` for
//! cooperative cancellation and progress reporting via atomic counters.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::types::WaypointMission;
use rpo_core::mission::{
    run_monte_carlo, MonteCarloConfig, MonteCarloControl, MonteCarloInput, MonteCarloReport,
};
use rpo_core::pipeline::{compute_mission_covariance, TransferResult, WaypointInput};
use rpo_core::propagation::covariance::{ManeuverUncertainty, NavigationAccuracy};
use rpo_core::propagation::PropagationModel;
use rpo_core::types::{SpacecraftConfig, StateVector};

use super::common::{replan_if_drag_changed, resolve_drag_and_propagator, send_progress};
use crate::error::ApiError;
use crate::protocol::{ProgressPhase, ProgressUpdate};

/// Request payload for Monte Carlo, cloned from session state.
///
/// Contains all data needed to run MC independently of the session,
/// since MC runs on a blocking thread.
pub struct McRequest {
    /// The planned waypoint mission (nominal reference).
    pub mission: WaypointMission,
    /// Chief ECI state at perch (mission start).
    pub perch_chief: StateVector,
    /// Deputy ECI state at perch (mission start).
    pub perch_deputy: StateVector,
    /// Chief spacecraft physical properties.
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft physical properties.
    pub deputy_config: SpacecraftConfig,
    /// Monte Carlo ensemble configuration.
    pub mc_config: MonteCarloConfig,
    /// Mission solver configuration.
    pub mission_config: MissionConfig,
    /// Propagation model.
    pub propagator: PropagationModel,
    /// Navigation accuracy for covariance (optional).
    pub navigation_accuracy: Option<NavigationAccuracy>,
    /// Maneuver uncertainty for covariance (optional).
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Auto-derive drag rates before MC.
    pub auto_drag: bool,
    /// Waypoint inputs (needed for drag-based replan).
    pub waypoints: Vec<WaypointInput>,
    /// Transfer result (needed for departure state in replan + covariance).
    pub transfer: TransferResult,
}

/// Run full-physics Monte Carlo with progress polling and cancellation.
///
/// # Errors
/// Returns [`ApiError`] if planning fails, propagation fails, or the operation is cancelled.
pub fn handle_mc(
    request: McRequest,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
) -> Result<MonteCarloReport, ApiError> {
    send_progress(progress_tx, ProgressPhase::Mc, "Planning mission...", 0.0);

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Resolve propagator (with optional auto-drag)
    let (propagator, derived_drag) = resolve_drag_and_propagator(
        request.auto_drag,
        &request.transfer,
        &request.chief_config,
        &request.deputy_config,
        almanac,
        &request.propagator,
    )?;

    // If auto_drag changed the propagator, replan the mission
    let mission = replan_if_drag_changed(
        request.auto_drag,
        derived_drag.as_ref(),
        request.mission,
        &request.transfer,
        &request.waypoints,
        &request.mission_config,
        &propagator,
    )?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Optional covariance propagation for MC cross-check
    let covariance_report = request.navigation_accuracy.as_ref().map(|nav| {
        compute_mission_covariance(
            &mission,
            &request.transfer.plan.chief_at_arrival,
            nav,
            request.maneuver_uncertainty.as_ref(),
            &propagator,
        )
    }).transpose()?;

    // Run Monte Carlo
    send_progress(
        progress_tx,
        ProgressPhase::Mc,
        &format!("Running {} MC samples...", request.mc_config.num_samples),
        0.1,
    );

    let mc_control = MonteCarloControl {
        progress: Arc::new(AtomicU32::new(0)),
        cancel: cancel.clone(),
    };

    let mc_input = MonteCarloInput {
        nominal_mission: &mission,
        initial_chief: &request.perch_chief,
        initial_deputy: &request.perch_deputy,
        config: &request.mc_config,
        mission_config: &request.mission_config,
        chief_config: &request.chief_config,
        deputy_config: &request.deputy_config,
        propagator: &propagator,
        almanac,
        covariance_report: covariance_report.as_ref(),
        control: Some(&mc_control),
    };

    let report = run_monte_carlo(&mc_input)?;

    send_progress(progress_tx, ProgressPhase::Mc, "Monte Carlo complete", 1.0);

    Ok(report)
}


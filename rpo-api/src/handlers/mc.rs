//! Monte Carlo handler: full-physics ensemble analysis with progress polling.
//!
//! Designed to be called from `spawn_blocking`. Uses `MonteCarloControl` for
//! cooperative cancellation and progress reporting via atomic counters.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::mission::{run_monte_carlo, MonteCarloControl, MonteCarloInput, MonteCarloReport};
use rpo_core::pipeline::{
    compute_mission_covariance, compute_transfer, plan_waypoints_from_transfer,
};

use super::common::resolve_drag_and_propagator;
use crate::error::{require_field, ApiError};
use crate::protocol::{MissionDefinition, ProgressPhase, ProgressUpdate};

/// Run full-physics Monte Carlo with progress polling and cancellation.
///
/// # Errors
/// Returns [`ApiError`] if required configs are missing, planning fails,
/// propagation fails, or the operation is cancelled.
pub fn handle_mc(
    def: &MissionDefinition,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
    auto_drag: bool,
) -> Result<MonteCarloReport, ApiError> {
    let chief_config = require_field(def.chief_config, "chief_config", "Monte Carlo")?.resolve();
    let deputy_config = require_field(def.deputy_config, "deputy_config", "Monte Carlo")?.resolve();
    let mc_config = require_field(def.monte_carlo.as_ref(), "monte_carlo", "Monte Carlo")?;

    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Mc,
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    // Phase 1: Compute transfer
    let transfer = compute_transfer(def)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Phase 2: Resolve propagator
    let (propagator, _derived_drag) =
        resolve_drag_and_propagator(auto_drag, &transfer, &chief_config, &deputy_config, almanac, def)?;

    // Phase 3: Plan waypoints
    let wp_mission = plan_waypoints_from_transfer(&transfer, def, &propagator)?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Optional covariance propagation
    let covariance_report = if let Some(ref nav) = def.navigation_accuracy {
        Some(compute_mission_covariance(
            &wp_mission,
            &transfer.plan.chief_at_arrival,
            nav,
            def.maneuver_uncertainty.as_ref(),
            &propagator,
        )?)
    } else {
        None
    };

    // Run Monte Carlo
    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Mc,
        detail: Some(format!("Running {} MC samples...", mc_config.num_samples)),
        fraction: Some(0.1),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    let mc_control = MonteCarloControl {
        progress: Arc::new(AtomicU32::new(0)),
        cancel: cancel.clone(),
    };

    let mc_input = MonteCarloInput {
        nominal_mission: &wp_mission,
        initial_chief: &transfer.perch_chief,
        initial_deputy: &transfer.perch_deputy,
        config: mc_config,
        mission_config: &def.config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &propagator,
        almanac,
        covariance_report: covariance_report.as_ref(),
        control: Some(&mc_control),
    };

    let report = run_monte_carlo(&mc_input)?;

    if let Err(e) = progress_tx.try_send(ProgressUpdate {
        phase: ProgressPhase::Mc,
        detail: Some("Monte Carlo complete".into()),
        fraction: Some(1.0),
    }) {
        tracing::debug!("Progress update dropped: {e}");
    }

    Ok(report)
}

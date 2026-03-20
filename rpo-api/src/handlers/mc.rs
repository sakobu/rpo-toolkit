//! Monte Carlo handler: full-physics ensemble analysis with progress polling.
//!
//! Designed to be called from `spawn_blocking`. Uses `MonteCarloControl` for
//! cooperative cancellation and progress reporting via atomic counters.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use tokio::sync::mpsc;

use rpo_core::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;
use rpo_core::mission::{
    propagate_mission_covariance, run_monte_carlo, MonteCarloControl, MonteCarloInput,
    MonteCarloReport,
};
use rpo_core::propagation::ric_accuracy_to_roe_covariance;

use crate::error::{require_field, ApiError};
use crate::handlers::validate::ProgressUpdate;
use crate::protocol::MissionDefinition;

/// Run full-physics Monte Carlo with progress polling and cancellation.
///
/// Plans the mission first, optionally runs covariance propagation,
/// then executes the MC ensemble. Progress is reported via `MonteCarloControl`
/// atomic counters, polled by the WebSocket handler on a timer.
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
    // Validate required inputs (mc-specific: configs are NOT optional)
    let chief_config = require_field(def.chief_config, "chief_config", "Monte Carlo")?;
    let deputy_config = require_field(def.deputy_config, "deputy_config", "Monte Carlo")?;
    let mc_config = require_field(def.monte_carlo.as_ref(), "monte_carlo", "Monte Carlo")?;

    let _ = progress_tx.blocking_send(ProgressUpdate {
        phase: "mc".into(),
        detail: Some("Planning mission...".into()),
        fraction: Some(0.0),
    });

    let prepared = super::common::plan_and_prepare(
        def, almanac, cancel, auto_drag, &chief_config, &deputy_config,
    )?;

    if cancel.load(Ordering::Relaxed) {
        return Err(ApiError::Cancelled);
    }

    // Optional covariance propagation
    let covariance_report = if let Some(ref nav) = def.navigation_accuracy {
        let initial_p = ric_accuracy_to_roe_covariance(nav, &prepared.plan.chief_at_arrival)?;
        let report = propagate_mission_covariance(
            &prepared.wp_mission,
            &initial_p,
            nav,
            def.maneuver_uncertainty.as_ref(),
            &prepared.propagator,
            DEFAULT_COVARIANCE_SAMPLES_PER_LEG,
        )?;
        Some(report)
    } else {
        None
    };

    // Run Monte Carlo with control hooks
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
        nominal_mission: &prepared.wp_mission,
        initial_chief: &prepared.perch_chief,
        initial_deputy: &prepared.perch_deputy,
        config: mc_config,
        mission_config: &def.config,
        chief_config: &chief_config,
        deputy_config: &deputy_config,
        propagator: &prepared.propagator,
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

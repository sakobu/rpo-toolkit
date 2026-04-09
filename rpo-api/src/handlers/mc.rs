//! Monte Carlo ensemble handler — blocking, minutes to hours.

use crate::error::ServerError;
use crate::protocol::{ProgressPhase, PROGRESS_COMPLETE, PROGRESS_EXECUTING, PROGRESS_START};
use anise::prelude::Almanac;
use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::types::{MonteCarloConfig, MonteCarloReport};
use rpo_core::mission::types::WaypointMission;
use rpo_core::pipeline::convert::to_propagation_model;
use rpo_core::pipeline::types::PropagatorChoice;
use rpo_core::propagation::covariance::types::MissionCovarianceReport;
use rpo_core::propagation::propagator::{DragConfig, PropagationModel};
use rpo_core::types::spacecraft::SpacecraftConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::monte_carlo::{run_monte_carlo, MonteCarloControl, MonteCarloInput};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use tokio::sync::mpsc;

use super::send_progress;

/// Domain inputs for a Monte Carlo ensemble job.
///
/// Groups the 10 domain parameters needed by [`handle_mc`], keeping the
/// handler signature at 4 arguments (input + almanac + progress + cancel).
pub(crate) struct McJobInput<'a> {
    /// Nominal mission plan (reference for dispersions).
    pub mission: &'a WaypointMission,
    /// Chief ECI state at mission start.
    pub chief: &'a StateVector,
    /// Deputy ECI state at mission start.
    pub deputy: &'a StateVector,
    /// Chief spacecraft physical properties.
    pub chief_config: &'a SpacecraftConfig,
    /// Deputy spacecraft physical properties.
    pub deputy_config: &'a SpacecraftConfig,
    /// Mission targeting configuration (for closed-loop re-targeting).
    pub mission_config: &'a MissionConfig,
    /// Propagator selection (J2 or J2+Drag).
    pub propagator_choice: PropagatorChoice,
    /// Optional drag config override (replaces the config embedded in
    /// `PropagatorChoice::J2Drag` when present, e.g., freshly extracted
    /// via `ExtractDrag`).
    pub drag_config: Option<DragConfig>,
    /// Monte Carlo configuration (samples, dispersions, mode, seed).
    pub mc_config: &'a MonteCarloConfig,
    /// Optional covariance predictions for cross-check.
    pub covariance_report: Option<&'a MissionCovarianceReport>,
}

/// Handle a `RunMc` message on a blocking thread.
///
/// Resolves `PropagatorChoice` → `PropagationModel`, constructs `MonteCarloInput`,
/// and delegates to `rpo_nyx::monte_carlo::run_monte_carlo()`.
///
/// Progress is tracked via `MonteCarloControl::progress` (atomic counter) and
/// cooperative cancellation via `cancel`.
///
/// # Errors
///
/// - [`ServerError::Cancelled`] if the cancel flag is set before the MC run.
/// - [`ServerError::MonteCarlo`] if the ensemble fails (zero samples, all samples
///   failed, nyx bridge error).
pub(crate) fn handle_mc(
    input: &McJobInput<'_>,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<crate::protocol::ProgressUpdate>,
    cancel: &Arc<AtomicBool>,
) -> Result<MonteCarloReport, ServerError> {
    send_progress(progress_tx, ProgressPhase::Mc, "Starting Monte Carlo...", PROGRESS_START);

    if cancel.load(Ordering::Relaxed) {
        return Err(ServerError::Cancelled);
    }

    // Resolve PropagatorChoice → PropagationModel, applying drag override if present.
    let propagator = resolve_propagation_model(input.propagator_choice, input.drag_config);

    // Create progress/cancel control for the MC engine
    let control = MonteCarloControl {
        progress: Arc::new(AtomicU32::new(0)),
        cancel: Arc::clone(cancel),
    };

    let num_samples = input.mc_config.num_samples;
    send_progress(
        progress_tx,
        ProgressPhase::Mc,
        &format!("Running {num_samples} MC samples..."),
        PROGRESS_EXECUTING,
    );

    let mc_input = MonteCarloInput {
        nominal_mission: input.mission,
        initial_chief: input.chief,
        initial_deputy: input.deputy,
        config: input.mc_config,
        mission_config: input.mission_config,
        chief_config: input.chief_config,
        deputy_config: input.deputy_config,
        propagator: &propagator,
        almanac,
        covariance_report: input.covariance_report,
        control: Some(&control),
    };

    let report = run_monte_carlo(&mc_input)?;

    send_progress(progress_tx, ProgressPhase::Mc, "Monte Carlo complete", PROGRESS_COMPLETE);

    Ok(report)
}

/// Resolve `PropagatorChoice` to `PropagationModel`, applying an optional drag override.
///
/// Delegates base conversion to [`rpo_core::pipeline::convert::to_propagation_model`],
/// then replaces the drag config when `drag_override` is `Some` (e.g., freshly
/// extracted via `ExtractDrag`).
fn resolve_propagation_model(
    choice: PropagatorChoice,
    drag_override: Option<DragConfig>,
) -> PropagationModel {
    let base = to_propagation_model(&choice);
    match (base, drag_override) {
        (PropagationModel::J2DragStm { .. }, Some(drag)) => {
            PropagationModel::J2DragStm { drag }
        }
        (model, _) => model,
    }
}

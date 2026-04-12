//! Server-only Monte Carlo input types: control hooks and bundled inputs.

use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32};
use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::MonteCarloConfig;
use rpo_core::mission::types::WaypointMission;
use rpo_core::propagation::covariance::types::MissionCovarianceReport;
use nalgebra::Vector3;
use rpo_core::propagation::propagator::{PropagatedState, PropagationModel};
use rpo_core::types::{SpacecraftConfig, StateVector};

/// Optional progress/cancel hooks for external callers (e.g., API server).
///
/// - `progress`: incremented per completed sample (poll for progress fraction)
/// - `cancel`: set to `true` to request cooperative cancellation
///
/// Not `Serialize`/`Deserialize` — runtime-only coordination.
pub struct MonteCarloControl {
    /// Incremented per completed sample. Poll to compute fraction: `progress.load() / num_samples`.
    pub progress: Arc<AtomicU32>,
    /// Set to `true` to request cancellation. Checked before each sample's nyx propagation.
    pub cancel: Arc<AtomicBool>,
}

/// Bundled inputs for Monte Carlo ensemble analysis.
///
/// Groups all arguments needed by [`crate::monte_carlo::run_monte_carlo`]
/// into a single struct to avoid long parameter lists.
///
/// Not `Serialize`/`Deserialize` because it contains borrows and `Arc`.
/// `Debug` is manually implemented because `Almanac` does not derive `Debug`.
pub struct MonteCarloInput<'a> {
    /// Nominal mission plan (reference Δv and TOFs).
    pub nominal_mission: &'a WaypointMission,
    /// Chief ECI state at mission start.
    pub initial_chief: &'a StateVector,
    /// Deputy ECI state at mission start.
    pub initial_deputy: &'a StateVector,
    /// Monte Carlo configuration (samples, dispersions, mode, seed).
    pub config: &'a MonteCarloConfig,
    /// Mission targeting/TOF/safety configuration (used for closed-loop re-targeting).
    pub mission_config: &'a MissionConfig,
    /// Chief spacecraft physical properties.
    pub chief_config: &'a SpacecraftConfig,
    /// Deputy spacecraft physical properties.
    pub deputy_config: &'a SpacecraftConfig,
    /// Propagation model for closed-loop re-targeting.
    pub propagator: &'a PropagationModel,
    /// Preloaded ANISE almanac for nyx propagation.
    pub almanac: &'a Arc<Almanac>,
    /// Optional covariance predictions for validation comparison.
    pub covariance_report: Option<&'a MissionCovarianceReport>,
    /// Optional progress/cancel hooks (API server use). `None` for CLI/test callers.
    pub control: Option<&'a MonteCarloControl>,
}

/// Lightweight trajectory summary projected from a full MC sample trajectory.
///
/// Replaces `Vec<PropagatedState>` in [`super::execution::SampleOutput`] to
/// reduce per-sample memory from ~19 f64 per state to ~4 f64 per state.
/// Full `PropagatedState` fields (ROE, chief mean elements, velocity) are
/// discarded after projection because downstream consumers only need RIC
/// positions (for dispersion envelope) and the terminal position (for
/// covariance cross-check).
///
/// # Invariants
/// - `positions_ric_km.len() == elapsed_s.len()`: both vectors are produced
///   from the same source trajectory in [`Self::from_trajectory`].
pub(crate) struct SampleTrajectorySummary {
    /// Per-time-step RIC positions (km), one per trajectory sample.
    /// Used by [`super::statistics::compute_dispersion_envelope`] for per-axis
    /// percentile statistics.
    pub(crate) positions_ric_km: Vec<Vector3<f64>>,
    /// Per-time-step elapsed seconds from mission start.
    /// Used by [`super::statistics::compute_dispersion_envelope`] for time
    /// axis labels on envelope entries.
    pub(crate) elapsed_s: Vec<f64>,
}

impl SampleTrajectorySummary {
    /// Project a full trajectory to a lightweight summary.
    ///
    /// Extracts only the RIC position and elapsed time at each step.
    /// Discards ROE, chief mean elements, and velocity.
    #[must_use]
    pub(crate) fn from_trajectory(trajectory: &[PropagatedState]) -> Self {
        let (positions_ric_km, elapsed_s) = trajectory
            .iter()
            .map(|s| (s.ric.position_ric_km, s.elapsed_s))
            .unzip();
        Self {
            positions_ric_km,
            elapsed_s,
        }
    }

    /// Terminal (last) RIC position (km), or `None` if the trajectory is empty.
    ///
    /// Used by [`super::statistics::compute_covariance_cross_check`] for
    /// 3-sigma box containment.
    #[must_use]
    pub(crate) fn terminal_position_ric_km(&self) -> Option<Vector3<f64>> {
        self.positions_ric_km.last().copied()
    }
}

impl fmt::Debug for MonteCarloInput<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MonteCarloInput")
            .field("config", &self.config)
            .field("mode", &self.config.mode)
            .field("num_samples", &self.config.num_samples)
            .field("almanac", &"<Almanac>")
            .finish_non_exhaustive()
    }
}

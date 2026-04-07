//! Server-only Monte Carlo input types: control hooks and bundled inputs.

use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32};
use std::sync::Arc;

use anise::prelude::Almanac;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::MonteCarloConfig;
use rpo_core::mission::types::WaypointMission;
use rpo_core::propagation::covariance::types::MissionCovarianceReport;
use rpo_core::propagation::propagator::PropagationModel;
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

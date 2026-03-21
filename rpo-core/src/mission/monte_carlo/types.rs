//! Monte Carlo domain types: dispersions, configurations, and reports.

use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32};
use std::sync::Arc;

use anise::prelude::Almanac;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::constants::{
    DEFAULT_MANEUVER_MAGNITUDE_SIGMA, DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
    DEFAULT_NAV_POSITION_SIGMA_KM, DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
};

use crate::mission::config::MissionConfig;
use crate::mission::types::{SafetyMetrics, WaypointMission};
use crate::propagation::covariance::types::{ManeuverUncertainty, MissionCovarianceReport};
use crate::propagation::propagator::PropagationModel;
use crate::types::{SpacecraftConfig, StateVector};

// ---------------------------------------------------------------------------
// Monte Carlo types
// ---------------------------------------------------------------------------

/// Distribution model for scalar uncertainty parameters.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum Distribution {
    /// Gaussian (normal) distribution with given 1-sigma.
    Gaussian {
        /// 1-sigma standard deviation (must be non-negative).
        sigma: f64,
    },
    /// Uniform distribution centered on nominal with given half-width.
    Uniform {
        /// Half-width of the uniform range (must be non-negative).
        half_width: f64,
    },
}

/// Initial state uncertainty model (RIC frame, applied to deputy).
///
/// Uses descriptive RIC axis names to avoid ambiguity.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct StateDispersion {
    /// 1-sigma radial position dispersion (km).
    pub position_radial_km: Distribution,
    /// 1-sigma in-track position dispersion (km).
    pub position_intrack_km: Distribution,
    /// 1-sigma cross-track position dispersion (km).
    pub position_crosstrack_km: Distribution,
    /// 1-sigma radial velocity dispersion (km/s).
    pub velocity_radial_km_s: Distribution,
    /// 1-sigma in-track velocity dispersion (km/s).
    pub velocity_intrack_km_s: Distribution,
    /// 1-sigma cross-track velocity dispersion (km/s).
    pub velocity_crosstrack_km_s: Distribution,
}

impl Default for StateDispersion {
    fn default() -> Self {
        Self {
            position_radial_km: Distribution::Gaussian {
                sigma: DEFAULT_NAV_POSITION_SIGMA_KM,
            },
            position_intrack_km: Distribution::Gaussian {
                sigma: DEFAULT_NAV_POSITION_SIGMA_KM,
            },
            position_crosstrack_km: Distribution::Gaussian {
                sigma: DEFAULT_NAV_POSITION_SIGMA_KM,
            },
            velocity_radial_km_s: Distribution::Gaussian {
                sigma: DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
            },
            velocity_intrack_km_s: Distribution::Gaussian {
                sigma: DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
            },
            velocity_crosstrack_km_s: Distribution::Gaussian {
                sigma: DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
            },
        }
    }
}

/// Maneuver execution error model (for MC sampling).
///
/// Shares the same (`magnitude_sigma`, `pointing_sigma_rad`) representation as
/// `ManeuverUncertainty` (covariance layer). A `From<ManeuverUncertainty>` impl
/// is provided for convenience.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ManeuverDispersion {
    /// Proportional 1-sigma magnitude error (dimensionless, e.g. 0.01 = 1%).
    pub magnitude_sigma: f64,
    /// 1-sigma pointing error (rad), applied as isotropic rotation about Δv axis.
    pub pointing_sigma_rad: f64,
}

impl Default for ManeuverDispersion {
    fn default() -> Self {
        Self {
            magnitude_sigma: DEFAULT_MANEUVER_MAGNITUDE_SIGMA,
            pointing_sigma_rad: DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
        }
    }
}

impl From<ManeuverUncertainty> for ManeuverDispersion {
    fn from(u: ManeuverUncertainty) -> Self {
        Self {
            magnitude_sigma: u.magnitude_sigma,
            pointing_sigma_rad: u.pointing_sigma_rad,
        }
    }
}

/// Spacecraft property uncertainty (deputy only, full-physics MC).
///
/// No `Default` impl — spacecraft dispersions are always explicitly specified
/// because there are no universal defaults for Cd/area/mass uncertainty.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SpacecraftDispersion {
    /// Drag coefficient dispersion.
    pub coeff_drag: Distribution,
    /// Drag cross-sectional area dispersion (m²).
    pub drag_area_m2: Distribution,
    /// Dry mass dispersion (kg).
    pub dry_mass_kg: Distribution,
}

/// Composite dispersion configuration.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct DispersionConfig {
    /// Initial state uncertainty (RIC frame).
    pub state: Option<StateDispersion>,
    /// Maneuver execution uncertainty.
    pub maneuver: Option<ManeuverDispersion>,
    /// Spacecraft property uncertainty (deputy only).
    pub spacecraft: Option<SpacecraftDispersion>,
}

/// Monte Carlo execution mode.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum MonteCarloMode {
    /// Apply nominal Δv plan with execution errors (no re-targeting).
    #[default]
    OpenLoop,
    /// Re-run targeting from each dispersed initial state.
    ClosedLoop,
}

impl std::fmt::Display for MonteCarloMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OpenLoop => write!(f, "Open Loop"),
            Self::ClosedLoop => write!(f, "Closed Loop"),
        }
    }
}

/// Monte Carlo configuration.
///
/// All MC runs use nyx full-physics propagation.
/// Must be explicitly constructed — no `Default` impl because `num_samples`
/// and `dispersions` have no meaningful defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MonteCarloConfig {
    /// Number of Monte Carlo samples to run.
    pub num_samples: u32,
    /// Dispersion models for state, maneuver, and spacecraft properties.
    pub dispersions: DispersionConfig,
    /// Open-loop (apply nominal plan) or closed-loop (re-target per sample).
    pub mode: MonteCarloMode,
    /// Master RNG seed for deterministic reproducibility (defaults to 42 if None).
    pub seed: Option<u64>,
    /// Number of trajectory sample points per leg for dispersion envelope.
    pub trajectory_steps: u32,
}

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
/// Groups all arguments needed by [`crate::mission::monte_carlo::run_monte_carlo`]
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

/// Per-sample result (no full trajectory stored).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SampleResult {
    /// Zero-based sample index.
    pub index: u32,
    /// Total Δv for this sample (km/s).
    pub total_dv_km_s: f64,
    /// Safety metrics for this sample (None if safety analysis not configured).
    pub safety: Option<SafetyMetrics>,
    /// Per-waypoint miss distance (km), one entry per waypoint.
    pub waypoint_miss_km: Vec<f64>,
    /// Whether the targeting solver converged for this sample.
    pub converged: bool,
}

/// Percentile statistics for a scalar quantity.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct PercentileStats {
    /// Minimum value.
    pub min: f64,
    /// 1st percentile.
    pub p01: f64,
    /// 5th percentile.
    pub p05: f64,
    /// 25th percentile (first quartile).
    pub p25: f64,
    /// 50th percentile (median).
    pub p50: f64,
    /// 75th percentile (third quartile).
    pub p75: f64,
    /// 95th percentile.
    pub p95: f64,
    /// 99th percentile.
    pub p99: f64,
    /// Maximum value.
    pub max: f64,
    /// Arithmetic mean.
    pub mean: f64,
    /// Sample standard deviation.
    pub std_dev: f64,
}

/// Trajectory dispersion envelope at a single time point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DispersionEnvelope {
    /// Elapsed time since mission start (seconds).
    pub elapsed_s: f64,
    /// Radial position dispersion statistics (km).
    pub radial_km: PercentileStats,
    /// In-track position dispersion statistics (km).
    pub in_track_km: PercentileStats,
    /// Cross-track position dispersion statistics (km).
    pub cross_track_km: PercentileStats,
}

/// Aggregate ensemble statistics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnsembleStatistics {
    /// Total Δv distribution across samples (km/s).
    pub total_dv_km_s: PercentileStats,
    /// Minimum radial/cross-track separation distribution (km).
    /// `None` when no samples produced safety data.
    pub min_rc_distance_km: Option<PercentileStats>,
    /// Minimum 3D distance distribution (km).
    /// `None` when no samples produced safety data.
    pub min_3d_distance_km: Option<PercentileStats>,
    /// Minimum e/i vector separation distribution (km).
    /// `None` when no samples produced safety data.
    pub min_ei_separation_km: Option<PercentileStats>,
    /// Per-waypoint miss distance distributions (km), one entry per waypoint.
    /// Individual entries are `None` when no finite miss data was available.
    pub waypoint_miss_km: Vec<Option<PercentileStats>>,
    /// Empirical collision probability (fraction of samples violating keep-out).
    pub collision_probability: f64,
    /// Fraction of samples where targeting converged.
    pub convergence_rate: f64,
    /// Fraction of samples where min e/i separation violated the configured threshold.
    /// 0.0 if no `SafetyConfig` was provided.
    pub ei_violation_rate: f64,
    /// Fraction of samples where min 3D distance violated the configured keep-out threshold.
    /// 0.0 if no `SafetyConfig` was provided.
    pub keepout_violation_rate: f64,
    /// Trajectory dispersion envelope at sampled time points.
    pub dispersion_envelope: Vec<DispersionEnvelope>,
}

/// Comparison between linear covariance prediction and MC ensemble.
///
/// The covariance model propagates uncertainty along the **nominal mission plan**
/// (open-loop: P = Phi P0 Phi^T). It does not model closed-loop re-targeting feedback.
///
/// - **`OpenLoop`**: Sigma ratios near 1.0 indicate a well-calibrated covariance.
///   Terminal containment should be near 99.7% for Gaussian-distributed errors.
/// - **`ClosedLoop`**: Sigma ratios << 1.0 because re-targeting from dispersed
///   states concentrates samples far within the open-loop uncertainty bounds.
///   Terminal containment often near 100%. These are expected, not a calibration failure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CovarianceCrossCheck {
    /// Fraction of MC samples whose terminal RIC position falls within
    /// the covariance-predicted axis-aligned 3-sigma box at mission end.
    ///
    /// For a well-calibrated Gaussian covariance in `OpenLoop` mode,
    /// expect approximately 0.997 (99.7%). In `ClosedLoop` mode, often near 1.0
    /// since retargeting tends to concentrate samples within the bounds.
    pub terminal_3sigma_containment: f64,
    /// Minimum Mahalanobis distance between nominal deputy and chief across
    /// the covariance-predicted (open-loop) trajectory. Measures closest
    /// approach in sigma-space; values < 1 indicate the deputy is within
    /// the 1-sigma covariance envelope of the chief.
    pub min_mahalanobis_distance: f64,
    /// Ratio of MC sigma to covariance sigma per RIC axis (expect ~1.0).
    pub sigma_ratio_ric: Vector3<f64>,
}

/// Complete Monte Carlo analysis report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MonteCarloReport {
    /// Configuration used for this MC run.
    pub config: MonteCarloConfig,
    /// Nominal (undispersed) total Δv (km/s).
    pub nominal_dv_km_s: f64,
    /// Nominal safety metrics (None if safety not configured).
    pub nominal_safety: Option<SafetyMetrics>,
    /// Aggregate ensemble statistics.
    pub statistics: EnsembleStatistics,
    /// Per-sample results (lightweight, no full trajectories).
    pub samples: Vec<SampleResult>,
    /// Number of samples that failed (propagation or convergence).
    pub num_failures: u32,
    /// Wall-clock time for the MC run (seconds).
    pub elapsed_wall_s: f64,
    /// Covariance cross-check against MC ensemble (if covariance report provided).
    pub covariance_cross_check: Option<CovarianceCrossCheck>,
}

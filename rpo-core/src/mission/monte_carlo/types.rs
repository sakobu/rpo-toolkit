//! Monte Carlo domain types: dispersions, configurations, and reports.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::constants::{
    DEFAULT_MANEUVER_MAGNITUDE_SIGMA, DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
    DEFAULT_NAV_POSITION_SIGMA_KM, DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
};

use crate::mission::types::SafetyMetrics;
use crate::propagation::covariance::types::{ManeuverUncertainty, NavigationAccuracy};

// ---------------------------------------------------------------------------
// Monte Carlo types
// ---------------------------------------------------------------------------

/// Distribution model for scalar uncertainty parameters.
///
/// # Invariants
///
/// - `Gaussian::sigma >= 0.0`
/// - `Uniform::half_width >= 0.0`
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
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

/// Initial state uncertainty model (RIC frame, applied to deputy relative to chief).
///
/// Dispersions are added to the deputy's initial RIC state offset.
/// Each axis is sampled independently (uncorrelated).
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

impl From<NavigationAccuracy> for StateDispersion {
    fn from(nav: NavigationAccuracy) -> Self {
        Self {
            position_radial_km: Distribution::Gaussian {
                sigma: nav.position_sigma_ric_km[0],
            },
            position_intrack_km: Distribution::Gaussian {
                sigma: nav.position_sigma_ric_km[1],
            },
            position_crosstrack_km: Distribution::Gaussian {
                sigma: nav.position_sigma_ric_km[2],
            },
            velocity_radial_km_s: Distribution::Gaussian {
                sigma: nav.velocity_sigma_ric_km_s[0],
            },
            velocity_intrack_km_s: Distribution::Gaussian {
                sigma: nav.velocity_sigma_ric_km_s[1],
            },
            velocity_crosstrack_km_s: Distribution::Gaussian {
                sigma: nav.velocity_sigma_ric_km_s[2],
            },
        }
    }
}

/// Spacecraft property uncertainty (deputy only, full-physics MC).
///
/// No `Default` impl — spacecraft dispersions are always explicitly specified
/// because there are no universal defaults for Cd/area/mass uncertainty.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SpacecraftDispersion {
    /// Drag coefficient (Cd) dispersion (dimensionless).
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

impl DispersionConfig {
    /// Fill `None` fields from top-level covariance types.
    ///
    /// When MC dispersion fields are not explicitly set, derive them from
    /// the top-level `NavigationAccuracy` and `ManeuverUncertainty` to avoid
    /// duplicating identical sigma values in the input JSON.
    /// Explicit dispersions are never overwritten.
    #[must_use]
    pub fn resolved(
        &self,
        nav: Option<&NavigationAccuracy>,
        maneuver_unc: Option<&ManeuverUncertainty>,
    ) -> Self {
        Self {
            state: self
                .state
                .or_else(|| nav.map(|n| StateDispersion::from(*n))),
            maneuver: self
                .maneuver
                .or_else(|| maneuver_unc.map(|m| ManeuverDispersion::from(*m))),
            spacecraft: self.spacecraft,
        }
    }
}

/// Monte Carlo execution mode.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
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
///
/// # Invariants
///
/// - `num_samples > 0`
/// - `trajectory_steps > 0`
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
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

impl MonteCarloConfig {
    /// Return a copy with dispersions resolved from top-level covariance types.
    ///
    /// When MC-specific dispersion fields are `None`, derives them from the
    /// top-level [`NavigationAccuracy`] and [`ManeuverUncertainty`] to avoid
    /// duplicating identical sigma values in input JSON. Explicit dispersions
    /// are never overwritten.
    #[must_use]
    pub fn with_resolved_dispersions(
        self,
        nav: Option<&NavigationAccuracy>,
        maneuver_unc: Option<&ManeuverUncertainty>,
    ) -> Self {
        Self {
            dispersions: self.dispersions.resolved(nav, maneuver_unc),
            ..self
        }
    }
}

/// Per-sample result (no full trajectory stored).
///
/// Produced by `rpo_nyx::monte_carlo::run_monte_carlo` per-sample propagation.
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
///
/// Empirical (non-parametric) summary computed from MC ensemble samples.
/// Percentiles use nearest-rank method; NaN/Inf values are filtered before
/// computation.
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
///
/// Percentile statistics of RIC position across the MC ensemble,
/// sampled at the given mission elapsed time.
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
///
/// Produced by `rpo_nyx::monte_carlo::run_monte_carlo` ensemble analysis.
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
///
/// Produced by `rpo_nyx::monte_carlo::run_monte_carlo`.
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::covariance::types::{ManeuverUncertainty, NavigationAccuracy};
    use nalgebra::Vector3;

    #[test]
    fn test_from_nav_accuracy_for_state_dispersion() {
        let nav = NavigationAccuracy {
            position_sigma_ric_km: Vector3::new(0.1, 0.2, 0.3),
            velocity_sigma_ric_km_s: Vector3::new(0.001, 0.002, 0.003),
        };
        let sd = StateDispersion::from(nav);
        assert_eq!(sd.position_radial_km, Distribution::Gaussian { sigma: 0.1 });
        assert_eq!(
            sd.position_intrack_km,
            Distribution::Gaussian { sigma: 0.2 }
        );
        assert_eq!(
            sd.position_crosstrack_km,
            Distribution::Gaussian { sigma: 0.3 }
        );
        assert_eq!(
            sd.velocity_radial_km_s,
            Distribution::Gaussian { sigma: 0.001 }
        );
        assert_eq!(
            sd.velocity_intrack_km_s,
            Distribution::Gaussian { sigma: 0.002 }
        );
        assert_eq!(
            sd.velocity_crosstrack_km_s,
            Distribution::Gaussian { sigma: 0.003 }
        );
    }

    #[test]
    fn test_dispersion_config_resolved_derives_from_nav() {
        let nav = NavigationAccuracy {
            position_sigma_ric_km: Vector3::new(0.1, 0.1, 0.1),
            velocity_sigma_ric_km_s: Vector3::new(0.0001, 0.0001, 0.0001),
        };
        let mu = ManeuverUncertainty {
            magnitude_sigma: 0.01,
            pointing_sigma_rad: 0.01745,
        };
        let config = DispersionConfig::default(); // all None
        let resolved = config.resolved(Some(&nav), Some(&mu));
        assert!(resolved.state.is_some(), "state should be derived from nav");
        assert!(
            resolved.maneuver.is_some(),
            "maneuver should be derived from uncertainty"
        );
    }

    #[test]
    fn test_dispersion_config_resolved_explicit_overrides() {
        let nav = NavigationAccuracy {
            position_sigma_ric_km: Vector3::new(0.1, 0.1, 0.1),
            velocity_sigma_ric_km_s: Vector3::new(0.0001, 0.0001, 0.0001),
        };
        let explicit_state = StateDispersion {
            position_radial_km: Distribution::Uniform { half_width: 0.5 },
            position_intrack_km: Distribution::Uniform { half_width: 0.5 },
            position_crosstrack_km: Distribution::Uniform { half_width: 0.5 },
            velocity_radial_km_s: Distribution::Uniform {
                half_width: 0.005,
            },
            velocity_intrack_km_s: Distribution::Uniform {
                half_width: 0.005,
            },
            velocity_crosstrack_km_s: Distribution::Uniform {
                half_width: 0.005,
            },
        };
        let config = DispersionConfig {
            state: Some(explicit_state),
            maneuver: None,
            spacecraft: None,
        };
        let resolved = config.resolved(Some(&nav), None);
        // Explicit state should NOT be overwritten
        assert_eq!(
            resolved.state.unwrap().position_radial_km,
            Distribution::Uniform { half_width: 0.5 }
        );
    }

    #[test]
    fn test_dispersion_config_resolved_none_stays_none() {
        let config = DispersionConfig::default();
        let resolved = config.resolved(None, None);
        assert!(resolved.state.is_none());
        assert!(resolved.maneuver.is_none());
    }
}

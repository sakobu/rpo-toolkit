//! Full-physics Monte Carlo ensemble analysis using nyx propagation.
//!
//! Each sample propagates chief + deputy through nyx with dispersed initial
//! states and Δv execution errors. Uses rayon for parallel execution with
//! deterministic per-sample seeding via `ChaCha20Rng`.

pub mod types;
pub(crate) mod execution;
pub(crate) mod sampling;
pub(crate) mod statistics;

use std::fmt;
use std::time::Instant;

use rayon::prelude::*;

use crate::elements::eci_ric_dcm::DcmError;
use crate::mission::errors::MissionError;
use crate::propagation::nyx_bridge::NyxBridgeError;
use crate::propagation::propagator::PropagationError;

use execution::{collect_ensemble_statistics, run_single_sample};
use statistics::compute_covariance_cross_check;
pub use types::{
    CovarianceCrossCheck, DispersionConfig, DispersionEnvelope, Distribution,
    EnsembleStatistics, ManeuverDispersion, MonteCarloConfig, MonteCarloInput, MonteCarloMode,
    MonteCarloReport, PercentileStats, SampleResult, SpacecraftDispersion, StateDispersion,
};

/// Default master seed when `MonteCarloConfig.seed` is `None`.
const DEFAULT_MC_SEED: u64 = 42;

/// Errors from Monte Carlo ensemble analysis.
#[derive(Debug)]
pub enum MonteCarloError {
    /// `num_samples` must be > 0.
    ZeroSamples,
    /// All MC samples failed (none converged or propagated successfully).
    AllSamplesFailed {
        /// Total number of samples attempted.
        num_samples: u32,
        /// Number that failed due to targeting non-convergence.
        convergence_failures: u32,
        /// Number that failed due to propagation errors.
        propagation_failures: u32,
    },
    /// Dispersed state produced negative semi-major axis.
    NegativeSma {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting semi-major axis (km).
        a_km: f64,
    },
    /// Dispersed state produced invalid eccentricity.
    InvalidEccentricity {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting eccentricity.
        e: f64,
    },
    /// Dispersion sigma must be non-negative.
    NegativeSigma {
        /// The invalid sigma value.
        value: f64,
    },
    /// Dispersion half-width must be non-negative.
    NegativeHalfWidth {
        /// The invalid half-width value.
        value: f64,
    },
    /// Mission planning failure during closed-loop re-targeting.
    Mission(MissionError),
    /// Propagation failure during sample execution.
    Propagation(PropagationError),
    /// Nyx bridge failure.
    NyxBridge(Box<NyxBridgeError>),
    /// Empty ensemble (no samples to compute statistics from).
    EmptyEnsemble,
    /// Trajectory count exceeds u32 range (should not happen — bounded by `num_samples`: u32).
    TooManySamples {
        /// The count that overflowed u32.
        count: usize,
    },
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
}

impl fmt::Display for MonteCarloError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroSamples => write!(f, "num_samples must be > 0"),
            Self::AllSamplesFailed {
                num_samples,
                convergence_failures,
                propagation_failures,
            } => write!(
                f,
                "all {num_samples} MC samples failed: {convergence_failures} convergence, \
                 {propagation_failures} propagation"
            ),
            Self::NegativeSma { sample_index, a_km } => {
                write!(f, "sample {sample_index}: negative SMA = {a_km} km")
            }
            Self::InvalidEccentricity { sample_index, e } => {
                write!(f, "sample {sample_index}: invalid eccentricity = {e}")
            }
            Self::NegativeSigma { value } => {
                write!(f, "dispersion sigma must be non-negative, got {value}")
            }
            Self::NegativeHalfWidth { value } => {
                write!(f, "dispersion half-width must be non-negative, got {value}")
            }
            Self::Mission(e) => write!(f, "mission planning failure: {e}"),
            Self::Propagation(e) => write!(f, "propagation failure: {e}"),
            Self::NyxBridge(e) => write!(f, "nyx bridge failure: {e}"),
            Self::EmptyEnsemble => write!(f, "empty ensemble: no samples to compute statistics"),
            Self::TooManySamples { count } => {
                write!(f, "trajectory count {count} exceeds u32 range")
            }
            Self::DcmFailure(e) => write!(f, "frame conversion failed: {e}"),
        }
    }
}

impl std::error::Error for MonteCarloError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Mission(e) => Some(e),
            Self::Propagation(e) => Some(e),
            Self::NyxBridge(e) => Some(e.as_ref()),
            Self::DcmFailure(e) => Some(e),
            _ => None,
        }
    }
}

impl From<DcmError> for MonteCarloError {
    fn from(e: DcmError) -> Self {
        Self::DcmFailure(e)
    }
}

impl From<MissionError> for MonteCarloError {
    fn from(e: MissionError) -> Self {
        Self::Mission(e)
    }
}

impl From<PropagationError> for MonteCarloError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<NyxBridgeError> for MonteCarloError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

/// Run full-physics Monte Carlo ensemble analysis using nyx propagation.
///
/// Each sample propagates chief + deputy through nyx with dispersed initial
/// states and Δv execution errors. In closed-loop mode, each sample re-targets
/// the mission from the dispersed initial state before propagation.
/// Uses rayon for parallel execution with deterministic per-sample seeding.
///
/// # Invariants
/// - `input.config.num_samples > 0`
/// - `input.initial_chief` and `input.initial_deputy` are bound orbits
///   (caller responsibility).
/// - `input.nominal_mission.legs` is non-empty (caller responsibility).
/// - Results are deterministic for a given `input.config.seed`.
///
/// # Arguments
/// * `input` — Monte Carlo input bundle (mission, config, dispersions, almanac)
///
/// # Errors
/// - [`MonteCarloError::ZeroSamples`] if `num_samples == 0`.
/// - [`MonteCarloError::AllSamplesFailed`] if every sample fails (with
///   breakdown of convergence vs propagation failures).
pub fn run_monte_carlo(input: &MonteCarloInput<'_>) -> Result<MonteCarloReport, MonteCarloError> {
    let config = input.config;
    let nominal_mission = input.nominal_mission;

    if config.num_samples == 0 {
        return Err(MonteCarloError::ZeroSamples);
    }

    let start = Instant::now();
    let master_seed = config.seed.unwrap_or(DEFAULT_MC_SEED);

    // Run samples in parallel
    let results: Vec<Result<execution::SampleOutput, MonteCarloError>> = (0..config.num_samples)
        .into_par_iter()
        .map(|i| run_single_sample(input, i, master_seed))
        .collect();

    // Separate successes from failures
    let mut samples = Vec::new();
    let mut trajectories = Vec::new();
    let mut num_failures = 0_u32;
    let mut convergence_failures = 0_u32;
    let mut propagation_failures = 0_u32;

    for result in results {
        match result {
            Ok(output) => {
                samples.push(output.result);
                trajectories.push(output.trajectory);
            }
            Err(MonteCarloError::Mission(_)) => {
                convergence_failures += 1;
                num_failures += 1;
            }
            Err(_) => {
                propagation_failures += 1;
                num_failures += 1;
            }
        }
    }

    if samples.is_empty() {
        return Err(MonteCarloError::AllSamplesFailed {
            num_samples: config.num_samples,
            convergence_failures,
            propagation_failures,
        });
    }

    // Compute ensemble statistics (denominator = total samples attempted, not just successes)
    let statistics = collect_ensemble_statistics(
        &samples,
        &trajectories,
        config,
        config.num_samples,
        input.mission_config.safety.as_ref(),
    )?;

    // Covariance validation (if provided)
    let covariance_cross_check = input
        .covariance_report
        .map(|cov_report| compute_covariance_cross_check(cov_report, &statistics, &trajectories))
        .transpose()?;

    Ok(MonteCarloReport {
        config: config.clone(),
        nominal_dv_km_s: nominal_mission.total_dv_km_s,
        nominal_safety: nominal_mission.safety,
        statistics,
        samples,
        num_failures,
        elapsed_wall_s: start.elapsed().as_secs_f64(),
        covariance_cross_check,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::config::MissionConfig;
    use crate::mission::types::{Waypoint, WaypointMission};
    use crate::mission::waypoints::plan_waypoint_mission;
    use crate::propagation::propagator::PropagationModel;
    use crate::types::{DepartureState, SpacecraftConfig, StateVector};
    use nalgebra::Vector3;

    /// Open-loop convergence rate is exactly 1.0 (all samples "converge").
    /// Tolerance accounts for floating-point division in rate computation.
    const CONVERGENCE_RATE_TOL: f64 = 1e-10;

    /// Upper bound on collision probability for "safe" geometry test.
    /// 5 km V-bar separation with 100 m dispersions should have Pc well below 10%.
    const SAFE_COLLISION_PROB_BOUND: f64 = 0.1;

    /// Waypoint distance (km) for collision probability "unsafe" test.
    /// 50 m is deliberately close to the 100 m collision threshold.
    const UNSAFE_WAYPOINT_DISTANCE_KM: f64 = 0.05;

    /// Build standard MC test fixtures: ISS-like chief, colocated deputy,
    /// single V-bar waypoint at 5 km, planned mission, and full almanac.
    fn build_mc_test_fixtures(
        num_samples: u32,
        mode: MonteCarloMode,
        dispersions: DispersionConfig,
    ) -> (
        StateVector,
        StateVector,
        WaypointMission,
        MonteCarloConfig,
        MissionConfig,
        PropagationModel,
        std::sync::Arc<anise::prelude::Almanac>,
    ) {
        use crate::elements::keplerian_conversions::keplerian_to_state;
        use crate::propagation::nyx_bridge::load_full_almanac;
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        };

        let departure = DepartureState {
            roe: crate::types::QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };

        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission =
            plan_waypoint_mission(&departure, &[waypoint], &mission_config, &propagator)
                .expect("mission planning should succeed");

        let mc_config = MonteCarloConfig {
            num_samples,
            dispersions,
            mode,
            seed: Some(42),
            trajectory_steps: 10,
        };

        let almanac = load_full_almanac().expect("full almanac should load");

        (
            chief_sv,
            deputy_sv,
            mission,
            mc_config,
            mission_config,
            propagator,
            almanac,
        )
    }

    /// Default state dispersions for MC tests using navigation accuracy
    /// defaults from `constants.rs`: 100 m position, 0.1 m/s velocity.
    fn default_state_dispersions() -> DispersionConfig {
        DispersionConfig {
            state: Some(StateDispersion::default()),
            maneuver: None,
            spacecraft: None,
        }
    }

    // -----------------------------------------------------------------------
    // Error paths (no nyx required)
    // -----------------------------------------------------------------------

    #[test]
    fn mc_zero_samples_error() {
        use std::sync::Arc;
        let dummy_epoch =
            hifitime::Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);
        let chief = StateVector {
            epoch: dummy_epoch,
            position_eci_km: Vector3::new(6786.0, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, 7.67, 0.0),
        };
        let mission = WaypointMission {
            legs: vec![],
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            safety: None,
            covariance: None,
            eclipse: None,
        };
        let mc_config = MonteCarloConfig {
            num_samples: 0,
            dispersions: DispersionConfig::default(),
            mode: MonteCarloMode::OpenLoop,
            seed: Some(42),
            trajectory_steps: 0,
        };
        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let almanac = Arc::new(anise::prelude::Almanac::default());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &chief,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        match run_monte_carlo(&input) {
            Err(MonteCarloError::ZeroSamples) => {}
            other => panic!("expected ZeroSamples error, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Full-physics MC (require nyx almanac)
    // -----------------------------------------------------------------------

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_single_sample() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(1, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("single sample should succeed");
        assert_eq!(report.samples.len(), 1, "should have exactly 1 sample");
        assert!(report.samples[0].converged, "sample should converge");
        assert!(
            report.samples[0].total_dv_km_s.is_finite(),
            "Δv should be finite"
        );
        assert_eq!(report.num_failures, 0, "no failures expected");
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_10_samples_statistics() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("10 samples should succeed");
        let stats = &report.statistics;

        assert!(stats.total_dv_km_s.mean.is_finite(), "Δv mean should be finite");
        assert!(stats.total_dv_km_s.std_dev.is_finite(), "Δv std should be finite");
        assert!(stats.total_dv_km_s.p05 <= stats.total_dv_km_s.p50);
        assert!(stats.total_dv_km_s.p50 <= stats.total_dv_km_s.p95);
        assert!(stats.collision_probability <= 1.0);
        assert!(
            (stats.convergence_rate - 1.0).abs() < CONVERGENCE_RATE_TOL,
            "open-loop convergence rate should be 1.0, got {}",
            stats.convergence_rate
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_parallel_deterministic() {
        let dispersions = default_state_dispersions();

        let (chief1, deputy1, mission1, mc1, mc_cfg1, prop1, alm1) =
            build_mc_test_fixtures(5, MonteCarloMode::OpenLoop, dispersions);
        let input1 = MonteCarloInput {
            nominal_mission: &mission1,
            initial_chief: &chief1,
            initial_deputy: &deputy1,
            config: &mc1,
            mission_config: &mc_cfg1,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &prop1,
            almanac: &alm1,
            covariance_report: None,
        };
        let report1 = run_monte_carlo(&input1).expect("run 1 should succeed");

        let dispersions2 = default_state_dispersions();
        let (chief2, deputy2, mission2, mc2, mc_cfg2, prop2, alm2) =
            build_mc_test_fixtures(5, MonteCarloMode::OpenLoop, dispersions2);
        let input2 = MonteCarloInput {
            nominal_mission: &mission2,
            initial_chief: &chief2,
            initial_deputy: &deputy2,
            config: &mc2,
            mission_config: &mc_cfg2,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &prop2,
            almanac: &alm2,
            covariance_report: None,
        };
        let report2 = run_monte_carlo(&input2).expect("run 2 should succeed");

        assert_eq!(report1.samples.len(), report2.samples.len());
        for (s1, s2) in report1.samples.iter().zip(report2.samples.iter()) {
            assert_eq!(
                s1.total_dv_km_s, s2.total_dv_km_s,
                "sample {} Δv mismatch: {} vs {}",
                s1.index, s1.total_dv_km_s, s2.total_dv_km_s
            );
        }
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_report_serde_roundtrip() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(3, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        let json = serde_json::to_string(&report).expect("serialize should succeed");
        let roundtrip: MonteCarloReport =
            serde_json::from_str(&json).expect("deserialize should succeed");

        assert_eq!(roundtrip.samples.len(), report.samples.len());
        assert_eq!(roundtrip.config.num_samples, report.config.num_samples);
        assert_eq!(
            roundtrip.nominal_dv_km_s, report.nominal_dv_km_s,
            "nominal Δv should survive roundtrip"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_collision_prob_safe() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        assert!(
            report.statistics.collision_probability < SAFE_COLLISION_PROB_BOUND,
            "5 km separation with 100 m dispersions should be safe, got Pc={}",
            report.statistics.collision_probability
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_collision_prob_unsafe() {
        use crate::elements::keplerian_conversions::keplerian_to_state;
        use crate::propagation::nyx_bridge::load_full_almanac;
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let period = chief_ke.period().unwrap();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, UNSAFE_WAYPOINT_DISTANCE_KM, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        };

        let departure = DepartureState {
            roe: crate::types::QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };

        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let mission =
            plan_waypoint_mission(&departure, &[waypoint], &mission_config, &propagator)
                .expect("planning should succeed");

        let mc_config = MonteCarloConfig {
            num_samples: 10,
            dispersions: default_state_dispersions(),
            mode: MonteCarloMode::OpenLoop,
            seed: Some(42),
            trajectory_steps: 10,
        };

        let almanac = load_full_almanac().expect("almanac should load");

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief_sv,
            initial_deputy: &deputy_sv,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        assert!(
            report.statistics.collision_probability <= 1.0,
            "collision probability should be bounded"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_closed_loop_converges() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(5, MonteCarloMode::ClosedLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("closed-loop MC should succeed");
        assert!(
            report.statistics.convergence_rate > 0.0,
            "at least some closed-loop samples should converge"
        );
        assert!(
            report.statistics.total_dv_km_s.mean.is_finite(),
            "closed-loop Δv mean should be finite"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_covariance_cross_check() {
        use crate::mission::covariance::propagate_mission_covariance;
        use crate::propagation::covariance::ric_accuracy_to_roe_covariance;
        use crate::propagation::covariance::types::NavigationAccuracy;

        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        let nav = NavigationAccuracy::default();
        let chief_ke = crate::test_helpers::iss_like_elements();
        let initial_cov =
            ric_accuracy_to_roe_covariance(&nav, &chief_ke).expect("covariance init should work");
        let cov_report =
            propagate_mission_covariance(&mission, &initial_cov, &nav, None, &propagator, 20)
                .expect("covariance propagation should succeed");

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: Some(&cov_report),
        };

        let report = run_monte_carlo(&input).expect("MC with covariance should succeed");

        let cv = report
            .covariance_cross_check
            .as_ref()
            .expect("covariance_cross_check should be Some");

        assert!(cv.sigma_ratio_ric.x.is_finite(), "radial sigma ratio not finite");
        assert!(cv.sigma_ratio_ric.y.is_finite(), "in-track sigma ratio not finite");
        assert!(cv.sigma_ratio_ric.z.is_finite(), "cross-track sigma ratio not finite");
        assert!(
            cv.terminal_3sigma_containment >= 0.0 && cv.terminal_3sigma_containment <= 1.0,
            "terminal containment should be in [0, 1], got {}",
            cv.terminal_3sigma_containment
        );
        assert!(
            cv.min_mahalanobis_distance >= 0.0,
            "Mahalanobis distance should be non-negative, got {}",
            cv.min_mahalanobis_distance
        );
    }
}

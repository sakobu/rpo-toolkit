//! Monte Carlo domain types and error definitions.
//!
//! The ensemble execution functions (`run_monte_carlo`, `execution`,
//! `sampling`, `statistics`) that depend on nyx-space and rayon live
//! in `rpo-nyx`. This module retains the domain types and error enum
//! used by both the analytical engine and the nyx-backed runner.

pub mod types;

use crate::elements::eci_ric_dcm::DcmError;
use crate::mission::errors::MissionError;
use crate::propagation::propagator::PropagationError;

pub use types::{
    CovarianceCrossCheck, DispersionConfig, DispersionEnvelope, Distribution,
    EnsembleStatistics, ManeuverDispersion, MonteCarloConfig,
    MonteCarloMode, MonteCarloReport, PercentileStats, SampleResult, SpacecraftDispersion,
    StateDispersion,
};

/// Errors from Monte Carlo ensemble analysis.
///
/// Defined in `rpo-core` so both the analytical engine and consumers can
/// inspect error variants in error chains. `MonteCarloError` values are
/// produced by the nyx-backed runner in `rpo-nyx::monte_carlo`.
#[derive(Debug, thiserror::Error)]
pub enum MonteCarloError {
    /// `num_samples` must be > 0.
    #[error("num_samples must be > 0")]
    ZeroSamples,
    /// `trajectory_steps` must be > 0.
    #[error("trajectory_steps must be > 0")]
    ZeroTrajectorySteps,
    /// All MC samples failed (none converged or propagated successfully).
    #[error(
        "all {num_samples} MC samples failed: {convergence_failures} convergence, \
         {propagation_failures} propagation"
    )]
    AllSamplesFailed {
        /// Total number of samples attempted.
        num_samples: u32,
        /// Number that failed due to targeting non-convergence.
        convergence_failures: u32,
        /// Number that failed due to propagation errors.
        propagation_failures: u32,
    },
    /// Dispersed state produced negative semi-major axis.
    #[error("sample {sample_index}: negative SMA = {a_km} km")]
    NegativeSma {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting semi-major axis (km).
        a_km: f64,
    },
    /// Dispersed state produced invalid eccentricity.
    #[error("sample {sample_index}: invalid eccentricity = {e}")]
    InvalidEccentricity {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting eccentricity.
        e: f64,
    },
    /// Dispersion sigma must be non-negative.
    #[error("dispersion sigma must be non-negative, got {value}")]
    NegativeSigma {
        /// The invalid sigma value.
        value: f64,
    },
    /// Dispersion half-width must be non-negative.
    #[error("dispersion half-width must be non-negative, got {value}")]
    NegativeHalfWidth {
        /// The invalid half-width value.
        value: f64,
    },
    /// Mission planning failure during closed-loop re-targeting.
    #[error(transparent)]
    Mission(#[from] MissionError),
    /// Propagation failure during sample execution.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Empty ensemble (no samples to compute statistics from).
    #[error("empty ensemble: no samples to compute statistics")]
    EmptyEnsemble,
    /// Trajectory count exceeds u32 range (should not happen — bounded by `num_samples`: u32).
    #[error("trajectory count {count} exceeds u32 range")]
    TooManySamples {
        /// The count that overflowed u32.
        count: usize,
    },
    /// ECI↔RIC frame conversion failed.
    #[error(transparent)]
    DcmFailure(#[from] DcmError),
    /// Operation was cancelled by the caller.
    #[error("Monte Carlo cancelled by caller")]
    Cancelled,
}

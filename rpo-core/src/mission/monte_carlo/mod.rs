//! Monte Carlo domain types and error definitions.
//!
//! The ensemble execution functions (`run_monte_carlo`, `execution`,
//! `sampling`, `statistics`) that depend on nyx-space and rayon live
//! in `rpo-nyx`. This module retains the domain types and error enum
//! used by both the analytical engine and the nyx-backed runner.

pub mod types;

use std::fmt;

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
    /// Empty ensemble (no samples to compute statistics from).
    EmptyEnsemble,
    /// Trajectory count exceeds u32 range (should not happen — bounded by `num_samples`: u32).
    TooManySamples {
        /// The count that overflowed u32.
        count: usize,
    },
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
    /// Operation was cancelled by the caller.
    Cancelled,
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
            Self::EmptyEnsemble => write!(f, "empty ensemble: no samples to compute statistics"),
            Self::TooManySamples { count } => {
                write!(f, "trajectory count {count} exceeds u32 range")
            }
            Self::DcmFailure(e) => write!(f, "frame conversion failed: {e}"),
            Self::Cancelled => write!(f, "Monte Carlo cancelled by caller"),
        }
    }
}

impl std::error::Error for MonteCarloError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Mission(e) => Some(e),
            Self::Propagation(e) => Some(e),
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

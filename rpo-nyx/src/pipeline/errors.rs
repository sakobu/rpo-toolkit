//! Server-side pipeline error type.
//!
//! Wraps `rpo_core::pipeline::PipelineError` (analytical) and adds
//! error variants for Lambert, validation, Monte Carlo, and nyx bridge operations.

use std::fmt;

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::mission::errors::MissionError;
use rpo_core::propagation::covariance::CovarianceError;
use rpo_core::propagation::lambert::LambertError;
use rpo_core::propagation::propagator::PropagationError;

use crate::monte_carlo::MonteCarloError;
use crate::nyx_bridge::NyxBridgeError;
use crate::validation::ValidationError;

/// Server-side pipeline error type.
///
/// Extends the analytical `rpo_core::pipeline::PipelineError` with
/// error variants for operations that require nyx-space.
#[derive(Debug)]
pub enum PipelineError {
    /// Error from the analytical pipeline.
    Core(rpo_core::pipeline::PipelineError),
    /// Lambert solver error.
    Lambert(LambertError),
    /// Nyx validation error.
    Validation(ValidationError),
    /// Monte Carlo error.
    MonteCarlo(MonteCarloError),
    /// Nyx bridge error (almanac, dynamics, propagation).
    NyxBridge(Box<NyxBridgeError>),
}

impl fmt::Display for PipelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Core(e) => write!(f, "{e}"),
            Self::Lambert(e) => write!(f, "{e}"),
            Self::Validation(e) => write!(f, "{e}"),
            Self::MonteCarlo(e) => write!(f, "{e}"),
            Self::NyxBridge(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for PipelineError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Core(e) => Some(e),
            Self::Lambert(e) => Some(e),
            Self::Validation(e) => Some(e),
            Self::MonteCarlo(e) => Some(e),
            Self::NyxBridge(e) => Some(e.as_ref()),
        }
    }
}

// ---- From impls: direct wrapping ----

impl From<rpo_core::pipeline::PipelineError> for PipelineError {
    fn from(e: rpo_core::pipeline::PipelineError) -> Self {
        Self::Core(e)
    }
}

impl From<LambertError> for PipelineError {
    fn from(e: LambertError) -> Self {
        Self::Lambert(e)
    }
}

impl From<ValidationError> for PipelineError {
    fn from(e: ValidationError) -> Self {
        Self::Validation(e)
    }
}

impl From<MonteCarloError> for PipelineError {
    fn from(e: MonteCarloError) -> Self {
        Self::MonteCarlo(e)
    }
}

impl From<NyxBridgeError> for PipelineError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

// ---- From impls: transitive through core PipelineError ----

impl From<MissionError> for PipelineError {
    fn from(e: MissionError) -> Self {
        Self::Core(rpo_core::pipeline::PipelineError::from(e))
    }
}

impl From<PropagationError> for PipelineError {
    fn from(e: PropagationError) -> Self {
        Self::Core(rpo_core::pipeline::PipelineError::from(e))
    }
}

impl From<CovarianceError> for PipelineError {
    fn from(e: CovarianceError) -> Self {
        Self::Core(rpo_core::pipeline::PipelineError::from(e))
    }
}

impl From<ConversionError> for PipelineError {
    fn from(e: ConversionError) -> Self {
        Self::Core(rpo_core::pipeline::PipelineError::from(e))
    }
}

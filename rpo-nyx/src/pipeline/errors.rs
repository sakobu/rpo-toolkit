//! Server-side pipeline error type.
//!
//! Wraps `rpo_core::pipeline::PipelineError` (analytical) and adds
//! error variants for Lambert, validation, Monte Carlo, and nyx bridge operations.

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::mission::errors::MissionError;
use rpo_core::mission::formation::FormationDesignError;
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
#[derive(Debug, thiserror::Error)]
pub enum PipelineError {
    /// Error from the analytical pipeline.
    #[error(transparent)]
    Core(#[from] rpo_core::pipeline::PipelineError),
    /// Lambert solver error.
    #[error(transparent)]
    Lambert(#[from] LambertError),
    /// Nyx validation error.
    #[error(transparent)]
    Validation(#[from] ValidationError),
    /// Monte Carlo error.
    #[error(transparent)]
    MonteCarlo(#[from] MonteCarloError),
    /// Nyx bridge error (almanac, dynamics, propagation).
    #[error(transparent)]
    NyxBridge(Box<NyxBridgeError>),
}

// Boxed target — hand-rolled per migration plan §4; thiserror's #[from]
// does not auto-box.
impl From<NyxBridgeError> for PipelineError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

// Transitive tunnels through rpo_core::pipeline::PipelineError — hand-rolled
// because thiserror's #[from] cannot traverse crate-boundary intermediate wrappers.
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

impl From<FormationDesignError> for PipelineError {
    fn from(e: FormationDesignError) -> Self {
        Self::Core(rpo_core::pipeline::PipelineError::from(e))
    }
}

//! Unified pipeline error type.
//!
//! Wraps all rpo-core error types that can occur during pipeline execution
//! into a single enum with `From` impls for ergonomic `?` propagation.

use std::fmt;

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::errors::MissionError;
use crate::mission::monte_carlo::MonteCarloError;
use crate::mission::validation::ValidationError;
use crate::propagation::covariance::CovarianceError;
use crate::propagation::lambert::LambertError;
use crate::propagation::nyx_bridge::NyxBridgeError;
use crate::propagation::propagator::PropagationError;

/// Unified error type for pipeline operations.
#[derive(Debug)]
pub enum PipelineError {
    /// Mission planning error (classification, targeting, waypoints).
    Mission(MissionError),
    /// Propagation error (STM, Keplerian).
    Propagation(PropagationError),
    /// Lambert solver error.
    Lambert(LambertError),
    /// Nyx validation error.
    Validation(ValidationError),
    /// Monte Carlo error (boxed to reduce enum size).
    MonteCarlo(Box<MonteCarloError>),
    /// Nyx bridge error (almanac, dynamics, propagation; boxed to reduce enum size).
    NyxBridge(Box<NyxBridgeError>),
    /// Covariance propagation error.
    Covariance(CovarianceError),
    /// A required field is missing for the requested operation.
    MissingField {
        /// Name of the missing field.
        field: &'static str,
        /// Operation that requires this field.
        context: &'static str,
    },
    /// Trajectory data is empty when a non-empty trajectory was expected.
    EmptyTrajectory,
}

impl fmt::Display for PipelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mission(e) => write!(f, "{e}"),
            Self::Propagation(e) => write!(f, "{e}"),
            Self::Lambert(e) => write!(f, "{e}"),
            Self::Validation(e) => write!(f, "{e}"),
            Self::MonteCarlo(e) => write!(f, "{}", *e),
            Self::NyxBridge(e) => write!(f, "{}", *e),
            Self::Covariance(e) => write!(f, "{e}"),
            Self::MissingField { field, context } => {
                write!(f, "{field} required for {context}")
            }
            Self::EmptyTrajectory => write!(f, "empty chief trajectory"),
        }
    }
}

impl std::error::Error for PipelineError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Mission(e) => Some(e),
            Self::Propagation(e) => Some(e),
            Self::Lambert(e) => Some(e),
            Self::Validation(e) => Some(e),
            Self::MonteCarlo(e) => Some(e.as_ref()),
            Self::NyxBridge(e) => Some(e.as_ref()),
            Self::Covariance(e) => Some(e),
            Self::MissingField { .. } | Self::EmptyTrajectory => None,
        }
    }
}

// ---- From impls ----

impl From<MissionError> for PipelineError {
    fn from(e: MissionError) -> Self {
        Self::Mission(e)
    }
}

impl From<PropagationError> for PipelineError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
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
        Self::MonteCarlo(Box::new(e))
    }
}

impl From<NyxBridgeError> for PipelineError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<CovarianceError> for PipelineError {
    fn from(e: CovarianceError) -> Self {
        Self::Covariance(e)
    }
}

impl From<ConversionError> for PipelineError {
    fn from(e: ConversionError) -> Self {
        Self::Mission(MissionError::Conversion(e))
    }
}

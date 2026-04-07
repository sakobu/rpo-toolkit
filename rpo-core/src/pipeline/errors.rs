//! Unified pipeline error type.
//!
//! Wraps all rpo-core error types that can occur during pipeline execution
//! into a single enum with `From` impls for ergonomic `?` propagation.

use std::fmt;

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::errors::MissionError;
#[cfg(feature = "server")]
use crate::mission::monte_carlo::MonteCarloError;
#[cfg(feature = "server")]
use crate::mission::validation::ValidationError;
use crate::propagation::covariance::CovarianceError;
// LambertError is always-available (pure type), but the PipelineError::Lambert
// variant is server-only (no solver to produce it without nyx). Gate the import
// to avoid unused-import warnings.
#[cfg(feature = "server")]
use crate::propagation::lambert::LambertError;
#[cfg(feature = "server")]
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
    #[cfg(feature = "server")]
    Lambert(LambertError),
    /// Nyx validation error.
    #[cfg(feature = "server")]
    Validation(ValidationError),
    /// Monte Carlo error (boxed to reduce enum size).
    #[cfg(feature = "server")]
    MonteCarlo(Box<MonteCarloError>),
    /// Nyx bridge error (almanac, dynamics, propagation; boxed to reduce enum size).
    #[cfg(feature = "server")]
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
            #[cfg(feature = "server")]
            Self::Lambert(e) => write!(f, "{e}"),
            #[cfg(feature = "server")]
            Self::Validation(e) => write!(f, "{e}"),
            #[cfg(feature = "server")]
            Self::MonteCarlo(e) => write!(f, "{}", *e),
            #[cfg(feature = "server")]
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
            #[cfg(feature = "server")]
            Self::Lambert(e) => Some(e),
            #[cfg(feature = "server")]
            Self::Validation(e) => Some(e),
            #[cfg(feature = "server")]
            Self::MonteCarlo(e) => Some(e.as_ref()),
            #[cfg(feature = "server")]
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

#[cfg(feature = "server")]
impl From<LambertError> for PipelineError {
    fn from(e: LambertError) -> Self {
        Self::Lambert(e)
    }
}

#[cfg(feature = "server")]
impl From<ValidationError> for PipelineError {
    fn from(e: ValidationError) -> Self {
        Self::Validation(e)
    }
}

#[cfg(feature = "server")]
impl From<MonteCarloError> for PipelineError {
    fn from(e: MonteCarloError) -> Self {
        Self::MonteCarlo(Box::new(e))
    }
}

#[cfg(feature = "server")]
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

//! Monte Carlo error type and `From` adapters.

use std::fmt;

use rpo_core::mission::monte_carlo::MonteCarloError as CoreMonteCarloError;

/// Errors from Monte Carlo ensemble analysis (nyx-backed).
///
/// Wraps the core analytical error and adds nyx-specific failure modes.
#[derive(Debug)]
pub enum MonteCarloError {
    /// Error from the analytical engine.
    Core(CoreMonteCarloError),
    /// Nyx bridge failure. Boxed because `NyxBridgeError` is significantly
    /// larger than `CoreMonteCarloError`; the box keeps the enum compact.
    NyxBridge(Box<crate::nyx_bridge::NyxBridgeError>),
}

impl fmt::Display for MonteCarloError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Core(e) => write!(f, "{e}"),
            Self::NyxBridge(e) => write!(f, "nyx bridge failure: {e}"),
        }
    }
}

impl std::error::Error for MonteCarloError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Core(e) => Some(e),
            Self::NyxBridge(e) => Some(e.as_ref()),
        }
    }
}

impl From<CoreMonteCarloError> for MonteCarloError {
    fn from(e: CoreMonteCarloError) -> Self {
        Self::Core(e)
    }
}

impl From<crate::nyx_bridge::NyxBridgeError> for MonteCarloError {
    fn from(e: crate::nyx_bridge::NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<rpo_core::elements::eci_ric_dcm::DcmError> for MonteCarloError {
    fn from(e: rpo_core::elements::eci_ric_dcm::DcmError) -> Self {
        Self::Core(CoreMonteCarloError::from(e))
    }
}

impl From<rpo_core::mission::errors::MissionError> for MonteCarloError {
    fn from(e: rpo_core::mission::errors::MissionError) -> Self {
        Self::Core(CoreMonteCarloError::from(e))
    }
}

impl From<rpo_core::propagation::propagator::PropagationError> for MonteCarloError {
    fn from(e: rpo_core::propagation::propagator::PropagationError) -> Self {
        Self::Core(CoreMonteCarloError::from(e))
    }
}

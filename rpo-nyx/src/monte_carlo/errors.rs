//! Monte Carlo error type and `From` adapters.

use rpo_core::mission::monte_carlo::MonteCarloError as CoreMonteCarloError;

/// Errors from Monte Carlo ensemble analysis (nyx-backed).
///
/// Wraps the core analytical error and adds nyx-specific failure modes.
#[derive(Debug, thiserror::Error)]
pub enum MonteCarloError {
    /// Error from the analytical engine.
    #[error(transparent)]
    Core(#[from] CoreMonteCarloError),
    /// Nyx bridge failure. Boxed because `NyxBridgeError` is significantly
    /// larger than `CoreMonteCarloError`; the box keeps the enum compact.
    #[error(transparent)]
    NyxBridge(Box<crate::nyx_bridge::NyxBridgeError>),
}

// Boxed + transitive From impls per migration plan §4. Box<NyxBridgeError>
// keeps the enum compact; the remaining three tunnel through Core(CoreMonteCarloError)
// because thiserror's #[from] cannot chain through an intermediate variant.
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

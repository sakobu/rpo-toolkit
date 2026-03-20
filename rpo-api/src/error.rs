//! API error types and conversion to wire-format error messages.
//!
//! `ApiError` wraps all rpo-core error types. Each variant converts to
//! `ServerMessage::Error` with a machine-readable `ErrorCode`, human-readable
//! message (from `Display`), and per-variant diagnostic `detail` fields.
//!
//! rpo-core error enums do NOT implement `Serialize` (they wrap third-party types
//! that don't). Serialization is handled here via manual field extraction.

use std::fmt;

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::mission::{MissionError, MonteCarloError, ValidationError};
use rpo_core::propagation::{CovarianceError, LambertError, NyxBridgeError, PropagationError};

use crate::protocol::{ErrorCode, ServerMessage};

/// Unified error type for the API layer.
#[derive(Debug)]
pub enum ApiError {
    /// Mission planning error (boxed to reduce enum size).
    Mission(Box<MissionError>),
    /// Propagation error.
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
    /// Invalid client input.
    InvalidInput(InvalidInputError),
    /// Operation was cancelled.
    Cancelled,
}

/// Structured error for invalid client input.
#[derive(Debug)]
pub enum InvalidInputError {
    /// JSON deserialization failed.
    MalformedJson {
        /// `serde_json` error detail.
        detail: String,
    },
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

impl fmt::Display for ApiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Mission(e) => write!(f, "{e}"),
            Self::Propagation(e) => write!(f, "{e}"),
            Self::Lambert(e) => write!(f, "{e}"),
            Self::Validation(e) => write!(f, "{e}"),
            Self::MonteCarlo(e) => write!(f, "{e}"),
            Self::NyxBridge(e) => write!(f, "{e}"),
            Self::Covariance(e) => write!(f, "{e}"),
            Self::InvalidInput(e) => write!(f, "invalid input: {e}"),
            Self::Cancelled => write!(f, "operation cancelled"),
        }
    }
}

impl std::error::Error for ApiError {}

impl fmt::Display for InvalidInputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedJson { detail } => write!(f, "malformed JSON: {detail}"),
            Self::MissingField { field, context } => {
                write!(f, "{field} required for {context}")
            }
            Self::EmptyTrajectory => write!(f, "empty chief trajectory"),
        }
    }
}

/// Require an `Option<T>` value, converting `None` to a `MissingField` error.
///
/// # Errors
/// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingField`]
/// when `value` is `None`.
pub fn require_field<T>(
    value: Option<T>,
    field: &'static str,
    context: &'static str,
) -> Result<T, ApiError> {
    value.ok_or(ApiError::InvalidInput(InvalidInputError::MissingField {
        field,
        context,
    }))
}

// From impls for each rpo-core error type
impl From<MissionError> for ApiError {
    fn from(e: MissionError) -> Self {
        Self::Mission(Box::new(e))
    }
}

impl From<PropagationError> for ApiError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<LambertError> for ApiError {
    fn from(e: LambertError) -> Self {
        Self::Lambert(e)
    }
}

impl From<ValidationError> for ApiError {
    fn from(e: ValidationError) -> Self {
        Self::Validation(e)
    }
}

impl From<MonteCarloError> for ApiError {
    fn from(e: MonteCarloError) -> Self {
        Self::MonteCarlo(Box::new(e))
    }
}

impl From<NyxBridgeError> for ApiError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<CovarianceError> for ApiError {
    fn from(e: CovarianceError) -> Self {
        Self::Covariance(e)
    }
}

impl From<ConversionError> for ApiError {
    fn from(e: ConversionError) -> Self {
        Self::Mission(Box::new(MissionError::Conversion(e)))
    }
}

impl From<rpo_core::pipeline::PipelineError> for ApiError {
    fn from(e: rpo_core::pipeline::PipelineError) -> Self {
        use rpo_core::pipeline::PipelineError;
        match e {
            PipelineError::Mission(e) => Self::Mission(Box::new(e)),
            PipelineError::Propagation(e) => Self::Propagation(e),
            PipelineError::Lambert(e) => Self::Lambert(e),
            PipelineError::Validation(e) => Self::Validation(e),
            PipelineError::MonteCarlo(e) => Self::MonteCarlo(e),
            PipelineError::NyxBridge(e) => Self::NyxBridge(e),
            PipelineError::Covariance(e) => Self::Covariance(e),
            PipelineError::MissingField { field, context } => {
                Self::InvalidInput(InvalidInputError::MissingField { field, context })
            }
            PipelineError::EmptyTrajectory => {
                Self::InvalidInput(InvalidInputError::EmptyTrajectory)
            }
        }
    }
}

impl ApiError {
    /// Convert to a `ServerMessage::Error` with per-variant diagnostic fields.
    #[must_use]
    pub fn to_server_message(&self, request_id: Option<u64>) -> ServerMessage {
        let (code, detail) = self.extract_code_and_detail();
        ServerMessage::Error {
            request_id,
            code,
            message: self.to_string(),
            detail,
        }
    }

    /// Extract machine-readable error code and structured diagnostic detail.
    fn extract_code_and_detail(&self) -> (ErrorCode, Option<serde_json::Value>) {
        match self {
            Self::Mission(inner) => match **inner {
                MissionError::TargetingConvergence {
                    final_error_km,
                    iterations,
                } => (
                    ErrorCode::TargetingConvergence,
                    Some(serde_json::json!({
                        "final_error_km": final_error_km,
                        "iterations": iterations,
                    })),
                ),
                MissionError::EmptyWaypoints => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({ "reason": "no waypoints provided" })),
                ),
                MissionError::InvalidReplanIndex {
                    index,
                    num_waypoints,
                } => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({
                        "reason": "replan index out of bounds",
                        "index": index,
                        "num_waypoints": num_waypoints,
                    })),
                ),
                MissionError::TofOptimizationFailure {
                    min_tof,
                    max_tof,
                    num_starts,
                } => (
                    ErrorCode::MissionError,
                    Some(serde_json::json!({
                        "reason": "tof_optimization_failure",
                        "min_tof_s": min_tof,
                        "max_tof_s": max_tof,
                        "num_starts": num_starts,
                    })),
                ),
                _ => (ErrorCode::MissionError, None),
            },
            Self::Propagation(_) => (ErrorCode::PropagationError, None),
            Self::Lambert(_) => (ErrorCode::LambertFailure, None),
            Self::Validation(_) => (ErrorCode::ValidationError, None),
            Self::MonteCarlo(inner) => match **inner {
                MonteCarloError::Cancelled => (ErrorCode::Cancelled, None),
                MonteCarloError::ZeroSamples => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({ "reason": "num_samples must be > 0" })),
                ),
                _ => (ErrorCode::MonteCarloError, None),
            },
            Self::NyxBridge(_) => (ErrorCode::NyxBridgeError, None),
            Self::Covariance(_) => (ErrorCode::CovarianceError, None),
            Self::InvalidInput(inner) => match inner {
                InvalidInputError::MalformedJson { detail } => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({ "reason": "malformed_json", "detail": detail })),
                ),
                InvalidInputError::MissingField { field, context } => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({ "reason": "missing_field", "field": field, "context": context })),
                ),
                InvalidInputError::EmptyTrajectory => (
                    ErrorCode::InvalidInput,
                    Some(serde_json::json!({ "reason": "empty_trajectory" })),
                ),
            },
            Self::Cancelled => (ErrorCode::Cancelled, None),
        }
    }
}

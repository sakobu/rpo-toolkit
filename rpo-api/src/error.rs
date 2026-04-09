//! Server error types — thin wrappers around rpo-nyx errors.

use crate::protocol::{ServerErrorCode, ServerMessage};
use rpo_core::propagation::lambert::LambertError;
use rpo_nyx::monte_carlo::MonteCarloError;
use rpo_nyx::nyx_bridge::NyxBridgeError;
use rpo_nyx::validation::ValidationError;
use std::fmt;

/// Errors that can occur during server-side operations.
///
/// Each variant wraps the native error type from `rpo-nyx`. Boxed variants
/// (`NyxBridge`, `MonteCarlo`) reduce enum size since those error types are large.
#[derive(Debug)]
pub enum ServerError {
    /// Lambert solver failure (convergence, degenerate geometry).
    Lambert(LambertError),
    /// Nyx bridge error (almanac load, dynamics setup, propagation).
    NyxBridge(Box<NyxBridgeError>),
    /// Full-physics validation error.
    Validation(ValidationError),
    /// Monte Carlo execution error (propagation, cancelled, zero samples).
    MonteCarlo(Box<MonteCarloError>),
    /// Client sent malformed JSON that could not be deserialized.
    ///
    /// The `serde_message` field carries the serde error description — a string
    /// is the appropriate representation here because serde errors are opaque.
    MalformedJson {
        /// serde deserialization error message.
        serde_message: String,
    },
    /// Non-Lambert pipeline or classification error.
    ///
    /// Wraps the `Display` output of upstream `rpo_nyx::pipeline::PipelineError`
    /// variants that are not Lambert-specific (e.g., classification failures).
    PipelineFailure {
        /// Upstream error description.
        source_message: String,
    },
    /// Operation cancelled by the client.
    Cancelled,
}

impl fmt::Display for ServerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Lambert(e) => write!(f, "Lambert solver error: {e}"),
            Self::NyxBridge(e) => write!(f, "Nyx bridge error: {e}"),
            Self::Validation(e) => write!(f, "Validation error: {e}"),
            Self::MonteCarlo(e) => write!(f, "Monte Carlo error: {e}"),
            Self::MalformedJson { serde_message } => {
                write!(f, "Malformed JSON: {serde_message}")
            }
            Self::PipelineFailure { source_message } => {
                write!(f, "Pipeline error: {source_message}")
            }
            Self::Cancelled => write!(f, "Operation cancelled"),
        }
    }
}

impl std::error::Error for ServerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Lambert(e) => Some(e),
            Self::NyxBridge(e) => Some(e.as_ref()),
            Self::Validation(e) => Some(e),
            Self::MonteCarlo(e) => Some(e.as_ref()),
            Self::MalformedJson { .. }
            | Self::PipelineFailure { .. }
            | Self::Cancelled => None,
        }
    }
}

impl ServerError {
    /// Convert to a `ServerMessage::Error` with structured diagnostic detail.
    #[must_use]
    pub fn to_server_message(&self, request_id: Option<u64>) -> ServerMessage {
        let (code, detail) = self.code_and_detail();
        ServerMessage::Error {
            request_id,
            code,
            message: self.to_string(),
            detail,
        }
    }

    fn code_and_detail(&self) -> (ServerErrorCode, Option<serde_json::Value>) {
        match self {
            Self::Lambert(e) => (ServerErrorCode::LambertFailure, lambert_detail(e)),
            Self::NyxBridge(_) => (ServerErrorCode::NyxBridgeError, None),
            Self::Validation(_) => (ServerErrorCode::ValidationError, None),
            Self::MonteCarlo(_) => (ServerErrorCode::MonteCarloError, None),
            Self::MalformedJson { serde_message } => (
                ServerErrorCode::InvalidInput,
                Some(serde_json::json!({ "reason": "malformed_json", "detail": serde_message })),
            ),
            Self::PipelineFailure { source_message } => (
                ServerErrorCode::InvalidInput,
                Some(serde_json::json!({ "reason": "pipeline_failure", "detail": source_message })),
            ),
            Self::Cancelled => (ServerErrorCode::Cancelled, None),
        }
    }
}

/// Extract structured diagnostic detail from Lambert errors.
fn lambert_detail(err: &LambertError) -> Option<serde_json::Value> {
    match err {
        LambertError::IzzoConvergenceFailure { details } => {
            Some(serde_json::json!({ "reason": "convergence_failure", "details": details }))
        }
        LambertError::InvalidInput { details } => {
            Some(serde_json::json!({ "reason": "invalid_input", "details": details }))
        }
        _ => None,
    }
}

// ---- From impls for ergonomic `?` ----

impl From<LambertError> for ServerError {
    fn from(e: LambertError) -> Self {
        Self::Lambert(e)
    }
}

impl From<NyxBridgeError> for ServerError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<ValidationError> for ServerError {
    fn from(e: ValidationError) -> Self {
        Self::Validation(e)
    }
}

impl From<MonteCarloError> for ServerError {
    fn from(e: MonteCarloError) -> Self {
        Self::MonteCarlo(Box::new(e))
    }
}

/// Convert nyx pipeline errors (from `compute_transfer`) into `ServerError`.
///
/// `rpo_nyx::pipeline::PipelineError` has a `Lambert(LambertError)` variant
/// which maps to `ServerError::Lambert`. All other variants (classification
/// failures, propagation errors) are input problems and map to `PipelineFailure`.
impl From<rpo_nyx::pipeline::PipelineError> for ServerError {
    fn from(e: rpo_nyx::pipeline::PipelineError) -> Self {
        match e {
            rpo_nyx::pipeline::PipelineError::Lambert(lambert_err) => {
                Self::Lambert(lambert_err)
            }
            other => Self::PipelineFailure {
                source_message: other.to_string(),
            },
        }
    }
}

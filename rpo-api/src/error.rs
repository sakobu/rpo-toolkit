//! Server error types — thin wrappers around rpo-nyx errors.

use crate::protocol::{ServerErrorCode, ServerMessage};
use rpo_core::propagation::lambert::LambertError;
use rpo_nyx::monte_carlo::MonteCarloError;
use rpo_nyx::nyx_bridge::NyxBridgeError;
use rpo_nyx::validation::ValidationError;

/// Errors that can occur during server-side operations.
///
/// Each variant wraps the native error type from `rpo-nyx`. Boxed variants
/// (`NyxBridge`, `MonteCarlo`) reduce enum size since those error types are large.
///
/// Display strings are the user-facing WebSocket API boundary and intentionally
/// use title-case phase prefixes (e.g. `"Lambert solver error:"`). Do **not**
/// convert these variants to `#[error(transparent)]` — the prefixes are what
/// client UIs render, and dropping them breaks the protocol contract.
#[derive(Debug, thiserror::Error)]
pub enum ServerError {
    /// Lambert solver failure (convergence, degenerate geometry).
    #[error("Lambert solver error: {0}")]
    Lambert(#[from] LambertError),
    /// Nyx bridge error (almanac load, dynamics setup, propagation).
    #[error("Nyx bridge error: {0}")]
    NyxBridge(#[source] Box<NyxBridgeError>),
    /// Full-physics validation error.
    #[error("Validation error: {0}")]
    Validation(#[from] ValidationError),
    /// Monte Carlo execution error (propagation, cancelled, zero samples).
    #[error("Monte Carlo error: {0}")]
    MonteCarlo(#[source] Box<MonteCarloError>),
    /// Client sent malformed JSON that could not be deserialized.
    ///
    /// The `serde_message` field carries the serde error description — a string
    /// is the appropriate representation here because serde errors are opaque.
    #[error("Malformed JSON: {serde_message}")]
    MalformedJson {
        /// serde deserialization error message.
        serde_message: String,
    },
    /// Non-Lambert pipeline or classification error.
    ///
    /// Wraps the `Display` output of upstream `rpo_nyx::pipeline::PipelineError`
    /// variants that are not Lambert-specific (e.g., classification failures).
    #[error("Pipeline error: {source_message}")]
    PipelineFailure {
        /// Upstream error description.
        source_message: String,
    },
    /// Operation cancelled by the client.
    #[error("Operation cancelled")]
    Cancelled,
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
            Self::Lambert(e) => (ServerErrorCode::LambertFailure, Some(lambert_detail(e))),
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
///
/// Exhaustive over `LambertError` so new upstream variants surface a compile
/// error here rather than silently dropping diagnostic fields on the wire.
fn lambert_detail(err: &LambertError) -> serde_json::Value {
    match err {
        LambertError::IzzoConvergenceFailure { details } => {
            serde_json::json!({ "reason": "convergence_failure", "details": details })
        }
        LambertError::InvalidInput { details } => {
            serde_json::json!({ "reason": "invalid_input", "details": details })
        }
        LambertError::NonPositiveTimeOfFlight { tof_s } => {
            serde_json::json!({ "reason": "non_positive_tof", "tof_s": tof_s })
        }
        LambertError::IdenticalPositions { separation_km } => {
            serde_json::json!({ "reason": "identical_positions", "separation_km": separation_km })
        }
    }
}

// Boxed From impls — hand-rolled because thiserror's #[from] does not auto-box.
// See migration plan §4.
impl From<NyxBridgeError> for ServerError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
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
/// Hand-rolled dispatch — not a straight `#[from]` — because Lambert failures
/// need a dedicated user-facing variant for protocol-level classification.
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

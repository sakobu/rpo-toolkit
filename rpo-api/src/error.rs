//! Server error types — thin wrappers around rpo-nyx errors.

use crate::protocol::{ServerErrorCode, ServerMessage};
use rpo_nyx::monte_carlo::MonteCarloError;
use rpo_nyx::nyx_bridge::NyxBridgeError;
use rpo_nyx::validation::ValidationError;

/// Errors that can occur during server-side operations.
///
/// Each variant wraps the native error type from `rpo-nyx`. Boxed variants
/// (`NyxBridge`, `MonteCarlo`) reduce enum size since those error types are large.
///
/// Display strings are the user-facing WebSocket API boundary and intentionally
/// use title-case phase prefixes (e.g. `"Validation error:"`). Do **not**
/// convert these variants to `#[error(transparent)]` — the prefixes are what
/// client UIs render, and dropping them breaks the protocol contract.
#[derive(Debug, thiserror::Error)]
pub enum ServerError {
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
    #[error("Malformed JSON: {serde_message}")]
    MalformedJson {
        /// serde deserialization error message.
        serde_message: String,
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
            Self::NyxBridge(_) => (ServerErrorCode::NyxBridgeError, None),
            Self::Validation(_) => (ServerErrorCode::ValidationError, None),
            Self::MonteCarlo(_) => (ServerErrorCode::MonteCarloError, None),
            Self::MalformedJson { serde_message } => (
                ServerErrorCode::InvalidInput,
                Some(serde_json::json!({ "reason": "malformed_json", "detail": serde_message })),
            ),
            Self::Cancelled => (ServerErrorCode::Cancelled, None),
        }
    }
}

// Boxed From impls — hand-rolled because thiserror's #[from] does not auto-box.
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

//! Wire protocol types for client ↔ server WebSocket communication.
//!
//! All messages are JSON with a `type` tag and a `request_id` (u64, client-assigned).
//! The `MissionDefinition` is the canonical `PipelineInput` type from rpo-core.

use serde::{Deserialize, Serialize};

use rpo_core::mission::{
    MissionPhase, MonteCarloReport, ValidationReport,
};
use rpo_core::propagation::DragConfig;

// ---------------------------------------------------------------------------
// Internal progress types (handler → WebSocket loop)
// ---------------------------------------------------------------------------

/// Phase label for long-running background operations.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProgressPhase {
    /// Per-leg nyx validation.
    Validate,
    /// Full-physics Monte Carlo ensemble analysis.
    Mc,
}

/// Progress update sent from background handlers to the WebSocket loop.
pub struct ProgressUpdate {
    /// Phase of the operation.
    pub phase: ProgressPhase,
    /// Human-readable detail.
    pub detail: Option<String>,
    /// Fraction complete (0.0 to 1.0).
    pub fraction: Option<f64>,
}

/// Central wire type sent by the client with every request.
///
/// Type alias to the canonical pipeline input type in rpo-core.
/// The server is a stateless compute engine — the client sends the
/// full mission definition with every message.
pub type MissionDefinition = rpo_core::pipeline::PipelineInput;

/// Payload for `MissionResult` responses.
///
/// Type alias to the canonical pipeline output type in rpo-core.
pub type MissionResultPayload = rpo_core::pipeline::PipelineOutput;

/// Payload for transfer result responses.
///
/// Type alias to the intermediate transfer result type in rpo-core.
pub type TransferResultPayload = rpo_core::pipeline::TransferResult;

// ---------------------------------------------------------------------------
// Client → Server messages
// ---------------------------------------------------------------------------

/// Messages sent from the client to the server.
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMessage {
    /// Classify chief/deputy separation (microseconds).
    Classify {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition (only chief + deputy + proximity used).
        mission: MissionDefinition,
    },
    /// Compute Lambert transfer only (no waypoint targeting).
    /// Returns classification, Lambert transfer, and perch handoff states.
    ComputeTransfer {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition (uses chief, deputy, perch, lambert fields).
        mission: MissionDefinition,
    },
    /// Full mission plan: classify → Lambert → waypoints → safety → covariance → eclipse.
    PlanMission {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Full mission definition.
        mission: MissionDefinition,
    },
    /// Replan from a moved waypoint (keeps earlier legs).
    MoveWaypoint {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Index of the modified waypoint.
        modified_index: usize,
        /// Full mission definition with updated waypoint.
        mission: MissionDefinition,
    },
    /// Re-solve all waypoints with new config/propagator.
    UpdateConfig {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Full mission definition with updated config.
        mission: MissionDefinition,
    },
    /// Extract differential drag rates via nyx (~3 seconds).
    ExtractDrag {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition (needs `chief_config` + `deputy_config`).
        mission: MissionDefinition,
    },
    /// Per-leg nyx validation with progress streaming (seconds).
    Validate {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition.
        mission: MissionDefinition,
        /// Number of sample points per leg (default: 50).
        #[serde(default)]
        samples_per_leg: Option<u32>,
        /// Auto-derive drag rates before validation.
        #[serde(default)]
        auto_drag: bool,
    },
    /// Full-physics Monte Carlo ensemble analysis (minutes).
    RunMC {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition (needs `monte_carlo` config).
        mission: MissionDefinition,
        /// Auto-derive drag rates before MC.
        #[serde(default)]
        auto_drag: bool,
    },
    /// Cancel a running background operation.
    Cancel {
        /// ID of the operation to cancel (or current if None).
        request_id: Option<u64>,
    },
}

// ---------------------------------------------------------------------------
// Server → Client messages
// ---------------------------------------------------------------------------

/// Messages sent from the server to the client.
#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    /// Classification result.
    ClassifyResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Classification: Proximity or `FarField` with full state info.
        phase: MissionPhase,
    },
    /// Lambert transfer result (no waypoint targeting).
    TransferResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Transfer result payload (boxed to reduce enum size).
        result: Box<TransferResultPayload>,
    },
    /// Mission planning result.
    MissionResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Full mission result payload (boxed to reduce enum size).
        result: Box<MissionResultPayload>,
    },
    /// Extracted differential drag rates.
    DragResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Extracted drag configuration.
        drag: DragConfig,
    },
    /// Progress update for long-running operations.
    Progress {
        /// Correlation ID of the operation in progress.
        request_id: u64,
        /// Phase of the operation.
        phase: ProgressPhase,
        /// Human-readable detail (e.g., "Validating leg 3/5").
        detail: Option<String>,
        /// Fraction complete (0.0 to 1.0).
        fraction: Option<f64>,
    },
    /// Validation result.
    ValidationResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Full validation report (boxed to reduce enum size).
        report: Box<ValidationReport>,
    },
    /// Monte Carlo result.
    MonteCarloResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Full MC report (boxed to reduce enum size).
        report: Box<MonteCarloReport>,
    },
    /// Operation was cancelled.
    Cancelled {
        /// Correlation ID of the cancelled operation.
        request_id: u64,
    },
    /// Periodic heartbeat sent during long-running operations.
    Heartbeat {
        /// Monotonic counter (increments each heartbeat).
        seq: u64,
    },
    /// Error response.
    Error {
        /// Correlation ID (None if error is not tied to a specific request).
        request_id: Option<u64>,
        /// Machine-readable error code.
        code: ErrorCode,
        /// Human-readable error message.
        message: String,
        /// Per-variant diagnostic fields.
        #[serde(skip_serializing_if = "Option::is_none")]
        detail: Option<serde_json::Value>,
    },
}

/// Machine-readable error codes for the frontend.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCode {
    /// Targeting solver did not converge.
    TargetingConvergence,
    /// Lambert solver failure.
    LambertFailure,
    /// Propagation error.
    PropagationError,
    /// Validation error.
    ValidationError,
    /// Monte Carlo error.
    MonteCarloError,
    /// Nyx bridge error.
    NyxBridgeError,
    /// Covariance propagation error.
    CovarianceError,
    /// Invalid input (malformed JSON, bad values).
    InvalidInput,
    /// Mission planning error (general).
    MissionError,
    /// Operation was cancelled.
    Cancelled,
}

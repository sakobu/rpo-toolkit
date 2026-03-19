//! Wire protocol types for client ↔ server WebSocket communication.
//!
//! All messages are JSON with a `type` tag and a `request_id` (u64, client-assigned).
//! The `MissionDefinition` is the central wire type sent by the client with every
//! request — the server is stateless and recomputes from the full definition each time.

use serde::{Deserialize, Serialize};

use rpo_core::mission::{
    MissionConfig, MissionPhase, MonteCarloConfig, MonteCarloReport, ValidationReport,
    WaypointMission,
};
use rpo_core::propagation::{
    DragConfig, LambertConfig, LambertTransfer, ManeuverUncertainty, MissionCovarianceReport,
    NavigationAccuracy,
};
use rpo_core::types::{SpacecraftConfig, StateVector, TransferEclipseData};

use crate::convert::WaypointInput;

// ---------------------------------------------------------------------------
// MissionDefinition wire type
// ---------------------------------------------------------------------------

/// Central wire type sent by the client with every request.
///
/// Closely mirrors the CLI's `MissionInput`. The server is a stateless compute
/// engine — the client sends the full mission definition with every message.
#[derive(Debug, Deserialize)]
pub struct MissionDefinition {
    /// Chief spacecraft ECI state.
    pub chief: StateVector,
    /// Deputy spacecraft ECI state.
    pub deputy: StateVector,
    /// Perch geometry for Lambert → proximity handoff (default: V-bar 5 km).
    #[serde(default)]
    pub perch: Option<rpo_core::mission::PerchGeometry>,
    /// Lambert transfer time-of-flight override (default: 3600 s).
    #[serde(default)]
    pub lambert_tof_s: Option<f64>,
    /// Lambert solver configuration (revolutions, direction).
    #[serde(default)]
    pub lambert_config: Option<LambertConfig>,
    /// Waypoint targets in RIC frame.
    pub waypoints: Vec<WaypointInput>,
    /// Classification threshold configuration.
    #[serde(default)]
    pub proximity: Option<rpo_core::mission::ProximityConfig>,
    /// Solver configuration (targeting, TOF optimization, safety).
    #[serde(default)]
    pub config: MissionConfig,
    /// Propagator selection: J2 or J2+Drag.
    #[serde(default)]
    pub propagator: PropagatorChoice,
    /// Chief spacecraft physical properties (required for validate/mc/drag).
    #[serde(default)]
    pub chief_config: Option<SpacecraftConfig>,
    /// Deputy spacecraft physical properties (required for validate/mc/drag).
    #[serde(default)]
    pub deputy_config: Option<SpacecraftConfig>,
    /// Navigation accuracy for covariance propagation.
    #[serde(default)]
    pub navigation_accuracy: Option<NavigationAccuracy>,
    /// Maneuver execution uncertainty for covariance propagation.
    #[serde(default)]
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Monte Carlo configuration (required for RunMC only).
    #[serde(default)]
    pub monte_carlo: Option<MonteCarloConfig>,
}

/// Propagator selection on the wire.
///
/// Wire format (serde externally tagged, default):
/// - `"j2"` → `PropagatorChoice::J2`
/// - `{ "j2_drag": { "drag": { ... } } }` → `PropagatorChoice::J2Drag { drag }`
#[derive(Debug, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum PropagatorChoice {
    /// J2-perturbed analytical propagation (no drag).
    #[default]
    J2,
    /// J2 + differential drag analytical propagation.
    J2Drag {
        /// Differential drag rates.
        drag: DragConfig,
    },
}

// ---------------------------------------------------------------------------
// Client → Server messages
// ---------------------------------------------------------------------------

/// Messages sent from the client to the server.
#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum ClientMessage {
    /// Classify chief/deputy separation (microseconds).
    Classify {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Mission definition (only chief + deputy + proximity used).
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
        /// Mission definition (needs chief_config + deputy_config).
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
        /// Mission definition (needs monte_carlo config).
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
#[serde(tag = "type")]
pub enum ServerMessage {
    /// Classification result.
    ClassifyResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Classification: Proximity or FarField with full state info.
        phase: MissionPhase,
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
        /// Phase label (e.g., "validate", "mc").
        phase: String,
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

/// Payload for `MissionResult` responses.
#[derive(Debug, Serialize)]
pub struct MissionResultPayload {
    /// Classification result.
    pub phase: MissionPhase,
    /// Lambert transfer (None if proximity).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer: Option<LambertTransfer>,
    /// Eclipse data along Lambert transfer arc.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer_eclipse: Option<TransferEclipseData>,
    /// Waypoint mission result.
    pub mission: WaypointMission,
    /// Combined Lambert + waypoint Δv.
    pub total_dv_km_s: f64,
    /// Auto-derived drag config (if auto-drag was used).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_drag_config: Option<DragConfig>,
    /// Covariance propagation report.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub covariance: Option<MissionCovarianceReport>,
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

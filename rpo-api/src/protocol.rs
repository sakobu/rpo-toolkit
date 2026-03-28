//! Wire protocol types for client ↔ server WebSocket communication.
//!
//! All messages are JSON with a `type` tag and a `request_id` (u64, client-assigned).
//! The server holds a [`Session`](crate::session::Session) per connection;
//! clients mutate it incrementally via state-setting messages and query
//! computed results via action messages.

use serde::{Deserialize, Serialize};

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::MonteCarloConfig;
use rpo_core::mission::types::PerchGeometry;
use rpo_core::mission::{MissionPhase, MonteCarloReport, ProximityConfig, ValidationReport};
use rpo_core::mission::free_drift::FreeDriftAnalysis;
use rpo_core::pipeline::{
    LeanPlanResult, LegTrajectory, SpacecraftChoice, TransferResult as CoreTransferResult,
    WaypointInput,
};
use rpo_core::propagation::covariance::{ManeuverUncertainty, MissionCovarianceReport, NavigationAccuracy};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::propagation::DragConfig;
use rpo_core::types::eclipse::{MissionEclipseData, TransferEclipseData};
use rpo_core::types::StateVector;

use crate::session::SessionSummary;

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

/// Payload for transfer result responses.
///
/// Type alias to the intermediate transfer result type in rpo-core.
pub type TransferResultPayload = CoreTransferResult;

// ---------------------------------------------------------------------------
// PropagatorToggle
// ---------------------------------------------------------------------------

/// User-facing propagator selection for the `update_config` message.
///
/// Unlike [`PropagatorChoice`](rpo_core::pipeline::PropagatorChoice), this does
/// not carry a `DragConfig` — the session's stored drag config is used when
/// `J2Drag` is selected.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropagatorToggle {
    /// J2-only analytical propagation.
    J2,
    /// J2 + differential drag (requires prior `extract_drag`).
    J2Drag,
}

// ---------------------------------------------------------------------------
// Client → Server messages
// ---------------------------------------------------------------------------

/// Messages sent from the client to the server.
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMessage {
    /// Set chief and deputy ECI states. Invalidates transfer, drag, mission.
    SetStates {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Chief spacecraft ECI state.
        chief: StateVector,
        /// Deputy spacecraft ECI state.
        deputy: StateVector,
    },
    /// Set spacecraft preset or custom configuration. Invalidates drag.
    SetSpacecraft {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Chief spacecraft choice (None = keep current).
        chief_config: Option<SpacecraftChoice>,
        /// Deputy spacecraft choice (None = keep current).
        deputy_config: Option<SpacecraftChoice>,
    },
    /// Classify chief/deputy separation (microseconds).
    Classify {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
    /// Compute Lambert transfer and store result in session.
    ComputeTransfer {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Override perch geometry (None = keep session default).
        perch: Option<PerchGeometry>,
        /// Override Lambert TOF (None = keep session default).
        lambert_tof_s: Option<f64>,
        /// Override Lambert config (None = keep session default).
        lambert_config: Option<LambertConfig>,
    },
    /// Extract differential drag rates via nyx (~3 seconds).
    ExtractDrag {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
    /// Set waypoints and plan/replan the mission.
    SetWaypoints {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Waypoint targets in RIC frame.
        waypoints: Vec<WaypointInput>,
        /// Index of the modified waypoint for incremental replan (None = full plan).
        #[serde(default)]
        changed_from: Option<usize>,
    },
    /// Update solver configuration and/or overlay parameters.
    UpdateConfig {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Solver configuration (None = keep current).
        config: Option<MissionConfig>,
        /// Propagator toggle (None = keep current).
        propagator: Option<PropagatorToggle>,
        /// Proximity classification thresholds (None = keep current).
        proximity: Option<ProximityConfig>,
        /// Navigation accuracy for covariance (None = keep current).
        navigation_accuracy: Option<NavigationAccuracy>,
        /// Maneuver uncertainty for covariance (None = keep current).
        maneuver_uncertainty: Option<ManeuverUncertainty>,
    },
    /// Fetch trajectory data for visualization (on-demand).
    GetTrajectory {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Leg indices to include (None = all legs).
        legs: Option<Vec<usize>>,
        /// Maximum points per leg (None = full resolution).
        max_points: Option<u32>,
    },
    /// Fetch covariance propagation report.
    GetCovariance {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
    /// Fetch eclipse data.
    GetEclipse {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
    /// Fetch free-drift (abort-case) trajectory and safety data.
    GetFreeDriftTrajectory {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Leg indices to include (None = all legs).
        #[serde(default)]
        legs: Option<Vec<usize>>,
        /// Maximum trajectory points per leg (None = full resolution).
        #[serde(default)]
        max_points: Option<u32>,
    },
    /// Per-leg nyx validation with progress streaming.
    Validate {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Number of sample points per leg (default: 50).
        #[serde(default)]
        samples_per_leg: Option<u32>,
        /// Auto-derive drag rates before validation.
        #[serde(default)]
        auto_drag: bool,
    },
    /// Full-physics Monte Carlo ensemble analysis.
    RunMc {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Monte Carlo config override (None = use session's).
        #[serde(default)]
        monte_carlo: Option<MonteCarloConfig>,
        /// Auto-derive drag rates before MC.
        #[serde(default)]
        auto_drag: bool,
    },
    /// Get a snapshot of current session state.
    GetSession {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
    /// Reset all session state to defaults.
    Reset {
        /// Client-assigned correlation ID.
        request_id: u64,
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
    /// Acknowledgement of a state-setting message with invalidation info.
    StateUpdated {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Fields that were set.
        updated: Vec<String>,
        /// Fields that were invalidated (cleared).
        invalidated: Vec<String>,
    },
    /// Classification result.
    ClassifyResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Classification: Proximity or `FarField` with full state info.
        phase: MissionPhase,
    },
    /// Lambert transfer result.
    TransferResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Transfer result payload (boxed to reduce enum size).
        result: Box<TransferResultPayload>,
    },
    /// Mission plan result (lean projection).
    PlanResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Lean plan result (boxed to reduce enum size).
        result: Box<LeanPlanResult>,
    },
    /// Extracted differential drag rates.
    DragResult {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Extracted drag configuration.
        drag: DragConfig,
    },
    /// Trajectory data for visualization.
    TrajectoryData {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Per-leg trajectory points.
        legs: Vec<LegTrajectory>,
    },
    /// Covariance propagation report.
    CovarianceData {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Full covariance report (boxed to reduce enum size).
        report: Box<MissionCovarianceReport>,
    },
    /// Free-drift trajectory and safety data.
    FreeDriftData {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Per-leg free-drift trajectory points.
        legs: Vec<LegTrajectory>,
        /// Per-leg free-drift safety analysis.
        analyses: Vec<FreeDriftSummary>,
    },
    /// Eclipse data for transfer and/or mission.
    EclipseData {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Transfer-phase eclipse data (None if proximity).
        #[serde(skip_serializing_if = "Option::is_none")]
        transfer: Option<TransferEclipseData>,
        /// Mission-phase eclipse data (None if no mission planned).
        #[serde(skip_serializing_if = "Option::is_none")]
        mission: Option<MissionEclipseData>,
    },
    /// Current session state snapshot.
    SessionState {
        /// Correlation ID from the client request.
        request_id: u64,
        /// Session summary.
        summary: SessionSummary,
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

/// Lean free-drift summary for API responses (no full trajectory).
#[derive(Debug, Clone, Serialize)]
pub struct FreeDriftSummary {
    /// Leg index.
    pub leg_index: usize,
    /// Min 3D distance on free-drift arc (km).
    pub min_distance_3d_km: f64,
    /// Min R/C distance on free-drift arc (km).
    pub min_rc_separation_km: f64,
    /// Min e/i separation on free-drift arc (km).
    pub min_ei_separation_km: f64,
    /// Bounded-motion residual (D'Amico Eq. 2.33). Dimensionless.
    pub bounded_motion_residual: f64,
}

impl FreeDriftSummary {
    /// Project a [`FreeDriftAnalysis`] to a lean summary.
    #[must_use]
    pub fn from_analysis(analysis: &FreeDriftAnalysis, leg_index: usize) -> Self {
        Self {
            leg_index,
            min_distance_3d_km: analysis.safety.operational.min_distance_3d_km,
            min_rc_separation_km: analysis.safety.operational.min_rc_separation_km,
            min_ei_separation_km: analysis.safety.passive.min_ei_separation_km,
            bounded_motion_residual: analysis.bounded_motion_residual,
        }
    }
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
    /// Required session state not available (call `set_states`, `compute_transfer`, etc. first).
    MissingSessionState,
}

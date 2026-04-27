//! WebSocket protocol types — 4 client message variants, 6 server message variants.
//!
//! The `ComputeTransfer` route (Lambert) was removed when the in-tree Izzo
//! solver moved to `rpo-core` and became WASM-callable; the browser now
//! computes transfers locally via `rpo-wasm` instead of round-tripping JSON
//! through this server.

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::types::{MonteCarloConfig, MonteCarloReport};
use rpo_core::mission::types::{ValidationReport, WaypointMission};
use rpo_core::pipeline::types::PropagatorChoice;
use rpo_core::propagation::covariance::types::MissionCovarianceReport;
use rpo_core::propagation::propagator::DragConfig;
use rpo_core::types::spacecraft::SpacecraftConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::validation::ColaBurn;
use serde::{Deserialize, Serialize};

// ---- Progress ----

/// Phase of a long-running background operation.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ProgressPhase {
    /// Full-physics nyx validation.
    Validate,
    /// Monte Carlo ensemble.
    Mc,
}

/// Internal progress update sent from background tasks to the WS loop.
pub(crate) struct ProgressUpdate {
    /// Which background operation is reporting.
    pub phase: ProgressPhase,
    /// Human-readable status (e.g., "Validating leg 3/5").
    pub detail: Option<String>,
    /// Completion fraction (0.0 to 1.0).
    pub fraction: Option<f64>,
}

/// Start of a long-running operation (0% complete).
pub(crate) const PROGRESS_START: f64 = 0.0;

/// Handoff to nyx engine (~10% — setup is done, heavy computation starting).
pub(crate) const PROGRESS_EXECUTING: f64 = 0.1;

/// Operation complete (100%).
pub(crate) const PROGRESS_COMPLETE: f64 = 1.0;

// ---- Client messages ----

/// Messages sent by the client over WebSocket.
///
/// Each variant is self-contained — the server holds no session state.
/// The browser manages all state via WASM and sends complete inputs.
///
/// `Validate` and `RunMc` carry full mission data (~1 KB), making the enum
/// large. This is acceptable: variants are deserialized once per message and
/// moved into handler closures — boxing would add indirection for the common
/// path without meaningful savings.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMessage {
    /// Extract differential drag rates via nyx full-physics propagation (~3s).
    ///
    /// Wraps `rpo_nyx::nyx_bridge::extract_dmf_rates()`.
    ExtractDrag {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Chief state vector in ECI.
        chief_eci: StateVector,
        /// Deputy state vector in ECI.
        deputy_eci: StateVector,
        /// Chief spacecraft physical properties.
        chief_config: SpacecraftConfig,
        /// Deputy spacecraft physical properties.
        deputy_config: SpacecraftConfig,
    },

    /// Full-physics nyx validation of an analytical mission plan.
    ///
    /// Wraps `rpo_nyx::validation::validate_mission_nyx()`.
    /// Streams `Progress` messages during execution.
    Validate {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Analytical mission to validate.
        mission: WaypointMission,
        /// Chief state vector in ECI at mission start.
        chief_eci: StateVector,
        /// Deputy state vector in ECI at mission start.
        deputy_eci: StateVector,
        /// Chief spacecraft physical properties.
        chief_config: SpacecraftConfig,
        /// Deputy spacecraft physical properties.
        deputy_config: SpacecraftConfig,
        /// Intermediate comparison samples per leg.
        samples_per_leg: u32,
        /// Optional COLA avoidance burns to inject during validation.
        #[serde(default)]
        cola_burns: Vec<ColaBurn>,
        /// Analytical COLA avoidance maneuvers for effectiveness comparison.
        /// The browser computes these via WASM `assess_cola()`; sending them
        /// here lets the server return a self-contained effectiveness comparison.
        #[serde(default)]
        analytical_cola: Vec<rpo_core::mission::AvoidanceManeuver>,
        /// Target COLA separation threshold (km) from `ColaConfig`.
        #[serde(default)]
        cola_target_distance_km: Option<f64>,
    },

    /// Monte Carlo ensemble with nyx full-physics propagation.
    ///
    /// Wraps `rpo_nyx::monte_carlo::run_monte_carlo()`.
    /// Streams `Progress` messages during execution.
    RunMc {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Nominal mission plan (reference for dispersions).
        mission: WaypointMission,
        /// Chief state vector in ECI at mission start.
        chief_eci: StateVector,
        /// Deputy state vector in ECI at mission start.
        deputy_eci: StateVector,
        /// Chief spacecraft physical properties.
        chief_config: SpacecraftConfig,
        /// Deputy spacecraft physical properties.
        deputy_config: SpacecraftConfig,
        /// Mission targeting configuration (for closed-loop re-targeting).
        mission_config: MissionConfig,
        /// Propagator selection (J2 or J2+Drag).
        propagator: PropagatorChoice,
        /// Drag config (required when propagator is J2+Drag).
        #[serde(default)]
        drag_config: Option<DragConfig>,
        /// Monte Carlo configuration (samples, dispersions, mode, seed).
        monte_carlo: MonteCarloConfig,
        /// Optional covariance predictions for cross-check.
        #[serde(default)]
        covariance_report: Option<MissionCovarianceReport>,
    },

    /// Cancel the active background job.
    Cancel {
        /// Client-assigned correlation ID.
        request_id: u64,
    },
}

// ---- Server messages ----

/// Messages sent by the server over WebSocket.
#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    /// Extracted differential drag configuration.
    DragResult {
        /// Echoed correlation ID.
        request_id: u64,
        /// DMF differential drag rates.
        drag: DragConfig,
    },

    /// Full-physics validation report.
    ValidationResult {
        /// Echoed correlation ID.
        request_id: u64,
        /// Per-leg comparison and aggregate error statistics.
        report: Box<ValidationReport>,
    },

    /// Monte Carlo ensemble report.
    MonteCarloResult {
        /// Echoed correlation ID.
        request_id: u64,
        /// Ensemble statistics and per-sample results.
        report: Box<MonteCarloReport>,
    },

    /// Progress update for long-running operations (validate, MC).
    Progress {
        /// Correlation ID of the active job.
        request_id: u64,
        /// Which operation is reporting.
        phase: ProgressPhase,
        /// Human-readable status detail.
        #[serde(skip_serializing_if = "Option::is_none")]
        detail: Option<String>,
        /// Completion fraction (0.0 to 1.0).
        #[serde(skip_serializing_if = "Option::is_none")]
        fraction: Option<f64>,
    },

    /// Error response.
    Error {
        /// Correlation ID (None for connection-level errors like malformed JSON).
        #[serde(skip_serializing_if = "Option::is_none")]
        request_id: Option<u64>,
        /// Machine-readable error code.
        code: ServerErrorCode,
        /// Human-readable error message.
        message: String,
        /// Optional structured diagnostic detail.
        #[serde(skip_serializing_if = "Option::is_none")]
        detail: Option<serde_json::Value>,
    },

    /// Confirmation that a background job was cancelled.
    Cancelled {
        /// Correlation ID of the cancelled job.
        request_id: u64,
    },

    /// Keep-alive during long-running operations.
    Heartbeat {
        /// Monotonically increasing sequence number.
        seq: u64,
    },
}

// ---- Error codes ----

/// Machine-readable error codes for server responses.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ServerErrorCode {
    /// Nyx bridge error (almanac, dynamics, propagation).
    NyxBridgeError,
    /// Full-physics validation error.
    ValidationError,
    /// Monte Carlo execution error.
    MonteCarloError,
    /// Invalid input (malformed JSON, missing fields, bad values).
    InvalidInput,
    /// Operation was cancelled by the client.
    Cancelled,
}

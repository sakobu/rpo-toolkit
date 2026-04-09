//! WebSocket protocol types — 5 client message variants, 8 server message variants.

use nalgebra::Vector3;
use rpo_core::mission::config::{MissionConfig, ProximityConfig};
use rpo_core::mission::monte_carlo::types::{MonteCarloConfig, MonteCarloReport};
use rpo_core::mission::types::{PerchGeometry, ValidationReport, WaypointMission};
use rpo_core::pipeline::types::{PropagatorChoice, TransferResult};
use rpo_core::propagation::covariance::types::MissionCovarianceReport;
use rpo_core::propagation::lambert::LambertConfig;
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

// ---- COLA burn protocol type ----

/// Protocol-level COLA burn input.
///
/// Mirrors `rpo_nyx::validation::ColaBurn` but with serde derives for
/// WebSocket deserialization. The nyx type uses `Vector3<f64>` which
/// serializes as `[f64; 3]` via nalgebra's serde support.
#[derive(Debug, Clone, Deserialize)]
pub struct ColaBurnInput {
    /// Index of the mission leg this burn applies to.
    pub leg_index: usize,
    /// Time from leg departure to COLA burn (seconds).
    pub elapsed_s: f64,
    /// Delta-v in RIC frame (km/s) as `[R, I, C]`.
    pub dv_ric_km_s: [f64; 3],
}

impl From<ColaBurnInput> for ColaBurn {
    fn from(input: ColaBurnInput) -> Self {
        Self {
            leg_index: input.leg_index,
            elapsed_s: input.elapsed_s,
            dv_ric_km_s: Vector3::new(
                input.dv_ric_km_s[0],
                input.dv_ric_km_s[1],
                input.dv_ric_km_s[2],
            ),
        }
    }
}

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
    /// Classify separation + solve Lambert transfer (far-field) or compute
    /// perch states (proximity).
    ///
    /// Wraps `rpo_nyx::pipeline::compute_transfer()`.
    /// ~100ms for far-field (Lambert), microseconds for proximity.
    ComputeTransfer {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Chief ECI state vector.
        chief: StateVector,
        /// Deputy ECI state vector (same epoch as chief).
        deputy: StateVector,
        /// Perch geometry for Lambert arrival.
        perch: PerchGeometry,
        /// Far-field vs. proximity classification thresholds.
        proximity: ProximityConfig,
        /// Lambert time-of-flight (seconds).
        lambert_tof_s: f64,
        /// Lambert solver configuration (direction, revolutions).
        lambert_config: LambertConfig,
    },

    /// Extract differential drag rates via nyx full-physics propagation (~3s).
    ///
    /// Wraps `rpo_nyx::nyx_bridge::extract_dmf_rates()`.
    ExtractDrag {
        /// Client-assigned correlation ID.
        request_id: u64,
        /// Chief ECI state vector.
        chief: StateVector,
        /// Deputy ECI state vector.
        deputy: StateVector,
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
        /// Chief ECI state at mission start.
        chief: StateVector,
        /// Deputy ECI state at mission start.
        deputy: StateVector,
        /// Chief spacecraft physical properties.
        chief_config: SpacecraftConfig,
        /// Deputy spacecraft physical properties.
        deputy_config: SpacecraftConfig,
        /// Intermediate comparison samples per leg.
        samples_per_leg: u32,
        /// Optional COLA avoidance burns to inject during validation.
        #[serde(default)]
        cola_burns: Vec<ColaBurnInput>,
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
        /// Chief ECI state at mission start.
        chief: StateVector,
        /// Deputy ECI state at mission start.
        deputy: StateVector,
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
    /// Lambert transfer result (classification + perch states).
    ///
    /// The variant name deliberately matches the inner `TransferResult` type;
    /// the serde tag `"transfer_result"` is the wire discriminant.
    TransferResult {
        /// Echoed correlation ID.
        request_id: u64,
        /// Transfer solution (boxed — largest payload, reduces enum size).
        result: Box<TransferResult>,
    },

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
    /// Lambert solver failed (convergence, degenerate geometry, etc.).
    LambertFailure,
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

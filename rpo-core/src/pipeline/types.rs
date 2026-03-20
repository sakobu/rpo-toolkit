//! Canonical pipeline types: input, output, and intermediate results.
//!
//! These types unify the CLI's `MissionInput` / `MissionOutput` and the
//! API's `MissionDefinition` / `MissionResultPayload` into a single set
//! of structs that live in `rpo-core`.

use serde::{Deserialize, Serialize};

use crate::mission::config::{MissionConfig, ProximityConfig};
use crate::mission::monte_carlo::{MonteCarloConfig, MonteCarloReport};
use crate::mission::types::{MissionPhase, MissionPlan, PerchGeometry, WaypointMission};
use crate::types::QuasiNonsingularROE;
use crate::propagation::covariance::types::{
    ManeuverUncertainty, MissionCovarianceReport, NavigationAccuracy,
};
use crate::propagation::lambert::{LambertConfig, LambertTransfer};
use crate::propagation::propagator::DragConfig;
use crate::types::{SpacecraftConfig, StateVector, TransferEclipseData};

// ---- Defaults ----

/// Default V-bar hold distance for Lambert → proximity handoff (km).
const DEFAULT_PERCH_ALONG_TRACK_KM: f64 = 5.0;

/// Default Lambert transfer time-of-flight (seconds, = 1 hour).
pub const DEFAULT_LAMBERT_TOF_S: f64 = 3600.0;

/// Default perch geometry: V-bar hold at [`DEFAULT_PERCH_ALONG_TRACK_KM`] km.
pub(crate) fn default_perch() -> PerchGeometry {
    PerchGeometry::VBar {
        along_track_km: DEFAULT_PERCH_ALONG_TRACK_KM,
    }
}

/// Default Lambert time-of-flight ([`DEFAULT_LAMBERT_TOF_S`] seconds).
fn default_lambert_tof_s() -> f64 {
    DEFAULT_LAMBERT_TOF_S
}

// ---- PropagatorChoice ----

/// User-facing propagator selection.
///
/// Serializes as `"j2"` or `{ "j2_drag": { "drag": { ... } } }`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
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

// ---- WaypointInput ----

/// Waypoint target from user input (array-based for JSON ergonomics).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaypointInput {
    /// RIC position target \[R, I, C\] in km.
    pub position_ric_km: [f64; 3],
    /// RIC velocity target \[R, I, C\] in km/s (defaults to zero).
    #[serde(default)]
    pub velocity_ric_km_s: Option<[f64; 3]>,
    /// Time-of-flight hint (seconds). If None, solver optimizes TOF.
    #[serde(default)]
    pub tof_s: Option<f64>,
    /// Human-readable label for display.
    #[serde(default)]
    pub label: Option<String>,
}

// ---- PipelineInput ----

/// Canonical mission input — single source of truth for CLI and API.
///
/// Replaces the CLI's `MissionInput` and the API's `MissionDefinition`.
/// Fields with serde defaults never need `unwrap_or` in pipeline code.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PipelineInput {
    /// Chief spacecraft ECI state.
    pub chief: StateVector,
    /// Deputy spacecraft ECI state.
    pub deputy: StateVector,
    /// Perch geometry for Lambert → proximity handoff (default: V-bar 5 km).
    #[serde(default = "default_perch")]
    pub perch: PerchGeometry,
    /// Lambert transfer time-of-flight (seconds, default: 3600).
    #[serde(default = "default_lambert_tof_s")]
    pub lambert_tof_s: f64,
    /// Lambert solver configuration (direction, revolutions).
    #[serde(default)]
    pub lambert_config: LambertConfig,
    /// Waypoint targets in RIC frame.
    pub waypoints: Vec<WaypointInput>,
    /// Classification threshold configuration.
    #[serde(default)]
    pub proximity: ProximityConfig,
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
    /// Monte Carlo configuration (required for MC only).
    #[serde(default)]
    pub monte_carlo: Option<MonteCarloConfig>,
}

// ---- TransferResult ----

/// Intermediate result from classification + Lambert + perch state computation.
///
/// Computed before waypoint targeting so that callers can optionally
/// extract differential drag rates from the perch states.
#[derive(Debug, Clone)]
pub struct TransferResult {
    /// Mission plan (phase classification, Lambert transfer, perch ROE).
    pub plan: MissionPlan,
    /// Chief ECI state at Lambert arrival (or original if proximity).
    pub perch_chief: StateVector,
    /// Deputy ECI state at perch (or original if proximity).
    pub perch_deputy: StateVector,
    /// Epoch at Lambert arrival.
    pub arrival_epoch: hifitime::Epoch,
    /// Lambert Δv magnitude (0.0 if proximity).
    pub lambert_dv_km_s: f64,
}

// ---- PipelineOutput ----

/// Canonical mission output — single source of truth for CLI and API.
///
/// Replaces the CLI's `MissionOutput` and the API's `MissionResultPayload`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PipelineOutput {
    /// Classification result.
    pub phase: MissionPhase,
    /// Lambert transfer (None if proximity).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer: Option<LambertTransfer>,
    /// Eclipse data along Lambert transfer arc.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transfer_eclipse: Option<TransferEclipseData>,
    /// Perch orbit target ROE (idealized geometric definition from `perch_to_roe()`).
    /// For V-bar: only δλ is nonzero. For R-bar: only δa is nonzero.
    pub perch_roe: QuasiNonsingularROE,
    /// Waypoint mission result.
    pub mission: WaypointMission,
    /// Combined Lambert + waypoint Δv.
    pub total_dv_km_s: f64,
    /// Combined Lambert + waypoint duration.
    pub total_duration_s: f64,
    /// Auto-derived drag config (if auto-drag was used).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_drag_config: Option<DragConfig>,
    /// Covariance propagation report.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub covariance: Option<MissionCovarianceReport>,
    /// Monte Carlo report (included only when MC is run through pipeline).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monte_carlo: Option<MonteCarloReport>,
}

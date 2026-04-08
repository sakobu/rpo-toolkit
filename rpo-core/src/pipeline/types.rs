//! Canonical pipeline types: input, output, and intermediate results.
//!
//! These types unify the CLI's `MissionInput` / `MissionOutput` and the
//! API's `MissionDefinition` / `MissionResultPayload` into a single set
//! of structs that live in `rpo-core`.

use serde::{Deserialize, Serialize};

use crate::mission::cola_assessment::{SecondaryViolation, SkippedLeg};
use crate::mission::avoidance::{AvoidanceManeuver, ColaConfig};
use crate::mission::closest_approach::ClosestApproach;
use crate::mission::config::{MissionConfig, ProximityConfig};
use crate::mission::formation::{
    FormationDesignReport, PerchEnrichmentResult, SafetyRequirements,
};
use crate::mission::free_drift::FreeDriftAnalysis;
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
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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

// ---- SpacecraftChoice ----

/// User-facing spacecraft configuration selection.
///
/// Serializes as `"cubesat_6u"`, `"servicer_500kg"`, or
/// `{ "custom": { "dry_mass_kg": ..., ... } }`.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, PartialEq)]
pub enum SpacecraftChoice {
    /// Typical 6U cubesat: 12 kg, 0.06 m² cross-section.
    #[serde(rename = "cubesat_6u")]
    Cubesat6U,
    /// Typical 500 kg servicer spacecraft.
    #[default]
    #[serde(rename = "servicer_500kg")]
    Servicer500Kg,
    /// Custom spacecraft physical properties.
    #[serde(rename = "custom")]
    Custom(SpacecraftConfig),
}

impl SpacecraftChoice {
    /// Resolve to a concrete [`SpacecraftConfig`].
    #[must_use]
    pub fn resolve(self) -> SpacecraftConfig {
        match self {
            Self::Cubesat6U => SpacecraftConfig::CUBESAT_6U,
            Self::Servicer500Kg => SpacecraftConfig::SERVICER_500KG,
            Self::Custom(config) => config,
        }
    }
}

// ---- WaypointInput ----

/// Waypoint target from user input (array-based for JSON ergonomics).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaypointInput {
    /// RIC position target \[R, I, C\] in km.
    pub position_ric_km: [f64; 3],
    /// RIC velocity target \[R, I, C\] in km/s. `None` = unspecified:
    /// targeting defaults to zero-velocity hold; enrichment interprets
    /// as 3-DOF null-space freedom.
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
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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
    /// Chief spacecraft selection: preset or custom (required for validate/mc/drag).
    #[serde(default)]
    pub chief_config: Option<SpacecraftChoice>,
    /// Deputy spacecraft selection: preset or custom (required for validate/mc/drag).
    #[serde(default)]
    pub deputy_config: Option<SpacecraftChoice>,
    /// Navigation accuracy for covariance propagation.
    #[serde(default)]
    pub navigation_accuracy: Option<NavigationAccuracy>,
    /// Maneuver execution uncertainty for covariance propagation.
    #[serde(default)]
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Monte Carlo configuration (required for MC only).
    #[serde(default)]
    pub monte_carlo: Option<MonteCarloConfig>,
    /// Collision avoidance (COLA) configuration.
    #[serde(default)]
    pub cola: Option<ColaConfig>,
    /// Safety requirements for formation design enrichment.
    /// When `Some`, perch ROE is enriched with safe e/i vectors before targeting
    /// (enforced), and waypoint enrichment + transit safety are computed (advisory).
    #[serde(default)]
    pub safety_requirements: Option<SafetyRequirements>,
}

// ---- PlanVariant ----

/// Identifies which plan variant the client wants active.
///
/// After `set_waypoints`, both a baseline (unenriched) and an enriched plan
/// may be available. `PlanVariant` selects which is active for downstream
/// operations (validation, Monte Carlo, COLA, etc.).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlanVariant {
    /// Unenriched geometric perch targeting.
    Baseline,
    /// Enriched perch targeting with safe e/i vectors.
    Enriched,
}

impl Default for PlanVariant {
    fn default() -> Self {
        Self::Baseline
    }
}

// ---- EnrichmentSuggestion ----

/// Read-only enrichment suggestion computed by `suggest_enrichment()`.
///
/// Contains the perch enrichment result and requirements.
/// Does NOT imply mutation — call `apply_perch_enrichment()` to apply.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnrichmentSuggestion {
    /// Perch enrichment result (enriched or fallback).
    pub perch: PerchEnrichmentResult,
    /// Safety requirements used for enrichment.
    pub requirements: SafetyRequirements,
}

// ---- TransferResult ----

/// Intermediate result from classification + Lambert + perch state computation.
///
/// Computed before waypoint targeting so that callers can optionally
/// extract differential drag rates from the perch states. On the server,
/// produced by the nyx Lambert pipeline; WASM clients may construct this
/// from client-side proximity classification.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransferResult {
    /// Mission plan (phase classification, Lambert transfer, perch ROE).
    pub plan: MissionPlan,
    /// Chief ECI state at Lambert arrival (or original if proximity).
    pub perch_chief: StateVector,
    /// Deputy ECI state at perch (or original if proximity).
    pub perch_deputy: StateVector,
    /// Epoch at Lambert arrival.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub arrival_epoch: hifitime::Epoch,
    /// Lambert Δv magnitude (0.0 if proximity).
    pub lambert_dv_km_s: f64,
}

// ---- PipelineOutput ----

/// Canonical mission output — single source of truth for CLI and API.
///
/// Replaces the CLI's `MissionOutput` and the API's `MissionResultPayload`.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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
    /// Safety analysis: free-drift, POCA, COLA (computed once, nested under `safety`).
    pub safety: SafetyAnalysis,
    /// Formation design report (perch enrichment, waypoint advisory, transit safety).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub formation_design: Option<FormationDesignReport>,
}

/// Complete safety analysis: free-drift, POCA, COLA.
///
/// Computed once, consumed by both validation (for COLA injection) and
/// `build_output` (for report assembly). Nested under `safety` in
/// [`PipelineOutput`].
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyAnalysis {
    /// Free-drift (abort-case) analysis per leg.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_drift: Option<Vec<FreeDriftAnalysis>>,
    /// Refined closest-approach points per leg (outer Vec = legs, inner = POCAs).
    /// Auto-computed when safety is enabled; empty inner Vec for diverging legs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub poca: Option<Vec<Vec<ClosestApproach>>>,
    /// Refined closest-approach points for free-drift trajectories per leg.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub free_drift_poca: Option<Vec<Vec<ClosestApproach>>>,
    /// Collision avoidance maneuvers computed for legs with POCA violations.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cola: Option<Vec<AvoidanceManeuver>>,
    /// Secondary conjunction warnings from post-avoidance multi-leg verification.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub secondary_conjunctions: Option<Vec<SecondaryViolation>>,
    /// Legs where COLA was attempted but failed (budget exceeded, degenerate geometry, etc.).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cola_skipped: Option<Vec<SkippedLeg>>,
}

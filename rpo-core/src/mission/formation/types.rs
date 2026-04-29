//! Types for formation design: configuration, enrichment results, transit safety.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::mission::safety::EiSeparation;
use crate::types::QuasiNonsingularROE;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Safety requirements for formation design enrichment.
///
/// No `Default` — every construction site must specify a positive
/// `min_separation_km` and an explicit alignment, matching the
/// `min_separation_km > 0` invariant documented in the `safety_envelope`
/// module. The "no requirements requested" state is modeled by the
/// [`EnrichmentSuggestion::Baseline`] variant rather than a zero-valued
/// sentinel of this struct.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SafetyRequirements {
    /// Minimum R/C separation (km) — maps to `d_min` in D'Amico Eq. 2.22.
    pub min_separation_km: f64,
    /// Preferred e/i alignment strategy.
    #[serde(default)]
    pub alignment: EiAlignment,
}

/// E/I vector alignment strategy for formation design.
///
/// Controls how the eccentricity vector is oriented relative to the
/// inclination vector after enrichment. Both `Parallel` and `AntiParallel`
/// maximize the passive safety metric (D'Amico Eq. 2.23).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum EiAlignment {
    /// Parallel e/i vectors (φ = θ). Maximizes passive safety.
    #[default]
    Parallel,
    /// Anti-parallel e/i vectors (φ = θ + π). Also maximizes passive safety.
    AntiParallel,
    /// Auto-select whichever alignment requires the smallest perturbation.
    Auto,
}

// ---------------------------------------------------------------------------
// Enrichment results
// ---------------------------------------------------------------------------

/// How the enrichment was applied.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnrichmentMode {
    /// Position-only waypoint: 3 DOF free, null-space projection applied.
    PositionOnly,
    /// Position + velocity specified: 0 DOF free, enrichment is advisory only.
    /// The enriched ROE shows what a safe state WOULD look like.
    VelocityConstrained,
}

/// Result of formation design enrichment for a single waypoint.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnrichedWaypoint {
    /// The safety-enriched ROE target.
    pub roe: QuasiNonsingularROE,
    /// The original (minimum-norm) ROE for comparison.
    pub baseline_roe: QuasiNonsingularROE,
    /// RIC position — identical to operator's input (null-space guarantee).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub position_ric_km: Vector3<f64>,
    /// E/I separation of the enriched state (D'Amico Eq. 2.22).
    pub enriched_ei: EiSeparation,
    /// E/I separation of the baseline (unenriched) state.
    pub baseline_ei: EiSeparation,
    /// Euclidean norm of the null-space perturbation (dimensionless ROE).
    pub perturbation_norm: f64,
    /// How enrichment was applied (position-only vs velocity-constrained).
    pub mode: EnrichmentMode,
    /// The e/i alignment that was resolved and applied.
    /// When `SafetyRequirements::alignment` is `Auto`, this reports which
    /// concrete alignment (`Parallel` or `AntiParallel`) was selected.
    /// Resolved once at perch, propagated to all downstream waypoints.
    pub resolved_alignment: EiAlignment,
}

/// Drift-aware e/i prediction for the first coast arc, evaluated at the
/// epoch of parallel alignment (mid-transit).
///
/// Populated by [`compute_formation_report`](crate::pipeline::execute::compute_formation_report):
/// [`enrich_with_drift_compensation`](super::transit::enrich_with_drift_compensation)
/// produces a pre-rotated departure ROE, which is then propagated forward
/// by `tof/2` under the J2 STM; the e/i separation is read at that epoch.
/// Fields reflect the mid-transit state — NOT the departure epoch (which
/// carries the pre-rotated, lagged phase by construction).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriftPrediction {
    /// Predicted minimum e/i separation at mid-transit (km).
    pub predicted_min_ei_km: f64,
    /// Predicted e/i phase angle at mid-transit (rad). Should be
    /// approximately zero when compensation is working correctly.
    pub predicted_phase_angle_rad: f64,
}

/// Result of enriching a perch geometry with safe e/i vectors.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafePerch {
    /// Geometric perch ROE before enrichment.
    pub baseline_roe: QuasiNonsingularROE,
    /// The enriched ROE (geometric perch + safe e/i vectors).
    pub roe: QuasiNonsingularROE,
    /// E-vector magnitude added (km, dimensional = a * |δe|).
    pub de_magnitude_km: f64,
    /// I-vector magnitude added (km, dimensional = a * |δi|).
    pub di_magnitude_km: f64,
    /// Resulting minimum R/C separation (D'Amico Eq. 2.23, km).
    pub min_rc_separation_km: f64,
    /// Which alignment was applied.
    pub alignment: EiAlignment,
}

/// Outcome of perch enrichment — what the engine suggests to the caller.
///
/// Two states only:
/// - [`Self::Enriched`]: requirements were supplied and the projection
///   succeeded; carries the safe perch and the requirements that produced
///   it (with `EiAlignment::Auto` already resolved to a concrete alignment).
/// - [`Self::Baseline`]: no requirements were supplied; carries the
///   geometric perch ROE so callers can render the "no enrichment" state
///   without nullable plumbing.
///
/// Enrichment-time errors propagate as [`super::FormationDesignError`] up
/// the call chain rather than being smuggled into a third "fallback"
/// variant.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum EnrichmentSuggestion {
    /// Enrichment succeeded — the enriched ROE replaces the geometric
    /// perch as the departure state on accept.
    Enriched {
        /// Safe perch geometry (e/i vectors aligned to maximize passive safety).
        safe_perch: SafePerch,
        /// Requirements that produced this suggestion. `alignment` reflects
        /// the resolved choice (`Auto` will be `Parallel` or `AntiParallel`).
        requirements: SafetyRequirements,
    },
    /// No requirements requested — geometric perch ROE preserved.
    Baseline {
        /// Geometric perch ROE (unchanged from `transfer.plan.perch_roe`).
        perch_roe: QuasiNonsingularROE,
    },
}

impl EnrichmentSuggestion {
    /// Resolved safety requirements when enrichment succeeded.
    ///
    /// Returns `Some` only for [`Self::Enriched`]; the requirements'
    /// `alignment` is the concrete choice made during projection (so a
    /// caller that requested `Auto` sees `Parallel` / `AntiParallel`
    /// here). Returns `None` for [`Self::Baseline`] — callers that need
    /// requirements at the baseline must source them elsewhere
    /// (e.g., `MissionInput::safety_requirements`).
    #[must_use]
    pub const fn resolved_requirements(&self) -> Option<&SafetyRequirements> {
        match self {
            Self::Enriched { requirements, .. } => Some(requirements),
            Self::Baseline { .. } => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Transit safety
// ---------------------------------------------------------------------------

/// Whether J2 drift compensation was applied or skipped.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriftCompensationStatus {
    /// Drift compensation was applied (pre-rotated e/i for mid-transit alignment).
    Applied,
    /// Drift compensation was skipped (arc too long, near-critical inclination, etc.).
    Skipped,
}

/// Single sample in the e/i separation profile.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EiSample {
    /// Elapsed time from leg departure (seconds).
    pub elapsed_s: f64,
    /// E/I separation at this point (km).
    pub ei_separation_km: f64,
    /// E/I phase angle at this point (rad).
    pub phase_angle_rad: f64,
}

/// E/I separation profile along a coast arc.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransitSafetyReport {
    /// Minimum e/i separation across the arc (km).
    pub min_ei_separation_km: f64,
    /// Elapsed time at minimum (seconds from leg departure).
    pub min_elapsed_s: f64,
    /// E/I phase angle at minimum (rad). 0 = parallel, π/2 = orthogonal.
    pub min_phase_angle_rad: f64,
    /// Whether the arc satisfies the safety requirement.
    pub satisfies_requirement: bool,
    /// The threshold used for `satisfies_requirement` (km). Makes the report self-documenting.
    pub threshold_km: f64,
    /// Per-sample e/i separation profile.
    pub profile: Vec<EiSample>,
}

// ---------------------------------------------------------------------------
// Mission-level report
// ---------------------------------------------------------------------------

/// Formation design report for a complete mission.
///
/// The `waypoints` and `transit_safety` vectors are indexed by leg — each
/// element corresponds to the leg at the same index. `None` entries indicate
/// that enrichment or assessment failed for that leg (non-fatal).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FormationDesignReport {
    /// Perch enrichment outcome — either enriched or baseline.
    pub perch: EnrichmentSuggestion,
    /// Per-waypoint enrichment results (advisory), indexed by leg.
    /// `None` if enrichment failed for that waypoint.
    pub waypoints: Vec<Option<EnrichedWaypoint>>,
    /// Per-leg transit safety profiles, indexed by leg.
    /// `None` if assessment failed for that leg.
    pub transit_safety: Vec<Option<TransitSafetyReport>>,
    /// Mission-wide minimum e/i separation (km).
    /// `None` when no transit assessments succeeded.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission_min_ei_separation_km: Option<f64>,
    /// Drift-aware e/i prediction for the leg-1 coast arc.
    /// Present when leg-1 TOF is within the J2 drift compensation regime.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drift_prediction: Option<DriftPrediction>,
}

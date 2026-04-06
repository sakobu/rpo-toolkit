//! Types for formation design: configuration, enrichment results, transit safety.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::mission::safety::EiSeparation;
use crate::types::QuasiNonsingularROE;

use super::errors::PerchFallbackReason;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Safety requirements for formation design enrichment.
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnrichedWaypoint {
    /// The safety-enriched ROE target.
    pub roe: QuasiNonsingularROE,
    /// The original (minimum-norm) ROE for comparison.
    pub baseline_roe: QuasiNonsingularROE,
    /// RIC position — identical to operator's input (null-space guarantee).
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DriftPrediction {
    /// Predicted minimum e/i separation at mid-transit (km).
    pub predicted_min_ei_km: f64,
    /// Predicted e/i phase angle at mid-transit (rad). Should be
    /// approximately zero when compensation is working correctly.
    pub predicted_phase_angle_rad: f64,
}

/// Result of enriching a perch geometry with safe e/i vectors.
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

/// Result of perch enrichment — distinguishes success from fallback.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum PerchEnrichmentResult {
    /// Enrichment succeeded — enriched ROE replaces geometric perch as departure state.
    Enriched(SafePerch),
    /// Baseline: geometric perch ROE, no enrichment applied.
    Baseline(QuasiNonsingularROE),
    /// Enrichment failed — pipeline continued with un-enriched geometric perch.
    Fallback {
        /// The un-enriched geometric perch ROE (used as-is for targeting).
        unenriched_roe: QuasiNonsingularROE,
        /// Why enrichment failed.
        reason: PerchFallbackReason,
    },
}

impl PerchEnrichmentResult {
    /// Resolve dynamic alignment from enrichment, falling back to original requirements.
    ///
    /// When perch enrichment succeeded and resolved `EiAlignment::Auto` to a
    /// concrete alignment, returns requirements with that resolved alignment.
    /// On `Baseline` or `Fallback`, returns the original requirements unchanged.
    #[must_use]
    pub fn resolve_requirements(&self, base: &SafetyRequirements) -> SafetyRequirements {
        match self {
            Self::Enriched(sp) => SafetyRequirements {
                min_separation_km: base.min_separation_km,
                alignment: sp.alignment,
            },
            Self::Baseline(_) | Self::Fallback { .. } => *base,
        }
    }
}

// ---------------------------------------------------------------------------
// Transit safety
// ---------------------------------------------------------------------------

/// Whether J2 drift compensation was applied or skipped.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriftCompensationStatus {
    /// Drift compensation was applied (pre-rotated e/i for mid-transit alignment).
    Applied,
    /// Drift compensation was skipped (arc too long, near-critical inclination, etc.).
    Skipped,
}

/// Single sample in the e/i separation profile.
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FormationDesignReport {
    /// Perch enrichment result — either enriched or fallback with reason.
    pub perch: PerchEnrichmentResult,
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

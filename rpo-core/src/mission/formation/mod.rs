//! Formation design: safe e/i vector enrichment for passive safety.
//!
//! Uses the `T_pos` null-space (D'Amico Eq. 2.17) to inject passively safe
//! e/i geometry into ROE states without changing the operator's requested
//! RIC position.
//!
//! # Architecture
//!
//! - **Perch enrichment** (enforced): replaces geometric perch ROE with
//!   safe e/i vectors before targeting.
//! - **Waypoint enrichment** (advisory): reports what safe ROE would look
//!   like at each waypoint, without modifying targeting.
//! - **Transit monitoring** (advisory): evaluates e/i separation along
//!   coast arcs.
//!
//! # References
//!
//! - D'Amico Eq. 2.17: `T_pos` matrix (ROE → RIC position mapping)
//! - D'Amico Eq. 2.22: e/i vector separation metric
//! - D'Amico Eq. 2.30: perigee rotation rate under J2

pub mod errors;
pub mod perch;
pub mod safety_envelope;
pub mod transit;
pub mod types;

pub use errors::FormationDesignError;
pub use types::{
    DriftPrediction, EiAlignment, EiSample,
    EnrichedWaypoint, EnrichmentMode, EnrichmentSuggestion, FormationDesignReport, SafePerch,
    SafetyRequirements, TransitSafetyReport,
};

/// Maximum ROE perturbation norm (dimensionless) before the `T_pos` linearization
/// is unreliable.
///
/// At norm = 0.01, the ROE separation is ~1% of SMA (~70 km at a 7000 km orbit).
/// Beyond this, second-order terms in the `T_pos` linearization (D'Amico Eq. 2.17)
/// produce position errors exceeding the sub-km accuracy target. Consistent with
/// the `dimensionless_norm()` threshold used for proximity classification.
///
/// Shared by every enrichment path in this module so that the two perch
/// branches (`enrich_simple_perch` and `enrich_custom_perch` via
/// `compute_safety_projection`) agree on the validity envelope.
///
/// # References
///
/// - D'Amico §2.3.4 (ROE linearization validity)
pub const LINEARIZATION_PERTURBATION_BOUND: f64 = 0.01;

/// Worst-case ratio of perturbation norm to dimensionless `d_min` for a
/// V-bar/R-bar parallel zero-baseline perch under the safety projection
/// in `safety_envelope`.
///
/// Closed-form derivation: with baseline ROE = 0 and `EiAlignment::Parallel`,
/// the projection produces enriched ROE
/// `(d_min, 0, d_min cos u, d_min sin u, d_min cos u, d_min sin u)` at any
/// chief argument-of-latitude `u`, giving
/// `‖Δ‖² = d_min² + d_min² + d_min² = 3·d_min²`, hence
/// `‖Δ‖ = √3 · d_min`. Combined with the
/// [`LINEARIZATION_PERTURBATION_BOUND`] clamp this gives the achievable cap
/// `min_separation_km ≤ a · LINEARIZATION_PERTURBATION_BOUND
/// / VBAR_RBAR_PERTURBATION_RATIO` for the default V-bar/R-bar perch modes.
///
/// Locked in by the
/// `vbar_rbar_parallel_perturbation_norm_equals_sqrt3_dmin` test in
/// `safety_envelope.rs`. The frontend `useAchievableCap` hook reads this
/// constant via the WASM `EngineConstants` bridge — do not duplicate the
/// `√3` literal in client code.
///
/// # Validity regime
///
/// Conservative for V-bar/R-bar (parallel, zero baseline). For Custom
/// perches with non-zero baseline e/i the actual ratio is geometry-
/// dependent: smaller when the baseline already points toward the target
/// e/i direction, and potentially larger when it points opposite. Callers
/// using this as a cap for Custom perches accept that occasional valid
/// geometries may be rejected; the engine remains the authoritative bound
/// via [`LINEARIZATION_PERTURBATION_BOUND`].
pub const VBAR_RBAR_PERTURBATION_RATIO: f64 = 1.732_050_807_568_877_2;

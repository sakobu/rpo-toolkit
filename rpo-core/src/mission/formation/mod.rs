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

pub use errors::{FormationDesignError, PerchFallbackReason};
pub use types::{
    DriftPrediction, EiAlignment, EiSample,
    EnrichedWaypoint, EnrichmentMode, FormationDesignReport, PerchEnrichmentResult, SafePerch,
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

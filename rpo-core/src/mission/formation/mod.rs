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
    DriftCompensationStatus, EiAlignment, EiSample, EnrichedWaypoint, EnrichmentMode,
    FormationDesignReport, PerchEnrichmentResult, SafePerch, SafetyRequirements,
    TransitSafetyReport,
};

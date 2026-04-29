//! Unified pipeline error type.
//!
//! Wraps all rpo-core error types that can occur during pipeline execution
//! into a single enum with `From` impls for ergonomic `?` propagation.

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::errors::MissionError;
use crate::mission::formation::FormationDesignError;
use crate::pipeline::transfer::ArcDensificationError;
use crate::propagation::covariance::CovarianceError;
use crate::propagation::propagator::PropagationError;

/// Unified error type for pipeline operations.
///
/// Lambert solver errors flow through [`MissionError::Lambert`] (auto-`#[from]`)
/// rather than appearing here directly — every pipeline call site routes Lambert
/// failures through the mission planner.
#[derive(Debug, thiserror::Error)]
pub enum PipelineError {
    /// Mission planning error (classification, targeting, waypoints, Lambert).
    #[error(transparent)]
    Mission(#[from] MissionError),
    /// Propagation error (STM, Keplerian).
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Covariance propagation error.
    #[error(transparent)]
    Covariance(#[from] CovarianceError),
    /// Lambert visualization-arc densification error.
    #[error(transparent)]
    ArcDensification(#[from] ArcDensificationError),
    /// Formation-design / perch-enrichment error.
    #[error(transparent)]
    FormationDesign(#[from] FormationDesignError),
    /// A required field is missing for the requested operation.
    #[error("{field} required for {context}")]
    MissingField {
        /// Name of the missing field.
        field: &'static str,
        /// Operation that requires this field.
        context: &'static str,
    },
    /// Trajectory data is empty when a non-empty trajectory was expected.
    #[error("empty chief trajectory")]
    EmptyTrajectory,
}

// Transitive tunnel: ConversionError → MissionError::Conversion → PipelineError::Mission.
// Hand-rolled because thiserror's #[from] only synthesizes one-hop conversions.
impl From<ConversionError> for PipelineError {
    fn from(e: ConversionError) -> Self {
        Self::Mission(MissionError::Conversion(e))
    }
}

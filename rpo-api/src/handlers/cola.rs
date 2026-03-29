//! COLA (collision avoidance) handler.
//!
//! Computes minimum-dv avoidance maneuvers for POCA violations on demand,
//! with multi-leg post-avoidance verification for secondary conjunctions.

use rpo_core::mission::{
    assess_cola, ColaAssessment, ColaConfig,
};
use rpo_core::pipeline::compute_poca_analysis;

use crate::error::ApiError;
use crate::protocol::{AvoidanceManeuverSummary, SecondaryViolationSummary, SkippedLegSummary};
use crate::session::Session;

/// Result of COLA evaluation, including secondary conjunction warnings.
pub struct ColaEvaluation {
    /// Avoidance maneuvers for POCA violations.
    pub maneuvers: Vec<AvoidanceManeuverSummary>,
    /// Downstream violations created by avoidance maneuvers.
    pub secondary_conjunctions: Vec<SecondaryViolationSummary>,
    /// Legs where COLA was attempted but failed.
    pub skipped: Vec<SkippedLegSummary>,
}

/// Compute collision avoidance maneuvers with multi-leg verification.
///
/// Delegates to [`assess_cola`] which checks all legs for POCA violations,
/// computes avoidance maneuvers, and verifies downstream legs for secondary
/// conjunctions.
///
/// # Errors
///
/// Returns [`ApiError`] if no mission is planned, or if POCA computation fails.
pub fn handle_run_cola(
    session: &Session,
    config: &ColaConfig,
) -> Result<ColaEvaluation, ApiError> {
    let mission = session.require_mission()?;
    let propagator = session.resolve_propagation_model();

    let all_poca = compute_poca_analysis(mission, &propagator).ok_or(
        ApiError::InvalidInput(crate::error::InvalidInputError::EmptyTrajectory),
    )?;

    let assessment = assess_cola(mission, &all_poca, &propagator, config);

    let (maneuvers, secondary, skipped) = match assessment {
        ColaAssessment::Nominal => (vec![], vec![], vec![]),
        ColaAssessment::Avoidance { maneuvers, skipped } => (maneuvers, vec![], skipped),
        ColaAssessment::SecondaryConjunction {
            maneuvers,
            secondary_violations,
            skipped,
        } => (maneuvers, secondary_violations, skipped),
    };

    Ok(ColaEvaluation {
        maneuvers: maneuvers
            .iter()
            .map(AvoidanceManeuverSummary::from_avoidance)
            .collect(),
        secondary_conjunctions: secondary
            .iter()
            .map(SecondaryViolationSummary::from_violation)
            .collect(),
        skipped: skipped
            .iter()
            .map(SkippedLegSummary::from_skipped)
            .collect(),
    })
}

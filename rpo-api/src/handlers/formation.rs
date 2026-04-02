//! Formation design handlers: on-demand enrichment and transit safety analysis.
//!
//! Inline handlers (microseconds) — pure analytical computation, no `spawn_blocking`.

use rpo_core::mission::formation::safety_envelope::enrich_waypoint;
use rpo_core::mission::formation::{
    EnrichedWaypoint, FormationDesignReport, PerchEnrichmentResult,
};
use rpo_core::pipeline::{compute_formation_report, PlanVariant};

use crate::error::{ApiError, InvalidInputError};
use crate::session::Session;

/// Compute the full formation design report for the currently selected plan variant.
///
/// Follows the POCA/COLA handler pattern: require session state, compute on
/// the fly, return lean result. Reads from stored `enrichment_suggestion`
/// rather than recomputing.
///
/// # Errors
///
/// Returns [`ApiError`] if mission, transfer, or safety requirements are
/// missing.
pub fn handle_get_formation_design(
    session: &Session,
) -> Result<FormationDesignReport, ApiError> {
    let mission = session.require_active_mission()?;
    let transfer = session.require_transfer()?;
    let requirements = session.require_safety_requirements()?;

    let perch = match session.selected_variant() {
        PlanVariant::Enriched => session
            .enrichment_suggestion()
            .map_or_else(
                || PerchEnrichmentResult::Baseline(transfer.plan.perch_roe),
                |s| s.perch.clone(),
            ),
        PlanVariant::Baseline => PerchEnrichmentResult::Baseline(transfer.plan.perch_roe),
    };

    Ok(compute_formation_report(perch, *requirements, &mission.legs))
}

/// Compute a safe alternative ROE for a single waypoint.
///
/// Resolves Auto alignment once at perch, then propagates to waypoints
/// so e/i phase is consistent across a mission. Reads alignment from
/// stored `enrichment_suggestion` rather than recomputing.
///
/// # Errors
///
/// Returns [`ApiError`] if mission, transfer, or safety requirements are
/// missing, if `waypoint_index` is out of bounds, or if enrichment fails.
pub fn handle_get_safe_alternative(
    session: &Session,
    waypoint_index: usize,
) -> Result<EnrichedWaypoint, ApiError> {
    let mission = session.require_active_mission()?;
    let requirements = session.require_safety_requirements()?;

    if waypoint_index >= mission.legs.len() {
        return Err(ApiError::InvalidInput(InvalidInputError::MissingSessionState {
            missing: "waypoint_index",
            context: "waypoint_index exceeds number of mission legs",
        }));
    }

    // Resolve alignment from stored enrichment suggestion
    let resolved_reqs = session
        .enrichment_suggestion()
        .map_or(*requirements, |s| s.perch.resolve_requirements(requirements));

    let leg = &mission.legs[waypoint_index];
    let enriched = enrich_waypoint(
        &leg.to_position_ric_km,
        Some(&leg.target_velocity_ric_km_s),
        &leg.arrival_chief_mean,
        &resolved_reqs,
    )?;

    Ok(enriched)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::Session;

    #[test]
    fn test_handle_get_formation_design_missing_mission() {
        let session = Session::default();
        let err = handle_get_formation_design(&session).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("mission"), "should require mission: {msg}");
    }

    #[test]
    fn test_handle_get_safe_alternative_missing_mission() {
        let session = Session::default();
        let err = handle_get_safe_alternative(&session, 0).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("mission"), "should require mission: {msg}");
    }
}

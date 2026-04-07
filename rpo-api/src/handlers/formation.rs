//! Formation design handlers: on-demand enrichment and transit safety analysis.
//!
//! Inline handlers (microseconds) — pure analytical computation, no `spawn_blocking`.

use rpo_core::mission::formation::safety_envelope::enrich_waypoint;
use rpo_core::mission::formation::{
    EnrichedWaypoint, FormationDesignReport, PerchEnrichmentResult,
};
use rpo_core::mission::types::WaypointMission;
use rpo_core::pipeline::convert::to_waypoints;
use rpo_core::pipeline::{
    accept_waypoint_enrichment, build_lean_plan_result, compute_formation_report, LeanPlanResult,
    PlanVariant,
};

use crate::error::{ApiError, InvalidInputError};
use crate::session::Session;

/// Validate that `waypoint_index` is within the mission's leg count.
///
/// # Errors
///
/// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::IndexOutOfBounds`]
/// when `waypoint_index >= mission.legs.len()`.
fn validate_waypoint_index(mission: &WaypointMission, waypoint_index: usize) -> Result<(), ApiError> {
    if waypoint_index >= mission.legs.len() {
        return Err(ApiError::InvalidInput(InvalidInputError::IndexOutOfBounds {
            index: waypoint_index,
            max: mission.legs.len(),
            context: "waypoint_index exceeds number of mission legs",
        }));
    }
    Ok(())
}

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

    let waypoints = to_waypoints(&session.waypoints);
    Ok(compute_formation_report(perch, *requirements, &mission.legs, &waypoints))
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
    validate_waypoint_index(mission, waypoint_index)?;

    let resolved_reqs = session.resolve_enriched_requirements(requirements);

    let leg = &mission.legs[waypoint_index];
    let enriched = enrich_waypoint(
        &leg.to_position_ric_km,
        Some(&leg.target_velocity_ric_km_s),
        &leg.arrival_chief_mean,
        &resolved_reqs,
    )?;

    Ok(enriched)
}

/// Accept the enrichment suggestion at a specific waypoint and replan.
///
/// Resolves the enriched ROE using the same alignment logic as
/// [`handle_get_safe_alternative`], converts it to a concrete RIC velocity
/// via `roe_to_ric` (D'Amico Eq. 2.17), updates the session's stored
/// waypoint, and replans the mission from the modified waypoint onward.
///
/// Acceptance collapses the dual-plan: the user commits to this enrichment,
/// so the result is stored as the new baseline with no enriched variant
/// and no lingering suggestion.
///
/// # Arguments
///
/// * `session` — mutable session state; waypoints and mission will be
///   replaced. Transfer remains intact (not invalidated by `set_waypoints`).
/// * `waypoint_index` — zero-indexed waypoint to enrich, bounds-checked
///   against `mission.legs.len()`.
///
/// # Invariants
///
/// - ROE linearization assumes small relative separation (dimensionless
///   ROE norm ≪ 1); see `roe_to_ric` (D'Amico Eq. 2.17).
/// - `session.transfer` must survive the `set_waypoints()` →
///   `store_missions()` sequence. This holds because `clear_mission_state`
///   does not invalidate the transfer tier.
///
/// # Errors
///
/// Returns [`ApiError`] if mission or safety requirements are missing, if
/// `waypoint_index` is out of bounds, if enrichment fails, or if
/// replanning fails.
pub fn handle_accept_waypoint_enrichment(
    session: &mut Session,
    waypoint_index: usize,
) -> Result<LeanPlanResult, ApiError> {
    let mission = session.require_active_mission()?;
    let requirements = session.require_safety_requirements()?;
    validate_waypoint_index(mission, waypoint_index)?;

    // Resolve enrichment in position-only mode (velocity=None): acceptance
    // is what populates the velocity via roe_to_ric in the pipeline layer.
    let resolved_reqs = session.resolve_enriched_requirements(requirements);
    let leg = &mission.legs[waypoint_index];
    let enriched = enrich_waypoint(
        &leg.to_position_ric_km,
        None,
        &leg.arrival_chief_mean,
        &resolved_reqs,
    )?;
    let chief_at_waypoint = leg.arrival_chief_mean;

    // Rebuild a PipelineInput from session, mutate the waypoint, replan.
    let mut input = session.assemble_pipeline_input()?;
    // Clone: accept_waypoint_enrichment takes &mut, but session ownership
    // is needed for set_waypoints/store_missions below.
    let mut transfer = session.require_transfer()?.clone();
    let output = accept_waypoint_enrichment(
        &mut input,
        &mut transfer,
        waypoint_index,
        &enriched.roe,
        &chief_at_waypoint,
    )?;
    let result = build_lean_plan_result(&transfer, &output.mission, &input.waypoints);

    session.set_waypoints(input.waypoints);
    session.store_missions(output.mission, None, None);

    Ok(result)
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

    #[test]
    fn test_handle_accept_waypoint_enrichment_missing_mission() {
        let mut session = Session::default();
        let err = handle_accept_waypoint_enrichment(&mut session, 0).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("mission"), "should require mission: {msg}");
    }
}

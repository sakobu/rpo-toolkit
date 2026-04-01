//! Formation design handlers: on-demand enrichment and transit safety analysis.
//!
//! Inline handlers (microseconds) — pure analytical computation, no `spawn_blocking`.

use rpo_core::mission::formation::perch::enrich_perch;
use rpo_core::mission::formation::safety_envelope::enrich_waypoint;
use rpo_core::mission::formation::transit::assess_transit_safety;
use rpo_core::mission::formation::{
    EnrichedWaypoint, FormationDesignReport, PerchEnrichmentResult, SafetyRequirements,
    TransitSafetyReport,
};
use rpo_core::mission::types::{ManeuverLeg, PerchGeometry};
use rpo_core::types::KeplerianElements;

use crate::error::{ApiError, InvalidInputError};
use crate::session::Session;

/// Try perch enrichment, returning [`PerchEnrichmentResult`] for both success and failure.
///
/// Wraps [`enrich_perch`] so callers don't repeat the `Ok`/`Err` → `Enriched`/`Fallback`
/// conversion. The `unenriched_roe` for fallback is read from `perch` + `chief_mean`.
pub(crate) fn try_enrich_perch(
    perch: &PerchGeometry,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
    unenriched_roe: rpo_core::types::QuasiNonsingularROE,
) -> PerchEnrichmentResult {
    match enrich_perch(perch, chief_mean, requirements) {
        Ok(safe_perch) => PerchEnrichmentResult::Enriched(safe_perch),
        Err(e) => PerchEnrichmentResult::Fallback {
            unenriched_roe,
            reason: e.into(),
        },
    }
}

/// Compute the full formation design report from session state.
///
/// Follows the POCA/COLA handler pattern: require session state, compute on
/// the fly, return lean result.
///
/// # Errors
///
/// Returns [`ApiError`] if mission, transfer, or safety requirements are
/// missing, or if perch enrichment fails fatally.
pub fn handle_get_formation_design(
    session: &Session,
) -> Result<FormationDesignReport, ApiError> {
    let mission = session.require_mission()?;
    let transfer = session.require_transfer()?;
    let requirements = session.require_safety_requirements()?;

    let perch_result = try_enrich_perch(
        &session.perch,
        &transfer.plan.chief_at_arrival,
        requirements,
        transfer.plan.perch_roe,
    );

    Ok(build_formation_report(
        perch_result,
        requirements,
        &mission.legs,
    ))
}

/// Compute a safe alternative ROE for a single waypoint.
///
/// Resolves Auto alignment once at perch, then propagates to waypoints
/// so e/i phase is consistent across a mission.
///
/// # Errors
///
/// Returns [`ApiError`] if mission, transfer, or safety requirements are
/// missing, if `waypoint_index` is out of bounds, or if enrichment fails.
pub fn handle_get_safe_alternative(
    session: &Session,
    waypoint_index: usize,
) -> Result<EnrichedWaypoint, ApiError> {
    let mission = session.require_mission()?;
    let transfer = session.require_transfer()?;
    let requirements = session.require_safety_requirements()?;

    if waypoint_index >= mission.legs.len() {
        return Err(ApiError::InvalidInput(InvalidInputError::MissingSessionState {
            missing: "waypoint_index",
            context: "waypoint_index exceeds number of mission legs",
        }));
    }

    // Resolve alignment from perch enrichment, propagate to waypoint
    let perch_result = try_enrich_perch(
        &session.perch,
        &transfer.plan.chief_at_arrival,
        requirements,
        transfer.plan.perch_roe,
    );
    let resolved_reqs = resolved_requirements(&perch_result, requirements);

    let leg = &mission.legs[waypoint_index];
    let enriched = enrich_waypoint(
        &leg.to_position_ric_km,
        Some(&leg.target_velocity_ric_km_s),
        &leg.arrival_chief_mean,
        &resolved_reqs,
    )?;

    Ok(enriched)
}

/// Build the formation design report from perch enrichment + mission legs.
///
/// Shared by [`handle_get_formation_design`] and the plan handler's
/// inline formation report computation.
///
/// Resolves Auto alignment once at perch, then propagates to all downstream
/// waypoint enrichment calls so e/i phase is consistent across a mission.
pub(crate) fn build_formation_report(
    perch_result: PerchEnrichmentResult,
    requirements: &SafetyRequirements,
    legs: &[ManeuverLeg],
) -> FormationDesignReport {
    let resolved_reqs = resolved_requirements(&perch_result, requirements);

    let waypoints: Vec<Option<EnrichedWaypoint>> = legs
        .iter()
        .map(|leg| {
            enrich_waypoint(
                &leg.to_position_ric_km,
                Some(&leg.target_velocity_ric_km_s),
                &leg.arrival_chief_mean,
                &resolved_reqs,
            )
            .ok()
        })
        .collect();

    let transit_safety: Vec<Option<TransitSafetyReport>> = legs
        .iter()
        .map(|leg| assess_transit_safety(&leg.trajectory, &resolved_reqs).ok())
        .collect();

    let mission_min_ei_separation_km = transit_safety
        .iter()
        .filter_map(|t| t.as_ref())
        .map(|t| t.min_ei_separation_km)
        .fold(None, |acc, v| Some(acc.map_or(v, |a: f64| a.min(v))));

    FormationDesignReport {
        perch: perch_result,
        waypoints,
        transit_safety,
        mission_min_ei_separation_km,
    }
}

/// Extract resolved alignment from a [`PerchEnrichmentResult`].
///
/// When perch enrichment succeeded and resolved `EiAlignment::Auto` to a
/// concrete alignment, propagate that to downstream waypoint enrichment.
/// On fallback, pass through the original requirements unchanged.
fn resolved_requirements(
    perch_result: &PerchEnrichmentResult,
    requirements: &SafetyRequirements,
) -> SafetyRequirements {
    match perch_result {
        PerchEnrichmentResult::Enriched(sp) => SafetyRequirements {
            min_separation_km: requirements.min_separation_km,
            alignment: sp.alignment,
        },
        PerchEnrichmentResult::Fallback { .. } => *requirements,
    }
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

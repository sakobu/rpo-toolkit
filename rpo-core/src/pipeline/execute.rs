//! Pipeline execution: `compute_transfer`, `execute_mission`, `replan_mission`, `build_output`.
//!
//! These compose the core planning primitives (classify, Lambert, waypoint
//! targeting, covariance, eclipse) into a complete mission pipeline.

use crate::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;
use crate::mission::cola_assessment::{assess_cola, ColaAssessment};
use crate::mission::avoidance::ColaConfig;
use crate::mission::closest_approach::{find_closest_approaches, ClosestApproach};
use crate::mission::formation::{
    EnrichedWaypoint, FormationDesignReport, PerchEnrichmentResult,
    SafetyRequirements, TransitSafetyReport,
};
use super::types::FormationContext;
use crate::mission::formation::perch::enrich_perch;
use crate::mission::formation::safety_envelope::enrich_waypoint;
use crate::mission::formation::transit::assess_transit_safety;
use crate::mission::free_drift::{compute_free_drift, FreeDriftAnalysis};
use crate::mission::planning::{compute_transfer_eclipse, plan_mission};
use crate::mission::types::PerchGeometry;
use crate::mission::waypoints::{plan_waypoint_mission, replan_from_waypoint};
use crate::propagation::covariance::{
    ric_accuracy_to_roe_covariance, CovarianceError, ManeuverUncertainty,
    MissionCovarianceReport, NavigationAccuracy,
};
use crate::propagation::keplerian::propagate_keplerian;
use crate::propagation::propagator::PropagationModel;
use crate::types::elements::KeplerianElements;
use crate::types::{DepartureState, StateVector};

use super::convert::{to_propagation_model, to_waypoints};
use super::errors::PipelineError;
use super::types::{PipelineInput, PipelineOutput, TransferResult};

/// Number of eclipse evaluation points along the Lambert transfer arc.
const DEFAULT_TRANSFER_ECLIPSE_SAMPLES: u32 = 200;

/// Default delta-v budget for auto-triggered COLA (km/s).
/// 10 m/s = 0.01 km/s — matches CLI default, sufficient for typical
/// proximity-phase collision avoidance maneuvers.
const AUTO_COLA_BUDGET_KM_S: f64 = 0.01;

/// Classify separation, solve Lambert if far-field, compute perch ECI states.
///
/// This is the first phase of the pipeline: resolve defaults, classify the
/// chief/deputy separation, solve Lambert if far-field, and compute the
/// perch handoff states.
///
/// Public primitive — used directly by validate/MC handlers that need
/// intermediate states for progress injection and propagator resolution.
///
/// # Errors
///
/// Returns [`PipelineError`] if classification or Lambert solving fails,
/// or if the chief trajectory is empty after Lambert propagation.
pub fn compute_transfer(input: &PipelineInput) -> Result<TransferResult, PipelineError> {
    let plan = plan_mission(
        &input.chief,
        &input.deputy,
        &input.perch,
        &input.proximity,
        input.lambert_tof_s,
        &input.lambert_config,
    )?;

    let lambert_dv_km_s = plan.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);

    let (perch_chief, perch_deputy, arrival_epoch) = if let Some(ref transfer) = plan.transfer {
        let arrival_epoch =
            input.chief.epoch + hifitime::Duration::from_seconds(input.lambert_tof_s);
        let chief_traj = propagate_keplerian(&input.chief, input.lambert_tof_s, 1)?;
        let chief_at_arrival = chief_traj
            .last()
            .ok_or(PipelineError::EmptyTrajectory)?
            .clone();

        let deputy_at_perch = StateVector {
            epoch: arrival_epoch,
            position_eci_km: transfer.arrival_state.position_eci_km,
            velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
                + transfer.arrival_dv_eci_km_s,
        };

        (chief_at_arrival, deputy_at_perch, arrival_epoch)
    } else {
        (input.chief.clone(), input.deputy.clone(), input.chief.epoch)
    };

    Ok(TransferResult {
        plan,
        perch_chief,
        perch_deputy,
        arrival_epoch,
        lambert_dv_km_s,
    })
}

/// Compute free-drift analysis for each leg of a waypoint mission.
///
/// For each leg, propagates from the pre-departure ROE (burn skipped) for
/// the same TOF and runs safety analysis on the resulting trajectory.
///
/// Returns `None` if any leg fails (non-fatal).
#[must_use]
pub fn compute_free_drift_analysis(
    mission: &crate::mission::types::WaypointMission,
    propagator: &PropagationModel,
) -> Option<Vec<FreeDriftAnalysis>> {
    let mut results = Vec::with_capacity(mission.legs.len());
    for leg in &mission.legs {
        match compute_free_drift(
            &leg.pre_departure_roe,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            leg.tof_s,
            propagator,
            leg.trajectory.len().saturating_sub(1).max(2),
        ) {
            Ok(fd) => results.push(fd),
            Err(_) => return None,
        }
    }
    Some(results)
}

/// Compute refined closest-approach (POCA) for each leg of a waypoint mission.
///
/// For each leg, runs Brent-method refinement on the grid-sampled trajectory
/// to find exact conjunction time/position/distance. Returns `None` if any
/// leg fails (non-fatal).
#[must_use]
pub fn compute_poca_analysis(
    mission: &crate::mission::types::WaypointMission,
    propagator: &PropagationModel,
) -> Option<Vec<Vec<ClosestApproach>>> {
    let mut results = Vec::with_capacity(mission.legs.len());
    for (i, leg) in mission.legs.iter().enumerate() {
        match find_closest_approaches(
            &leg.trajectory,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            propagator,
            &leg.post_departure_roe,
            i,
        ) {
            Ok(pocas) => results.push(pocas),
            Err(_) => return None,
        }
    }
    flag_global_minimum(&mut results);
    Some(results)
}

/// Compute refined closest-approach (POCA) on free-drift trajectories.
///
/// For each free-drift trajectory, runs bracket detection + Brent refinement
/// using the pre-departure ROE (the free-drift departure state). Returns
/// `None` if any leg fails (non-fatal).
#[must_use]
pub fn compute_free_drift_poca(
    free_drift: &[FreeDriftAnalysis],
    legs: &[crate::mission::types::ManeuverLeg],
    propagator: &PropagationModel,
) -> Option<Vec<Vec<ClosestApproach>>> {
    let mut results = Vec::with_capacity(free_drift.len());
    for (i, (fd, leg)) in free_drift.iter().zip(legs.iter()).enumerate() {
        match find_closest_approaches(
            &fd.trajectory,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            propagator,
            &leg.pre_departure_roe,
            i,
        ) {
            Ok(pocas) => results.push(pocas),
            Err(_) => return None,
        }
    }
    flag_global_minimum(&mut results);
    Some(results)
}

/// Set `is_global_minimum` on the single closest approach across all legs.
fn flag_global_minimum(results: &mut [Vec<ClosestApproach>]) {
    let mut best_dist = f64::INFINITY;
    let mut best_leg = 0;
    let mut best_idx = 0;
    for (leg_i, pocas) in results.iter().enumerate() {
        for (poca_i, poca) in pocas.iter().enumerate() {
            if poca.distance_km < best_dist {
                best_dist = poca.distance_km;
                best_leg = leg_i;
                best_idx = poca_i;
            }
        }
    }
    if best_dist < f64::INFINITY {
        results[best_leg][best_idx].is_global_minimum = true;
    }
}

/// Assemble a [`PipelineOutput`] from pipeline components.
///
/// Runs covariance propagation (if `navigation_accuracy` is present)
/// and eclipse computation. Covariance and eclipse failures are non-fatal —
/// the result is returned without that data.
///
/// # Arguments
///
/// * `transfer` — Transfer result from `compute_transfer()`
/// * `wp_mission` — Planned waypoint mission
/// * `input` — Original pipeline input (for covariance/eclipse config)
/// * `propagator` — Resolved propagation model
/// * `auto_drag` — Auto-derived drag config, if any
/// * `formation_context` — Perch enrichment result + requirements, if formation design was requested
#[must_use]
pub fn build_output(
    transfer: &TransferResult,
    wp_mission: crate::mission::types::WaypointMission,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: Option<crate::propagation::propagator::DragConfig>,
    formation_context: Option<FormationContext>,
) -> PipelineOutput {
    let transfer_eclipse = transfer.plan.transfer.as_ref().and_then(|t| {
        compute_transfer_eclipse(t, &input.chief, DEFAULT_TRANSFER_ECLIPSE_SAMPLES).ok()
    });

    let total_dv_km_s = transfer.lambert_dv_km_s + wp_mission.total_dv_km_s;
    let lambert_tof_s = transfer
        .plan
        .transfer
        .as_ref()
        .map_or(0.0, |t| t.tof_s);
    let total_duration_s = lambert_tof_s + wp_mission.total_duration_s;

    let covariance = input.navigation_accuracy.as_ref().and_then(|nav| {
        compute_mission_covariance(
            &wp_mission,
            &transfer.plan.chief_at_arrival,
            nav,
            input.maneuver_uncertainty.as_ref(),
            propagator,
        )
        .ok()
    });

    let free_drift = input.config.safety.as_ref().and_then(|_| {
        compute_free_drift_analysis(&wp_mission, propagator)
    });

    let poca = input.config.safety.as_ref().and_then(|_| {
        compute_poca_analysis(&wp_mission, propagator)
    });

    let free_drift_poca = free_drift.as_ref().and_then(|fd| {
        compute_free_drift_poca(fd, &wp_mission.legs, propagator)
    });

    // COLA: explicit config takes priority; auto-derive from safety thresholds
    // otherwise. When safety is enabled and POCA detects violations, COLA
    // auto-computes avoidance using min_distance_3d_km as the target separation.
    let cola_config = input.cola.or_else(|| {
        input.config.safety.as_ref().map(|safety| ColaConfig {
            target_distance_km: safety.min_distance_3d_km,
            max_dv_km_s: AUTO_COLA_BUDGET_KM_S,
        })
    });

    let (cola, secondary_conjunctions, cola_skipped) = cola_config
        .zip(poca.as_ref())
        .map_or((None, None, None), |(config, poca_data)| {
            let decision =
                assess_cola(&wp_mission, poca_data, propagator, &config);
            match decision {
                ColaAssessment::Nominal => (None, None, None),
                ColaAssessment::Avoidance { maneuvers, skipped } => {
                    let skipped_opt =
                        if skipped.is_empty() { None } else { Some(skipped) };
                    (Some(maneuvers), None, skipped_opt)
                }
                ColaAssessment::SecondaryConjunction {
                    maneuvers,
                    secondary_violations,
                    skipped,
                } => {
                    let skipped_opt =
                        if skipped.is_empty() { None } else { Some(skipped) };
                    (Some(maneuvers), Some(secondary_violations), skipped_opt)
                }
            }
        });

    let formation_design = formation_context.map(|ctx| {
        compute_formation_report(ctx.perch, ctx.requirements, &wp_mission.legs)
    });

    PipelineOutput {
        phase: transfer.plan.phase.clone(),
        transfer: transfer.plan.transfer.clone(),
        transfer_eclipse,
        perch_roe: transfer.plan.perch_roe,
        mission: wp_mission,
        total_dv_km_s,
        total_duration_s,
        auto_drag_config: auto_drag,
        covariance,
        monte_carlo: None,
        free_drift,
        poca,
        free_drift_poca,
        cola,
        secondary_conjunctions,
        cola_skipped,
        formation_design,
    }
}

/// Compute mission covariance from navigation accuracy and a planned mission.
///
/// Composes [`ric_accuracy_to_roe_covariance`] and
/// [`propagate_mission_covariance`](crate::mission::covariance::propagate_mission_covariance)
/// into a single call. Callers decide error policy: fatal callers use `?`,
/// non-fatal callers use `.ok()`.
///
/// # Errors
///
/// Returns [`CovarianceError`] if initial covariance conversion or propagation fails.
pub fn compute_mission_covariance(
    mission: &crate::mission::types::WaypointMission,
    chief_at_arrival: &KeplerianElements,
    navigation_accuracy: &NavigationAccuracy,
    maneuver_uncertainty: Option<&ManeuverUncertainty>,
    propagator: &PropagationModel,
) -> Result<MissionCovarianceReport, CovarianceError> {
    let initial_p = ric_accuracy_to_roe_covariance(navigation_accuracy, chief_at_arrival)?;
    crate::mission::covariance::propagate_mission_covariance(
        mission,
        &initial_p,
        navigation_accuracy,
        maneuver_uncertainty,
        propagator,
        DEFAULT_COVARIANCE_SAMPLES_PER_LEG,
    )
}

/// Build the [`DepartureState`] from a transfer result's perch ROE and chief elements.
fn departure_from_transfer(transfer: &TransferResult) -> DepartureState {
    DepartureState {
        roe: transfer.plan.perch_roe,
        chief: transfer.plan.chief_at_arrival,
        epoch: transfer.arrival_epoch,
    }
}

/// Plan waypoints from a transfer result.
///
/// Builds the [`DepartureState`] from the transfer's perch ROE and chief elements,
/// converts the pipeline waypoint inputs to core [`crate::mission::types::Waypoint`]s, and runs
/// waypoint targeting. Consolidates boilerplate shared by CLI and API handlers.
///
/// # Errors
///
/// Returns [`PipelineError`] if waypoint targeting fails.
pub fn plan_waypoints_from_transfer(
    transfer: &TransferResult,
    input: &PipelineInput,
    propagator: &PropagationModel,
) -> Result<crate::mission::types::WaypointMission, PipelineError> {
    let waypoints = to_waypoints(&input.waypoints);
    let departure = departure_from_transfer(transfer);
    Ok(plan_waypoint_mission(
        &departure,
        &waypoints,
        &input.config,
        propagator,
    )?)
}

/// Prepare formation design context from a transfer result and safety requirements.
///
/// Enriches the perch ROE in-place (enforced) and returns a [`FormationContext`]
/// for passing to [`build_output`]. Call this between [`compute_transfer`] and
/// [`plan_waypoints_from_transfer`] so the enriched perch flows into targeting.
///
/// Returns `None` if `safety_requirements` is not set on the input.
pub fn prepare_formation_context(
    transfer: &mut TransferResult,
    input: &PipelineInput,
) -> Option<FormationContext> {
    input.safety_requirements.as_ref().map(|reqs| FormationContext {
        perch: enrich_pipeline_perch(transfer, &input.perch, reqs),
        requirements: *reqs,
    })
}

/// Attempt perch enrichment. On success, replaces `transfer.plan.perch_roe`
/// with the enriched ROE (enforced). Returns the result for reporting.
fn enrich_pipeline_perch(
    transfer: &mut TransferResult,
    perch: &PerchGeometry,
    requirements: &SafetyRequirements,
) -> PerchEnrichmentResult {
    match enrich_perch(perch, &transfer.plan.chief_at_arrival, requirements) {
        Ok(safe_perch) => {
            transfer.plan.perch_roe = safe_perch.roe;
            PerchEnrichmentResult::Enriched(safe_perch)
        }
        Err(e) => PerchEnrichmentResult::Fallback {
            unenriched_roe: transfer.plan.perch_roe,
            reason: e.into(),
        },
    }
}

/// Build the advisory formation design report from perch enrichment results.
///
/// Applies R2 (resolve alignment once at perch, propagate to all waypoints),
/// then enriches each waypoint and assesses transit safety per leg.
/// Uses per-leg chief mean elements (which evolve under J2) rather than a
/// single chief epoch, so that the T-matrix and e/i geometry are evaluated
/// at the correct epoch for each leg.
fn compute_formation_report(
    perch_result: PerchEnrichmentResult,
    reqs: SafetyRequirements,
    legs: &[crate::mission::types::ManeuverLeg],
) -> FormationDesignReport {
    // R2: resolve alignment once at perch, propagate to all waypoints.
    // If perch enrichment succeeded, use the resolved concrete alignment
    // (Auto → Parallel or AntiParallel). On fallback, use original reqs.
    let resolved_reqs = match &perch_result {
        PerchEnrichmentResult::Enriched(sp) => SafetyRequirements {
            min_separation_km: reqs.min_separation_km,
            alignment: sp.alignment,
        },
        PerchEnrichmentResult::Fallback { .. } => reqs,
    };

    // Advisory: enrich each waypoint (velocity-constrained path — all
    // waypoints currently carry velocity via to_waypoints() zero-fill).
    // Uses per-leg arrival chief mean so T-matrix matches J2-evolved epoch.
    // Indexed by leg — None if enrichment failed for that waypoint.
    let waypoints: Vec<Option<EnrichedWaypoint>> = legs.iter().map(|leg| {
        enrich_waypoint(
            &leg.to_position_ric_km,
            Some(&leg.target_velocity_ric_km_s),
            &leg.arrival_chief_mean,
            &resolved_reqs,
        ).ok()
    }).collect();

    // Advisory: transit e/i separation per leg.
    // assess_transit_safety reads per-sample chief from trajectory internally.
    // Indexed by leg — None if assessment failed for that leg.
    let transit_safety: Vec<Option<TransitSafetyReport>> = legs.iter().map(|leg| {
        assess_transit_safety(&leg.trajectory, &resolved_reqs).ok()
    }).collect();

    let mission_min_ei_separation_km = transit_safety.iter()
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

/// Execute the full mission pipeline: classify → Lambert → waypoints → covariance → eclipse.
///
/// Single entry point for the CLI `mission` command and the API `PlanMission` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn execute_mission(input: &PipelineInput) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);

    let formation_context = prepare_formation_context(&mut transfer, input);

    let wp_mission = plan_waypoints_from_transfer(&transfer, input, &propagator)?;

    Ok(build_output(&transfer, wp_mission, input, &propagator, None, formation_context))
}

/// Incremental replan pipeline: classify → Lambert → replan from index.
///
/// When `cached_mission` is provided, reuses the kept legs (`0..modified_index`)
/// without re-solving them. When `None`, plans the full mission first as a
/// fallback (prior behavior).
///
/// Used by the API's `MoveWaypoint` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn replan_mission(
    input: &PipelineInput,
    modified_index: usize,
    cached_mission: Option<crate::mission::types::WaypointMission>,
) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);

    let formation_context = prepare_formation_context(&mut transfer, input);

    let waypoints = to_waypoints(&input.waypoints);
    let departure = departure_from_transfer(&transfer);

    let base_mission = match cached_mission {
        Some(m) => m,
        None => plan_waypoint_mission(&departure, &waypoints, &input.config, &propagator)?,
    };

    let wp_mission = replan_from_waypoint(
        base_mission,
        modified_index,
        &waypoints,
        &departure,
        &input.config,
        &propagator,
    )?;

    Ok(build_output(&transfer, wp_mission, input, &propagator, None, formation_context))
}

#[cfg(test)]
mod tests {
    use crate::mission::config::ProximityConfig;
    use crate::propagation::lambert::LambertConfig;

    use super::*;

    /// Tolerance for pure data-copy fidelity tests (no arithmetic, exact IEEE 754 roundtrip).
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;

    /// Minimum e/i magnitude expected after enrichment (dimensionless ROE).
    /// For a 100m separation at a = 7000 km, d_min/a ~ 1.4e-5. 1e-10 provides
    /// 5 orders of margin above floating-point noise.
    const ENRICHMENT_NONZERO_TOL: f64 = 1e-10;

    /// Exact-equality tolerance for ROE that should be bitwise identical.
    /// The enriched ROE is assigned directly (no arithmetic), so the only
    /// difference is f64 representation noise from serialization roundtrips.
    const ROE_IDENTITY_TOL: f64 = 1e-15;

    /// Build a minimal `PipelineInput` from the standard far-field test scenario.
    fn far_field_input() -> PipelineInput {
        use crate::types::StateVector;
        use hifitime::Epoch;
        use nalgebra::Vector3;

        let chief = StateVector {
            epoch: Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap(),
            position_eci_km: Vector3::new(5876.261, 3392.661, 0.0),
            velocity_eci_km_s: Vector3::new(-2.380512, 4.123167, 6.006917),
        };

        let deputy = StateVector {
            epoch: Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap(),
            position_eci_km: Vector3::new(5199.839421, 4281.648523, 1398.070066),
            velocity_eci_km_s: Vector3::new(-3.993103, 2.970313, 5.764540),
        };

        PipelineInput {
            chief,
            deputy,
            perch: crate::pipeline::types::default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![
                crate::pipeline::types::WaypointInput {
                    position_ric_km: [0.5, 2.0, 0.5],
                    velocity_ric_km_s: None,
                    tof_s: Some(4200.0),
                    label: Some("WP1".into()),
                },
                crate::pipeline::types::WaypointInput {
                    position_ric_km: [0.5, 0.5, 0.5],
                    velocity_ric_km_s: Some([0.0, 0.001, 0.0]),
                    tof_s: Some(4200.0),
                    label: Some("WP2".into()),
                },
            ],
            proximity: ProximityConfig::default(),
            config: crate::mission::config::MissionConfig::default(),
            propagator: crate::pipeline::types::PropagatorChoice::J2,
            chief_config: None,
            deputy_config: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo: None,
            cola: None,
            safety_requirements: None,
        }
    }

    #[test]
    fn test_compute_transfer_far_field() {
        let input = far_field_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");

        // Far-field scenario should produce a Lambert transfer
        assert!(result.plan.transfer.is_some());
        assert!(result.lambert_dv_km_s > 0.0);
    }

    #[test]
    fn test_execute_mission_far_field() {
        let input = far_field_input();
        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Should have a Lambert transfer and waypoint legs
        assert!(output.transfer.is_some());
        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
        assert!(output.total_duration_s > 0.0);
    }

    #[test]
    fn test_replan_mission() {
        let input = far_field_input();
        let output = replan_mission(&input, 0, None).expect("replan_mission should succeed");

        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
    }

    #[test]
    fn test_replan_mission_with_cache() {
        let input = far_field_input();
        // First run a full mission to get a cached result
        let full_output = execute_mission(&input).expect("execute_mission should succeed");

        // Replan from waypoint 1 using the cached mission
        let output = replan_mission(&input, 1, Some(full_output.mission))
            .expect("replan_mission with cache should succeed");

        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
    }

    /// Build a proximity-regime `PipelineInput` (deputy 1 km higher than chief).
    fn proximity_input() -> PipelineInput {
        use crate::elements::keplerian_conversions::keplerian_to_state;
        use crate::test_helpers::iss_like_elements;
        use hifitime::Epoch;

        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher → Proximity regime
        deputy_ke.mean_anomaly_rad += 0.01;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        PipelineInput {
            chief,
            deputy,
            perch: crate::pipeline::types::default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![crate::pipeline::types::WaypointInput {
                position_ric_km: [0.5, 2.0, 0.5],
                velocity_ric_km_s: None,
                tof_s: Some(4200.0),
                label: Some("WP1".into()),
            }],
            proximity: ProximityConfig::default(),
            config: crate::mission::config::MissionConfig::default(),
            propagator: crate::pipeline::types::PropagatorChoice::J2,
            chief_config: None,
            deputy_config: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo: None,
            cola: None,
            safety_requirements: None,
        }
    }

    #[test]
    fn test_compute_transfer_proximity_arrival_epoch() {
        let input = proximity_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");

        // Proximity scenario: no Lambert transfer
        assert!(result.plan.transfer.is_none());
        assert!(
            result.lambert_dv_km_s.abs() < f64::EPSILON,
            "proximity should have zero Lambert delta-v"
        );

        // arrival_epoch must equal the chief epoch (no Lambert offset)
        assert_eq!(
            result.arrival_epoch, input.chief.epoch,
            "proximity arrival_epoch should equal chief epoch, not chief + lambert_tof_s"
        );
    }

    #[test]
    fn test_pipeline_input_serde_roundtrip() {
        let input = far_field_input();
        let json = serde_json::to_string(&input).expect("serialize");
        let roundtrip: PipelineInput = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(input.chief.epoch, roundtrip.chief.epoch);
        assert_eq!(input.waypoints.len(), roundtrip.waypoints.len());
        assert!(
            (input.lambert_tof_s - roundtrip.lambert_tof_s).abs() < COPY_FIDELITY_TOL,
            "lambert_tof_s serde roundtrip mismatch"
        );
    }

    #[test]
    fn test_transfer_result_serde_roundtrip() {
        let input = far_field_input();
        let result = compute_transfer(&input).expect("compute_transfer should succeed");
        let json = serde_json::to_string(&result).expect("serialize TransferResult");
        let roundtrip: TransferResult =
            serde_json::from_str(&json).expect("deserialize TransferResult");

        assert_eq!(result.arrival_epoch, roundtrip.arrival_epoch);
        assert!(
            (result.lambert_dv_km_s - roundtrip.lambert_dv_km_s).abs() < COPY_FIDELITY_TOL,
            "lambert_dv_km_s serde roundtrip mismatch"
        );
        assert!(result.plan.transfer.is_some() == roundtrip.plan.transfer.is_some());
    }

    #[test]
    fn test_to_waypoints_conversion() {
        let inputs = vec![
            crate::pipeline::types::WaypointInput {
                position_ric_km: [1.0, 2.0, 3.0],
                velocity_ric_km_s: Some([0.1, 0.2, 0.3]),
                tof_s: Some(100.0),
                label: None,
            },
            crate::pipeline::types::WaypointInput {
                position_ric_km: [4.0, 5.0, 6.0],
                velocity_ric_km_s: None,
                tof_s: None,
                label: None,
            },
        ];

        let waypoints = to_waypoints(&inputs);
        assert_eq!(waypoints.len(), 2);
        assert!(
            (waypoints[0].position_ric_km.x - 1.0).abs() < COPY_FIDELITY_TOL,
            "position_ric_km.x copy mismatch"
        );
        assert!(
            (waypoints[0].velocity_ric_km_s.y - 0.2).abs() < COPY_FIDELITY_TOL,
            "velocity_ric_km_s.y copy mismatch"
        );
        assert!(
            (waypoints[0].tof_s.unwrap() - 100.0).abs() < COPY_FIDELITY_TOL,
            "tof_s copy mismatch"
        );
        // Second waypoint should default velocity to zero
        assert!(
            waypoints[1].velocity_ric_km_s.x.abs() < COPY_FIDELITY_TOL,
            "default velocity should be zero"
        );
        assert!(waypoints[1].tof_s.is_none());
    }

    #[test]
    fn test_execute_mission_with_formation_design() {
        use crate::mission::formation::{
            EiAlignment, PerchEnrichmentResult, SafetyRequirements,
        };

        let mut input = far_field_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.1,
            alignment: EiAlignment::Parallel,
        });

        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Formation design report must be present
        let report = output.formation_design
            .as_ref()
            .expect("formation_design should be Some when safety_requirements is set");

        // Perch should be enriched (V-bar perch has zero e/i → enrichment adds them)
        assert!(
            matches!(report.perch, PerchEnrichmentResult::Enriched(_)),
            "V-bar perch should enrich successfully"
        );

        // Enriched perch ROE should have nonzero e/i vectors
        let roe = &output.perch_roe;
        let ei_magnitude = (roe.dex * roe.dex + roe.dey * roe.dey
            + roe.dix * roe.dix + roe.diy * roe.diy).sqrt();

        assert!(
            ei_magnitude > ENRICHMENT_NONZERO_TOL,
            "enriched perch should have nonzero e/i vectors, got {ei_magnitude}"
        );

        // Enforced enrichment must flow through: output.perch_roe == report enriched ROE
        if let PerchEnrichmentResult::Enriched(ref sp) = report.perch {
            let diff = (sp.roe.to_vector() - roe.to_vector()).norm();
            assert!(
                diff < ROE_IDENTITY_TOL,
                "output.perch_roe must match enriched perch ROE, diff = {diff:.2e}"
            );
        }

        // Vectors indexed by leg — same length as legs
        assert_eq!(
            report.waypoints.len(),
            output.mission.legs.len(),
            "waypoints should have one entry per leg"
        );
        assert_eq!(
            report.transit_safety.len(),
            output.mission.legs.len(),
            "transit_safety should have one entry per leg"
        );

        // All entries should be Some (no failures in this scenario)
        assert!(
            report.waypoints.iter().all(Option::is_some),
            "all waypoint enrichments should succeed"
        );
        assert!(
            report.transit_safety.iter().all(Option::is_some),
            "all transit assessments should succeed"
        );

        // Mission-wide minimum e/i separation should be present and positive
        let mission_min = report.mission_min_ei_separation_km
            .expect("mission_min should be Some when transit assessments succeed");
        assert!(
            mission_min > 0.0,
            "mission min e/i separation should be positive, got {mission_min}"
        );
    }
}

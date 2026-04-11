//! Server-side pipeline: Lambert-dependent mission orchestration.
//!
//! Contains the 4 server-only pipeline functions that require nyx-space
//! (via the Lambert solver). The WASM-eligible pipeline functions
//! ([`execute_mission_from_transfer`],
//! [`replan_from_transfer`]) remain in `rpo-core`.
//!
//! ## Functions
//!
//! - [`compute_transfer`] -- classify separation, solve Lambert, compute perch states
//! - [`compute_validation_burns`] -- derive COLA burns for validation injection
//! - [`execute_mission`] -- full pipeline: Lambert + mission planning
//! - [`replan_mission`] -- full pipeline: Lambert + replanning from a modified waypoint

pub mod errors;

pub use errors::PipelineError;

use rpo_core::mission::config::SafetyConfig;
use rpo_core::mission::avoidance::ColaConfig;
use rpo_core::pipeline::{
    execute_mission_from_transfer, replan_from_transfer,
    PipelineInput, PipelineOutput, SafetyAnalysis, TransferResult,
    compute_safety_analysis,
};
use rpo_core::propagation::keplerian::propagate_keplerian;
use rpo_core::types::StateVector;

use crate::planning::plan_mission;
use crate::validation::convert_cola_to_burns;

/// Classify separation, solve Lambert if far-field, compute perch ECI states.
///
/// This is the first phase of the pipeline: resolve defaults, classify the
/// chief/deputy separation, solve Lambert if far-field, and compute the
/// perch handoff states.
///
/// Public primitive -- used directly by validate/MC handlers that need
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
            .ok_or(rpo_core::pipeline::PipelineError::EmptyTrajectory)?
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

/// Compute safety analysis and derive COLA burns for validation injection.
///
/// Combines [`compute_safety_analysis`]
/// and [`convert_cola_to_burns`] into a
/// single call, ensuring COLA burns are always derived consistently from the
/// safety analysis. Used by both CLI and API validate handlers.
///
/// # Errors
/// Returns [`crate::validation::ValidationError`] if a COLA burn epoch is out of bounds.
pub fn compute_validation_burns(
    wp_mission: &rpo_core::mission::types::WaypointMission,
    safety: Option<&SafetyConfig>,
    cola: Option<&ColaConfig>,
    propagator: &rpo_core::propagation::propagator::PropagationModel,
) -> Result<(SafetyAnalysis, Vec<crate::validation::ColaBurn>), crate::validation::ValidationError>
{
    let analysis = compute_safety_analysis(wp_mission, safety, cola, propagator);
    let burns = convert_cola_to_burns(analysis.cola.as_deref(), wp_mission)?;
    Ok((analysis, burns))
}

/// Server-side full pipeline: computes Lambert transfer, then delegates to
/// [`execute_mission_from_transfer`].
///
/// Used by the CLI `mission` command and the API `PlanMission` handler.
/// For WASM clients that already hold a [`TransferResult`], call
/// [`execute_mission_from_transfer`] directly.
///
/// # Errors
///
/// Returns [`PipelineError`] if Lambert solving or any pipeline phase fails.
pub fn execute_mission(input: &PipelineInput) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    Ok(execute_mission_from_transfer(&mut transfer, input)?)
}

/// Server-side replan: computes Lambert transfer, then delegates to
/// [`replan_from_transfer`].
///
/// For WASM clients that already hold a [`TransferResult`], call
/// [`replan_from_transfer`] directly.
///
/// # Errors
///
/// Returns [`PipelineError`] if Lambert solving or replanning fails.
pub fn replan_mission(
    input: &PipelineInput,
    modified_index: usize,
    cached_mission: Option<rpo_core::mission::types::WaypointMission>,
) -> Result<PipelineOutput, PipelineError> {
    let mut transfer = compute_transfer(input)?;
    Ok(replan_from_transfer(&mut transfer, input, modified_index, cached_mission)?)
}

#[cfg(test)]
mod tests {
    use rpo_core::mission::config::ProximityConfig;
    use rpo_core::mission::types::PerchGeometry;
    use rpo_core::pipeline::{
        apply_perch_enrichment, compute_safety_analysis, plan_waypoints_from_transfer,
        suggest_enrichment, to_propagation_model, EnrichmentSuggestion, PipelineInput,
        PropagatorChoice, WaypointInput,
    };
    use rpo_core::mission::formation::{
        EiAlignment, PerchEnrichmentResult, PerchFallbackReason, SafetyRequirements,
    };
    use rpo_core::propagation::lambert::LambertConfig;

    use super::*;
    use rpo_core::test_helpers::iss_like_elements;

    /// Default V-bar perch at 5 km along-track.
    /// Inlined from `rpo_core::pipeline::types::default_perch` (pub(crate)).
    fn default_perch() -> PerchGeometry {
        PerchGeometry::VBar {
            along_track_km: 5.0,
        }
    }

    /// Tolerance for pure data-copy fidelity tests (no arithmetic, exact IEEE 754 roundtrip).
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;

    /// Minimum e/i magnitude expected after enrichment (dimensionless ROE).
    /// For a 100m separation at a = 7000 km, `d_min/a` ~ 1.4e-5. 1e-10 provides
    /// 5 orders of margin above floating-point noise.
    const ENRICHMENT_NONZERO_TOL: f64 = 1e-10;

    /// Exact-equality tolerance for ROE that should be bitwise identical.
    /// The enriched ROE is assigned directly (no arithmetic), so the only
    /// difference is f64 representation noise from serialization roundtrips.
    const ROE_IDENTITY_TOL: f64 = 1e-15;

    /// Upper bound on residual e/i phase at mid-transit after drift
    /// compensation reaches the pipeline output (rad). Looser than the
    /// `COMPENSATED_PHASE_TOL_RAD` used in transit.rs unit tests because
    /// the pipeline path adds leg-chain arithmetic (`period()`, STM
    /// composition). 1e-3 rad ~ 0.06 deg is conservative for a sub-orbit arc.
    const PIPELINE_DRIFT_PHASE_TOL_RAD: f64 = 1.0e-3;

    /// Upper bound on residual e/i phase at mid-transit when the enrichment
    /// is computed inside `execute_mission` (full pipeline round-trip, rad).
    /// 100x looser than [`PIPELINE_DRIFT_PHASE_TOL_RAD`] because the
    /// pipeline round-trip adds Newton-Raphson solver targeting noise
    /// (~1e-3 km position residual -> ~1e-2 rad phase at a ~ 7 000 km,
    /// de ~ 1e-5 scale) on top of the analytical drift compensation.
    /// 0.05 rad ~ 2.9 deg keeps enough headroom without masking a broken
    /// compensation (which would produce pi/2 ~ 1.57 rad).
    const PIPELINE_ENRICHMENT_PHASE_TOL_RAD: f64 = 0.05;

    /// Upper bound on |`delta_da`| introduced by e/i enrichment (dimensionless).
    /// The null-space projection (D'Amico Eq. 2.17) preserves position but
    /// da is a free DOF that shifts during e/i alignment -- typically
    /// O(1e-4) for proximity-regime geometries. 1e-3 (~7 km at a = 7 000 km)
    /// catches gross errors while allowing normal null-space adjustment.
    const ENRICHMENT_DA_DRIFT_TOL: f64 = 1e-3;

    /// Default time-of-flight for test waypoints (seconds). ~1.2 orbital
    /// periods for ISS-like orbits (T ~ 5 570 s), giving meaningful J2
    /// drift without multi-revolution ambiguity.
    const TEST_WAYPOINT_TOF_S: f64 = 4200.0;

    /// Lower bound on |delta(dv)| between baseline and enriched downstream legs
    /// (km/s). When enrichment modifies waypoint 0's arrival velocity, the
    /// solver must re-target leg 1 from a different initial state, producing
    /// a different dv. 1e-10 km/s is 5+ orders below typical maneuver scale
    /// (O(1e-3 km/s)) -- just above f64 noise -- so any physically meaningful
    /// cascade will exceed it.
    const CASCADE_DV_CHANGE_TOL_KM_S: f64 = 1e-10;

    /// Build a minimal `PipelineInput` from the standard far-field test scenario.
    fn far_field_input() -> PipelineInput {
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
            perch: default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![
                WaypointInput {
                    position_ric_km: [0.5, 2.0, 0.5],
                    velocity_ric_km_s: None,
                    tof_s: Some(TEST_WAYPOINT_TOF_S),
                    label: Some("WP1".into()),
                },
                WaypointInput {
                    position_ric_km: [0.5, 0.5, 0.5],
                    velocity_ric_km_s: Some([0.0, 0.001, 0.0]),
                    tof_s: Some(TEST_WAYPOINT_TOF_S),
                    label: Some("WP2".into()),
                },
            ],
            proximity: ProximityConfig::default(),
            config: rpo_core::mission::config::MissionConfig::default(),
            propagator: PropagatorChoice::J2,
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
        use rpo_core::elements::keplerian_conversions::keplerian_to_state;
        use hifitime::Epoch;

        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher -> Proximity regime
        deputy_ke.mean_anomaly_rad += 0.01;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        PipelineInput {
            chief,
            deputy,
            perch: default_perch(),
            lambert_tof_s: 3600.0,
            lambert_config: LambertConfig::default(),
            waypoints: vec![WaypointInput {
                position_ric_km: [0.5, 2.0, 0.5],
                velocity_ric_km_s: None,
                tof_s: Some(TEST_WAYPOINT_TOF_S),
                label: Some("WP1".into()),
            }],
            proximity: ProximityConfig::default(),
            config: rpo_core::mission::config::MissionConfig::default(),
            propagator: PropagatorChoice::J2,
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
    fn test_execute_mission_with_formation_design() {
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

        // Perch should be enriched (V-bar perch has zero e/i -> enrichment adds them)
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

        // Vectors indexed by leg -- same length as legs
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

    /// Formation report includes drift prediction for leg-1 coast arc.
    ///
    /// `proximity_input` has `tof_s` = 4200s (~0.78 orbits of ISS-like chief),
    /// well within the 10-orbit drift compensation regime, so a prediction
    /// must be present.
    #[test]
    fn formation_report_has_drift_prediction() {
        let mut input = proximity_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.10,
            alignment: EiAlignment::Parallel,
        });

        let output = execute_mission(&input).expect("execute_mission should succeed");
        let report = output.formation_design
            .as_ref()
            .expect("formation_design should be present");

        assert!(
            report.drift_prediction.is_some(),
            "drift prediction should be present for sub-orbit arc with non-critical inclination"
        );
        let pred = report.drift_prediction.as_ref().unwrap();
        assert!(
            pred.predicted_min_ei_km > 0.0,
            "predicted min e/i must be positive"
        );
        // After compensation + propagation by tof/2, the e/i phase at
        // mid-transit should be approximately zero (parallel aligned);
        // the residual is bounded by higher-order J2 terms.
        assert!(
            pred.predicted_phase_angle_rad.abs() < PIPELINE_DRIFT_PHASE_TOL_RAD,
            "compensated mid-transit phase should be near zero, got {:.4e} rad",
            pred.predicted_phase_angle_rad,
        );
    }

    // ---- suggest + apply tests ----

    fn proximity_input_with_enrichment() -> PipelineInput {
        let mut input = proximity_input();
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        });
        input
    }

    #[test]
    fn test_suggest_enrichment_does_not_mutate() {
        let input = proximity_input_with_enrichment();
        let transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input);

        assert!(suggestion.is_some(), "should produce suggestion when safety_requirements set");
        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
            "suggest_enrichment must not mutate transfer"
        );
    }

    #[test]
    fn test_suggest_enrichment_none_without_requirements() {
        let input = far_field_input();
        let transfer = compute_transfer(&input).expect("transfer");
        let suggestion = suggest_enrichment(&transfer, &input);
        assert!(suggestion.is_none());
    }

    #[test]
    fn test_apply_perch_enrichment_mutates() {
        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input).expect("suggestion");

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.dey - original_perch_roe.dey).abs()
                > ENRICHMENT_NONZERO_TOL,
            "apply should mutate perch_roe dey"
        );
    }

    #[test]
    fn test_apply_perch_enrichment_fallback_no_mutate() {
        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer(&input).expect("transfer");

        let original_perch_roe = transfer.plan.perch_roe;

        let suggestion = EnrichmentSuggestion {
            perch: PerchEnrichmentResult::Fallback {
                unenriched_roe: original_perch_roe,
                reason: PerchFallbackReason::SingularGeometry {
                    mean_arg_lat_rad: 0.0,
                },
            },
            requirements: SafetyRequirements {
                min_separation_km: 0.15,
                alignment: EiAlignment::Parallel,
            },
        };

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
            "fallback should not mutate perch_roe"
        );
    }

    /// Verify `compute_safety_analysis` produces identical results to the
    /// safety computation that was previously inlined in `build_output`.
    #[test]
    fn test_compute_safety_analysis_matches_build_output() {
        use rpo_core::mission::config::SafetyConfig;

        let mut input = far_field_input();
        input.config.safety = Some(SafetyConfig {
            min_ei_separation_km: 0.1,
            min_distance_3d_km: 0.05,
        });

        // execute_mission calls compute_safety_analysis internally and
        // unpacks it into PipelineOutput. Verify the fields match.
        let output = execute_mission(&input).expect("execute_mission should succeed");

        // Separately compute safety assessment from the same mission
        let transfer = compute_transfer(&input).expect("transfer");
        let propagator = to_propagation_model(&input.propagator);
        let mut transfer2 = transfer;
        let suggestion = suggest_enrichment(&transfer2, &input);
        if let Some(ref s) = suggestion {
            apply_perch_enrichment(&mut transfer2, s);
        }
        let wp_mission = plan_waypoints_from_transfer(&transfer2, &input, &propagator)
            .expect("waypoints");
        let safety = compute_safety_analysis(
            &wp_mission, input.config.safety.as_ref(), input.cola.as_ref(), &propagator,
        );

        // Free-drift: both should be Some when safety is enabled
        assert_eq!(
            output.safety.free_drift.is_some(),
            safety.free_drift.is_some(),
            "free_drift presence should match"
        );

        // POCA: both should be Some when safety is enabled
        assert_eq!(
            output.safety.poca.is_some(),
            safety.poca.is_some(),
            "poca presence should match"
        );

        // Free-drift POCA
        assert_eq!(
            output.safety.free_drift_poca.is_some(),
            safety.free_drift_poca.is_some(),
            "free_drift_poca presence should match"
        );

        // COLA maneuvers (may or may not be present depending on POCA distances)
        assert_eq!(
            output.safety.cola.is_some(),
            safety.cola.is_some(),
            "cola presence should match"
        );

        // If POCA data exists, check leg counts match
        if let (Some(out_poca), Some(sa_poca)) = (&output.safety.poca, &safety.poca) {
            assert_eq!(out_poca.len(), sa_poca.len(), "POCA leg count should match");
        }
    }

    #[test]
    fn accept_waypoint_enrichment_replans_with_safe_ei() {
        use rpo_core::mission::safety::compute_ei_separation;
        use rpo_core::mission::formation::safety_envelope::enrich_waypoint;
        use rpo_core::pipeline::accept_waypoint_enrichment;

        let input = proximity_input_with_enrichment();
        let baseline_output = execute_mission(&input).expect("baseline");

        // Compute an enrichment suggestion at waypoint 0 using the baseline's
        // arrival chief.
        let reqs = input.safety_requirements.expect("safety reqs set by fixture");
        let leg = &baseline_output.mission.legs[0];
        let enriched = enrich_waypoint(
            &leg.to_position_ric_km,
            None,
            &leg.arrival_chief_mean,
            &reqs,
        ).expect("enrichment");

        // Accept the enrichment -> replan
        let mut updated_input = input.clone();
        let mut transfer = compute_transfer(&updated_input).expect("transfer");
        let enriched_output = accept_waypoint_enrichment(
            &mut updated_input,
            &mut transfer,
            0,
            &enriched.roe,
            &leg.arrival_chief_mean,
        ).expect("accept enrichment");

        // Enriched arrival should have better e/i than baseline
        let enriched_leg = &enriched_output.mission.legs[0];
        let ei_enriched = compute_ei_separation(
            &enriched_leg.post_arrival_roe,
            &enriched_leg.arrival_chief_mean,
        );
        let ei_baseline = compute_ei_separation(
            &leg.post_arrival_roe,
            &leg.arrival_chief_mean,
        );
        assert!(
            ei_enriched.min_separation_km > ei_baseline.min_separation_km,
            "enriched e/i ({:.4} km) should exceed baseline ({:.4} km)",
            ei_enriched.min_separation_km, ei_baseline.min_separation_km,
        );

        // Waypoint velocity must now be set (accepted enrichment -> concrete target)
        assert!(
            updated_input.waypoints[0].velocity_ric_km_s.is_some(),
            "accepted enrichment should populate waypoint velocity",
        );

        // Enrichment is an e/i layer -- da should stay near-zero
        let delta_da = enriched.roe.da - leg.post_arrival_roe.da;
        assert!(
            delta_da.abs() < ENRICHMENT_DA_DRIFT_TOL,
            "enrichment should not inflate da: baseline={:.3e}, enriched={:.3e}, \
             delta={:.3e} (limit={:.0e})",
            leg.post_arrival_roe.da, enriched.roe.da, delta_da, ENRICHMENT_DA_DRIFT_TOL,
        );
    }

    #[test]
    fn accept_waypoint_enrichment_rejects_out_of_bounds_index() {
        use rpo_core::mission::errors::MissionError;
        use rpo_core::mission::formation::safety_envelope::enrich_waypoint;
        use rpo_core::pipeline::accept_waypoint_enrichment;

        let input = proximity_input_with_enrichment();
        let baseline_output = execute_mission(&input).expect("baseline");
        let leg = &baseline_output.mission.legs[0];
        let reqs = input.safety_requirements.expect("safety reqs set by fixture");
        let enriched = enrich_waypoint(
            &leg.to_position_ric_km, None, &leg.arrival_chief_mean, &reqs,
        ).expect("enrichment");

        let mut updated_input = input.clone();
        let mut transfer = compute_transfer(&updated_input).expect("transfer");
        let result = accept_waypoint_enrichment(
            &mut updated_input, &mut transfer, 99, &enriched.roe, &leg.arrival_chief_mean,
        );
        assert!(matches!(
            result,
            Err(rpo_core::pipeline::PipelineError::Mission(MissionError::InvalidReplanIndex { index: 99, .. }))
        ));
    }

    #[test]
    fn formation_report_enrichment_uses_drift_compensation() {
        use rpo_core::mission::formation::types::EnrichmentMode;

        let input = proximity_input_with_enrichment();
        let output = execute_mission(&input).expect("pipeline");

        let report = output.formation_design.expect("report should exist");

        // Diagnostic drift prediction on first leg departure (unchanged behavior).
        if let Some(ref drift) = report.drift_prediction {
            assert!(drift.predicted_min_ei_km > 0.0);
        }

        // Position-only waypoint enrichments must be tagged as PositionOnly mode.
        for enriched in report.waypoints.iter().flatten() {
            assert_eq!(enriched.mode, EnrichmentMode::PositionOnly);
        }
    }

    #[test]
    fn waypoint_enrichment_aligns_at_next_leg_midpoint() {
        use rpo_core::mission::safety::compute_ei_separation;
        use rpo_core::propagation::stm::propagate_roe_stm;

        // Requires a mission with 2+ waypoints so waypoint 0 has a following leg.
        let mut input = proximity_input_with_enrichment();
        input.waypoints.push(WaypointInput {
            position_ric_km: [0.3, 1.5, 0.2],
            velocity_ric_km_s: None,
            tof_s: Some(TEST_WAYPOINT_TOF_S),
            label: Some("WP2".into()),
        });

        let output = execute_mission(&input).expect("pipeline");
        let report = output.formation_design.expect("report");
        let enriched_wp0 = report.waypoints[0].as_ref().expect("enrichment for wp0");

        // Propagate the drift-compensated ROE forward by HALF of the NEXT leg's TOF.
        let next_leg = &output.mission.legs[1];
        let (mid_roe, mid_chief) = propagate_roe_stm(
            &enriched_wp0.roe,
            &output.mission.legs[0].arrival_chief_mean,
            next_leg.tof_s / 2.0,
        ).expect("propagation");
        let ei_mid = compute_ei_separation(&mid_roe, &mid_chief);

        assert!(
            ei_mid.phase_angle_rad.abs() < PIPELINE_ENRICHMENT_PHASE_TOL_RAD,
            "e/i phase at mid-transit of next leg should be ~0 rad (parallel aligned), got {:.4} rad",
            ei_mid.phase_angle_rad,
        );
        assert!(
            ei_mid.min_separation_km >= enriched_wp0.baseline_ei.min_separation_km,
            "drift-compensated mid-transit separation ({:.4} km) should not regress \
             below uncompensated baseline ({:.4} km)",
            ei_mid.min_separation_km, enriched_wp0.baseline_ei.min_separation_km,
        );
    }

    #[test]
    fn full_enrichment_cycle_suggest_accept_verify() {
        use rpo_core::mission::safety::compute_ei_separation;
        use rpo_core::pipeline::accept_waypoint_enrichment;

        // 1. Plan baseline mission with TWO position-only waypoints (cascade test).
        let mut input = proximity_input_with_enrichment();
        input.waypoints.push(WaypointInput {
            position_ric_km: [0.3, 1.5, 0.2],
            velocity_ric_km_s: None,
            tof_s: Some(TEST_WAYPOINT_TOF_S),
            label: Some("WP2".into()),
        });

        let baseline = execute_mission(&input).expect("baseline");
        let report = baseline.formation_design.as_ref().expect("report");

        // 2. Formation report should have position-only enrichment suggestions.
        let suggestion_0 = report.waypoints[0].as_ref().expect("suggestion for wp 0");
        assert!(
            suggestion_0.enriched_ei.min_separation_km > suggestion_0.baseline_ei.min_separation_km,
            "enriched should be safer than baseline"
        );

        // 3. Accept enrichment at waypoint 0.
        let leg_0 = &baseline.mission.legs[0];
        let mut transfer = compute_transfer(&input).expect("transfer");
        let enriched = accept_waypoint_enrichment(
            &mut input,
            &mut transfer,
            0,
            &suggestion_0.roe,
            &leg_0.arrival_chief_mean,
        ).expect("accept");

        // 4. Verify: arrival at waypoint 0 should be at least as safe as baseline.
        let enriched_leg_0 = &enriched.mission.legs[0];
        let ei_enriched = compute_ei_separation(
            &enriched_leg_0.post_arrival_roe,
            &enriched_leg_0.arrival_chief_mean,
        );
        let ei_baseline = compute_ei_separation(
            &leg_0.post_arrival_roe,
            &leg_0.arrival_chief_mean,
        );
        assert!(
            ei_enriched.min_separation_km >= ei_baseline.min_separation_km,
            "enriched arrival e/i ({:.4} km) must not regress below baseline ({:.4} km)",
            ei_enriched.min_separation_km, ei_baseline.min_separation_km,
        );

        // 5. Waypoint 0's velocity should now be set (no longer position-only).
        assert!(
            input.waypoints[0].velocity_ric_km_s.is_some(),
            "accepted enrichment should set waypoint velocity"
        );

        // 6. Downstream leg 1 should cascade (waypoint 0's departure state changed).
        let baseline_leg_1 = &baseline.mission.legs[1];
        let enriched_leg_1 = &enriched.mission.legs[1];
        assert!(
            (enriched_leg_1.total_dv_km_s - baseline_leg_1.total_dv_km_s).abs()
                > CASCADE_DV_CHANGE_TOL_KM_S,
            "downstream leg should differ after enrichment acceptance"
        );
    }
}

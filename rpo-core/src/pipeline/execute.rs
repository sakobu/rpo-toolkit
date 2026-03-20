//! Pipeline execution: `compute_transfer`, `execute_mission`, `replan_mission`, `build_output`.
//!
//! These compose the core planning primitives (classify, Lambert, waypoint
//! targeting, covariance, eclipse) into a complete mission pipeline.

use crate::constants::DEFAULT_COVARIANCE_SAMPLES_PER_LEG;
use crate::mission::planning::{compute_transfer_eclipse, plan_mission};
use crate::mission::waypoints::{plan_waypoint_mission, replan_from_waypoint};
use crate::propagation::covariance::ric_accuracy_to_roe_covariance;
use crate::propagation::keplerian::propagate_keplerian;
use crate::propagation::propagator::PropagationModel;
use crate::types::{DepartureState, StateVector};

use super::convert::{to_propagation_model, to_waypoints};
use super::errors::PipelineError;
use super::types::{PipelineInput, PipelineOutput, TransferResult};

/// Number of eclipse evaluation points along the Lambert transfer arc.
const DEFAULT_TRANSFER_ECLIPSE_SAMPLES: u32 = 200;

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
    let arrival_epoch =
        input.chief.epoch + hifitime::Duration::from_seconds(input.lambert_tof_s);

    let (perch_chief, perch_deputy) = if let Some(ref transfer) = plan.transfer {
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

        (chief_at_arrival, deputy_at_perch)
    } else {
        (input.chief.clone(), input.deputy.clone())
    };

    Ok(TransferResult {
        plan,
        perch_chief,
        perch_deputy,
        arrival_epoch,
        lambert_dv_km_s,
    })
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
#[must_use]
pub fn build_output(
    transfer: &TransferResult,
    wp_mission: crate::mission::types::WaypointMission,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: Option<crate::propagation::propagator::DragConfig>,
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
        let initial_p =
            ric_accuracy_to_roe_covariance(nav, &transfer.plan.chief_at_arrival).ok()?;
        crate::mission::covariance::propagate_mission_covariance(
            &wp_mission,
            &initial_p,
            nav,
            input.maneuver_uncertainty.as_ref(),
            propagator,
            DEFAULT_COVARIANCE_SAMPLES_PER_LEG,
        )
        .ok()
    });

    PipelineOutput {
        phase: transfer.plan.phase.clone(),
        transfer: transfer.plan.transfer.clone(),
        transfer_eclipse,
        mission: wp_mission,
        total_dv_km_s,
        total_duration_s,
        auto_drag_config: auto_drag,
        covariance,
        monte_carlo: None,
    }
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
/// converts the pipeline waypoint inputs to core [`Waypoint`]s, and runs
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

/// Execute the full mission pipeline: classify → Lambert → waypoints → covariance → eclipse.
///
/// Single entry point for the CLI `mission` command and the API `PlanMission` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn execute_mission(input: &PipelineInput) -> Result<PipelineOutput, PipelineError> {
    let transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);
    let wp_mission = plan_waypoints_from_transfer(&transfer, input, &propagator)?;

    Ok(build_output(&transfer, wp_mission, input, &propagator, None))
}

/// Incremental replan pipeline: classify → Lambert → full plan → replan from index.
///
/// Plans the full waypoint mission first (needed as base for replan),
/// then calls `replan_from_waypoint()` from the modified index.
/// Used by the API's `MoveWaypoint` handler.
///
/// # Errors
///
/// Returns [`PipelineError`] if any pipeline phase fails.
pub fn replan_mission(
    input: &PipelineInput,
    modified_index: usize,
) -> Result<PipelineOutput, PipelineError> {
    let transfer = compute_transfer(input)?;
    let propagator = to_propagation_model(&input.propagator);
    let waypoints = to_waypoints(&input.waypoints);

    let departure = departure_from_transfer(&transfer);

    // First plan the full mission to get the existing legs
    let full_mission =
        plan_waypoint_mission(&departure, &waypoints, &input.config, &propagator)?;

    // Then replan from the modified index
    let wp_mission = replan_from_waypoint(
        &full_mission,
        modified_index,
        &waypoints,
        &departure,
        &input.config,
        &propagator,
    )?;

    Ok(build_output(&transfer, wp_mission, input, &propagator, None))
}

#[cfg(test)]
mod tests {
    use crate::mission::config::ProximityConfig;
    use crate::propagation::lambert::LambertConfig;

    use super::*;

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
        let output = replan_mission(&input, 0).expect("replan_mission should succeed");

        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
    }

    /// Tolerance for pure data-copy fidelity tests (no arithmetic, exact IEEE 754 roundtrip).
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;

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
}

//! Smoke tests for rpo-wasm WASM exports.
//!
//! Tests are split into two groups:
//! - **Native tests** (`#[test]`): error mapping and serde roundtrips that don't
//!   touch `JsValue`. Run via `cargo test -p rpo-wasm`.
//! - **WASM tests** (`#[wasm_bindgen_test]`): end-to-end boundary tests that
//!   exercise `serde_wasm_bindgen` serialization. Run via `wasm-pack test --node`.
//!
//! Algorithm-level tests live in rpo-core; these focus on the WASM boundary.

use hifitime::Epoch;
use nalgebra::Vector3;
use rpo_core::elements::keplerian_to_state;
use rpo_core::mission::config::{MissionConfig, ProximityConfig};
use rpo_core::mission::formation::EiAlignment;
use rpo_core::mission::types::{MissionPhase, MissionPlan, PerchGeometry, Waypoint, WaypointMission};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::pipeline::types::{PipelineInput, PropagatorChoice, TransferResult, WaypointInput};
use rpo_core::propagation::PropagationModel;
use rpo_core::test_helpers::{iss_like_elements, test_epoch};
use rpo_core::types::{DepartureState, KeplerianElements, QuasiNonsingularROE, StateVector};

// ---------------------------------------------------------------------------
// Test constants — named per CLAUDE.md Tolerance Policy
// ---------------------------------------------------------------------------

/// SMA offset to place the deputy in the proximity regime.
const TEST_SMA_OFFSET_KM: f64 = 1.0;

/// SMA offset for synthetic Lambert transfer in eclipse test.
const TEST_LAMBERT_SMA_OFFSET_KM: f64 = 10.0;

/// Proximity-regime separation (matches classification output for 1 km SMA offset).
const TEST_PROXIMITY_SEPARATION_KM: f64 = 1.0;

/// Proximity-regime `delta_r/r` (matches classification output for 1 km SMA offset).
const TEST_PROXIMITY_DELTA_R_OVER_R: f64 = 0.0001;

/// Safety config: minimum e/i vector separation.
const TEST_MIN_EI_SEPARATION_KM: f64 = 0.2;

/// Safety config: minimum 3D distance.
const TEST_MIN_DISTANCE_3D_KM: f64 = 0.1;

/// Formation safety: minimum separation threshold.
const TEST_MIN_SEPARATION_KM: f64 = 0.15;

/// COLA config: target avoidance distance.
const TEST_COLA_TARGET_DIST_KM: f64 = 0.2;

/// COLA config: maximum delta-v budget.
const TEST_COLA_MAX_DV_KM_S: f64 = 0.01;

/// COLA config: maximum delta-v for avoidance-nominal test (larger budget).
const TEST_COLA_MAX_DV_LARGE_KM_S: f64 = 1.0;

/// Multiplier on POCA distance to set COLA target above actual, ensuring solver has work.
const COLA_TARGET_MARGIN_FACTOR: f64 = 2.0;

/// Elapsed time known to exceed any test mission duration.
const BEYOND_ANY_MISSION_DURATION_S: f64 = 1e9;

/// Number of resampling steps for trajectory tests.
const TEST_RESAMPLE_STEPS: usize = 50;

/// Number of arc densification steps for eclipse tests.
const TEST_ARC_STEPS: u32 = 20;

/// V-bar perch along-track distance for test scenarios.
const TEST_PERCH_ALONG_TRACK_KM: f64 = 5.0;

/// Lambert time-of-flight for test scenarios.
const TEST_LAMBERT_TOF_S: f64 = 3600.0;

// ---------------------------------------------------------------------------
// Shared test fixtures
// ---------------------------------------------------------------------------

/// Small nonzero ROE for structural/boundary tests (not paper-traced).
/// Same values used across native and WASM-target test suites.
fn test_roe() -> QuasiNonsingularROE {
    QuasiNonsingularROE {
        da: 0.0,
        dlambda: 0.001,
        dex: 0.0001,
        dey: -0.0001,
        dix: 0.0001,
        diy: -0.0001,
    }
}

/// ISS-like proximity pair: deputy [`TEST_SMA_OFFSET_KM`] higher than chief.
fn proximity_pair() -> (KeplerianElements, KeplerianElements, StateVector, StateVector, Epoch) {
    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let mut deputy_ke = chief_ke;
    deputy_ke.a_km += TEST_SMA_OFFSET_KM;
    let chief = keplerian_to_state(&chief_ke, epoch).expect("chief state");
    let deputy = keplerian_to_state(&deputy_ke, epoch).expect("deputy state");
    (chief_ke, deputy_ke, chief, deputy, epoch)
}

/// Single in-track waypoint at \[0, 0.5, 0\] km — shared between native and WASM tests.
fn test_waypoint_input() -> WaypointInput {
    WaypointInput {
        position_ric_km: [0.0, 0.5, 0.0],
        velocity_ric_km_s: None,
        tof_s: None,
        label: None,
    }
}

/// `DepartureState` built from `iss_like_elements()` + `test_epoch()` + `test_roe()`.
fn test_departure() -> DepartureState {
    DepartureState {
        roe: test_roe(),
        chief: iss_like_elements(),
        epoch: test_epoch(),
    }
}

/// Plan a single-waypoint mission at \[0, 0.5, 0\] km in-track.
/// Calls rpo-core directly (not the WASM wrapper).
fn test_mission() -> WaypointMission {
    let departure = test_departure();
    let waypoints = vec![Waypoint {
        position_ric_km: Vector3::new(0.0, 0.5, 0.0),
        velocity_ric_km_s: None,
        tof_s: None,
    }];
    rpo_core::mission::waypoints::plan_waypoint_mission(
        &departure,
        &waypoints,
        &MissionConfig::default(),
        &PropagationModel::J2Stm,
    )
    .expect("test fixture: plan_waypoint_mission must succeed")
}

/// Proximity `TransferResult` — no Lambert, ISS-like orbit, same ROE as `test_departure`.
fn test_transfer_result() -> TransferResult {
    let (chief_ke, deputy_ke, chief, deputy, epoch) = proximity_pair();
    let roe = test_roe();

    TransferResult {
        plan: MissionPlan {
            phase: MissionPhase::Proximity {
                roe,
                chief_elements: chief_ke,
                deputy_elements: deputy_ke,
                separation_km: TEST_PROXIMITY_SEPARATION_KM,
                delta_r_over_r: TEST_PROXIMITY_DELTA_R_OVER_R,
            },
            transfer: None,
            perch_roe: roe,
            chief_at_arrival: chief_ke,
        },
        perch_chief: chief.clone(),
        perch_deputy: deputy.clone(),
        arrival_epoch: epoch,
        lambert_dv_km_s: 0.0,
    }
}

/// `PipelineInput` matching `test_transfer_result` — single waypoint at \[0, 0.5, 0\] km.
fn test_pipeline_input() -> PipelineInput {
    let (_chief_ke, _deputy_ke, chief, deputy, _epoch) = proximity_pair();

    PipelineInput {
        chief,
        deputy,
        waypoints: vec![test_waypoint_input()],
        config: MissionConfig::default(),
        propagator: PropagatorChoice::J2,
        perch: PerchGeometry::VBar { along_track_km: TEST_PERCH_ALONG_TRACK_KM },
        lambert_tof_s: TEST_LAMBERT_TOF_S,
        lambert_config: LambertConfig::default(),
        proximity: ProximityConfig::default(),
        chief_config: None,
        deputy_config: None,
        navigation_accuracy: None,
        maneuver_uncertainty: None,
        monte_carlo: None,
        cola: None,
        safety_requirements: None,
    }
}

// ---------------------------------------------------------------------------
// Native tests: error mapping + serde roundtrips
// ---------------------------------------------------------------------------

#[test]
fn pipeline_error_maps_to_correct_code() {
    use rpo_core::pipeline::PipelineError;
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = PipelineError::MissingField {
        field: "chief",
        context: "classification",
    };
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::MissingField));

    let err = PipelineError::EmptyTrajectory;
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::EmptyTrajectory));
}

#[test]
fn mission_error_maps_to_mission_code() {
    use rpo_core::mission::MissionError;
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = MissionError::EmptyWaypoints;
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::Mission));
}

#[test]
fn propagation_error_maps_to_propagation_code() {
    use rpo_core::propagation::PropagationError;
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = PropagationError::ZeroSteps;
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::Propagation));
}

#[test]
fn covariance_error_maps_to_covariance_code() {
    use rpo_core::propagation::CovarianceError;
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = CovarianceError::SingularTMatrix;
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::Covariance));
}

#[test]
fn eclipse_error_maps_to_eclipse_code() {
    use rpo_core::mission::EclipseComputeError;
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = EclipseComputeError::EmptyTrajectory;
    let wasm_err = WasmError::from(err);
    assert!(matches!(wasm_err.code, WasmErrorCode::Eclipse));
}

#[test]
fn wasm_error_serde_roundtrip() {
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = WasmError {
        code: WasmErrorCode::Mission,
        message: "test error".to_string(),
        details: Some("inner cause".to_string()),
    };
    let json = serde_json::to_string(&err).expect("serialize WasmError");
    let roundtrip: WasmError = serde_json::from_str(&json).expect("deserialize WasmError");
    assert!(matches!(roundtrip.code, WasmErrorCode::Mission));
    assert_eq!(roundtrip.message, "test error");
    assert_eq!(roundtrip.details.as_deref(), Some("inner cause"));
}

#[test]
fn wasm_error_without_details_omits_field() {
    use rpo_wasm::error::{WasmError, WasmErrorCode};

    let err = WasmError {
        code: WasmErrorCode::Internal,
        message: "something broke".to_string(),
        details: None,
    };
    let json = serde_json::to_string(&err).expect("serialize");
    assert!(
        !json.contains(r#""details""#),
        "None details should omit the key: {json}"
    );
}

#[test]
fn wasm_error_code_all_variants_roundtrip() {
    use rpo_wasm::error::WasmErrorCode;

    let variants = [
        WasmErrorCode::Mission,
        WasmErrorCode::Propagation,
        WasmErrorCode::Covariance,
        WasmErrorCode::Avoidance,
        WasmErrorCode::MissingField,
        WasmErrorCode::EmptyTrajectory,
        WasmErrorCode::Lambert,
        WasmErrorCode::Eclipse,
        WasmErrorCode::Formation,
        WasmErrorCode::Deserialization,
        WasmErrorCode::Internal,
    ];
    for variant in variants {
        let json = serde_json::to_string(&variant).expect("serialize");
        let roundtrip: WasmErrorCode = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(
            format!("{roundtrip:?}"),
            format!("{variant:?}"),
            "roundtrip failed for {variant:?}"
        );
    }
}

#[test]
fn classify_separation_proximity() {
    let (_chief_ke, _deputy_ke, chief, deputy, _epoch) = proximity_pair();

    let config = rpo_core::mission::config::ProximityConfig::default();
    let result = rpo_wasm::planning::classify_separation(chief, deputy, config);
    assert!(result.is_ok(), "classify_separation should succeed for proximity pair");
}

// ---------------------------------------------------------------------------
// mission.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn execute_mission_from_transfer_nominal() {
    let transfer = test_transfer_result();
    let input = test_pipeline_input();
    let result = rpo_wasm::mission::execute_mission_from_transfer(transfer, input);
    assert!(result.is_ok(), "execute_mission_from_transfer should succeed: {result:?}");
    let mission_result = result.unwrap();
    assert!(
        !mission_result.output.mission.legs.is_empty(),
        "mission should have at least one leg"
    );
}

#[test]
fn execute_mission_from_transfer_empty_waypoints() {
    let transfer = test_transfer_result();
    let mut input = test_pipeline_input();
    input.waypoints.clear();
    let result = rpo_wasm::mission::execute_mission_from_transfer(transfer, input);
    assert!(result.is_err(), "should fail with empty waypoints");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Mission),
        "expected Mission error code, got {:?}",
        err.code
    );
}

#[test]
fn replan_from_transfer_nominal() {
    let transfer = test_transfer_result();
    let input = test_pipeline_input();
    let result = rpo_wasm::mission::replan_from_transfer(
        transfer,
        input,
        0,
        None,
    );
    assert!(result.is_ok(), "replan_from_transfer should succeed: {result:?}");
}

#[test]
fn replan_from_transfer_index_out_of_bounds() {
    let transfer = test_transfer_result();
    let input = test_pipeline_input();
    let result = rpo_wasm::mission::replan_from_transfer(
        transfer,
        input,
        999,
        None,
    );
    assert!(result.is_err(), "should fail with out-of-bounds index");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Mission),
        "expected Mission error code, got {:?}",
        err.code
    );
}

// ---------------------------------------------------------------------------
// analysis.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn compute_mission_covariance_nominal() {
    use rpo_core::propagation::covariance::types::NavigationAccuracy;

    let mission = test_mission();
    let chief_at_arrival = iss_like_elements();
    let nav = NavigationAccuracy::default();
    let result = rpo_wasm::analysis::compute_mission_covariance(
        mission,
        chief_at_arrival,
        nav,
        None,
        PropagatorChoice::J2,
    );
    assert!(result.is_ok(), "compute_mission_covariance should succeed: {result:?}");
}

#[test]
fn compute_mission_covariance_invalid_chief() {
    use rpo_core::propagation::covariance::types::NavigationAccuracy;
    use rpo_core::types::KeplerianElements;

    let mission = test_mission();
    // Zero semi-major axis → invalid chief → CovarianceError
    let bad_chief = KeplerianElements {
        a_km: 0.0,
        ..iss_like_elements()
    };
    let nav = NavigationAccuracy::default();
    let result = rpo_wasm::analysis::compute_mission_covariance(
        mission,
        bad_chief,
        nav,
        None,
        PropagatorChoice::J2,
    );
    assert!(result.is_err(), "should fail with invalid chief elements");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Covariance),
        "expected Covariance error code, got {:?}",
        err.code
    );
}

#[test]
fn compute_free_drift_analysis_nominal() {
    let mission = test_mission();
    let result = rpo_wasm::analysis::compute_free_drift_analysis(
        mission,
        PropagatorChoice::J2,
    );
    assert!(result.is_some(), "free-drift analysis should succeed");
    let fd = result.unwrap();
    assert!(
        !fd.analyses.is_empty(),
        "should have at least one per-leg analysis"
    );
}

#[test]
fn compute_free_drift_analysis_empty_legs() {
    // A mission with no legs returns Some(empty), not None.
    let mut mission = test_mission();
    mission.legs.clear();
    let result = rpo_wasm::analysis::compute_free_drift_analysis(
        mission,
        PropagatorChoice::J2,
    );
    assert!(result.is_some(), "empty legs should return Some, not None");
    assert!(result.unwrap().analyses.is_empty(), "analyses should be empty");
}

#[test]
fn compute_poca_analysis_nominal() {
    let mission = test_mission();
    let result = rpo_wasm::analysis::compute_poca_analysis(
        mission,
        PropagatorChoice::J2,
    );
    assert!(result.is_some(), "POCA analysis should succeed");
    let poca = result.unwrap();
    assert!(!poca.legs.is_empty(), "should have at least one leg of POCAs");
}

#[test]
fn compute_poca_analysis_empty_legs() {
    let mut mission = test_mission();
    mission.legs.clear();
    let result = rpo_wasm::analysis::compute_poca_analysis(
        mission,
        PropagatorChoice::J2,
    );
    assert!(result.is_some(), "empty legs should return Some, not None");
    assert!(result.unwrap().legs.is_empty(), "legs should be empty");
}

#[test]
fn resample_leg_trajectory_nominal() {
    let mission = test_mission();
    let leg = mission.legs[0].clone();
    let result = rpo_wasm::analysis::resample_leg_trajectory(
        leg,
        TEST_RESAMPLE_STEPS,
        PropagatorChoice::J2,
    );
    assert!(result.is_ok(), "resample should succeed: {result:?}");
    let resampled = result.unwrap();
    assert_eq!(resampled.states.len(), TEST_RESAMPLE_STEPS + 1, "expected n_steps + 1 resampled states (including endpoints)");
}

#[test]
fn resample_leg_trajectory_zero_steps() {
    let mission = test_mission();
    let leg = mission.legs[0].clone();
    let result = rpo_wasm::analysis::resample_leg_trajectory(
        leg,
        0,
        PropagatorChoice::J2,
    );
    assert!(result.is_err(), "should fail with zero steps");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Propagation),
        "expected Propagation error code, got {:?}",
        err.code
    );
}

// ---------------------------------------------------------------------------
// eclipse.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn compute_mission_eclipse_nominal() {
    let mission = test_mission();
    let result = rpo_wasm::eclipse::compute_mission_eclipse(mission);
    assert!(result.is_ok(), "compute_mission_eclipse should succeed: {result:?}");
}

#[test]
fn compute_transfer_eclipse_nominal() {
    use rpo_core::propagation::lambert::{LambertTransfer, TransferDirection};

    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let chief_state = keplerian_to_state(&chief_ke, epoch).expect("chief state");

    // Construct a synthetic LambertTransfer with the ISS-like departure.
    // densify_arc only uses departure_state + tof_s, so other fields
    // are set to physically plausible but arbitrary values.
    let mut deputy_ke = chief_ke;
    deputy_ke.a_km += TEST_LAMBERT_SMA_OFFSET_KM;
    let arrival_epoch = epoch + hifitime::Duration::from_seconds(TEST_LAMBERT_TOF_S);
    let arrival_state = keplerian_to_state(&deputy_ke, arrival_epoch).expect("arrival state");
    let transfer = LambertTransfer {
        departure_state: chief_state.clone(),
        arrival_state,
        departure_dv_eci_km_s: Vector3::new(0.001, 0.0, 0.0),
        arrival_dv_eci_km_s: Vector3::new(-0.001, 0.0, 0.0),
        total_dv_km_s: 0.002,
        tof_s: TEST_LAMBERT_TOF_S,
        c3_km2_s2: -30.0,
        direction: TransferDirection::Auto,
    };

    let result = rpo_wasm::eclipse::compute_transfer_eclipse(transfer, chief_state, TEST_ARC_STEPS);
    assert!(result.is_ok(), "compute_transfer_eclipse should succeed: {result:?}");
}

// ---------------------------------------------------------------------------
// query.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn get_mission_state_at_time_nominal() {
    let mission = test_mission();
    let result = rpo_wasm::query::get_mission_state_at_time(
        mission,
        100.0, // 100 seconds into the mission
        PropagatorChoice::J2,
    );
    assert!(result.is_ok(), "get_mission_state_at_time should succeed: {result:?}");
    assert!(result.unwrap().is_some(), "100s should be within mission duration");
}

#[test]
fn get_mission_state_at_time_beyond_mission() {
    let mission = test_mission();
    let result = rpo_wasm::query::get_mission_state_at_time(
        mission,
        BEYOND_ANY_MISSION_DURATION_S,
        PropagatorChoice::J2,
    );
    assert!(result.is_ok(), "should succeed but return None");
    assert!(result.unwrap().is_none(), "time beyond mission should return None");
}

// ---------------------------------------------------------------------------
// safety.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn compute_safety_analysis_with_config() {
    use rpo_core::mission::config::SafetyConfig;

    let mission = test_mission();
    let safety = SafetyConfig {
        min_ei_separation_km: TEST_MIN_EI_SEPARATION_KM,
        min_distance_3d_km: TEST_MIN_DISTANCE_3D_KM,
    };
    let result = rpo_wasm::safety::compute_safety_analysis(
        mission,
        Some(safety),
        None,
        PropagatorChoice::J2,
    );
    // SafetyAnalysis is always returned (no Result), just verify no panic.
    // With safety config, free-drift and POCA should be populated.
    assert!(
        result.free_drift.is_some() || result.poca.is_some(),
        "safety analysis with config should produce free-drift or POCA data"
    );
}

#[test]
fn compute_safety_analysis_without_config() {
    let mission = test_mission();
    let result = rpo_wasm::safety::compute_safety_analysis(
        mission,
        None,
        None,
        PropagatorChoice::J2,
    );
    // Should return without panic even with no safety config.
    let _ = format!("{result:?}");
}

/// Tests `assess_cola` via rpo-core directly because the WASM wrapper
/// takes `JsValue` for the `poca` parameter (cannot construct in native test).
#[test]
fn assess_cola_nominal() {
    use rpo_core::mission::avoidance::ColaConfig;
    use rpo_core::mission::cola_assessment::assess_cola;

    let mission = test_mission();
    // Get real POCA data from the test mission.
    let poca_result = rpo_wasm::analysis::compute_poca_analysis(
        mission.clone(),
        PropagatorChoice::J2,
    );
    assert!(poca_result.is_some(), "POCA analysis should succeed");
    let poca_data = poca_result.unwrap();

    let model = PropagationModel::J2Stm;
    let config = ColaConfig {
        target_distance_km: TEST_COLA_TARGET_DIST_KM,
        max_dv_km_s: TEST_COLA_MAX_DV_KM_S,
    };

    // assess_cola returns ColaAssessment (not Result), always succeeds.
    let result = assess_cola(&mission, &poca_data.legs, &model, &config);
    let _ = format!("{result:?}");
}

#[test]
fn compute_avoidance_nominal() {
    use rpo_core::mission::avoidance::ColaConfig;

    let mission = test_mission();
    let poca_result = rpo_wasm::analysis::compute_poca_analysis(
        mission.clone(),
        PropagatorChoice::J2,
    );
    assert!(poca_result.is_some(), "need POCA data for avoidance test");
    let poca_data = poca_result.unwrap();

    // Find the first POCA across all legs.
    let first_poca = poca_data.legs.iter()
        .flatten()
        .next()
        .expect("should have at least one POCA");

    let leg = &mission.legs[0];
    let config = ColaConfig {
        target_distance_km: first_poca.distance_km * COLA_TARGET_MARGIN_FACTOR,
        max_dv_km_s: TEST_COLA_MAX_DV_LARGE_KM_S,
    };
    let epoch_str = "2024-01-01T00:00:00 UTC".to_string();

    let result = rpo_wasm::safety::compute_avoidance(
        first_poca.clone(),
        leg.post_departure_roe,
        leg.departure_chief_mean,
        epoch_str,
        leg.tof_s,
        PropagatorChoice::J2,
        config,
    );
    assert!(result.is_ok(), "compute_avoidance should succeed: {result:?}");
}

#[test]
fn compute_avoidance_bad_epoch() {
    use rpo_core::mission::avoidance::ColaConfig;
    use rpo_core::mission::closest_approach::ClosestApproach;

    let mission = test_mission();
    let leg = &mission.legs[0];
    // Construct a minimal ClosestApproach — fields don't matter because
    // epoch parsing fails before the avoidance solver runs.
    let poca = ClosestApproach {
        epoch: test_epoch(),
        elapsed_s: 100.0,
        distance_km: 0.05,
        position_ric_km: Vector3::new(0.01, 0.03, 0.01),
        velocity_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
        leg_index: 0,
        is_global_minimum: true,
    };
    let config = ColaConfig {
        target_distance_km: TEST_COLA_TARGET_DIST_KM,
        max_dv_km_s: TEST_COLA_MAX_DV_KM_S,
    };

    let result = rpo_wasm::safety::compute_avoidance(
        poca,
        leg.post_departure_roe,
        leg.departure_chief_mean,
        "not-a-valid-epoch".to_string(),
        leg.tof_s,
        PropagatorChoice::J2,
        config,
    );
    assert!(result.is_err(), "should fail with invalid epoch");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Deserialization),
        "expected Deserialization error code, got {:?}",
        err.code
    );
}

// ---------------------------------------------------------------------------
// enrichment.rs boundary tests
// ---------------------------------------------------------------------------

#[test]
fn apply_perch_enrichment_nominal() {
    use rpo_core::mission::formation::SafetyRequirements;

    let transfer = test_transfer_result();
    let mut input = test_pipeline_input();
    input.safety_requirements = Some(SafetyRequirements {
        min_separation_km: TEST_MIN_SEPARATION_KM,
        alignment: EiAlignment::default(),
    });

    // Get a suggestion first (needs safety_requirements).
    let suggestion = rpo_wasm::enrichment::suggest_enrichment(
        transfer.clone(),
        input,
    );
    assert!(suggestion.is_some(), "should produce enrichment suggestion with safety_requirements");
    let suggestion = suggestion.unwrap();

    // Apply it — returns modified transfer (no error path).
    let enriched = rpo_wasm::enrichment::apply_perch_enrichment(transfer, suggestion);
    // The enriched transfer may or may not differ from original depending
    // on whether the suggestion found a better perch. Just verify no panic.
    let _ = format!("{:?}", enriched.plan.perch_roe);
}

#[test]
fn accept_waypoint_enrichment_nominal() {
    let input = test_pipeline_input();
    let transfer = test_transfer_result();
    let enriched_roe = QuasiNonsingularROE {
        da: 0.0,
        dlambda: 0.001,
        dex: 0.0002,
        dey: -0.0002,
        dix: 0.0002,
        diy: -0.0002,
    };
    let chief_at_waypoint = iss_like_elements();

    let result = rpo_wasm::enrichment::accept_waypoint_enrichment(
        input,
        transfer,
        0, // waypoint_index: first and only waypoint
        enriched_roe,
        chief_at_waypoint,
    );
    assert!(result.is_ok(), "accept_waypoint_enrichment should succeed");
}

#[test]
fn accept_waypoint_enrichment_index_out_of_bounds() {
    let input = test_pipeline_input();
    let transfer = test_transfer_result();
    let roe = test_roe();

    let result = rpo_wasm::enrichment::accept_waypoint_enrichment(
        input,
        transfer,
        999, // way beyond the 1 waypoint
        roe,
        iss_like_elements(),
    );
    assert!(result.is_err(), "should fail with out-of-bounds index");
    let err = result.unwrap_err();
    assert!(
        matches!(err.code, rpo_wasm::error::WasmErrorCode::Mission),
        "expected Mission error code, got {:?}",
        err.code
    );
}

// ---------------------------------------------------------------------------
// WASM boundary tests: require wasm-pack test --node
//
// These use serde_wasm_bindgen which panics on non-wasm32 targets.
// Gated behind cfg(target_arch = "wasm32") so they're ignored by cargo test.
// ---------------------------------------------------------------------------

#[cfg(target_arch = "wasm32")]
mod wasm_boundary {
    use wasm_bindgen_test::*;

    use rpo_core::mission::config::MissionConfig;
    use rpo_core::mission::formation::SafetyRequirements;
    use rpo_core::pipeline::types::PropagatorChoice;
    use rpo_core::test_helpers::iss_like_elements;
    use rpo_core::types::DepartureState;

    use super::{
        TEST_MIN_SEPARATION_KM, TEST_PROXIMITY_DELTA_R_OVER_R, TEST_PROXIMITY_SEPARATION_KM,
    };

    #[wasm_bindgen_test]
    fn plan_waypoint_mission_nominal() {
        let departure = DepartureState {
            roe: super::test_roe(),
            chief: iss_like_elements(),
            epoch: rpo_core::test_helpers::test_epoch(),
        };
        let waypoints = vec![super::test_waypoint_input()];
        let waypoints_js = serde_wasm_bindgen::to_value(&waypoints)
            .expect("waypoints serialization");

        let result = rpo_wasm::planning::plan_waypoint_mission(
            departure,
            waypoints_js,
            MissionConfig::default(),
            PropagatorChoice::J2,
        );
        assert!(result.is_ok(), "plan_waypoint_mission should succeed: {result:?}");
    }

    #[wasm_bindgen_test]
    fn enrich_waypoint_nominal() {
        let chief_mean = iss_like_elements();
        let pos_js = serde_wasm_bindgen::to_value(&[0.0_f64, 0.5, 0.0]).expect("pos ser");
        let vel_js =
            serde_wasm_bindgen::to_value(&Option::<[f64; 3]>::None).expect("vel ser");
        let requirements = SafetyRequirements {
            min_separation_km: TEST_MIN_SEPARATION_KM,
            alignment: Default::default(),
        };

        let result =
            rpo_wasm::enrichment::enrich_waypoint(pos_js, vel_js, chief_mean, requirements);
        assert!(result.is_ok(), "enrich_waypoint should succeed: {result:?}");
    }

    #[wasm_bindgen_test]
    fn enrich_waypoint_bad_position_returns_deserialization_error() {
        let chief_mean = iss_like_elements();
        let bad_pos = serde_wasm_bindgen::to_value("not an array").expect("ser");
        let vel_js =
            serde_wasm_bindgen::to_value(&Option::<[f64; 3]>::None).expect("ser");
        let requirements = SafetyRequirements {
            min_separation_km: TEST_MIN_SEPARATION_KM,
            alignment: Default::default(),
        };

        let result =
            rpo_wasm::enrichment::enrich_waypoint(bad_pos, vel_js, chief_mean, requirements);
        assert!(result.is_err(), "should fail on bad position");
        let err = result.unwrap_err();
        assert!(
            matches!(err.code, rpo_wasm::error::WasmErrorCode::Deserialization),
            "expected Deserialization error code, got {:?}",
            err.code
        );
    }

    #[wasm_bindgen_test]
    fn suggest_enrichment_returns_none_without_safety() {
        let transfer = super::test_transfer_result();
        let input = super::test_pipeline_input();

        let result = rpo_wasm::enrichment::suggest_enrichment(transfer, input);
        assert!(result.is_none(), "should be None without safety_requirements");
    }
}

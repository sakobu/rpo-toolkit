//! Smoke tests for rpo-wasm WASM exports.
//!
//! Tests are split into two groups:
//! - **Native tests** (`#[test]`): error mapping and serde roundtrips that don't
//!   touch `JsValue`. Run via `cargo test -p rpo-wasm`.
//! - **WASM tests** (`#[wasm_bindgen_test]`): end-to-end boundary tests that
//!   exercise `serde_wasm_bindgen` serialization. Run via `wasm-pack test --node`.
//!
//! Algorithm-level tests live in rpo-core; these focus on the WASM boundary.

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
    use rpo_core::elements::keplerian_to_state;
    use rpo_core::test_helpers::{iss_like_elements, test_epoch};

    let epoch = test_epoch();
    let chief_ke = iss_like_elements();
    let mut deputy_ke = chief_ke;
    deputy_ke.a_km += 1.0;
    let chief = keplerian_to_state(&chief_ke, epoch).expect("chief state");
    let deputy = keplerian_to_state(&deputy_ke, epoch).expect("deputy state");

    let config = rpo_core::mission::config::ProximityConfig::default();
    let result = rpo_wasm::planning::classify_separation(chief, deputy, config);
    assert!(result.is_ok(), "classify_separation should succeed for proximity pair");
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
    use rpo_core::mission::types::{MissionPhase, MissionPlan, PerchGeometry};
    use rpo_core::pipeline::types::{
        PipelineInput, PropagatorChoice, TransferResult, WaypointInput,
    };
    use rpo_core::test_helpers::{iss_like_elements, test_epoch};
    use rpo_core::types::{DepartureState, QuasiNonsingularROE};
    use rpo_core::elements::keplerian_to_state;

    fn proximity_pair() -> (rpo_core::types::StateVector, rpo_core::types::StateVector) {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;
        let chief = keplerian_to_state(&chief_ke, epoch).expect("chief state");
        let deputy = keplerian_to_state(&deputy_ke, epoch).expect("deputy state");
        (chief, deputy)
    }

    fn test_waypoint_input() -> WaypointInput {
        WaypointInput {
            position_ric_km: [0.0, 0.5, 0.0],
            velocity_ric_km_s: None,
            tof_s: None,
            label: None,
        }
    }

    #[wasm_bindgen_test]
    fn plan_waypoint_mission_nominal() {
        let departure = DepartureState {
            roe: QuasiNonsingularROE {
                da: 0.0,
                dlambda: 0.001,
                dex: 0.0001,
                dey: -0.0001,
                dix: 0.0001,
                diy: -0.0001,
            },
            chief: iss_like_elements(),
            epoch: test_epoch(),
        };
        let waypoints = vec![test_waypoint_input()];
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
            min_separation_km: 0.15,
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
            min_separation_km: 0.15,
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
        let (chief, deputy) = proximity_pair();
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let zero_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.001,
            dex: 0.0001,
            dey: -0.0001,
            dix: 0.0001,
            diy: -0.0001,
        };

        let transfer = TransferResult {
            plan: MissionPlan {
                phase: MissionPhase::Proximity {
                    roe: zero_roe,
                    chief_elements: chief_ke,
                    deputy_elements: chief_ke,
                    separation_km: 1.0,
                    delta_r_over_r: 0.0001,
                },
                transfer: None,
                perch_roe: zero_roe,
                chief_at_arrival: chief_ke,
            },
            perch_chief: chief.clone(),
            perch_deputy: deputy.clone(),
            arrival_epoch: epoch,
            lambert_dv_km_s: 0.0,
        };

        let input = PipelineInput {
            chief,
            deputy,
            waypoints: vec![test_waypoint_input()],
            config: MissionConfig::default(),
            propagator: PropagatorChoice::J2,
            perch: PerchGeometry::VBar {
                along_track_km: 5.0,
            },
            lambert_tof_s: 3600.0,
            lambert_config: Default::default(),
            proximity: Default::default(),
            chief_config: None,
            deputy_config: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo: None,
            cola: None,
            safety_requirements: None,
        };

        let result = rpo_wasm::enrichment::suggest_enrichment(transfer, input);
        assert!(result.is_none(), "should be None without safety_requirements");
    }
}

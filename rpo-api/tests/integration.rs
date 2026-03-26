//! Integration tests for the rpo-api WebSocket server.
//!
//! Start server on random port, connect via WebSocket, verify message flow.
//! Each test creates its own server because `#[tokio::test]` uses a per-test
//! runtime — a shared server would be killed when the spawning test completes.

use std::sync::Arc;
use std::time::Duration;

use axum::extract::ws::WebSocketUpgrade;
use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::time::timeout;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use tower_http::cors::CorsLayer;

#[derive(Clone)]
struct TestState {
    almanac: Arc<anise::prelude::Almanac>,
}

async fn ws_upgrade(ws: WebSocketUpgrade, State(state): State<TestState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| rpo_api::ws::handle_ws(socket, state.almanac))
}

async fn health() -> &'static str {
    "ok"
}

/// Start a test server and return the WebSocket URL.
async fn start_test_server() -> String {
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind");
    let port = listener.local_addr().unwrap().port();

    let almanac = Arc::new(anise::prelude::Almanac::default());
    let state = TestState { almanac };

    let app = Router::new()
        .route("/ws", get(ws_upgrade))
        .route("/health", get(health))
        .layer(CorsLayer::permissive())
        .with_state(state);

    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    // Give server a moment to start
    tokio::time::sleep(Duration::from_millis(50)).await;

    format!("ws://127.0.0.1:{port}/ws")
}

/// Send a message and receive the response.
async fn send_recv(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    msg: Value,
) -> Value {
    ws.send(Message::Text(msg.to_string().into()))
        .await
        .expect("send");

    let response = timeout(Duration::from_secs(10), ws.next())
        .await
        .expect("timeout")
        .expect("recv")
        .expect("msg");

    let text = response.into_text().expect("text");
    serde_json::from_str(&text).expect("json")
}

// ---------------------------------------------------------------------------
// JSON state helpers
// ---------------------------------------------------------------------------

/// Far-field chief ECI state.
fn far_field_chief() -> Value {
    json!({
        "epoch": "2024-01-01T00:00:00 UTC",
        "position_eci_km": [5876.261, 3392.661, 0.0],
        "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
    })
}

/// Far-field deputy ECI state (~1790 km from chief).
fn far_field_deputy() -> Value {
    json!({
        "epoch": "2024-01-01T00:00:00 UTC",
        "position_eci_km": [5199.839421, 4281.648523, 1398.070066],
        "velocity_eci_km_s": [-3.993103, 2.970313, 5.764540]
    })
}

/// Proximity chief ECI state (same as far-field chief).
fn proximity_chief() -> Value {
    far_field_chief()
}

/// Proximity deputy ECI state (very close to chief).
fn proximity_deputy() -> Value {
    json!({
        "epoch": "2024-01-01T00:00:00 UTC",
        "position_eci_km": [5876.561, 3392.261, 0.3],
        "velocity_eci_km_s": [-2.380612, 4.123067, 6.006817]
    })
}

// ---------------------------------------------------------------------------
// Message flow helpers
// ---------------------------------------------------------------------------

type WsStream = tokio_tungstenite::WebSocketStream<
    tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
>;

/// Send `set_states` and expect `state_updated` response.
async fn send_set_states(ws: &mut WsStream, chief: Value, deputy: Value, request_id: u64) -> Value {
    let resp = send_recv(
        ws,
        json!({
            "type": "set_states",
            "request_id": request_id,
            "chief": chief,
            "deputy": deputy,
        }),
    )
    .await;
    assert_eq!(resp["type"], "state_updated", "expected state_updated, got: {resp}");
    assert_eq!(resp["request_id"], request_id);
    resp
}

/// Send `classify` and expect `classify_result` response.
async fn send_classify(ws: &mut WsStream, request_id: u64) -> Value {
    let resp = send_recv(
        ws,
        json!({
            "type": "classify",
            "request_id": request_id,
        }),
    )
    .await;
    assert_eq!(resp["type"], "classify_result", "expected classify_result, got: {resp}");
    assert_eq!(resp["request_id"], request_id);
    resp
}

/// Send `compute_transfer` and expect `transfer_result` response.
async fn send_compute_transfer(ws: &mut WsStream, request_id: u64) -> Value {
    let resp = send_recv(
        ws,
        json!({
            "type": "compute_transfer",
            "request_id": request_id,
        }),
    )
    .await;
    assert_eq!(resp["type"], "transfer_result", "expected transfer_result, got: {resp}");
    assert_eq!(resp["request_id"], request_id);
    resp
}

/// Send `set_waypoints` and expect `plan_result` response.
async fn send_set_waypoints(ws: &mut WsStream, waypoints: Value, request_id: u64) -> Value {
    let resp = send_recv(
        ws,
        json!({
            "type": "set_waypoints",
            "request_id": request_id,
            "waypoints": waypoints,
        }),
    )
    .await;
    assert_eq!(resp["type"], "plan_result", "expected plan_result, got: {resp}");
    assert_eq!(resp["request_id"], request_id);
    resp
}

// ---------------------------------------------------------------------------
// 1. far_field_full_flow
// ---------------------------------------------------------------------------

#[tokio::test]
async fn far_field_full_flow() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Step 1: set_states
    let state_resp = send_set_states(
        &mut ws,
        far_field_chief(),
        far_field_deputy(),
        1,
    )
    .await;
    assert!(state_resp["updated"].as_array().unwrap().contains(&json!("chief")));
    assert!(state_resp["updated"].as_array().unwrap().contains(&json!("deputy")));
    assert!(state_resp["invalidated"].as_array().unwrap().contains(&json!("transfer")));

    // Step 2: classify => far_field
    let classify_resp = send_classify(&mut ws, 2).await;
    assert!(
        classify_resp["phase"]["far_field"].is_object(),
        "expected far_field, got: {}",
        classify_resp["phase"]
    );

    // Step 3: compute_transfer
    let transfer_resp = send_compute_transfer(&mut ws, 3).await;
    let result = &transfer_resp["result"];
    assert!(result["plan"]["transfer"].is_object(), "far-field transfer should include Lambert data");
    let lambert_dv = result["lambert_dv_km_s"].as_f64().expect("lambert_dv_km_s");
    assert!(lambert_dv > 0.0, "Lambert dv should be positive for far-field");
    assert!(result["perch_chief"]["position_eci_km"].is_array());
    assert!(result["perch_deputy"]["position_eci_km"].is_array());

    // Step 4: set_waypoints => plan_result
    let plan_resp = send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.5, 2.0, 0.5], "tof_s": 4200.0 }]),
        4,
    )
    .await;
    let plan = &plan_resp["result"];
    assert!(plan["legs"].is_array());
    let legs = plan["legs"].as_array().unwrap();
    assert_eq!(legs.len(), 1, "expected 1 leg for 1 waypoint");
    assert!(plan["total_dv_km_s"].as_f64().unwrap() > 0.0);
    assert!(plan["total_duration_s"].as_f64().unwrap() > 0.0);
}

// ---------------------------------------------------------------------------
// 2. proximity_full_flow
// ---------------------------------------------------------------------------

#[tokio::test]
async fn proximity_full_flow() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Step 1: set_states
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;

    // Step 2: classify => proximity
    let classify_resp = send_classify(&mut ws, 2).await;
    assert!(
        classify_resp["phase"]["proximity"].is_object(),
        "expected proximity, got: {}",
        classify_resp["phase"]
    );

    // Step 3: set_waypoints directly (auto-computes transfer for proximity)
    let plan_resp = send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.0, 2.0, 0.0], "tof_s": 4200.0 }]),
        3,
    )
    .await;
    let plan = &plan_resp["result"];
    assert!(plan["legs"].is_array());
    let legs = plan["legs"].as_array().unwrap();
    assert_eq!(legs.len(), 1, "expected 1 leg for 1 waypoint");
    assert!(plan["total_dv_km_s"].as_f64().unwrap() > 0.0);

    // Proximity should have no transfer_summary (no Lambert)
    assert!(
        plan["transfer_summary"].is_null(),
        "proximity should have no Lambert transfer_summary"
    );
}

// ---------------------------------------------------------------------------
// 3. state_invalidation
// ---------------------------------------------------------------------------

#[tokio::test]
async fn state_invalidation() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Set states and compute transfer
    send_set_states(&mut ws, far_field_chief(), far_field_deputy(), 1).await;
    send_compute_transfer(&mut ws, 2).await;

    // Check session has transfer
    let session_resp = send_recv(
        &mut ws,
        json!({ "type": "get_session", "request_id": 3 }),
    )
    .await;
    assert_eq!(session_resp["type"], "session_state");
    assert!(session_resp["summary"]["has_transfer"].as_bool().unwrap(), "should have transfer");

    // Set new states => transfer should be invalidated
    let state_resp = send_set_states(
        &mut ws,
        proximity_chief(),
        proximity_deputy(),
        4,
    )
    .await;
    assert!(
        state_resp["invalidated"].as_array().unwrap().contains(&json!("transfer")),
        "set_states should invalidate transfer"
    );

    // Verify transfer is gone via get_session
    let session_resp = send_recv(
        &mut ws,
        json!({ "type": "get_session", "request_id": 5 }),
    )
    .await;
    assert_eq!(session_resp["type"], "session_state");
    assert!(
        !session_resp["summary"]["has_transfer"].as_bool().unwrap(),
        "transfer should be cleared after set_states"
    );
}

// ---------------------------------------------------------------------------
// 4. missing_state_errors
// ---------------------------------------------------------------------------

#[tokio::test]
async fn missing_state_errors() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Classify before set_states => error
    let resp = send_recv(
        &mut ws,
        json!({ "type": "classify", "request_id": 1 }),
    )
    .await;
    assert_eq!(resp["type"], "error");
    assert_eq!(resp["code"], "missing_session_state");
    assert_eq!(resp["request_id"], 1);
    assert!(resp["detail"]["missing"].is_string(), "detail should have 'missing' field");
    assert!(resp["detail"]["context"].is_string(), "detail should have 'context' field");
}

// ---------------------------------------------------------------------------
// 5. far_field_without_transfer
// ---------------------------------------------------------------------------

#[tokio::test]
async fn far_field_without_transfer() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Set far-field states
    send_set_states(&mut ws, far_field_chief(), far_field_deputy(), 1).await;

    // Try set_waypoints without compute_transfer => error
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "set_waypoints",
            "request_id": 2,
            "waypoints": [{ "position_ric_km": [0.5, 2.0, 0.5], "tof_s": 4200.0 }]
        }),
    )
    .await;
    assert_eq!(resp["type"], "error", "expected error, got: {resp}");
    assert_eq!(resp["code"], "missing_session_state");
    assert_eq!(resp["request_id"], 2);
}

// ---------------------------------------------------------------------------
// 6. update_config_replan
// ---------------------------------------------------------------------------

#[tokio::test]
async fn update_config_replan() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Build a proximity mission first
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;
    send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.0, 2.0, 0.0], "tof_s": 4200.0 }]),
        2,
    )
    .await;

    // Update config with mission-affecting change => replan => plan_result
    // MissionConfig requires all TargetingConfig fields (no per-field defaults).
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "update_config",
            "request_id": 3,
            "config": {
                "targeting": {
                    "max_iterations": 80,
                    "position_tol_km": 1e-6,
                    "initial_damping": 1.0,
                    "dv_cap_km_s": 1.0,
                    "trajectory_steps": 200
                }
            }
        }),
    )
    .await;
    assert_eq!(resp["type"], "plan_result", "mission-affecting update_config should trigger replan, got: {resp}");
    assert_eq!(resp["request_id"], 3);
    assert!(resp["result"]["legs"].is_array());
}

// ---------------------------------------------------------------------------
// 7. update_config_overlay_only
// ---------------------------------------------------------------------------

#[tokio::test]
async fn update_config_overlay_only() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Build a proximity mission first
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;
    send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.0, 2.0, 0.0], "tof_s": 4200.0 }]),
        2,
    )
    .await;

    // Update only navigation_accuracy (overlay, no replan) => state_updated
    // NavigationAccuracy requires Vector3 fields for position and velocity sigma.
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "update_config",
            "request_id": 3,
            "navigation_accuracy": {
                "position_sigma_ric_km": [0.01, 0.01, 0.01],
                "velocity_sigma_ric_km_s": [0.0001, 0.0001, 0.0001]
            }
        }),
    )
    .await;
    assert_eq!(resp["type"], "state_updated", "overlay-only update_config should return state_updated, got: {resp}");
    assert_eq!(resp["request_id"], 3);
    assert!(resp["updated"].as_array().unwrap().contains(&json!("config")));
    assert!(resp["invalidated"].as_array().unwrap().is_empty());
}

// ---------------------------------------------------------------------------
// 8. get_trajectory_resampling
// ---------------------------------------------------------------------------

#[tokio::test]
async fn get_trajectory_resampling() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Build a proximity mission
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;
    send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.0, 2.0, 0.0], "tof_s": 4200.0 }]),
        2,
    )
    .await;

    // Get trajectory with max_points: 5
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "get_trajectory",
            "request_id": 3,
            "max_points": 5
        }),
    )
    .await;
    assert_eq!(resp["type"], "trajectory_data", "expected trajectory_data, got: {resp}");
    assert_eq!(resp["request_id"], 3);
    let legs = resp["legs"].as_array().expect("legs should be array");
    assert_eq!(legs.len(), 1, "should have 1 leg");
    let points = legs[0]["points"].as_array().expect("points should be array");
    assert!(
        points.len() <= 5,
        "resampled trajectory should have at most 5 points, got {}",
        points.len()
    );
}

// ---------------------------------------------------------------------------
// 9. reset_clears_session
// ---------------------------------------------------------------------------

#[tokio::test]
async fn reset_clears_session() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Set states and classify
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;
    send_classify(&mut ws, 2).await;

    // Reset
    let resp = send_recv(
        &mut ws,
        json!({ "type": "reset", "request_id": 3 }),
    )
    .await;
    assert_eq!(resp["type"], "state_updated");
    assert_eq!(resp["request_id"], 3);
    assert!(resp["invalidated"].as_array().unwrap().contains(&json!("all")));

    // Classify again => error (no states)
    let resp = send_recv(
        &mut ws,
        json!({ "type": "classify", "request_id": 4 }),
    )
    .await;
    assert_eq!(resp["type"], "error");
    assert_eq!(resp["code"], "missing_session_state");
}

// ---------------------------------------------------------------------------
// 10. cancel_without_active_job (connection survives)
// ---------------------------------------------------------------------------

#[tokio::test]
async fn cancel_without_active_job_no_crash() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Send Cancel with no active job — server should not crash or respond
    ws.send(Message::Text(
        json!({"type": "cancel", "request_id": 99}).to_string().into(),
    ))
    .await
    .expect("send");

    // Small delay to let server process the cancel
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Verify connection still alive by setting states
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 100).await;

    // Verify we can still do operations
    let resp = send_classify(&mut ws, 101).await;
    assert_eq!(resp["type"], "classify_result");
}

// ---------------------------------------------------------------------------
// 11. error_details_missing_session_state
// ---------------------------------------------------------------------------

#[tokio::test]
async fn error_details_missing_session_state() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // compute_transfer without states => missing_session_state error with detail
    let resp = send_recv(
        &mut ws,
        json!({ "type": "compute_transfer", "request_id": 1 }),
    )
    .await;
    assert_eq!(resp["type"], "error");
    assert_eq!(resp["code"], "missing_session_state");
    assert_eq!(resp["request_id"], 1);

    let detail = &resp["detail"];
    assert!(detail["missing"].is_string(), "detail should have 'missing' field: {detail}");
    assert!(detail["context"].is_string(), "detail should have 'context' field: {detail}");
}

// ---------------------------------------------------------------------------
// 12. get_eclipse_on_demand
// ---------------------------------------------------------------------------

#[tokio::test]
async fn get_eclipse_on_demand() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Build a proximity mission (eclipse data is pre-computed on WaypointMission)
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 1).await;
    send_set_waypoints(
        &mut ws,
        json!([{ "position_ric_km": [0.0, 2.0, 0.0], "tof_s": 4200.0 }]),
        2,
    )
    .await;

    // Get eclipse
    let resp = send_recv(
        &mut ws,
        json!({ "type": "get_eclipse", "request_id": 3 }),
    )
    .await;
    assert_eq!(resp["type"], "eclipse_data", "expected eclipse_data, got: {resp}");
    assert_eq!(resp["request_id"], 3);
    // For proximity, transfer eclipse may be absent, but mission eclipse should exist
    assert!(
        resp["mission"].is_object() || resp["transfer"].is_object(),
        "eclipse_data should have at least mission or transfer data: {resp}"
    );
}

// ---------------------------------------------------------------------------
// 13. get_session_summary
// ---------------------------------------------------------------------------

#[tokio::test]
async fn get_session_summary() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Fresh session
    let resp = send_recv(
        &mut ws,
        json!({ "type": "get_session", "request_id": 1 }),
    )
    .await;
    assert_eq!(resp["type"], "session_state");
    assert_eq!(resp["request_id"], 1);
    let summary = &resp["summary"];
    assert_eq!(summary["has_chief"], false);
    assert_eq!(summary["has_deputy"], false);
    assert_eq!(summary["has_transfer"], false);
    assert_eq!(summary["has_mission"], false);
    assert_eq!(summary["waypoint_count"], 0);

    // Set states and check again
    send_set_states(&mut ws, proximity_chief(), proximity_deputy(), 2).await;

    let resp = send_recv(
        &mut ws,
        json!({ "type": "get_session", "request_id": 3 }),
    )
    .await;
    let summary = &resp["summary"];
    assert_eq!(summary["has_chief"], true);
    assert_eq!(summary["has_deputy"], true);
    assert_eq!(summary["has_transfer"], false);
    assert_eq!(summary["has_mission"], false);
}

// ---------------------------------------------------------------------------
// 14. binary_frame_returns_error
// ---------------------------------------------------------------------------

#[tokio::test]
async fn binary_frame_returns_error() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    ws.send(Message::Binary(vec![0x00, 0x01, 0x02].into()))
        .await
        .expect("send");

    let response = timeout(Duration::from_secs(5), ws.next())
        .await
        .expect("timeout")
        .expect("stream")
        .expect("msg");
    let text = response.into_text().expect("text");
    let parsed: Value = serde_json::from_str(&text).expect("json");

    assert_eq!(parsed["type"], "error");
    assert_eq!(parsed["code"], "invalid_input");
    assert!(parsed["message"].as_str().unwrap().contains("text frame"));
}

// ---------------------------------------------------------------------------
// 15. malformed_json_error_detail
// ---------------------------------------------------------------------------

#[tokio::test]
async fn malformed_json_error_detail() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    ws.send(Message::Text("{not valid json}".into()))
        .await
        .expect("send");

    let response = timeout(Duration::from_secs(5), ws.next())
        .await
        .expect("timeout")
        .expect("recv")
        .expect("msg");

    let text = response.into_text().expect("text");
    let parsed: Value = serde_json::from_str(&text).expect("json");

    assert_eq!(parsed["code"], "invalid_input");
    assert_eq!(parsed["detail"]["reason"], "malformed_json");
    assert!(
        parsed["detail"]["detail"].is_string(),
        "malformed_json detail should include serde error"
    );
}

// ---------------------------------------------------------------------------
// 16. error_detail_targeting_convergence (unit test)
// ---------------------------------------------------------------------------

#[test]
fn error_detail_targeting_convergence() {
    use rpo_api::error::ApiError;
    use rpo_core::mission::MissionError;

    let err = ApiError::Mission(Box::new(MissionError::TargetingConvergence {
        final_error_km: 1.23,
        iterations: 50,
    }));
    let msg = err.to_server_message(Some(99));
    let json = serde_json::to_value(&msg).unwrap();

    assert_eq!(json["code"], "targeting_convergence");
    assert_eq!(json["request_id"], 99);
    assert_eq!(json["detail"]["final_error_km"], 1.23);
    assert_eq!(json["detail"]["iterations"], 50);
}

// ---------------------------------------------------------------------------
// 17. error_detail_missing_field (unit test)
// ---------------------------------------------------------------------------

#[test]
fn error_detail_missing_field() {
    use rpo_api::error::require_field;

    let err = require_field::<u32>(None, "chief_config", "Monte Carlo").unwrap_err();
    let msg = err.to_server_message(Some(42));
    let json = serde_json::to_value(&msg).unwrap();

    assert_eq!(json["code"], "invalid_input");
    assert_eq!(json["detail"]["reason"], "missing_field");
    assert_eq!(json["detail"]["field"], "chief_config");
    assert_eq!(json["detail"]["context"], "Monte Carlo");
}

// ---------------------------------------------------------------------------
// 18. all_error_codes_serialize_to_snake_case
// ---------------------------------------------------------------------------

#[test]
fn all_error_codes_serialize_to_snake_case() {
    use rpo_api::protocol::ErrorCode;

    let codes = [
        (ErrorCode::TargetingConvergence, "targeting_convergence"),
        (ErrorCode::LambertFailure, "lambert_failure"),
        (ErrorCode::PropagationError, "propagation_error"),
        (ErrorCode::ValidationError, "validation_error"),
        (ErrorCode::MonteCarloError, "monte_carlo_error"),
        (ErrorCode::NyxBridgeError, "nyx_bridge_error"),
        (ErrorCode::CovarianceError, "covariance_error"),
        (ErrorCode::InvalidInput, "invalid_input"),
        (ErrorCode::MissionError, "mission_error"),
        (ErrorCode::Cancelled, "cancelled"),
        (ErrorCode::MissingSessionState, "missing_session_state"),
    ];

    for (code, expected) in &codes {
        let json = serde_json::to_value(code).unwrap();
        assert_eq!(json.as_str().unwrap(), *expected, "ErrorCode::{code:?}");
    }
}

// ---------------------------------------------------------------------------
// 19. api_error_to_server_message_never_panics (unit test)
// ---------------------------------------------------------------------------

#[test]
fn api_error_to_server_message_never_panics() {
    use rpo_api::error::ApiError;
    use rpo_core::mission::MissionError;

    let errors: Vec<ApiError> = vec![
        ApiError::from(MissionError::EmptyWaypoints),
        ApiError::from(MissionError::TargetingConvergence {
            final_error_km: 1.0,
            iterations: 50,
        }),
        ApiError::Cancelled,
    ];

    for err in &errors {
        let msg = err.to_server_message(Some(1));
        let json = serde_json::to_value(&msg).unwrap();
        assert_eq!(json["type"], "error");
        assert!(json["code"].is_string());
        assert!(json["message"].is_string());
    }
}

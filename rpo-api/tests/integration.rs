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

/// Far-field mission definition (chief and deputy ~1790 km apart).
///
/// ECI states from `examples/mission.json`. Deputy is in a significantly
/// different orbit (higher SMA, different inclination), producing ROE
/// magnitudes well above the default proximity threshold (0.005).
fn far_field_mission() -> Value {
    json!({
        "chief": {
            "epoch": "2024-01-01T00:00:00 UTC",
            "position_eci_km": [5876.261, 3392.661, 0.0],
            "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
        },
        "deputy": {
            "epoch": "2024-01-01T00:00:00 UTC",
            "position_eci_km": [5199.839421, 4281.648523, 1398.070066],
            "velocity_eci_km_s": [-3.993103, 2.970313, 5.764540]
        },
        "waypoints": [
            {
                "position_ric_km": [0.5, 2.0, 0.5],
                "tof_s": 4200.0
            }
        ]
    })
}

/// Proximity mission definition (chief ≈ deputy).
fn proximity_mission() -> Value {
    json!({
        "chief": {
            "epoch": "2024-01-01T00:00:00 UTC",
            "position_eci_km": [5876.261, 3392.661, 0.0],
            "velocity_eci_km_s": [-2.380512, 4.123167, 6.006917]
        },
        "deputy": {
            "epoch": "2024-01-01T00:00:00 UTC",
            "position_eci_km": [5876.561, 3392.261, 0.3],
            "velocity_eci_km_s": [-2.380612, 4.123067, 6.006817]
        },
        "waypoints": [
            {
                "position_ric_km": [0.0, 2.0, 0.0],
                "tof_s": 4200.0
            }
        ]
    })
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

    let response = timeout(Duration::from_secs(5), ws.next())
        .await
        .expect("timeout")
        .expect("recv")
        .expect("msg");

    let text = response.into_text().expect("text");
    serde_json::from_str(&text).expect("json")
}

#[tokio::test]
async fn classify_returns_proximity() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "Classify",
            "request_id": 1,
            "mission": proximity_mission()
        }),
    )
    .await;

    assert_eq!(resp["type"], "ClassifyResult");
    assert_eq!(resp["request_id"], 1);
    // Nearby chief/deputy should classify as Proximity
    assert!(
        resp["phase"]["Proximity"].is_object(),
        "expected Proximity, got: {}",
        resp["phase"]
    );
}

#[tokio::test]
async fn plan_mission_returns_legs() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "PlanMission",
            "request_id": 2,
            "mission": proximity_mission()
        }),
    )
    .await;

    assert_eq!(resp["type"], "MissionResult");
    assert_eq!(resp["request_id"], 2);
    assert!(resp["result"]["mission"]["legs"].is_array());
    assert!(resp["result"]["total_dv_km_s"].is_number());

    // Should have exactly 1 leg (1 waypoint)
    let legs = resp["result"]["mission"]["legs"].as_array().unwrap();
    assert_eq!(legs.len(), 1, "expected 1 leg for 1 waypoint");
}

#[tokio::test]
async fn move_waypoint_returns_result() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "MoveWaypoint",
            "request_id": 3,
            "modified_index": 0,
            "mission": proximity_mission()
        }),
    )
    .await;

    assert_eq!(resp["type"], "MissionResult");
    assert_eq!(resp["request_id"], 3);
}

#[tokio::test]
async fn multiple_requests_on_same_connection() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // First request
    let resp1 = send_recv(
        &mut ws,
        json!({
            "type": "Classify",
            "request_id": 10,
            "mission": proximity_mission()
        }),
    )
    .await;
    assert_eq!(resp1["type"], "ClassifyResult");
    assert_eq!(resp1["request_id"], 10);

    // Second request on same connection
    let resp2 = send_recv(
        &mut ws,
        json!({
            "type": "PlanMission",
            "request_id": 11,
            "mission": proximity_mission()
        }),
    )
    .await;
    assert_eq!(resp2["type"], "MissionResult");
    assert_eq!(resp2["request_id"], 11);
}

// ---------------------------------------------------------------------------
// Far-field path (Lambert transfer + perch + waypoints)
// ---------------------------------------------------------------------------

#[tokio::test]
async fn far_field_classify_and_plan() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Classify should return FarField (not Proximity)
    let classify = send_recv(
        &mut ws,
        json!({
            "type": "Classify",
            "request_id": 20,
            "mission": far_field_mission()
        }),
    )
    .await;
    assert_eq!(classify["type"], "ClassifyResult");
    assert!(
        classify["phase"]["FarField"].is_object(),
        "expected FarField, got: {}",
        classify["phase"]
    );

    // Plan should include a Lambert transfer
    let plan = send_recv(
        &mut ws,
        json!({
            "type": "PlanMission",
            "request_id": 21,
            "mission": far_field_mission()
        }),
    )
    .await;
    assert_eq!(plan["type"], "MissionResult");
    assert!(
        plan["result"]["transfer"].is_object(),
        "far-field plan should include Lambert transfer"
    );
    let transfer_dv = plan["result"]["transfer"]["total_dv_km_s"]
        .as_f64()
        .expect("transfer should have total_dv_km_s");
    assert!(transfer_dv > 0.0, "Lambert Δv should be positive");

    let total_dv = plan["result"]["total_dv_km_s"]
        .as_f64()
        .expect("result should have total_dv_km_s");
    assert!(
        total_dv > transfer_dv,
        "total Δv ({total_dv}) should exceed Lambert Δv ({transfer_dv}) due to waypoint maneuvers"
    );

    let legs = plan["result"]["mission"]["legs"].as_array().unwrap();
    assert_eq!(legs.len(), 1, "expected 1 leg for 1 waypoint");
}

// ---------------------------------------------------------------------------
// Error code mapping (unit tests — no WebSocket needed)
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
// Strengthened error detail assertions
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

#[tokio::test]
async fn empty_waypoints_error_detail() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    let mut mission = proximity_mission();
    mission["waypoints"] = json!([]);

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "PlanMission",
            "request_id": 30,
            "mission": mission
        }),
    )
    .await;

    assert_eq!(resp["type"], "Error");
    assert_eq!(resp["code"], "invalid_input");
    assert_eq!(
        resp["detail"]["reason"], "no waypoints provided",
        "detail: {}",
        resp["detail"]
    );
}

// ---------------------------------------------------------------------------
// Binary frame rejection
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

    assert_eq!(parsed["type"], "Error");
    assert_eq!(parsed["code"], "invalid_input");
    assert!(parsed["message"].as_str().unwrap().contains("text frame"));
}

// ---------------------------------------------------------------------------
// Cancel without active job (connection survives)
// ---------------------------------------------------------------------------

#[tokio::test]
async fn cancel_without_active_job_no_crash() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    // Send Cancel with no active job
    ws.send(Message::Text(
        json!({"type": "Cancel", "request_id": 99}).to_string().into(),
    ))
    .await
    .expect("send");

    // Small delay to let server process the cancel
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Verify connection still alive by sending a classify
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "Classify",
            "request_id": 100,
            "mission": proximity_mission()
        }),
    )
    .await;
    assert_eq!(resp["type"], "ClassifyResult");
}

// ---------------------------------------------------------------------------
// ErrorCode serialization completeness
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
    ];

    for (code, expected) in &codes {
        let json = serde_json::to_value(code).unwrap();
        assert_eq!(json.as_str().unwrap(), *expected, "ErrorCode::{code:?}");
    }
}

// ---------------------------------------------------------------------------
// ApiError::to_server_message coverage
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
        assert_eq!(json["type"], "Error");
        assert!(json["code"].is_string());
        assert!(json["message"].is_string());
    }
}

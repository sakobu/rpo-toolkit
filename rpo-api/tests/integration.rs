//! Integration tests for the rpo-api WebSocket server.
//!
//! Start server on random port, connect via WebSocket, verify message flow.

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
                "position": [0.0, 2.0, 0.0],
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
async fn invalid_json_returns_error() {
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

    assert_eq!(parsed["type"], "Error");
    assert_eq!(parsed["code"], "invalid_input");
}

#[tokio::test]
async fn empty_waypoints_returns_error() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.expect("connect");

    let mut mission = proximity_mission();
    mission["waypoints"] = json!([]);

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "PlanMission",
            "request_id": 4,
            "mission": mission
        }),
    )
    .await;

    assert_eq!(resp["type"], "Error");
    assert_eq!(resp["request_id"], 4);
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

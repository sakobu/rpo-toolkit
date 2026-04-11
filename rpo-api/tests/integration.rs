//! Integration tests for the stateless WebSocket API.
//!
//! Tests the stateless protocol: 5 ClientMessage variants (compute_transfer,
//! extract_drag, validate, run_mc, cancel) and 8 ServerMessage variants
//! (transfer_result, drag_result, validation_result, monte_carlo_result,
//! progress, error, cancelled, heartbeat).
//!
//! Each test creates its own server because `#[tokio::test]` uses a per-test
//! runtime — a shared server would be killed when the spawning test completes.
//!
//! Tests are grouped by endpoint in mission-workflow order:
//! transfer → drag → validate → MC → cancel → errors.

use futures_util::{SinkExt, StreamExt};
use rpo_core::pipeline::types::PipelineInput;
use serde_json::{json, Value};
use std::net::SocketAddr;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio_tungstenite::{connect_async, tungstenite::Message};

/// Default timeout for synchronous request-response exchanges (transfer, drag
/// short-circuit, error cases). 10s is generous for operations that complete
/// in < 1s on any hardware.
const DEFAULT_RECV_TIMEOUT: Duration = Duration::from_secs(10);

/// Timeout for nyx full-physics validation (~5-10s typical, 30s worst case).
const VALIDATE_TIMEOUT_SECS: u64 = 30;

/// Timeout for nyx drag extraction (~3s typical, 30s worst case).
const DRAG_TIMEOUT_SECS: u64 = 30;

/// Timeout for Monte Carlo ensemble (3 samples, ~30-60s typical).
const MC_TIMEOUT_SECS: u64 = 120;

/// Timeout for cancel race condition test.
const CANCEL_TIMEOUT: Duration = Duration::from_secs(15);

/// Short timeout for error response tests (immediate server reply expected).
const ERROR_RECV_TIMEOUT: Duration = Duration::from_secs(5);

// ===========================================================================
// Test infrastructure
// ===========================================================================

#[derive(Clone)]
struct TestState {
    almanac: std::sync::Arc<anise::prelude::Almanac>,
}

async fn ws_upgrade(
    ws: axum::extract::ws::WebSocketUpgrade,
    axum::extract::State(state): axum::extract::State<TestState>,
) -> impl axum::response::IntoResponse {
    ws.on_upgrade(move |socket| rpo_api::ws::handle_ws(socket, state.almanac))
}

/// Start a test server on a random port and return the WebSocket URL.
async fn start_test_server() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();

    // load_default_almanac() returns Arc<Almanac> — no network download needed.
    let almanac = rpo_nyx::nyx_bridge::load_default_almanac();
    let state = TestState { almanac };

    let app = axum::Router::new()
        .route("/ws", axum::routing::get(ws_upgrade))
        .with_state(state);

    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    format!("ws://{addr}/ws")
}

/// Start a test server with the full ANISE almanac (downloads on first run).
///
/// Required for tests that invoke nyx full-physics propagation (validate, MC),
/// which need planetary frame data (Earth ID 399) not present in the default almanac.
async fn start_test_server_full() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr: SocketAddr = listener.local_addr().unwrap();

    // load_full_almanac() may download kernels — run on blocking thread.
    let almanac = tokio::task::spawn_blocking(rpo_nyx::nyx_bridge::load_full_almanac)
        .await
        .unwrap()
        .expect("load_full_almanac");
    let state = TestState { almanac };

    let app = axum::Router::new()
        .route("/ws", axum::routing::get(ws_upgrade))
        .with_state(state);

    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    format!("ws://{addr}/ws")
}

/// Send a JSON message and receive the next non-heartbeat text response within 10 seconds.
///
/// Heartbeat messages are skipped — background jobs (extract_drag) may send one
/// before the result arrives if the 30-second heartbeat fires.
async fn send_recv(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    msg: Value,
) -> Value {
    ws.send(Message::Text(msg.to_string().into()))
        .await
        .expect("send");

    recv_next(ws).await
}

/// Receive the next non-heartbeat text response within 10 seconds.
async fn recv_next(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
) -> Value {
    let deadline = std::time::Instant::now() + DEFAULT_RECV_TIMEOUT;
    loop {
        let remaining = deadline.saturating_duration_since(std::time::Instant::now());
        if remaining.is_zero() {
            panic!("Timed out waiting for non-heartbeat response");
        }
        match tokio::time::timeout(remaining, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                let val: Value = serde_json::from_str(&text).unwrap();
                if val["type"] == "heartbeat" {
                    continue;
                }
                return val;
            }
            other => panic!("Expected text response, got: {other:?}"),
        }
    }
}

/// Receive WS messages until one with a matching `type` field arrives.
///
/// Skips `progress` and `heartbeat` messages. Panics if any other message type
/// arrives before the expected one, or if the deadline expires.
async fn recv_until_type(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    target_type: &str,
    timeout_secs: u64,
) -> Value {
    let deadline = std::time::Instant::now() + Duration::from_secs(timeout_secs);
    loop {
        let remaining = deadline.saturating_duration_since(std::time::Instant::now());
        if remaining.is_zero() {
            panic!("Timed out waiting for '{target_type}'");
        }
        match tokio::time::timeout(remaining, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                let val: Value = serde_json::from_str(&text).unwrap();
                let msg_type = val["type"].as_str().unwrap_or("");
                if msg_type == target_type {
                    return val;
                }
                assert!(
                    msg_type == "progress" || msg_type == "heartbeat",
                    "unexpected message type '{msg_type}' while waiting for '{target_type}': {val}"
                );
            }
            other => panic!("Expected text response while waiting for '{target_type}', got: {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// State helpers
// ---------------------------------------------------------------------------

/// ISS-like far-field test states: chief at LEO (~6778 km), deputy at GEO-like
/// altitude (~42164 km). Separation >> proximity threshold so Lambert is called.
fn far_field_states() -> (Value, Value) {
    let chief = json!({
        "epoch": "2024-01-15T12:00:00.000000000 UTC",
        "position_eci_km": [6778.137, 0.0, 0.0],
        "velocity_eci_km_s": [0.0, 7.6126, 0.0]
    });
    let deputy = json!({
        "epoch": "2024-01-15T12:00:00.000000000 UTC",
        "position_eci_km": [42164.0, 0.0, 0.0],
        "velocity_eci_km_s": [0.0, 3.0747, 0.0]
    });
    (chief, deputy)
}

/// Proximity test states: two states ~0.3 km apart at ISS-like altitude.
/// δr/r ≈ 0.3 / 6778 ≈ 4.4e-5 << roe_threshold = 0.005 → proximity.
fn proximity_states() -> (Value, Value) {
    let chief = json!({
        "epoch": "2024-01-15T12:00:00.000000000 UTC",
        "position_eci_km": [6778.137, 0.0, 0.0],
        "velocity_eci_km_s": [0.0, 7.6126, 0.0]
    });
    let deputy = json!({
        "epoch": "2024-01-15T12:00:00.000000000 UTC",
        "position_eci_km": [6778.437, 0.0, 0.0],
        "velocity_eci_km_s": [0.0, 7.6123, 0.0]
    });
    (chief, deputy)
}

// ===========================================================================
// Transfer (ComputeTransfer)
// ===========================================================================

/// Far-field: expect `transfer_result` with a Lambert solution.
///
/// Uses GEO-like deputy (42164 km) vs ISS-like chief (6778 km).
/// Lambert is required; `plan.transfer` must be an object and
/// `lambert_dv_km_s` must be positive.
#[tokio::test]
async fn transfer_far_field() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();
    let (chief, deputy) = far_field_states();

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "compute_transfer",
            "request_id": 1,
            "chief": chief,
            "deputy": deputy,
            "perch": { "v_bar": { "along_track_km": 1.0 } },
            "proximity": { "roe_threshold": 0.005 },
            "lambert_tof_s": 3600.0,
            "lambert_config": { "direction": "auto", "revolutions": 0 }
        }),
    )
    .await;

    assert_eq!(resp["type"], "transfer_result", "got: {resp}");
    assert_eq!(resp["request_id"], 1);
    assert!(
        resp["result"]["plan"]["transfer"].is_object(),
        "far-field should have Lambert transfer, got: {}",
        resp["result"]["plan"]["transfer"]
    );
    assert!(
        resp["result"]["lambert_dv_km_s"].as_f64().unwrap() > 0.0,
        "lambert_dv_km_s should be positive"
    );
}

/// Proximity: expect `transfer_result` with no Lambert (transfer is null).
///
/// States are ~0.3 km apart; δr/r << roe_threshold so no Lambert is run.
#[tokio::test]
async fn transfer_proximity() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();
    let (chief, deputy) = proximity_states();

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "compute_transfer",
            "request_id": 2,
            "chief": chief,
            "deputy": deputy,
            "perch": { "v_bar": { "along_track_km": 1.0 } },
            "proximity": { "roe_threshold": 0.005 },
            "lambert_tof_s": 3600.0,
            "lambert_config": { "direction": "auto", "revolutions": 0 }
        }),
    )
    .await;

    assert_eq!(resp["type"], "transfer_result", "got: {resp}");
    assert_eq!(resp["request_id"], 2);
    assert!(
        resp["result"]["plan"]["transfer"].is_null(),
        "proximity should have no Lambert transfer, got: {}",
        resp["result"]["plan"]["transfer"]
    );
    assert_eq!(
        resp["result"]["lambert_dv_km_s"].as_f64().unwrap(),
        0.0,
        "lambert_dv_km_s should be zero for proximity"
    );
}

// ===========================================================================
// Drag extraction (ExtractDrag)
// ===========================================================================

/// Identical spacecraft configs → short-circuit to `DragConfig::zero()`.
///
/// When `chief_config == deputy_config`, `handle_extract_drag` bypasses nyx
/// and returns zero rates immediately. No almanac download required.
#[tokio::test]
async fn extract_drag_identical_configs_returns_zero() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();
    let (chief, deputy) = proximity_states();

    // Identical configs → DragConfig::zero() short-circuit.
    // Field names match SpacecraftConfig: dry_mass_kg, drag_area_m2, coeff_drag,
    // srp_area_m2, coeff_reflectivity.
    let config = json!({
        "dry_mass_kg": 12.0,
        "drag_area_m2": 0.06,
        "coeff_drag": 2.2,
        "srp_area_m2": 0.06,
        "coeff_reflectivity": 1.5
    });

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "extract_drag",
            "request_id": 10,
            "chief": chief,
            "deputy": deputy,
            "chief_config": config,
            "deputy_config": config
        }),
    )
    .await;

    assert_eq!(resp["type"], "drag_result", "got: {resp}");
    assert_eq!(resp["request_id"], 10);
    assert_eq!(resp["drag"]["da_dot"], 0.0, "da_dot should be zero");
    assert_eq!(resp["drag"]["dex_dot"], 0.0, "dex_dot should be zero");
    assert_eq!(resp["drag"]["dey_dot"], 0.0, "dey_dot should be zero");
}

/// Different spacecraft configs → real nyx DMF extraction (not the short-circuit).
///
/// Uses `examples/validate.json` which has a 500 kg servicer chief and a 12 kg
/// cubesat deputy. Because the configs differ, `handle_extract_drag` cannot
/// short-circuit and must run the full nyx propagation.
/// The resulting `da_dot` must be non-zero because the mass ratio is ~42×.
#[tokio::test]
#[ignore] // nyx full-physics drag extraction (~3s)
async fn extract_drag_different_configs() {
    let input: PipelineInput = serde_json::from_reader(
        std::fs::File::open("../examples/validate.json").expect("open examples/validate.json"),
    )
    .expect("parse validate.json as PipelineInput");

    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    // Verify this test exercises the real nyx path (configs must differ).
    assert_ne!(
        chief_config, deputy_config,
        "test needs different configs to bypass the short-circuit"
    );

    let url = start_test_server_full().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    let msg = json!({
        "type": "extract_drag",
        "request_id": 40,
        "chief": serde_json::to_value(&input.chief).unwrap(),
        "deputy": serde_json::to_value(&input.deputy).unwrap(),
        "chief_config": serde_json::to_value(chief_config).unwrap(),
        "deputy_config": serde_json::to_value(deputy_config).unwrap()
    });

    ws.send(Message::Text(msg.to_string().into()))
        .await
        .expect("send extract_drag message");

    // Drag runs as a background job — skip progress/heartbeat until result arrives.
    let resp = recv_until_type(&mut ws, "drag_result", DRAG_TIMEOUT_SECS).await;

    assert_eq!(resp["type"], "drag_result", "got: {resp}");
    assert_eq!(resp["request_id"], 40);
    let da_dot = resp["drag"]["da_dot"].as_f64().unwrap();
    assert!(
        da_dot != 0.0,
        "da_dot should be non-zero with different configs (500 kg vs 12 kg), got {da_dot}"
    );
}

// ===========================================================================
// Validation (Validate)
// ===========================================================================

/// Full roundtrip: plan a mission from example data, validate via nyx.
///
/// Loads `examples/validate.json` and plans the mission using the analytical
/// pipeline (the same code the browser runs via WASM). The planned mission
/// is then sent to the WS Validate endpoint for nyx full-physics comparison.
/// Verifies that a `ValidationResult` with meaningful position errors is returned.
#[tokio::test]
#[ignore] // nyx full-physics propagation (~5-10s)
async fn validate_mission_roundtrip() {
    let url = start_test_server_full().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    // Load PipelineInput from the example JSON fixture.
    let input: PipelineInput = serde_json::from_reader(
        std::fs::File::open("../examples/validate.json").expect("open examples/validate.json"),
    )
    .expect("parse validate.json as PipelineInput");

    // Plan mission analytically — same pipeline as WASM/CLI.
    let output = rpo_nyx::pipeline::execute_mission(&input).expect("execute_mission");

    // Extract spacecraft configs from SpacecraftChoice.
    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    // Build the Validate WS message with the planned mission.
    let msg = json!({
        "type": "validate",
        "request_id": 20,
        "mission": serde_json::to_value(&output.mission).unwrap(),
        "chief": serde_json::to_value(&input.chief).unwrap(),
        "deputy": serde_json::to_value(&input.deputy).unwrap(),
        "chief_config": serde_json::to_value(chief_config).unwrap(),
        "deputy_config": serde_json::to_value(deputy_config).unwrap(),
        "samples_per_leg": 2
    });

    ws.send(Message::Text(msg.to_string().into()))
        .await
        .expect("send validate message");

    // Collect result — skip progress and heartbeat messages.
    let resp = recv_until_type(&mut ws, "validation_result", VALIDATE_TIMEOUT_SECS).await;

    assert_eq!(resp["type"], "validation_result", "got: {resp}");
    assert_eq!(resp["request_id"], 20);
    let report = &resp["report"];
    assert!(report["leg_points"].is_array(), "should have per-leg validation data");
    assert!(
        report["max_position_error_km"].as_f64().unwrap() > 0.0,
        "max position error should be positive (analytical vs nyx)"
    );
    assert!(
        report["mean_position_error_km"].as_f64().unwrap() > 0.0,
        "mean position error should be positive"
    );
}

// ===========================================================================
// Monte Carlo (RunMc)
// ===========================================================================

/// Full roundtrip: plan a mission, run a 3-sample MC ensemble via nyx.
///
/// Uses `examples/mc.json` for realistic states and configs, but overrides
/// to minimal settings: 3 samples, open-loop mode, J2 propagator, seed 42.
#[tokio::test]
#[ignore] // nyx full-physics propagation (~30-60s)
async fn mc_ensemble_roundtrip() {
    let url = start_test_server_full().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    // Load PipelineInput from the MC example fixture.
    let input: PipelineInput = serde_json::from_reader(
        std::fs::File::open("../examples/mc.json").expect("open examples/mc.json"),
    )
    .expect("parse mc.json as PipelineInput");

    // Plan mission analytically.
    let output = rpo_nyx::pipeline::execute_mission(&input).expect("execute_mission");

    // Extract spacecraft configs.
    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    // Build the RunMc WS message — minimal settings for speed.
    let msg = json!({
        "type": "run_mc",
        "request_id": 30,
        "mission": serde_json::to_value(&output.mission).unwrap(),
        "chief": serde_json::to_value(&input.chief).unwrap(),
        "deputy": serde_json::to_value(&input.deputy).unwrap(),
        "chief_config": serde_json::to_value(chief_config).unwrap(),
        "deputy_config": serde_json::to_value(deputy_config).unwrap(),
        "mission_config": serde_json::to_value(&input.config).unwrap(),
        "propagator": "j2",
        "monte_carlo": {
            "num_samples": 3,
            "dispersions": {},
            "mode": "open_loop",
            "seed": 42,
            "trajectory_steps": 10
        }
    });

    ws.send(Message::Text(msg.to_string().into()))
        .await
        .expect("send run_mc message");

    // Collect result — longer timeout for MC ensemble.
    let resp = recv_until_type(&mut ws, "monte_carlo_result", MC_TIMEOUT_SECS).await;

    assert_eq!(resp["type"], "monte_carlo_result", "got: {resp}");
    assert_eq!(resp["request_id"], 30);
    let report = &resp["report"];
    assert!(report["statistics"].is_object(), "should have ensemble statistics");
    assert!(report["samples"].is_array(), "should have per-sample results");
    let samples = report["samples"].as_array().unwrap();
    assert_eq!(samples.len(), 3, "should have 3 MC samples");
    assert!(
        report["statistics"]["convergence_rate"].as_f64().unwrap() > 0.0,
        "at least some samples should converge"
    );
}

// ===========================================================================
// Cancellation (Cancel)
// ===========================================================================

/// Cancel with no active background job — server replies `cancelled`,
/// then the connection remains alive and can process further messages.
#[tokio::test]
async fn cancel_without_active_job() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "cancel",
            "request_id": 99
        }),
    )
    .await;

    assert_eq!(resp["type"], "cancelled", "got: {resp}");
    assert_eq!(resp["request_id"], 99);

    // Connection should still be alive — send a transfer to verify.
    let (chief, deputy) = proximity_states();
    let resp = send_recv(
        &mut ws,
        json!({
            "type": "compute_transfer",
            "request_id": 100,
            "chief": chief,
            "deputy": deputy,
            "perch": { "v_bar": { "along_track_km": 1.0 } },
            "proximity": { "roe_threshold": 0.005 },
            "lambert_tof_s": 3600.0,
            "lambert_config": { "direction": "auto", "revolutions": 0 }
        }),
    )
    .await;

    assert_eq!(resp["type"], "transfer_result", "got: {resp}");
    assert_eq!(resp["request_id"], 100);
}

/// Start a Validate job then immediately cancel it.
///
/// Cancellation is cooperative: the job may complete before the cancel flag is
/// checked. Both `cancelled` and `validation_result` are valid outcomes.
/// The key invariant is that the server returns a response and does not hang.
#[tokio::test]
#[ignore] // nyx full-physics propagation (cancelled quickly)
async fn cancel_active_validation() {
    let url = start_test_server_full().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    let input: PipelineInput = serde_json::from_reader(
        std::fs::File::open("../examples/validate.json").expect("open examples/validate.json"),
    )
    .expect("parse validate.json as PipelineInput");

    let output = rpo_nyx::pipeline::execute_mission(&input).expect("execute_mission");
    let chief_config = input.chief_config.unwrap_or_default().resolve();
    let deputy_config = input.deputy_config.unwrap_or_default().resolve();

    // Send Validate — starts a background nyx propagation job.
    let validate_msg = json!({
        "type": "validate",
        "request_id": 50,
        "mission": serde_json::to_value(&output.mission).unwrap(),
        "chief": serde_json::to_value(&input.chief).unwrap(),
        "deputy": serde_json::to_value(&input.deputy).unwrap(),
        "chief_config": serde_json::to_value(chief_config).unwrap(),
        "deputy_config": serde_json::to_value(deputy_config).unwrap(),
        "samples_per_leg": 2
    });
    ws.send(Message::Text(validate_msg.to_string().into()))
        .await
        .expect("send validate message");

    // Immediately cancel — the job may already be running.
    ws.send(
        Message::Text(
            json!({ "type": "cancel", "request_id": 50 })
                .to_string()
                .into(),
        ),
    )
    .await
    .expect("send cancel message");

    // Poll until we get a terminal response (cancelled or validation_result).
    // Progress and heartbeat messages are skipped.
    let deadline = std::time::Instant::now() + CANCEL_TIMEOUT;
    let mut got_cancelled = false;
    let mut got_result = false;
    loop {
        let remaining = deadline.saturating_duration_since(std::time::Instant::now());
        if remaining.is_zero() {
            break;
        }
        match tokio::time::timeout(remaining, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                let val: Value = serde_json::from_str(&text).unwrap();
                match val["type"].as_str().unwrap_or("") {
                    "cancelled" => {
                        got_cancelled = true;
                        break;
                    }
                    "validation_result" => {
                        got_result = true;
                        break;
                    }
                    "progress" | "heartbeat" => continue,
                    other => panic!("unexpected message type: {other}"),
                }
            }
            Ok(Some(Ok(Message::Close(_)))) | Ok(None) => break,
            Ok(Some(Err(e))) => panic!("ws error: {e}"),
            Err(_) => break, // timeout
            Ok(Some(Ok(_))) => continue, // ping/pong
        }
    }

    assert!(
        got_cancelled || got_result,
        "expected either 'cancelled' or 'validation_result' within 15s"
    );
}

// ===========================================================================
// Error handling
// ===========================================================================

/// Send malformed JSON — server responds with `error` and `invalid_input` code.
#[tokio::test]
async fn malformed_json_error() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    ws.send(Message::Text("not valid json".into()))
        .await
        .unwrap();

    let resp: Value = {
        let timeout = ERROR_RECV_TIMEOUT;
        match tokio::time::timeout(timeout, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => serde_json::from_str(&text).unwrap(),
            other => panic!("Expected text response, got: {other:?}"),
        }
    };

    assert_eq!(resp["type"], "error", "got: {resp}");
    assert_eq!(resp["code"], "invalid_input");
    assert!(resp["request_id"].is_null(), "connection-level error should have null request_id");
}

/// Send a binary WebSocket frame — server responds with `error` and `invalid_input` code.
#[tokio::test]
async fn binary_frame_error() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    ws.send(Message::Binary(vec![0, 1, 2].into()))
        .await
        .unwrap();

    let resp: Value = {
        let timeout = ERROR_RECV_TIMEOUT;
        match tokio::time::timeout(timeout, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => serde_json::from_str(&text).unwrap(),
            other => panic!("Expected text response, got: {other:?}"),
        }
    };

    assert_eq!(resp["type"], "error", "got: {resp}");
    assert_eq!(resp["code"], "invalid_input");
}

/// Sending a `compute_transfer` message with missing required fields must
/// produce a structured `error` response rather than a server crash or hang.
///
/// serde fails to deserialize the full `ClientMessage` (including `request_id`),
/// so the server cannot echo a request_id and returns `null`.
#[tokio::test]
async fn missing_required_fields() {
    let url = start_test_server().await;
    let (mut ws, _) = connect_async(&url).await.unwrap();

    let resp = send_recv(
        &mut ws,
        json!({
            "type": "compute_transfer",
            "request_id": 60
            // missing: chief, deputy, perch, proximity, lambert_tof_s, lambert_config
        }),
    )
    .await;

    assert_eq!(resp["type"], "error", "got: {resp}");
    assert_eq!(resp["code"], "invalid_input");
    // serde failed to parse the full message, so no request_id could be extracted.
    assert!(
        resp["request_id"].is_null(),
        "request_id should be null when serde fails to parse: got {}",
        resp["request_id"]
    );
}

/// All `ServerErrorCode` variants must serialize to snake_case strings.
#[test]
fn error_codes_serialize_to_snake_case() {
    use rpo_api::protocol::ServerErrorCode;

    let cases = [
        (ServerErrorCode::LambertFailure, "lambert_failure"),
        (ServerErrorCode::NyxBridgeError, "nyx_bridge_error"),
        (ServerErrorCode::ValidationError, "validation_error"),
        (ServerErrorCode::MonteCarloError, "monte_carlo_error"),
        (ServerErrorCode::InvalidInput, "invalid_input"),
        (ServerErrorCode::Cancelled, "cancelled"),
    ];

    for (code, expected) in &cases {
        let json = serde_json::to_string(code).unwrap();
        assert_eq!(
            json,
            format!("\"{expected}\""),
            "ServerErrorCode::{code:?} should serialize to \"{expected}\""
        );
    }
}

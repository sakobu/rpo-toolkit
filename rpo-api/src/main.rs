//! RPO API Server — WebSocket backend for the R3F mission planner frontend.
//!
//! Loads the ANISE almanac at startup, then serves a WebSocket endpoint at `/ws`
//! and a health check at `/health`. Single-user desktop tool: one connection
//! at a time, one mission at a time.

use std::sync::Arc;

use axum::extract::ws::WebSocketUpgrade;
use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use tower_http::cors::CorsLayer;

/// Shared application state.
#[derive(Clone)]
struct AppState {
    /// Preloaded ANISE almanac (cached after first download).
    almanac: Arc<anise::prelude::Almanac>,
}

/// WebSocket upgrade handler.
async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| rpo_api::ws::handle_ws(socket, state.almanac))
}

/// Health check endpoint.
async fn health() -> &'static str {
    "ok"
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    // Parse CLI args
    let bind = std::env::var("RPO_BIND").unwrap_or_else(|_| "127.0.0.1".into());
    let port = std::env::var("RPO_PORT")
        .ok()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(3001);

    // Load almanac
    tracing::info!("Loading almanac (may download on first run)...");
    let almanac = rpo_nyx::nyx_bridge::load_full_almanac()
        .map_err(|e| format!("failed to load almanac: {e}"))?;
    tracing::info!("Almanac loaded");

    let state = AppState { almanac };

    // Build router
    let app = Router::new()
        .route("/ws", get(ws_handler))
        .route("/health", get(health))
        .layer(CorsLayer::permissive())
        .with_state(state);

    // Bind and serve
    let addr = format!("{bind}:{port}");
    tracing::info!("Listening on {addr}");
    let listener = tokio::net::TcpListener::bind(&addr)
        .await
        .map_err(|e| format!("failed to bind {addr}: {e}"))?;

    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await?;

    Ok(())
}

/// Wait for SIGINT or SIGTERM for graceful shutdown.
async fn shutdown_signal() {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
    };

    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to install SIGTERM handler")
            .recv()
            .await;
    };

    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        () = ctrl_c => {},
        () = terminate => {},
    }

    tracing::info!("Shutting down...");
}

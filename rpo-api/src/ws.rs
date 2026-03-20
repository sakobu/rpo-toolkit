//! WebSocket upgrade handler and per-connection message loop.
//!
//! Each WebSocket connection spawns a single tokio task that uses `tokio::select!`
//! to concurrently handle incoming messages and background job progress/results.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use axum::extract::ws::{Message, WebSocket};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use rpo_core::mission::types::WaypointMission;

use crate::error::ApiError;
use crate::handlers;
use crate::protocol::{ClientMessage, ProgressUpdate, ServerMessage};

/// Active background job state.
struct ActiveJob {
    /// The `request_id` this job is associated with.
    request_id: u64,
    /// Cancel flag — set to true to request cooperative cancellation.
    cancel: Arc<AtomicBool>,
    /// Join handle for the spawned blocking task.
    _handle: JoinHandle<()>,
}

/// Result from a background job, sent via channel.
enum JobResult {
    /// Validation completed.
    Validation {
        request_id: u64,
        result: Result<Box<rpo_core::mission::ValidationReport>, ApiError>,
    },
    /// Monte Carlo completed.
    MonteCarlo {
        request_id: u64,
        result: Result<Box<rpo_core::mission::MonteCarloReport>, ApiError>,
    },
    /// Drag extraction completed.
    Drag {
        request_id: u64,
        result: Result<rpo_core::propagation::DragConfig, ApiError>,
    },
}

/// Handle a WebSocket connection.
///
/// Runs a `tokio::select!` loop that processes:
/// 1. Incoming WebSocket messages from the client
/// 2. Progress updates from background tasks
/// 3. Final results from background tasks
pub async fn handle_ws(mut ws: WebSocket, almanac: Arc<Almanac>) {
    let (progress_tx, mut progress_rx) = mpsc::channel::<ProgressUpdate>(64);
    let (result_tx, mut result_rx) = mpsc::channel::<JobResult>(4);
    let mut active_job: Option<ActiveJob> = None;
    let mut last_mission: Option<WaypointMission> = None;

    loop {
        tokio::select! {
            // Incoming WebSocket message
            msg = ws.recv() => {
                let msg = match msg {
                    Some(Ok(msg)) => msg,
                    Some(Err(e)) => {
                        tracing::warn!("WebSocket recv error: {e}");
                        break;
                    }
                    None => break, // Connection closed
                };

                match msg {
                    Message::Text(text) => {
                        handle_text_message(
                            &text,
                            &mut ws,
                            &almanac,
                            &mut active_job,
                            &mut last_mission,
                            &progress_tx,
                            &result_tx,
                        ).await;
                    }
                    Message::Close(_) => break,
                    _ => {
                        let err = ServerMessage::Error {
                            request_id: None,
                            code: crate::protocol::ErrorCode::InvalidInput,
                            message: "expected text frame".into(),
                            detail: None,
                        };
                        send_message(&mut ws, &err).await;
                    }
                }
            }

            // Progress from background job
            Some(progress) = progress_rx.recv() => {
                if let Some(job) = &active_job {
                    let msg = ServerMessage::Progress {
                        request_id: job.request_id,
                        phase: progress.phase,
                        detail: progress.detail,
                        fraction: progress.fraction,
                    };
                    send_message(&mut ws, &msg).await;
                }
            }

            // Result from background job
            Some(result) = result_rx.recv() => {
                let msg = match result {
                    JobResult::Validation { request_id, result } => {
                        match result {
                            Ok(report) => ServerMessage::ValidationResult { request_id, report },
                            Err(ApiError::Cancelled) => ServerMessage::Cancelled { request_id },
                            Err(e) => e.to_server_message(Some(request_id)),
                        }
                    }
                    JobResult::MonteCarlo { request_id, result } => {
                        match result {
                            Ok(report) => ServerMessage::MonteCarloResult { request_id, report },
                            Err(ApiError::Cancelled) => ServerMessage::Cancelled { request_id },
                            Err(e) => e.to_server_message(Some(request_id)),
                        }
                    }
                    JobResult::Drag { request_id, result } => {
                        match result {
                            Ok(drag) => ServerMessage::DragResult { request_id, drag },
                            Err(e) => e.to_server_message(Some(request_id)),
                        }
                    }
                };
                send_message(&mut ws, &msg).await;
                active_job = None;
            }
        }
    }

    // Cleanup: cancel any active background job
    if let Some(job) = active_job.take() {
        job.cancel.store(true, Ordering::Relaxed);
    }

    tracing::info!("WebSocket connection closed");
}

/// Process a text WebSocket message.
#[allow(clippy::too_many_lines)]
async fn handle_text_message(
    text: &str,
    ws: &mut WebSocket,
    almanac: &Arc<Almanac>,
    active_job: &mut Option<ActiveJob>,
    last_mission: &mut Option<WaypointMission>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    result_tx: &mpsc::Sender<JobResult>,
) {
    let client_msg: ClientMessage = match serde_json::from_str(text) {
        Ok(msg) => msg,
        Err(e) => {
            let err = ApiError::InvalidInput(crate::error::InvalidInputError::MalformedJson {
                detail: e.to_string(),
            });
            send_message(ws, &err.to_server_message(None)).await;
            return;
        }
    };

    match client_msg {
        // Inline handlers — respond immediately
        ClientMessage::Classify {
            request_id,
            mission,
        } => {
            let msg = match handlers::handle_classify(&mission) {
                Ok(phase) => ServerMessage::ClassifyResult { request_id, phase },
                Err(e) => e.to_server_message(Some(request_id)),
            };
            send_message(ws, &msg).await;
        }

        ClientMessage::PlanMission {
            request_id,
            mission,
        } => {
            let msg = match handlers::handle_plan(&mission, almanac) {
                Ok(result) => {
                    *last_mission = Some(result.mission.clone());
                    ServerMessage::MissionResult { request_id, result: Box::new(result) }
                }
                Err(e) => {
                    *last_mission = None;
                    e.to_server_message(Some(request_id))
                }
            };
            send_message(ws, &msg).await;
        }

        ClientMessage::MoveWaypoint {
            request_id,
            modified_index,
            mission,
        } => {
            let cached = last_mission.as_ref();
            let msg = match handlers::handle_move_waypoint(&mission, modified_index, cached, almanac) {
                Ok(result) => {
                    *last_mission = Some(result.mission.clone());
                    ServerMessage::MissionResult { request_id, result: Box::new(result) }
                }
                Err(e) => e.to_server_message(Some(request_id)),
            };
            send_message(ws, &msg).await;
        }

        ClientMessage::UpdateConfig {
            request_id,
            mission,
        } => {
            let msg = match handlers::handle_update_config(&mission, almanac) {
                Ok(result) => {
                    *last_mission = Some(result.mission.clone());
                    ServerMessage::MissionResult { request_id, result: Box::new(result) }
                }
                Err(e) => {
                    *last_mission = None;
                    e.to_server_message(Some(request_id))
                }
            };
            send_message(ws, &msg).await;
        }

        // Background handlers — spawn blocking task
        ClientMessage::ExtractDrag {
            request_id,
            mission,
        } => {
            cancel_active_job(active_job);
            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let tx = result_tx.clone();
            let handle = tokio::task::spawn_blocking(move || {
                let result = handlers::handle_extract_drag(&mission, &alm);
                let _ = tx.blocking_send(JobResult::Drag { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel,
                _handle: handle,
            });
        }

        ClientMessage::Validate {
            request_id,
            mission,
            samples_per_leg,
            auto_drag,
        } => {
            cancel_active_job(active_job);
            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let ptx = progress_tx.clone();
            let rtx = result_tx.clone();
            let cancel_clone = cancel.clone();
            let samples = samples_per_leg.unwrap_or(50);
            let handle = tokio::task::spawn_blocking(move || {
                let result = handlers::handle_validate(
                    &mission, &alm, &ptx, &cancel_clone, samples, auto_drag,
                );
                let result = result.map(Box::new);
                let _ = rtx.blocking_send(JobResult::Validation { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel,
                _handle: handle,
            });
        }

        ClientMessage::RunMC {
            request_id,
            mission,
            auto_drag,
        } => {
            cancel_active_job(active_job);
            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let ptx = progress_tx.clone();
            let rtx = result_tx.clone();
            let cancel_clone = cancel.clone();
            let handle = tokio::task::spawn_blocking(move || {
                let result =
                    handlers::handle_mc(&mission, &alm, &ptx, &cancel_clone, auto_drag);
                let result = result.map(Box::new);
                let _ = rtx.blocking_send(JobResult::MonteCarlo { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel,
                _handle: handle,
            });
        }

        ClientMessage::Cancel { request_id } => {
            if let Some(job) = &active_job {
                let cancelled_id = request_id.unwrap_or(job.request_id);
                job.cancel.store(true, Ordering::Relaxed);
                send_message(
                    ws,
                    &ServerMessage::Cancelled {
                        request_id: cancelled_id,
                    },
                )
                .await;
                *active_job = None;
            }
        }
    }
}

/// Cancel the active background job, if any.
fn cancel_active_job(active_job: &mut Option<ActiveJob>) {
    if let Some(job) = active_job.take() {
        job.cancel.store(true, Ordering::Relaxed);
    }
}

/// Serialize and send a `ServerMessage` over the WebSocket.
async fn send_message(ws: &mut WebSocket, msg: &ServerMessage) {
    match serde_json::to_string(msg) {
        Ok(json) => {
            if let Err(e) = ws.send(Message::Text(json.into())).await {
                tracing::warn!("WebSocket send error: {e}");
            }
        }
        Err(e) => {
            tracing::error!("Failed to serialize ServerMessage: {e}");
        }
    }
}

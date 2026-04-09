//! Stateless WebSocket handler — dispatches 4 nyx operations + cancel.

use crate::error::ServerError;
use crate::handlers;
use crate::handlers::mc::McJobInput;
use crate::handlers::validate::ValidateJobInput;
use crate::protocol::{ClientMessage, ProgressUpdate, ServerMessage};
use anise::prelude::Almanac;
use axum::extract::ws::{Message, WebSocket};
use rpo_core::mission::monte_carlo::types::MonteCarloReport;
use rpo_core::mission::types::ValidationReport;
use rpo_core::propagation::propagator::DragConfig;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{interval, Duration};
use tracing::{debug, error, info, warn};

/// Progress updates are fire-and-forget (`try_send`); buffer handles short bursts
/// without blocking the sender. Handlers send ~3 messages per job; 32 provides
/// ample margin without wasting memory.
const PROGRESS_CHANNEL_CAPACITY: usize = 32;

/// Only one background job is active at a time; capacity > 1 handles the edge
/// case where a result arrives between cancel and result-channel read.
const RESULT_CHANNEL_CAPACITY: usize = 4;

/// WebSocket proxies typically time out at 60s; 30s keeps the connection alive
/// with margin. Heartbeats are only sent while a background job is active.
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(30);

/// Tracks a spawned background job for cancellation and result collection.
struct ActiveJob {
    /// Correlation ID from the client request.
    request_id: u64,
    /// Cooperative cancellation flag — set to `true` to request stop.
    cancel: Arc<AtomicBool>,
    /// Handle to the spawned blocking task (kept alive).
    _handle: JoinHandle<()>,
}

/// Result from a completed background job, sent back via channel.
enum JobResult {
    /// Drag extraction completed.
    Drag {
        request_id: u64,
        result: Result<DragConfig, ServerError>,
    },
    /// Validation completed.
    Validation {
        request_id: u64,
        result: Result<Box<ValidationReport>, ServerError>,
    },
    /// Monte Carlo completed.
    MonteCarlo {
        request_id: u64,
        result: Result<Box<MonteCarloReport>, ServerError>,
    },
}

/// Main per-connection WebSocket handler.
///
/// Runs the message loop until the client disconnects. Holds no persistent
/// state — each incoming message is self-contained.
pub async fn handle_ws(mut ws: WebSocket, almanac: Arc<Almanac>) {
    info!("WebSocket connected");

    let (progress_tx, mut progress_rx) = mpsc::channel::<ProgressUpdate>(PROGRESS_CHANNEL_CAPACITY);
    let (result_tx, mut result_rx) = mpsc::channel::<JobResult>(RESULT_CHANNEL_CAPACITY);
    let mut active_job: Option<ActiveJob> = None;
    let mut heartbeat = interval(HEARTBEAT_INTERVAL);
    // tokio::time::interval fires the first tick immediately; this is harmless
    // because heartbeats are only sent when active_job.is_some() (which is false
    // on a fresh connection).
    let mut heartbeat_seq: u64 = 0;

    loop {
        tokio::select! {
            // ---- Incoming client message ----
            msg = ws.recv() => {
                let Some(msg) = msg else {
                    info!("WebSocket disconnected");
                    break;
                };
                let msg = match msg {
                    Ok(m) => m,
                    Err(e) => {
                        debug!("WebSocket recv error: {e}");
                        break;
                    }
                };
                match msg {
                    Message::Text(text) => {
                        handle_text_message(
                            &text,
                            &mut ws,
                            &almanac,
                            &progress_tx,
                            &result_tx,
                            &mut active_job,
                        ).await;
                    }
                    Message::Binary(_) => {
                        let err = ServerError::MalformedJson {
                            serde_message: "binary frames are not supported; send JSON text".to_owned(),
                        };
                        send_message(&mut ws, err.to_server_message(None)).await;
                    }
                    Message::Close(_) => break,
                    _ => {} // Ping/Pong handled by axum
                }
            }

            // ---- Progress from background job ----
            Some(progress) = progress_rx.recv() => {
                if let Some(job) = &active_job {
                    let msg = ServerMessage::Progress {
                        request_id: job.request_id,
                        phase: progress.phase,
                        detail: progress.detail,
                        fraction: progress.fraction,
                    };
                    send_message(&mut ws, msg).await;
                }
            }

            // ---- Result from background job ----
            Some(result) = result_rx.recv() => {
                let response = match result {
                    JobResult::Drag { request_id, result } => match result {
                        Ok(drag) => ServerMessage::DragResult { request_id, drag },
                        Err(ServerError::Cancelled) => ServerMessage::Cancelled { request_id },
                        Err(e) => e.to_server_message(Some(request_id)),
                    },
                    JobResult::Validation { request_id, result } => match result {
                        Ok(report) => ServerMessage::ValidationResult { request_id, report },
                        Err(ServerError::Cancelled) => ServerMessage::Cancelled { request_id },
                        Err(e) => e.to_server_message(Some(request_id)),
                    },
                    JobResult::MonteCarlo { request_id, result } => match result {
                        Ok(report) => ServerMessage::MonteCarloResult { request_id, report },
                        Err(ServerError::Cancelled) => ServerMessage::Cancelled { request_id },
                        Err(e) => e.to_server_message(Some(request_id)),
                    },
                };
                send_message(&mut ws, response).await;
                active_job = None;
                heartbeat_seq = 0;
                heartbeat.reset();
            }

            // ---- Heartbeat during long operations ----
            _ = heartbeat.tick() => {
                if active_job.is_some() {
                    heartbeat_seq += 1;
                    send_message(&mut ws, ServerMessage::Heartbeat { seq: heartbeat_seq }).await;
                }
            }
        }
    }

    // Cancel any active job on disconnect
    if let Some(job) = active_job.take() {
        job.cancel.store(true, Ordering::Relaxed);
        info!("Cancelled active job on disconnect");
    }
}

/// Dispatch a JSON text message to the appropriate handler.
#[allow(clippy::too_many_lines)]
async fn handle_text_message(
    text: &str,
    ws: &mut WebSocket,
    almanac: &Arc<Almanac>,
    progress_tx: &mpsc::Sender<ProgressUpdate>,
    result_tx: &mpsc::Sender<JobResult>,
    active_job: &mut Option<ActiveJob>,
) {
    let msg: ClientMessage = match serde_json::from_str(text) {
        Ok(m) => m,
        Err(e) => {
            let err = ServerError::MalformedJson {
                serde_message: e.to_string(),
            };
            send_message(ws, err.to_server_message(None)).await;
            return;
        }
    };

    match msg {
        // ---- Synchronous: ComputeTransfer (~100ms) ----
        ClientMessage::ComputeTransfer {
            request_id,
            chief,
            deputy,
            perch,
            proximity,
            lambert_tof_s,
            lambert_config,
        } => {
            let response = match handlers::handle_compute_transfer(
                chief, deputy, perch, proximity, lambert_tof_s, lambert_config,
            ) {
                Ok(result) => ServerMessage::TransferResult {
                    request_id,
                    result: Box::new(result),
                },
                Err(e) => e.to_server_message(Some(request_id)),
            };
            send_message(ws, response).await;
        }

        // ---- Background: ExtractDrag (~3s) ----
        // Cancel flag is stored on ActiveJob but handle_extract_drag does not
        // check it — drag extraction is short (~3s) and CPU-bound, so
        // cancellation is best-effort.
        ClientMessage::ExtractDrag {
            request_id,
            chief,
            deputy,
            chief_config,
            deputy_config,
        } => {
            cancel_active_job(active_job);
            let almanac = Arc::clone(almanac);
            let tx = result_tx.clone();
            let cancel = Arc::new(AtomicBool::new(false));
            let cancel_clone = Arc::clone(&cancel);
            let handle = tokio::task::spawn_blocking(move || {
                let result =
                    handlers::handle_extract_drag(&chief, &deputy, &chief_config, &deputy_config, &almanac);
                let _ = tx.blocking_send(JobResult::Drag { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel: cancel_clone,
                _handle: handle,
            });
        }

        // ---- Background: Validate (seconds) ----
        ClientMessage::Validate {
            request_id,
            mission,
            chief,
            deputy,
            chief_config,
            deputy_config,
            samples_per_leg,
            cola_burns,
            analytical_cola,
            cola_target_distance_km,
        } => {
            cancel_active_job(active_job);
            let almanac = Arc::clone(almanac);
            let ptx = progress_tx.clone();
            let tx = result_tx.clone();
            let cancel = Arc::new(AtomicBool::new(false));
            let cancel_clone = Arc::clone(&cancel);

            let job_input = ValidateJobInput {
                mission,
                chief,
                deputy,
                chief_config,
                deputy_config,
                samples_per_leg,
                cola_burn_inputs: cola_burns,
                analytical_cola,
                cola_target_distance_km,
            };

            let handle = tokio::task::spawn_blocking(move || {
                let result = handlers::handle_validate(job_input, &almanac, &ptx, &cancel);
                let result = result.map(Box::new);
                let _ = tx.blocking_send(JobResult::Validation { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel: cancel_clone,
                _handle: handle,
            });
        }

        // ---- Background: Monte Carlo (minutes) ----
        ClientMessage::RunMc {
            request_id,
            mission,
            chief,
            deputy,
            chief_config,
            deputy_config,
            mission_config,
            propagator,
            drag_config,
            monte_carlo,
            covariance_report,
        } => {
            cancel_active_job(active_job);
            let almanac = Arc::clone(almanac);
            let ptx = progress_tx.clone();
            let tx = result_tx.clone();
            let cancel = Arc::new(AtomicBool::new(false));
            let cancel_clone = Arc::clone(&cancel);
            let handle = tokio::task::spawn_blocking(move || {
                let input = McJobInput {
                    mission: &mission,
                    chief: &chief,
                    deputy: &deputy,
                    chief_config: &chief_config,
                    deputy_config: &deputy_config,
                    mission_config: &mission_config,
                    propagator_choice: propagator,
                    drag_config,
                    mc_config: &monte_carlo,
                    covariance_report: covariance_report.as_ref(),
                };
                let result = handlers::handle_mc(&input, &almanac, &ptx, &cancel);
                let result = result.map(Box::new);
                let _ = tx.blocking_send(JobResult::MonteCarlo { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel: cancel_clone,
                _handle: handle,
            });
        }

        // ---- Cancel ----
        ClientMessage::Cancel { request_id } => {
            if let Some(job) = active_job.take() {
                job.cancel.store(true, Ordering::Relaxed);
            }
            send_message(ws, ServerMessage::Cancelled { request_id }).await;
        }
    }
}

/// Cancel any active background job by setting its cancel flag.
fn cancel_active_job(active_job: &mut Option<ActiveJob>) {
    if let Some(job) = active_job.take() {
        job.cancel.store(true, Ordering::Relaxed);
        warn!(request_id = job.request_id, "Cancelled active job for new request");
    }
}

/// Serialize and send a `ServerMessage` over the WebSocket.
async fn send_message(ws: &mut WebSocket, msg: ServerMessage) {
    match serde_json::to_string(&msg) {
        Ok(json) => {
            if let Err(e) = ws.send(Message::Text(json.into())).await {
                error!("Failed to send WebSocket message: {e}");
            }
        }
        Err(e) => {
            error!("Failed to serialize ServerMessage: {e}");
        }
    }
}

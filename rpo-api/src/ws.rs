//! WebSocket upgrade handler and per-connection message loop.
//!
//! Each WebSocket connection spawns a single tokio task that uses `tokio::select!`
//! to concurrently handle incoming messages and background job progress/results.
//! The connection owns a [`Session`] that holds mutable state across messages.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use anise::prelude::Almanac;
use axum::extract::ws::{Message, WebSocket};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{interval, Duration};

use crate::error::ApiError;
use crate::handlers;
use crate::protocol::{ClientMessage, ProgressUpdate, ServerMessage};
use crate::session::Session;

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
    let (progress_tx, mut progress_rx) = mpsc::channel::<ProgressUpdate>(256);
    let (result_tx, mut result_rx) = mpsc::channel::<JobResult>(4);
    let mut active_job: Option<ActiveJob> = None;
    let mut session = Session::default();
    let mut heartbeat = interval(Duration::from_secs(30));
    heartbeat.tick().await; // consume the immediate first tick
    let mut heartbeat_seq: u64 = 0;

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
                            &mut session,
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
                            Ok(drag) => {
                                session.store_drag(drag);
                                ServerMessage::DragResult { request_id, drag }
                            }
                            Err(e) => e.to_server_message(Some(request_id)),
                        }
                    }
                };
                send_message(&mut ws, &msg).await;
                active_job = None;
                heartbeat_seq = 0;
            }

            // Heartbeat during long-running operations
            _ = heartbeat.tick(), if active_job.is_some() => {
                heartbeat_seq += 1;
                let msg = ServerMessage::Heartbeat { seq: heartbeat_seq };
                send_message(&mut ws, &msg).await;
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
    session: &mut Session,
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
        // ---- State-setting messages ----

        ClientMessage::SetStates {
            request_id,
            chief,
            deputy,
        } => {
            cancel_active_job(active_job);
            session.set_states(chief, deputy);
            let msg = ServerMessage::StateUpdated {
                request_id,
                updated: vec!["chief".into(), "deputy".into()],
                invalidated: vec!["transfer".into(), "drag_config".into(), "mission".into()],
            };
            send_message(ws, &msg).await;
        }

        ClientMessage::SetSpacecraft {
            request_id,
            chief_config,
            deputy_config,
        } => {
            let updated_fields = session.update_spacecraft(chief_config, deputy_config);
            let invalidated = if updated_fields.is_empty() {
                vec![]
            } else {
                vec!["drag_config".into()]
            };
            let msg = ServerMessage::StateUpdated {
                request_id,
                updated: updated_fields.into_iter().map(Into::into).collect(),
                invalidated,
            };
            send_message(ws, &msg).await;
        }

        ClientMessage::Reset { request_id } => {
            cancel_active_job(active_job);
            session.reset();
            let msg = ServerMessage::StateUpdated {
                request_id,
                updated: vec![],
                invalidated: vec!["all".into()],
            };
            send_message(ws, &msg).await;
        }

        // ---- Inline compute handlers ----

        ClientMessage::Classify { request_id } => {
            match handlers::handle_classify(session) {
                Ok(phase) => {
                    send_message(ws, &ServerMessage::ClassifyResult { request_id, phase }).await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::ComputeTransfer {
            request_id,
            perch,
            lambert_tof_s,
            lambert_config,
        } => {
            // Apply optional overrides to session (transfer/mission invalidation
            // is handled by store_transfer inside handle_compute_transfer)
            if let Some(p) = perch {
                session.perch = p;
            }
            if let Some(tof) = lambert_tof_s {
                session.lambert_tof_s = tof;
            }
            if let Some(lc) = lambert_config {
                session.lambert_config = lc;
            }

            match handlers::handle_compute_transfer(session) {
                Ok(result) => {
                    send_message(
                        ws,
                        &ServerMessage::TransferResult {
                            request_id,
                            result: Box::new(result),
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::SetWaypoints {
            request_id,
            waypoints,
            changed_from,
        } => {
            // Extract cached mission before set_waypoints clears it
            let cached_mission = session.mission.take();
            session.set_waypoints(waypoints);
            match handlers::handle_set_waypoints(session, changed_from, cached_mission.as_ref()) {
                Ok(result) => {
                    send_message(
                        ws,
                        &ServerMessage::PlanResult {
                            request_id,
                            result: Box::new(result),
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::UpdateConfig {
            request_id,
            config,
            propagator,
            proximity,
            navigation_accuracy,
            maneuver_uncertainty,
        } => {
            // Resolve propagator toggle if present
            let mut propagator_changed = false;
            if let Some(toggle) = propagator {
                match handlers::resolve_propagator_toggle(toggle, session.drag_config) {
                    Ok(choice) => {
                        session.set_propagator(choice);
                        propagator_changed = true;
                    }
                    Err(e) => {
                        send_message(ws, &e.to_server_message(Some(request_id))).await;
                        return;
                    }
                }
            }

            let update = handlers::ConfigUpdate {
                config,
                propagator_changed,
                proximity,
                navigation_accuracy,
                maneuver_uncertainty,
            };

            match handlers::handle_update_config(session, update) {
                Ok(Some(result)) => {
                    send_message(
                        ws,
                        &ServerMessage::PlanResult {
                            request_id,
                            result: Box::new(result),
                        },
                    )
                    .await;
                }
                Ok(None) => {
                    send_message(
                        ws,
                        &ServerMessage::StateUpdated {
                            request_id,
                            updated: vec!["config".into()],
                            invalidated: vec![],
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetTrajectory {
            request_id,
            legs,
            max_points,
        } => {
            match handlers::handle_get_trajectory(session, legs.as_deref(), max_points) {
                Ok(leg_data) => {
                    send_message(
                        ws,
                        &ServerMessage::TrajectoryData {
                            request_id,
                            legs: leg_data,
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetFreeDriftTrajectory {
            request_id,
            legs,
            max_points,
        } => {
            match handlers::handle_get_free_drift(session, legs.as_deref(), max_points) {
                Ok(response) => {
                    send_message(
                        ws,
                        &ServerMessage::FreeDriftData {
                            request_id,
                            legs: response.legs,
                            analyses: response.analyses,
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetPoca { request_id, legs } => {
            match handlers::handle_get_poca(session, legs.as_deref()) {
                Ok(points) => {
                    send_message(
                        ws,
                        &ServerMessage::PocaData {
                            request_id,
                            points,
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetCovariance { request_id } => {
            match handlers::handle_get_covariance(session) {
                Ok(report) => {
                    send_message(
                        ws,
                        &ServerMessage::CovarianceData {
                            request_id,
                            report: Box::new(report),
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetEclipse { request_id } => {
            match handlers::handle_get_eclipse(session) {
                Ok(eclipse) => {
                    send_message(
                        ws,
                        &ServerMessage::EclipseData {
                            request_id,
                            transfer: eclipse.transfer,
                            mission: eclipse.mission,
                        },
                    )
                    .await;
                }
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                }
            }
        }

        ClientMessage::GetSession { request_id } => {
            let summary = session.to_summary();
            send_message(
                ws,
                &ServerMessage::SessionState {
                    request_id,
                    summary,
                },
            )
            .await;
        }

        // ---- Background handlers ----

        ClientMessage::ExtractDrag { request_id } => {
            cancel_active_job(active_job);

            // Determine states: use perch states if transfer exists, else original
            let (chief, deputy) = match session.transfer.as_ref() {
                Some(t) => (t.perch_chief.clone(), t.perch_deputy.clone()),
                None => match (session.require_chief(), session.require_deputy()) {
                    (Ok(c), Ok(d)) => (c.clone(), d.clone()),
                    (Err(e), _) | (_, Err(e)) => {
                        send_message(ws, &e.to_server_message(Some(request_id))).await;
                        return;
                    }
                },
            };
            let chief_config = session.chief_config().resolve();
            let deputy_config = session.deputy_config().resolve();

            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let tx = result_tx.clone();
            let handle = tokio::task::spawn_blocking(move || {
                let result =
                    handlers::handle_extract_drag(&chief, &deputy, &chief_config, &deputy_config, &alm);
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
            samples_per_leg,
            auto_drag,
        } => {
            cancel_active_job(active_job);

            let request = match build_validate_request(session, samples_per_leg, auto_drag) {
                Ok(r) => r,
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                    return;
                }
            };

            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let ptx = progress_tx.clone();
            let rtx = result_tx.clone();
            let cancel_clone = cancel.clone();
            let handle = tokio::task::spawn_blocking(move || {
                let result = handlers::handle_validate(request, &alm, &ptx, &cancel_clone);
                let result = result.map(Box::new);
                let _ = rtx.blocking_send(JobResult::Validation { request_id, result });
            });
            *active_job = Some(ActiveJob {
                request_id,
                cancel,
                _handle: handle,
            });
        }

        ClientMessage::RunMc {
            request_id,
            monte_carlo,
            auto_drag,
        } => {
            cancel_active_job(active_job);

            // Store MC config override if provided
            if let Some(mc) = monte_carlo {
                session.set_monte_carlo_config(mc);
            }

            let request = match build_mc_request(session, auto_drag) {
                Ok(r) => r,
                Err(e) => {
                    send_message(ws, &e.to_server_message(Some(request_id))).await;
                    return;
                }
            };

            let cancel = Arc::new(AtomicBool::new(false));
            let alm = almanac.clone();
            let ptx = progress_tx.clone();
            let rtx = result_tx.clone();
            let cancel_clone = cancel.clone();
            let handle = tokio::task::spawn_blocking(move || {
                let result = handlers::handle_mc(request, &alm, &ptx, &cancel_clone);
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

/// Build a [`ValidateRequest`](handlers::ValidateRequest) from session state.
fn build_validate_request(
    session: &Session,
    samples_per_leg: Option<u32>,
    auto_drag: bool,
) -> Result<handlers::ValidateRequest, ApiError> {
    let mission = session.require_mission()?.clone();
    let transfer = session.require_transfer()?.clone();

    Ok(handlers::ValidateRequest {
        mission,
        perch_chief: transfer.perch_chief.clone(),
        perch_deputy: transfer.perch_deputy.clone(),
        chief_config: session.chief_config().resolve(),
        deputy_config: session.deputy_config().resolve(),
        samples_per_leg: samples_per_leg.unwrap_or(50),
        auto_drag,
        waypoints: session.waypoints.clone(),
        config: session.config.clone(),
        propagator: session.resolve_propagation_model(),
        transfer,
    })
}

/// Build an [`McRequest`](handlers::McRequest) from session state.
fn build_mc_request(
    session: &Session,
    auto_drag: bool,
) -> Result<handlers::McRequest, ApiError> {
    let mission = session.require_mission()?.clone();
    let transfer = session.require_transfer()?.clone();
    let mc_config = session.require_monte_carlo_config()?.clone();

    Ok(handlers::McRequest {
        mission,
        perch_chief: transfer.perch_chief.clone(),
        perch_deputy: transfer.perch_deputy.clone(),
        chief_config: session.chief_config().resolve(),
        deputy_config: session.deputy_config().resolve(),
        mc_config,
        mission_config: session.config.clone(),
        propagator: session.resolve_propagation_model(),
        navigation_accuracy: session.navigation_accuracy,
        maneuver_uncertainty: session.maneuver_uncertainty,
        auto_drag,
        waypoints: session.waypoints.clone(),
        transfer,
    })
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

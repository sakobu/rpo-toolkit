//! Handler modules for the 4 server-side operations.

pub mod drag;
pub mod mc;
pub mod transfer;
pub mod validate;

pub use drag::handle_extract_drag;
pub(crate) use mc::handle_mc;
pub use transfer::handle_compute_transfer;
pub(crate) use validate::handle_validate;

use crate::protocol::{ProgressPhase, ProgressUpdate};
use tokio::sync::mpsc;

/// Best-effort progress update — dropped silently if channel is full.
///
/// Used by background handlers (validate, MC) to stream progress to the WS loop.
/// `try_send` is non-blocking: if the channel is full, the update is discarded
/// rather than blocking the computation thread.
pub(crate) fn send_progress(
    tx: &mpsc::Sender<ProgressUpdate>,
    phase: ProgressPhase,
    detail: &str,
    fraction: f64,
) {
    let _ = tx.try_send(ProgressUpdate {
        phase,
        detail: Some(detail.to_owned()),
        fraction: Some(fraction),
    });
}

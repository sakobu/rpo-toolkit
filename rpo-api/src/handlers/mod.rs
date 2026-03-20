//! Request handlers for the WebSocket API.
//!
//! Each handler is a pure function, testable without WebSocket infrastructure.
//! Inline handlers (classify, plan, `move_waypoint`, `update_config`) return immediately.
//! Background handlers (drag, validate, mc) are designed to be spawned on a blocking thread.

pub mod classify;
pub mod drag;
pub mod mc;
pub mod plan;
pub mod validate;

pub use classify::handle_classify;
pub use drag::handle_extract_drag;
pub use mc::handle_mc;
pub use plan::{handle_move_waypoint, handle_plan, handle_update_config};
pub use validate::handle_validate;

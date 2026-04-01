//! Request handlers for the WebSocket API.
//!
//! Each handler is a function that operates on session state, testable without
//! WebSocket infrastructure. Inline handlers (classify, transfer, plan, config)
//! return immediately. Background handlers (drag, validate, mc) are designed to
//! be spawned on a blocking thread.

pub mod classify;
pub mod cola;
pub mod common;
pub mod drag;
pub mod formation;
pub mod free_drift;
pub mod mc;
pub mod poca;
pub mod plan;
pub mod transfer;
pub mod validate;

pub use classify::handle_classify;
pub use cola::handle_run_cola;
pub use common::resolve_propagator_toggle;
pub use drag::handle_extract_drag;
pub use formation::{handle_get_formation_design, handle_get_safe_alternative};
pub use free_drift::handle_get_free_drift;
pub use poca::handle_get_poca;
pub use mc::{handle_mc, McRequest};
pub use plan::{
    handle_get_covariance, handle_get_eclipse, handle_get_trajectory, handle_set_waypoints,
    handle_update_config, ConfigUpdate, EclipseResponse, PlanResponse,
};
pub use transfer::handle_compute_transfer;
pub use validate::{handle_validate, ValidateRequest};

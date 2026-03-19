//! RPO API server library — WebSocket backend for the R3F mission planner.
//!
//! Exposes the WebSocket handler, protocol types, and conversion layers
//! for use by the binary and integration tests.

pub mod convert;
pub mod error;
pub mod handlers;
pub mod protocol;
pub mod ws;

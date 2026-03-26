#![warn(clippy::pedantic)]
#![warn(missing_docs)]

//! RPO API server library — WebSocket backend for the R3F mission planner.
//!
//! Exposes the WebSocket handler, protocol types, and error types
//! for use by the binary and integration tests.

pub mod error;
pub mod handlers;
pub mod protocol;
pub mod session;
pub mod ws;

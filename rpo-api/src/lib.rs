#![warn(clippy::pedantic)]
#![warn(missing_docs)]

//! RPO API server — stateless WebSocket backend for nyx-dependent operations.
//!
//! Handles 4 operations that require nyx-space: Lambert transfer, drag
//! extraction, full-physics validation, and Monte Carlo ensemble. All
//! analytical operations run in the browser via WASM.

pub(crate) mod error;
pub(crate) mod handlers;
pub mod protocol;
pub mod ws;

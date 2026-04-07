//! nyx-space integration for RPO mission planning.
//!
//! Provides full-physics propagation, Lambert solver, nyx validation,
//! and Monte Carlo ensemble analysis on top of `rpo-core`'s analytical engine.
//!
//! # Crate boundary
//!
//! `rpo-core` contains the analytical engine (STMs, targeting, safety, formation
//! design) and compiles to WASM with no AGPL dependencies. This crate wraps
//! nyx-space (AGPL-3.0) for operations that require numerical integration:
//!
//! - **Lambert solver** — Izzo algorithm via nyx
//! - **Validation** — full-physics comparison against analytical predictions
//! - **Monte Carlo** — ensemble nyx propagation with dispersions
//! - **Drag extraction** — DMF differential-drag rates via nyx dynamics
//!
#![warn(clippy::pedantic)]
#![allow(clippy::module_name_repetitions)]
#![warn(missing_docs)]

pub mod lambert;
pub mod monte_carlo;
pub mod nyx_bridge;
pub mod pipeline;
pub mod planning;
pub mod validation;

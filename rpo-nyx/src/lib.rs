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

// Primitives -- direct nyx-space wrappers, no internal dependencies.
pub mod nyx_bridge;
pub mod lambert;

// Utilities -- internal support, used by orchestration layers.
pub(crate) mod statistics;

// Orchestration -- composes primitives + utilities into higher-level flows.
pub mod monte_carlo;
pub mod validation;
pub mod pipeline;

//! WASM bindings for the RPO analytical engine.
//!
//! Thin glue layer exposing rpo-core's analytical functions to JavaScript
//! with auto-generated TypeScript definitions via `tsify-next`.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]
// wasm-bindgen functions take owned values across the WASM boundary;
// clippy's pass-by-value lint does not apply.
#![allow(clippy::needless_pass_by_value)]

pub mod error;
pub mod analysis;
pub mod eclipse;
pub mod enrichment;
pub mod mission;
pub mod planning;
pub mod query;
pub mod safety;

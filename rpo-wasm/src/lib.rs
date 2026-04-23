//! WASM bindings for the RPO analytical engine.
//!
//! Thin glue layer exposing rpo-core's analytical functions to JavaScript
//! with auto-generated TypeScript definitions via `tsify-next`.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]
// Every module in this crate is a wasm-bindgen binding layer; `#[wasm_bindgen]`
// exports cross the WASM ABI boundary, which requires owned values. The lint
// does not apply to binding signatures, so allow it crate-wide here.
#![allow(clippy::needless_pass_by_value)]

pub mod error;
pub mod analysis;
pub mod constants;
pub mod eclipse;
pub mod elements;
pub mod enrichment;
pub mod frames;
pub mod mission;
pub mod planning;
pub mod query;
pub mod safety;

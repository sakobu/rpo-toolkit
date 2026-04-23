//! WASM export for engine policy/physics constants.
//!
//! See `rpo-core::engine_constants` for the rationale and field list.

use wasm_bindgen::prelude::*;

use rpo_core::engine_constants::{
    engine_constants as core_engine_constants, EngineConstants,
};

/// Snapshot the engine policy/physics constants used by the frontend.
///
/// Frontend callers (typically via a module-singleton cache, e.g.
/// `rpo-app/src/wasm/constants.ts`) read this once after WASM init
/// instead of mirroring the values in TypeScript.
///
/// # Returns
/// - [`EngineConstants`] — see that type for the current field list.
///   Add fields there when a frontend mirror would otherwise need to
///   be hand-maintained.
#[must_use]
#[wasm_bindgen]
pub fn engine_constants() -> EngineConstants {
    core_engine_constants()
}

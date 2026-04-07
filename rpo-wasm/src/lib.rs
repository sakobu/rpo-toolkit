//! WASM bindings for the RPO analytical engine.
//!
//! Thin glue layer: deserialize `JsValue` → rpo-core types, call core
//! functions, serialize results back. No business logic here.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

use wasm_bindgen::prelude::*;

/// Classify the separation between chief and deputy spacecraft.
///
/// Returns a [`MissionPhase`](rpo_core::mission::types::MissionPhase)
/// (either `far_field` or `proximity`) with orbital elements, separation
/// distance, and δr/r ratio.
///
/// # Errors
///
/// Returns `JsError` if deserialization of chief/deputy fails or if
/// the classification computation encounters invalid orbital elements.
#[wasm_bindgen]
pub fn classify_separation(chief: JsValue, deputy: JsValue) -> Result<JsValue, JsError> {
    let chief: rpo_core::types::StateVector = serde_wasm_bindgen::from_value(chief)?;
    let deputy: rpo_core::types::StateVector = serde_wasm_bindgen::from_value(deputy)?;
    let config = rpo_core::mission::config::ProximityConfig::default();
    let phase = rpo_core::mission::planning::classify_separation(&chief, &deputy, &config)
        .map_err(|e| JsError::new(&e.to_string()))?;
    Ok(serde_wasm_bindgen::to_value(&phase)?)
}

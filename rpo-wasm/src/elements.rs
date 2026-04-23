//! WASM exports for Keplerian element conversions.
//!
//! Thin glue over `rpo-core::elements::keplerian_conversions`. Currently exposes
//! the ECI state → classical Keplerian elements direction; the inverse is added
//! when a frontend caller needs it.

use wasm_bindgen::prelude::*;

use rpo_core::elements::state_to_keplerian as core_state_to_keplerian;
use rpo_core::types::{KeplerianElements, StateVector};

use crate::error::WasmError;

/// Convert an ECI state vector (km, km/s, epoch) to classical Keplerian elements.
///
/// Wraps [`rpo_core::elements::state_to_keplerian`] (Vallado Algorithm 9). Edge
/// cases for near-circular and near-equatorial orbits are documented in the core
/// function; the WASM caller sees the same singular-axis fallbacks.
///
/// # Inputs
/// - `state` — ECI state (position km, velocity km/s, epoch UTC).
///
/// # Returns
/// - [`KeplerianElements`] — `a_km`, `e`, `i_rad`, `raan_rad`, `aop_rad`,
///   `mean_anomaly_rad`. Frontend callers reading `a_km` get the chief
///   semi-major axis directly without having to mirror `MU_EARTH` client-side.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the position vector is zero or
/// the orbit is unbound (specific energy ≥ 0).
#[wasm_bindgen]
pub fn state_to_keplerian(state: StateVector) -> Result<KeplerianElements, WasmError> {
    core_state_to_keplerian(&state).map_err(WasmError::from)
}

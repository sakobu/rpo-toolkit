//! WASM exports for Keplerian element conversions and orbit sampling.
//!
//! Thin glue over `rpo-core`. Exposes two capabilities to the frontend:
//! converting a state vector to classical elements, and sampling a closed
//! orbit polyline directly from those elements (for 3D rendering).

use wasm_bindgen::prelude::*;

use rpo_core::elements::state_to_keplerian as core_state_to_keplerian;
use rpo_core::propagation::sample_orbit_eci as core_sample_orbit_eci;
use rpo_core::types::{KeplerianElements, StateVector};

use crate::error::WasmError;
use crate::frames::Vec3;

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

/// Sample one revolution of a Keplerian orbit as ECI positions.
///
/// Wraps [`rpo_core::propagation::sample_orbit_eci`]. Samples are uniform in
/// mean anomaly starting at the supplied `mean_anomaly_rad`, advancing by
/// `2π / n_points` per step. The polyline closes when consumers connect the
/// last point back to the first (the wrap maps to identical mean anomaly).
///
/// # Inputs
/// - `elements` — classical Keplerian elements (see `state_to_keplerian`).
/// - `n_points` — number of samples. Caller should use a smooth-curvature
///   value (e.g. 128); low counts produce visibly polygonal ellipses.
///
/// # Returns
/// - `[x, y, z]` positions in ECI (km), length `n_points`.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the elements are invalid
/// (non-positive semi-major axis or eccentricity outside `[0, 1)`).
#[wasm_bindgen]
pub fn sample_orbit_eci(
    elements: KeplerianElements,
    n_points: u32,
) -> Result<Vec<Vec3>, WasmError> {
    core_sample_orbit_eci(&elements, n_points)
        .map(|pts| pts.into_iter().map(Vec3::from).collect())
        .map_err(WasmError::from)
}

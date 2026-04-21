//! WASM exports for Earth-rotation, Sun/Moon positions, and ECI↔ECEF /
//! geodetic conversions.
//!
//! Thin glue over the pure-analytical frame math in `rpo-core::elements`.
//! All functions are synchronous and microsecond-scale — they run in the
//! browser without blocking the UI thread. Precision is arcsecond-class
//! (no EOP corrections); see the module docs in
//! `rpo-core::elements::earth_rotation` for the regime.
//!
//! # Crate boundary
//! Per CLAUDE.md Crate Boundary Rules: every WASM-exported function below
//! states its input frame + units, output frame + units, and the
//! [`WasmError`] variants it can return. Position-bearing returns use the
//! [`Vec3`] newtype so the JS-side type is `[number, number, number]`
//! rather than a heap-allocated `number[]`.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use rpo_core::elements::earth_rotation::earth_rotation_angle_rad as core_era;
use rpo_core::elements::eci_ecef::EciEcefTransform;
use rpo_core::elements::eci_ric_dcm::{
    eci_to_ric_dcm as core_eci_to_ric_dcm, eci_to_ric_relative as core_eci_to_ric_relative,
};
use rpo_core::elements::eclipse::{
    moon_position_eci_km as core_moon_eci, sun_position_eci_km as core_sun_eci,
};
use rpo_core::elements::geodetic::{
    ecef_to_geodetic as core_ecef_to_geodetic, geodetic_to_ecef_km as core_geodetic_to_ecef,
};
use rpo_core::types::{EcefState, GeodeticCoord, Matrix3, RICState, StateVector};

use crate::error::WasmError;

/// 3-element vector exposed at the WASM boundary as
/// `[number, number, number]`.
///
/// Wraps a fixed `[f64; 3]` and is mapped through tsify-next's
/// `tsify(type = ...)` annotation so the JS side sees a tuple, avoiding
/// the heap allocation of `Vec<f64>` returns. This matches the convention
/// already used inside [`StateVector`] / [`EcefState`] field annotations.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Tsify)]
#[tsify(into_wasm_abi, from_wasm_abi)]
pub struct Vec3(#[tsify(type = "[number, number, number]")] pub [f64; 3]);

impl From<Vector3<f64>> for Vec3 {
    fn from(v: Vector3<f64>) -> Self {
        Self([v.x, v.y, v.z])
    }
}

impl From<Vec3> for Vector3<f64> {
    fn from(v: Vec3) -> Self {
        Vector3::new(v.0[0], v.0[1], v.0[2])
    }
}

/// 3×3 matrix exposed at the WASM boundary as
/// `[[number, number, number], [number, number, number], [number, number, number]]`.
///
/// Row-major: `rows[0]` = R̂, `rows[1]` = Î, `rows[2]` = Ĉ when returned
/// from [`eci_to_ric_dcm`], each expressed in ECI coordinates. Applying
/// to a vector: `v_ric = M · v_eci`.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Tsify)]
#[tsify(into_wasm_abi, from_wasm_abi)]
pub struct Matrix3Rows(
    #[tsify(
        type = "[[number, number, number], [number, number, number], [number, number, number]]"
    )]
    pub [[f64; 3]; 3],
);

impl From<Matrix3> for Matrix3Rows {
    fn from(m: Matrix3) -> Self {
        Self([
            [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
            [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
            [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
        ])
    }
}

/// Parse a hifitime-compatible epoch string into an `Epoch`.
///
/// Accepts the same formats as the canonical `StateVector` deserializer
/// (`epoch_serde::deserialize` in `rpo-core::types::state`), which uses
/// `Epoch::from_gregorian_str`: ISO 8601 with optional timescale suffix
/// (`UTC | TAI | TDB | TT | ET | GPS`) or `Z` / `±HH:MM` offset.
fn parse_epoch(s: &str) -> Result<Epoch, WasmError> {
    Epoch::from_gregorian_str(s).map_err(|e| WasmError {
        code: crate::error::WasmErrorCode::Frame,
        message: format!("invalid epoch string {s:?}: {e}"),
        details: std::error::Error::source(&e).map(ToString::to_string),
    })
}

/// Earth Rotation Angle at the given epoch, in radians `[0, 2π)`.
///
/// IERS-2010 formula (TN 36 Eq. 5.15) with UT1 ≈ UTC — arcsecond-class.
///
/// # Inputs
/// - `epoch` — ISO 8601 string (UTC).
///
/// # Returns
/// - `f64` ERA in radians.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn earth_rotation_angle_rad(epoch: &str) -> Result<f64, WasmError> {
    let epoch = parse_epoch(epoch)?;
    Ok(core_era(epoch))
}

/// Sun position in ECI (J2000), km. Arcsecond-class (Meeus Ch. 25).
///
/// # Inputs
/// - `epoch` — ISO 8601 string.
///
/// # Returns
/// - `[x, y, z]` km in ECI J2000.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn sun_position_eci_km(epoch: &str) -> Result<Vec3, WasmError> {
    let epoch = parse_epoch(epoch)?;
    Ok(core_sun_eci(epoch).into())
}

/// Moon position in ECI (J2000), km. Approximately 1° accurate (Meeus Ch. 47 truncated).
///
/// # Inputs
/// - `epoch` — ISO 8601 string.
///
/// # Returns
/// - `[x, y, z]` km in ECI J2000.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn moon_position_eci_km(epoch: &str) -> Result<Vec3, WasmError> {
    let epoch = parse_epoch(epoch)?;
    Ok(core_moon_eci(epoch).into())
}

/// Convert an ECI position (km) to ECEF at the given epoch.
///
/// # Inputs
/// - `r_eci_km` — `[x, y, z]` in ECI (km).
/// - `epoch` — ISO 8601 string.
///
/// # Returns
/// - `[x, y, z]` in ECEF (km).
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn eci_to_ecef_position_km(r_eci_km: Vec3, epoch: &str) -> Result<Vec3, WasmError> {
    let epoch = parse_epoch(epoch)?;
    let r: Vector3<f64> = r_eci_km.into();
    Ok(EciEcefTransform::at(epoch).forward_position(&r).into())
}

/// Convert an ECEF position (km) to ECI at the given epoch.
///
/// # Inputs
/// - `r_ecef_km` — `[x, y, z]` in ECEF (km).
/// - `epoch` — ISO 8601 string.
///
/// # Returns
/// - `[x, y, z]` in ECI J2000 (km).
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn ecef_to_eci_position_km(r_ecef_km: Vec3, epoch: &str) -> Result<Vec3, WasmError> {
    let epoch = parse_epoch(epoch)?;
    let r: Vector3<f64> = r_ecef_km.into();
    Ok(EciEcefTransform::at(epoch).inverse_position(&r).into())
}

/// Convert a full ECI state (position + velocity + epoch) to ECEF, applying
/// the ω × r velocity correction.
///
/// # Inputs
/// - `state` — ECI state (position km, velocity km/s, epoch UTC).
///
/// # Returns
/// - `EcefState` with position and velocity in ECEF.
#[must_use]
#[wasm_bindgen]
pub fn eci_to_ecef_state(state: StateVector) -> EcefState {
    EciEcefTransform::at(state.epoch).forward_state(&state)
}

/// Convert a full ECEF state back to ECI. Inverse of [`eci_to_ecef_state`].
///
/// # Inputs
/// - `state` — ECEF state.
///
/// # Returns
/// - `StateVector` in ECI J2000.
#[must_use]
#[wasm_bindgen]
pub fn ecef_to_eci_state(state: EcefState) -> StateVector {
    EciEcefTransform::at(state.epoch).inverse_state(&state)
}

/// Batch ECI → ECEF state transform at a single epoch.
///
/// Builds the [`EciEcefTransform`] once (single ERA polynomial + sin/cos),
/// then applies it to every input state. Intended for trajectory rendering
/// in the viewport, where N samples typically share an epoch range close
/// enough that the per-state ERA recomputation is wasted work.
///
/// **Caller invariant:** every state's `epoch` is treated as the SAME
/// epoch as `epoch` (the cached transform's epoch). Callers needing
/// per-state-epoch fidelity must call [`eci_to_ecef_state`] per state.
///
/// # Inputs
/// - `epoch` — ISO 8601 string.
/// - `states` — Vec of ECI states.
///
/// # Returns
/// - `Vec<EcefState>` indexed parallel to `states`.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the epoch string cannot be parsed.
#[wasm_bindgen]
pub fn eci_to_ecef_states_at_epoch(
    epoch: &str,
    states: Vec<StateVector>,
) -> Result<Vec<EcefState>, WasmError> {
    let epoch = parse_epoch(epoch)?;
    let xform = EciEcefTransform::at(epoch);
    Ok(states.iter().map(|s| xform.forward_state(s)).collect())
}

/// Convert WGS-84 geodetic coordinates (lat rad, lon rad, alt km) to ECEF
/// position (km).
///
/// # Inputs
/// - `latitude_rad`, `longitude_rad`, `altitude_km` — WGS-84 geodetic.
///
/// # Returns
/// - `[x, y, z]` ECEF (km).
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if `|latitude_rad| > π/2`.
#[wasm_bindgen]
pub fn geodetic_to_ecef_km(
    latitude_rad: f64,
    longitude_rad: f64,
    altitude_km: f64,
) -> Result<Vec3, WasmError> {
    let coord = GeodeticCoord {
        latitude_rad,
        longitude_rad,
        altitude_km,
    };
    let ecef = core_geodetic_to_ecef(&coord).map_err(WasmError::from)?;
    Ok(ecef.into())
}

/// Convert ECEF position (km) to WGS-84 geodetic coordinates.
///
/// # Inputs
/// - `r_ecef_km` — `[x, y, z]` ECEF (km).
///
/// # Returns
/// - `GeodeticCoord` with latitude, longitude, altitude.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the position magnitude is
/// below the zero-vector guard, or if Bowring's latitude iteration fails
/// to converge (does not occur for well-posed inputs).
#[wasm_bindgen]
pub fn ecef_to_geodetic(r_ecef_km: Vec3) -> Result<GeodeticCoord, WasmError> {
    let r: Vector3<f64> = r_ecef_km.into();
    core_ecef_to_geodetic(&r).map_err(WasmError::from)
}

/// ECI → RIC direction cosine matrix at the chief state.
///
/// Rows of the returned 3×3 are R̂, Î, Ĉ expressed in ECI — apply to a
/// vector as `v_ric = M · v_eci`. The RIC frame is the chief-centered
/// rotating frame (radial / in-track / cross-track) used throughout the
/// proximity viewport.
///
/// Intended use: callers that need to rotate several ECI directions into
/// RIC per frame (e.g. Sun + Moon direction indicators) compute the DCM
/// once and apply it to each vector in JS. For a single one-shot
/// rotation, the JS-side matrix-vector multiply (9 muls + 6 adds) is
/// trivially cheap.
///
/// # Inputs
/// - `chief` — chief ECI state (position km, velocity km/s, epoch).
///
/// # Returns
/// - `Matrix3Rows` (row-major 3×3).
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the chief position vector
/// is zero (radial direction undefined) or if `r × v` is zero
/// (rectilinear orbit; orbital plane undefined).
#[wasm_bindgen]
pub fn eci_to_ric_dcm(chief: StateVector) -> Result<Matrix3Rows, WasmError> {
    let m = core_eci_to_ric_dcm(&chief).map_err(WasmError::from)?;
    Ok(m.into())
}

/// Deputy's relative state in the chief-centered RIC frame (km, km/s).
///
/// Wraps [`rpo_core::elements::eci_ric_dcm::eci_to_ric_relative`], which
/// returns the rotating-frame relative state including the `ω × ρ`
/// velocity correction (Vallado Ch. 3). Intended for the proximity
/// viewport: chief sits at RIC origin, deputy renders at
/// `position_ric_km × 1000` meters.
///
/// # Inputs
/// - `chief` — chief ECI state (position km, velocity km/s, epoch).
/// - `deputy` — deputy ECI state (position km, velocity km/s, epoch).
///
/// # Returns
/// - [`RICState`] — deputy position and velocity expressed in the
///   chief-centered RIC frame. Position in km, velocity in km/s.
///
/// # Errors
/// Returns [`WasmError`] with code `Frame` if the chief state cannot
/// define a valid RIC frame (zero position or rectilinear orbit).
#[wasm_bindgen]
pub fn eci_to_ric_relative_state(
    chief: StateVector,
    deputy: StateVector,
) -> Result<RICState, WasmError> {
    core_eci_to_ric_relative(&chief, &deputy).map_err(WasmError::from)
}

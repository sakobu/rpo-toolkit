//! Core state types: ECI state vectors, RIC states, and departure states.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::elements::KeplerianElements;
use super::roe::QuasiNonsingularROE;

/// ECI J2000 state vector (position in km, velocity in km/s)
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct StateVector {
    /// Epoch of the state
    #[serde(with = "epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub epoch: Epoch,
    /// Position vector in ECI frame (km)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub position_eci_km: Vector3<f64>,
    /// Velocity vector in ECI frame (km/s)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub velocity_eci_km_s: Vector3<f64>,
}

/// Relative state in the RIC (Radial-In-track-Cross-track) frame
/// Also known as Hill frame or LVLH frame
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RICState {
    /// Relative position in RIC frame (km): [radial, in-track, cross-track]
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub position_ric_km: Vector3<f64>,
    /// Time derivative of relative position in the rotating RIC frame, ρ̇ (km/s)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub velocity_ric_km_s: Vector3<f64>,
}

/// State vector in the Earth-Centered Earth-Fixed (ECEF) frame.
///
/// ECEF co-rotates with Earth: `+X` through Greenwich (lon 0°, lat 0°),
/// `+Z` along the mean rotation axis. The velocity is the time derivative
/// in the rotating frame — so a satellite in geostationary orbit at 0°
/// longitude has ECEF velocity ≈ 0.
///
/// Conversions to/from [`StateVector`] (ECI) live in
/// [`crate::elements::eci_ecef`].
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EcefState {
    /// Epoch of the state (needed because ECEF orientation tracks Earth rotation).
    #[serde(with = "epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub epoch: Epoch,
    /// Position vector in the ECEF frame (km).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub position_ecef_km: Vector3<f64>,
    /// Velocity vector in the ECEF frame (km/s).
    ///
    /// Includes the Earth-rotation correction: `v_ecef = R(-θ)·(v_eci − ω × r_eci)`.
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub velocity_ecef_km_s: Vector3<f64>,
}

/// WGS-84 geodetic coordinates: latitude, longitude, altitude above the
/// reference ellipsoid.
///
/// - Latitude is geodetic (measured from the equator to the normal of the
///   ellipsoid, not to the geocentric position), in radians ∈ `[-π/2, π/2]`.
/// - Longitude is east-positive, in radians ∈ `(-π, π]` by convention, though
///   no wrap normalization is enforced.
/// - Altitude is height above the ellipsoid, in km (positive outward).
///
/// Conversions to/from ECEF live in [`crate::elements::geodetic`].
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GeodeticCoord {
    /// Geodetic latitude (rad).
    pub latitude_rad: f64,
    /// Geodetic longitude (rad), east-positive.
    pub longitude_rad: f64,
    /// Altitude above the WGS-84 ellipsoid (km).
    pub altitude_km: f64,
}

/// Departure orbital state for targeting: groups ROE, chief elements, and epoch.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DepartureState {
    /// Deputy quasi-nonsingular ROE relative to chief
    pub roe: QuasiNonsingularROE,
    /// Chief mean Keplerian elements
    pub chief: KeplerianElements,
    /// Epoch of this state
    #[serde(with = "epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub epoch: Epoch,
}

/// Serde support for `hifitime::Epoch` (serialize as ISO 8601 string)
pub(crate) mod epoch_serde {
    use hifitime::Epoch;
    use serde::{self, Deserialize, Deserializer, Serializer};

    /// Serialize an `Epoch` as an ISO 8601 string.
    pub fn serialize<S>(epoch: &Epoch, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let s = format!("{epoch}");
        serializer.serialize_str(&s)
    }

    /// Deserialize an `Epoch` from an ISO 8601 string.
    pub fn deserialize<'de, D>(deserializer: D) -> Result<Epoch, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Epoch::from_gregorian_str(&s).map_err(serde::de::Error::custom)
    }
}

//! WGS-84 geodetic ↔ ECEF coordinate conversions.
//!
//! Uses the WGS-84 reference ellipsoid (equatorial radius
//! [`crate::constants::R_EARTH`], flattening
//! [`crate::constants::EARTH_FLATTENING_WGS84`]).
//!
//! Forward conversion (geodetic → ECEF) is closed-form. Inverse
//! (ECEF → geodetic) uses iterative refinement on the geodetic latitude
//! (Bowring-style direct iteration). Both are pure analytical, WASM-safe,
//! sub-microsecond per call.
//!
//! # References
//!
//! - National Imagery and Mapping Agency (NIMA), *Department of Defense
//!   World Geodetic System 1984*, Technical Report TR8350.2.
//! - Bowring, B. R. (1976). "Transformation from spatial to geographical
//!   coordinates", *Survey Review*, 23(181), 323–327.
//! - Vallado, D. A., *Fundamentals of Astrodynamics and Applications*,
//!   4e, Ch. 3, §3.4 (ellipsoidal Earth geometry).

use core::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::constants::{
    EARTH_ECCENTRICITY_SQUARED_WGS84, EARTH_POLAR_RADIUS_KM, GEODETIC_LAT_TOL_RAD,
    GEODETIC_MAX_ITER, GEODETIC_POLAR_EQUATORIAL_RADIUS_TOL_KM, MIN_POSITION_NORM_KM, R_EARTH,
};
use crate::types::GeodeticCoord;

/// Errors from geodetic ↔ ECEF conversions.
#[derive(Debug, Clone, thiserror::Error)]
pub enum GeodeticError {
    /// ECEF position vector magnitude is below the zero-vector guard.
    #[error("ECEF position magnitude {norm_km} km is below the zero threshold")]
    ZeroPosition {
        /// Magnitude of the offending position (km).
        norm_km: f64,
    },
    /// Geodetic latitude is outside `[-π/2, π/2]`.
    #[error("geodetic latitude {lat_rad} rad is outside [-π/2, π/2]")]
    LatitudeOutOfRange {
        /// The offending latitude (rad).
        lat_rad: f64,
    },
    /// Bowring's geodetic-latitude iteration failed to converge within
    /// [`GEODETIC_MAX_ITER`] steps.
    #[error("geodetic iteration did not converge: {iters} iters, residual {residual_rad:.3e} rad")]
    NoConvergence {
        /// Number of iterations performed before giving up.
        iters: u32,
        /// Final `|Δφ|` residual on the geodetic latitude (rad).
        residual_rad: f64,
    },
}

/// Prime vertical radius of curvature `N(φ) = a / sqrt(1 − e² sin²φ)`, in km.
///
/// Source: NIMA TR8350.2 §3.5.4 / Vallado §3.4.
fn prime_vertical_radius_km(lat_rad: f64) -> f64 {
    let sin_lat = lat_rad.sin();
    R_EARTH / (1.0 - EARTH_ECCENTRICITY_SQUARED_WGS84 * sin_lat * sin_lat).sqrt()
}

/// Convert WGS-84 geodetic coordinates to ECEF (km).
///
/// Closed-form formula:
///
/// ```text
/// N(φ) = a / sqrt(1 - e² sin²φ)
/// x = (N + h) cos φ cos λ
/// y = (N + h) cos φ sin λ
/// z = ((1 - e²) N + h) sin φ
/// ```
///
/// # Invariants
/// - `|latitude_rad| ≤ π/2`. Inputs outside this range return an error.
/// - Longitude is not wrap-normalized (any real value is accepted).
/// - Output magnitude `|r_ecef|` is positive for any non-degenerate input.
///
/// # Errors
/// Returns [`GeodeticError::LatitudeOutOfRange`] if `|latitude_rad| > π/2`.
///
/// # References
/// - NIMA TR8350.2 §3.5.4.
/// - Vallado §3.4.
pub fn geodetic_to_ecef_km(coord: &GeodeticCoord) -> Result<Vector3<f64>, GeodeticError> {
    if coord.latitude_rad.abs() > FRAC_PI_2 {
        return Err(GeodeticError::LatitudeOutOfRange {
            lat_rad: coord.latitude_rad,
        });
    }

    let n = prime_vertical_radius_km(coord.latitude_rad);
    let (sin_lat, cos_lat) = coord.latitude_rad.sin_cos();
    let (sin_lon, cos_lon) = coord.longitude_rad.sin_cos();

    let x = (n + coord.altitude_km) * cos_lat * cos_lon;
    let y = (n + coord.altitude_km) * cos_lat * sin_lon;
    let z = ((1.0 - EARTH_ECCENTRICITY_SQUARED_WGS84) * n + coord.altitude_km) * sin_lat;

    Ok(Vector3::new(x, y, z))
}

/// Convert ECEF coordinates to WGS-84 geodetic coordinates.
///
/// Longitude is computed exactly from `atan2(y, x)`. Latitude and altitude
/// are refined by iterative solution of:
///
/// ```text
/// p = sqrt(x² + y²)
/// φ = atan2(z + e²·N(φ)·sin φ, p)
/// ```
///
/// # Invariants
/// - Bowring's iteration converges quadratically; 2–3 iterations are typical
///   and 10 is the hard ceiling ([`GEODETIC_MAX_ITER`]).
/// - Polar inputs (`p` below [`GEODETIC_POLAR_EQUATORIAL_RADIUS_TOL_KM`])
///   take the closed-form polar branch instead of iterating.
/// - Returned `latitude_rad ∈ [−π/2, π/2]`; `longitude_rad ∈ (−π, π]`
///   (per `atan2`); `altitude_km` is height above the WGS-84 ellipsoid.
///
/// # Errors
/// - [`GeodeticError::ZeroPosition`] if `|r_ecef| < MIN_POSITION_NORM_KM`.
/// - [`GeodeticError::NoConvergence`] if Bowring's iteration fails to reach
///   [`GEODETIC_LAT_TOL_RAD`] within [`GEODETIC_MAX_ITER`] steps (does not
///   occur for well-posed inputs).
///
/// # References
/// - NIMA TR8350.2 (WGS-84).
/// - Bowring, B. R. (1976). "Transformation from spatial to geographical
///   coordinates", *Survey Review*, 23(181), 323–327.
pub fn ecef_to_geodetic(r_ecef_km: &Vector3<f64>) -> Result<GeodeticCoord, GeodeticError> {
    let norm_km = r_ecef_km.norm();
    if norm_km < MIN_POSITION_NORM_KM {
        return Err(GeodeticError::ZeroPosition { norm_km });
    }

    let z_km = r_ecef_km.z;
    let longitude_rad = r_ecef_km.y.atan2(r_ecef_km.x);
    let p_km = (r_ecef_km.x * r_ecef_km.x + r_ecef_km.y * r_ecef_km.y).sqrt();

    // Pole branch: equatorial radius is below the singularity threshold;
    // altitude is the vertical distance from the polar radius.
    if p_km < GEODETIC_POLAR_EQUATORIAL_RADIUS_TOL_KM {
        let latitude_rad = if z_km >= 0.0 { FRAC_PI_2 } else { -FRAC_PI_2 };
        let altitude_km = z_km.abs() - EARTH_POLAR_RADIUS_KM;
        return Ok(GeodeticCoord {
            latitude_rad,
            longitude_rad,
            altitude_km,
        });
    }

    // Initial guess: geocentric latitude adjusted for flattening.
    let mut latitude_rad = z_km.atan2(p_km * (1.0 - EARTH_ECCENTRICITY_SQUARED_WGS84));
    let mut residual_rad = f64::INFINITY;

    for _ in 0..GEODETIC_MAX_ITER {
        let n_km = prime_vertical_radius_km(latitude_rad);
        let sin_lat = latitude_rad.sin();
        let new_lat = (z_km + EARTH_ECCENTRICITY_SQUARED_WGS84 * n_km * sin_lat).atan2(p_km);
        residual_rad = (new_lat - latitude_rad).abs();
        latitude_rad = new_lat;
        if residual_rad < GEODETIC_LAT_TOL_RAD {
            let cos_lat = latitude_rad.cos();
            let altitude_km = p_km / cos_lat - n_km;
            return Ok(GeodeticCoord {
                latitude_rad,
                longitude_rad,
                altitude_km,
            });
        }
    }

    Err(GeodeticError::NoConvergence {
        iters: GEODETIC_MAX_ITER,
        residual_rad,
    })
}

#[cfg(test)]
mod tests {
    use core::f64::consts::FRAC_PI_2;

    use nalgebra::Vector3;

    use super::{ecef_to_geodetic, geodetic_to_ecef_km, GeodeticError};
    use crate::constants::{EARTH_POLAR_RADIUS_KM, R_EARTH};
    use crate::types::GeodeticCoord;

    // ---------------------------------------------------------------------
    // Named tolerance constants
    // ---------------------------------------------------------------------

    /// Closed-form ECEF-at-equator must equal `(R_EARTH, 0, 0)` exactly.
    /// Pure f64 product of a few constants; 1e-12 km is tight.
    const EQUATOR_CLOSED_FORM_TOL_KM: f64 = 1e-12;

    /// Geodetic ↔ ECEF latitude round-trip (any latitude away from poles).
    /// Dominated by Bowring's convergence tolerance (`GEODETIC_LAT_TOL_RAD`);
    /// 1e-11 rad provides margin.
    const LATITUDE_ROUNDTRIP_TOL_RAD: f64 = 1e-11;

    /// Geodetic ↔ ECEF altitude round-trip at typical station altitudes.
    /// Altitude is derived from `p / cos(φ) − N`; error scales with
    /// `|alt_error| ≈ p · |Δφ·tan φ| + …`; 1e-9 km = 1 μm is conservative.
    const ALTITUDE_ROUNDTRIP_TOL_KM: f64 = 1e-9;

    /// Longitude round-trip error — `atan2` is exact to f64 round-off.
    const LONGITUDE_ROUNDTRIP_TOL_RAD: f64 = 1e-14;

    /// Equator at (0°, 0°, 0 km) maps to `(R_EARTH, 0, 0)` ECEF exactly.
    #[test]
    fn equator_zero_altitude_closed_form() {
        let coord = GeodeticCoord {
            latitude_rad: 0.0,
            longitude_rad: 0.0,
            altitude_km: 0.0,
        };
        let ecef = geodetic_to_ecef_km(&coord).unwrap();
        let err_x = (ecef.x - R_EARTH).abs();
        let err_yz = ecef.y.abs() + ecef.z.abs();
        assert!(
            err_x < EQUATOR_CLOSED_FORM_TOL_KM && err_yz < EQUATOR_CLOSED_FORM_TOL_KM,
            "equator ECEF = {ecef:?}, expected ({R_EARTH}, 0, 0)"
        );
    }

    /// North pole at (+90°, 0°, 0 km) maps to `(0, 0, EARTH_POLAR_RADIUS_KM)`.
    #[test]
    fn north_pole_zero_altitude_closed_form() {
        let coord = GeodeticCoord {
            latitude_rad: FRAC_PI_2,
            longitude_rad: 0.0,
            altitude_km: 0.0,
        };
        let ecef = geodetic_to_ecef_km(&coord).unwrap();
        assert!(ecef.x.abs() < EQUATOR_CLOSED_FORM_TOL_KM);
        assert!(ecef.y.abs() < EQUATOR_CLOSED_FORM_TOL_KM);
        let err_z = (ecef.z - EARTH_POLAR_RADIUS_KM).abs();
        assert!(
            err_z < EQUATOR_CLOSED_FORM_TOL_KM,
            "north pole ECEF.z = {}, expected {EARTH_POLAR_RADIUS_KM}",
            ecef.z
        );
    }

    /// Roundtrip at Goddard Space Flight Center: 38.9921°N, -76.8481°E, 0 km.
    /// Mid-latitude, non-polar; stresses the typical Bowring regime.
    #[test]
    fn gsfc_roundtrip() {
        let original = GeodeticCoord {
            latitude_rad: 38.9921_f64.to_radians(),
            longitude_rad: (-76.8481_f64).to_radians(),
            altitude_km: 0.0,
        };
        let ecef = geodetic_to_ecef_km(&original).unwrap();
        let back = ecef_to_geodetic(&ecef).unwrap();

        assert!(
            (back.latitude_rad - original.latitude_rad).abs() < LATITUDE_ROUNDTRIP_TOL_RAD,
            "lat err = {}",
            (back.latitude_rad - original.latitude_rad).abs()
        );
        assert!(
            (back.longitude_rad - original.longitude_rad).abs() < LONGITUDE_ROUNDTRIP_TOL_RAD
        );
        assert!(
            (back.altitude_km - original.altitude_km).abs() < ALTITUDE_ROUNDTRIP_TOL_KM,
            "alt err = {}",
            (back.altitude_km - original.altitude_km).abs()
        );
    }

    /// Roundtrip at 89° latitude (near pole) to stress Bowring convergence.
    #[test]
    fn near_pole_roundtrip() {
        for (lat_deg, alt_km) in [(89.0, 0.0), (-89.0, 10.0), (89.5, 0.5)] {
            let original = GeodeticCoord {
                latitude_rad: f64::to_radians(lat_deg),
                longitude_rad: 1.0,
                altitude_km: alt_km,
            };
            let ecef = geodetic_to_ecef_km(&original).unwrap();
            let back = ecef_to_geodetic(&ecef).unwrap();
            assert!(
                (back.latitude_rad - original.latitude_rad).abs() < LATITUDE_ROUNDTRIP_TOL_RAD,
                "near-pole lat err at {lat_deg}° = {}",
                (back.latitude_rad - original.latitude_rad).abs()
            );
            assert!(
                (back.altitude_km - original.altitude_km).abs() < ALTITUDE_ROUNDTRIP_TOL_KM,
                "near-pole alt err at {lat_deg}° = {}",
                (back.altitude_km - original.altitude_km).abs()
            );
        }
    }

    /// Roundtrip at non-zero altitude to stress altitude accuracy.
    #[test]
    fn roundtrip_with_altitude() {
        let original = GeodeticCoord {
            latitude_rad: 45.0_f64.to_radians(),
            longitude_rad: 90.0_f64.to_radians(),
            altitude_km: 408.0, // ISS altitude
        };
        let ecef = geodetic_to_ecef_km(&original).unwrap();
        let back = ecef_to_geodetic(&ecef).unwrap();
        assert!((back.latitude_rad - original.latitude_rad).abs() < LATITUDE_ROUNDTRIP_TOL_RAD);
        assert!((back.altitude_km - original.altitude_km).abs() < ALTITUDE_ROUNDTRIP_TOL_KM);
    }

    /// Latitude just outside `[-π/2, π/2]` returns `LatitudeOutOfRange`.
    #[test]
    fn latitude_out_of_range_rejected() {
        let coord = GeodeticCoord {
            latitude_rad: FRAC_PI_2 * 1.01,
            longitude_rad: 0.0,
            altitude_km: 0.0,
        };
        let result = geodetic_to_ecef_km(&coord);
        assert!(
            matches!(result, Err(GeodeticError::LatitudeOutOfRange { .. })),
            "latitude > π/2 should error; got {result:?}"
        );
    }

    /// Zero ECEF vector returns `ZeroPosition`.
    #[test]
    fn zero_ecef_rejected() {
        let result = ecef_to_geodetic(&Vector3::zeros());
        assert!(
            matches!(result, Err(GeodeticError::ZeroPosition { .. })),
            "zero ECEF should error; got {result:?}"
        );
    }

    /// Exactly on the pole (Bowring's degenerate case). Should return
    /// lat = ±π/2 without iterating.
    #[test]
    fn pole_inverse_is_exact() {
        let r_north = Vector3::new(0.0, 0.0, EARTH_POLAR_RADIUS_KM);
        let coord = ecef_to_geodetic(&r_north).unwrap();
        assert!((coord.latitude_rad - FRAC_PI_2).abs() < LATITUDE_ROUNDTRIP_TOL_RAD);
        assert!(coord.altitude_km.abs() < ALTITUDE_ROUNDTRIP_TOL_KM);
    }
}

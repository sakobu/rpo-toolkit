//! Conversions between ECI state vectors and Keplerian orbital elements.
//!
//! Implements the standard Keplerian↔ECI transformations from Vallado,
//! "Fundamentals of Astrodynamics and Applications", 4th ed., Algorithms 9 and 10.
//! Edge-case handling for circular and equatorial orbits follows Vallado Sec. 2.6.

use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use crate::constants::{ECC_TOL, INC_TOL, MIN_POSITION_NORM_KM, MU_EARTH, TWO_PI};
use crate::types::{KeplerianElements, StateVector};

/// Errors from ECI ↔ Keplerian conversions.
#[derive(Debug, Clone)]
pub enum ConversionError {
    /// Position vector is zero; cannot compute orbital elements.
    ZeroPositionVector,
    /// Orbit is unbound (specific energy >= 0); semi-major axis would be negative.
    UnboundOrbit {
        /// Specific orbital energy (km²/s²).
        energy_km2_s2: f64,
    },
    /// Semi-major axis must be positive.
    InvalidSemiMajorAxis {
        /// The invalid semi-major axis (km).
        a_km: f64,
    },
    /// Eccentricity must be in [0, 1).
    InvalidEccentricity {
        /// The invalid eccentricity value.
        e: f64,
    },
}

impl std::fmt::Display for ConversionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ZeroPositionVector => {
                write!(f, "ConversionError: position vector is zero")
            }
            Self::UnboundOrbit { energy_km2_s2 } => {
                write!(
                    f,
                    "ConversionError: unbound orbit — specific energy = {energy_km2_s2:.6e} km²/s² (must be negative)"
                )
            }
            Self::InvalidSemiMajorAxis { a_km } => {
                write!(
                    f,
                    "ConversionError: semi-major axis = {a_km} km must be positive"
                )
            }
            Self::InvalidEccentricity { e } => {
                write!(
                    f,
                    "ConversionError: eccentricity = {e} out of range [0, 1)"
                )
            }
        }
    }
}

impl std::error::Error for ConversionError {}

/// Validate that Keplerian elements have positive SMA and bound eccentricity.
///
/// # Errors
/// Returns `ConversionError::InvalidSemiMajorAxis` if `ke.a_km <= 0`.
/// Returns `ConversionError::InvalidEccentricity` if `ke.e < 0` or `ke.e >= 1`.
pub(crate) fn validate_elements(ke: &KeplerianElements) -> Result<(), ConversionError> {
    if ke.a_km <= 0.0 {
        return Err(ConversionError::InvalidSemiMajorAxis { a_km: ke.a_km });
    }
    if ke.e < 0.0 || ke.e >= 1.0 {
        return Err(ConversionError::InvalidEccentricity { e: ke.e });
    }
    Ok(())
}

/// Convert an ECI state vector to Keplerian orbital elements.
///
/// Handles edge cases:
/// - Near-circular orbits (e < `ECC_TOL`): ω set to 0, true anomaly measured from ascending node
/// - Near-equatorial orbits (i < `INC_TOL`): Ω set to 0, ω measured from x-axis
///
/// # Invariants
/// - `sv.position_eci_km` must be non-zero (zero position → division by zero)
/// - Orbit must be bound (`e < 1`); escape trajectories produce `Err(UnboundOrbit)`
/// - Kepler's equation inversion (via `true_anomaly()`) may not converge for `e` near 1
///
/// # Errors
/// Returns `ConversionError::ZeroPositionVector` if the position vector norm is < `MIN_POSITION_NORM_KM`.
/// Returns `ConversionError::UnboundOrbit` if specific energy >= 0 (escape trajectory).
#[allow(clippy::many_single_char_names)]
pub fn state_to_keplerian(sv: &StateVector) -> Result<KeplerianElements, ConversionError> {
    let r_vec = sv.position_eci_km;
    let v_vec = sv.velocity_eci_km_s;
    let r = r_vec.norm();
    let v = v_vec.norm();

    if r < MIN_POSITION_NORM_KM {
        return Err(ConversionError::ZeroPositionVector);
    }

    // Specific angular momentum
    let h_vec = r_vec.cross(&v_vec);
    let h = h_vec.norm();

    // Node vector: k × h
    let k = Vector3::new(0.0, 0.0, 1.0);
    let n_vec = k.cross(&h_vec);
    let n = n_vec.norm();

    // Eccentricity vector
    let e_vec = ((v * v - MU_EARTH / r) * r_vec - r_vec.dot(&v_vec) * v_vec) / MU_EARTH;
    let e = e_vec.norm();

    // Semi-major axis (vis-viva)
    let energy = v * v / 2.0 - MU_EARTH / r;
    if energy >= 0.0 {
        return Err(ConversionError::UnboundOrbit { energy_km2_s2: energy });
    }
    let a = -MU_EARTH / (2.0 * energy);

    // Inclination
    let i = (h_vec.z / h).acos();

    // RAAN
    let raan = if i.abs() < INC_TOL {
        0.0
    } else if n_vec.y >= 0.0 {
        (n_vec.x / n).acos()
    } else {
        TWO_PI - (n_vec.x / n).acos()
    };

    // Argument of perigee
    let aop = if e < ECC_TOL {
        0.0
    } else if i.abs() < INC_TOL {
        // Equatorial: measure ω from x-axis
        let aop = e_vec.y.atan2(e_vec.x);
        aop.rem_euclid(TWO_PI)
    } else {
        let cos_aop = n_vec.dot(&e_vec) / (n * e);
        let aop = cos_aop.clamp(-1.0, 1.0).acos();
        if e_vec.z >= 0.0 {
            aop
        } else {
            TWO_PI - aop
        }
    };

    // True anomaly
    let nu = if e < ECC_TOL {
        // Circular: measure from node line (or x-axis if equatorial)
        if i.abs() < INC_TOL {
            let nu = r_vec.y.atan2(r_vec.x);
            nu.rem_euclid(TWO_PI)
        } else {
            let cos_nu = n_vec.dot(&r_vec) / (n * r);
            let nu = cos_nu.clamp(-1.0, 1.0).acos();
            if r_vec.z >= 0.0 {
                nu
            } else {
                TWO_PI - nu
            }
        }
    } else {
        let cos_nu = e_vec.dot(&r_vec) / (e * r);
        let nu = cos_nu.clamp(-1.0, 1.0).acos();
        if r_vec.dot(&v_vec) >= 0.0 {
            nu
        } else {
            TWO_PI - nu
        }
    };

    // True anomaly → eccentric anomaly → mean anomaly
    let ea = 2.0
        * ((1.0 - e).sqrt() * (nu / 2.0).sin()).atan2((1.0 + e).sqrt() * (nu / 2.0).cos());
    let mean_anomaly = (ea - e * ea.sin()).rem_euclid(TWO_PI);

    Ok(KeplerianElements {
        a_km: a,
        e,
        i_rad: i,
        raan_rad: raan,
        aop_rad: aop,
        mean_anomaly_rad: mean_anomaly,
    })
}

/// Convert Keplerian orbital elements to an ECI state vector.
///
/// # Invariants
/// - `ke.a_km > 0` (negative SMA produces invalid geometry)
/// - `0 <= ke.e < 1` (parabolic/hyperbolic elements are not supported)
/// - Delegates to `true_anomaly()` for Kepler's equation; see its invariants
///
/// # Errors
/// Returns `ConversionError::InvalidSemiMajorAxis` if `ke.a_km <= 0`.
/// Returns `ConversionError::InvalidEccentricity` if `ke.e` is outside [0, 1).
pub fn keplerian_to_state(ke: &KeplerianElements, epoch: Epoch) -> Result<StateVector, ConversionError> {
    validate_elements(ke)?;

    let nu = ke.true_anomaly();
    let p = ke.a_km * (1.0 - ke.e * ke.e); // semi-latus rectum
    let r = p / (1.0 + ke.e * nu.cos());

    // Position and velocity in perifocal frame (PQW)
    let r_pqw = Vector3::new(r * nu.cos(), r * nu.sin(), 0.0);
    let v_pqw = Vector3::new(
        -(MU_EARTH / p).sqrt() * nu.sin(),
        (MU_EARTH / p).sqrt() * (ke.e + nu.cos()),
        0.0,
    );

    // Rotation matrix: perifocal → ECI
    // R = R3(-Ω) · R1(-i) · R3(-ω)
    let cos_o = ke.raan_rad.cos();
    let sin_o = ke.raan_rad.sin();
    let cos_i = ke.i_rad.cos();
    let sin_i = ke.i_rad.sin();
    let cos_w = ke.aop_rad.cos();
    let sin_w = ke.aop_rad.sin();

    let rot = Matrix3::new(
        cos_o * cos_w - sin_o * sin_w * cos_i,
        -cos_o * sin_w - sin_o * cos_w * cos_i,
        0.0,
        sin_o * cos_w + cos_o * sin_w * cos_i,
        -sin_o * sin_w + cos_o * cos_w * cos_i,
        0.0,
        sin_w * sin_i,
        cos_w * sin_i,
        0.0,
    );

    let position = rot * r_pqw;
    let velocity = rot * v_pqw;

    Ok(StateVector {
        epoch,
        position_eci_km: position,
        velocity_eci_km_s: velocity,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{eccentric_elements, iss_like_elements, test_epoch};

    #[test]
    fn roundtrip_eci_keplerian() {
        let epoch = test_epoch();
        let ke = iss_like_elements();
        let sv = keplerian_to_state(&ke, epoch).unwrap();
        let ke2 = state_to_keplerian(&sv).unwrap();
        let sv2 = keplerian_to_state(&ke2, sv.epoch).unwrap();

        let pos_err = (sv.position_eci_km - sv2.position_eci_km).norm();
        let vel_err = (sv.velocity_eci_km_s - sv2.velocity_eci_km_s).norm();

        assert!(
            pos_err < 1e-10,
            "Position roundtrip error: {pos_err} km"
        );
        assert!(
            vel_err < 1e-10,
            "Velocity roundtrip error: {vel_err} km/s"
        );
    }

    #[test]
    fn roundtrip_eccentric_orbit() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let sv = keplerian_to_state(&ke, epoch).unwrap();
        let ke2 = state_to_keplerian(&sv).unwrap();
        let sv2 = keplerian_to_state(&ke2, epoch).unwrap();

        let pos_err = (sv.position_eci_km - sv2.position_eci_km).norm();
        let vel_err = (sv.velocity_eci_km_s - sv2.velocity_eci_km_s).norm();

        assert!(pos_err < 1e-10, "Position roundtrip error: {pos_err} km");
        assert!(vel_err < 1e-10, "Velocity roundtrip error: {vel_err} km/s");
    }

    #[test]
    fn roundtrip_circular_equatorial() {
        let epoch = test_epoch();
        let ke = KeplerianElements {
            a_km: 7000.0,
            e: 0.0,
            i_rad: 0.0,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 90.0_f64.to_radians(),
        };
        let sv = keplerian_to_state(&ke, epoch).unwrap();
        let ke2 = state_to_keplerian(&sv).unwrap();
        let sv2 = keplerian_to_state(&ke2, epoch).unwrap();

        let pos_err = (sv.position_eci_km - sv2.position_eci_km).norm();
        let vel_err = (sv.velocity_eci_km_s - sv2.velocity_eci_km_s).norm();

        assert!(pos_err < 1e-10, "Position roundtrip error: {pos_err} km");
        assert!(vel_err < 1e-10, "Velocity roundtrip error: {vel_err} km/s");
    }

    #[test]
    fn keplerian_elements_correct() {
        let epoch = test_epoch();
        let ke_orig = KeplerianElements {
            a_km: 8000.0,
            e: 0.1,
            i_rad: 30.0_f64.to_radians(),
            raan_rad: 60.0_f64.to_radians(),
            aop_rad: 90.0_f64.to_radians(),
            mean_anomaly_rad: 45.0_f64.to_radians(),
        };
        let sv = keplerian_to_state(&ke_orig, epoch).unwrap();
        let ke = state_to_keplerian(&sv).unwrap();

        assert!((ke.a_km - ke_orig.a_km).abs() < 1e-8, "SMA mismatch");
        assert!((ke.e - ke_orig.e).abs() < 1e-10, "Eccentricity mismatch");
        assert!((ke.i_rad - ke_orig.i_rad).abs() < 1e-10, "Inclination mismatch");
        assert!((ke.raan_rad - ke_orig.raan_rad).abs() < 1e-10, "RAAN mismatch");
        assert!((ke.aop_rad - ke_orig.aop_rad).abs() < 1e-10, "AoP mismatch");
        assert!(
            (ke.mean_anomaly_rad - ke_orig.mean_anomaly_rad).abs() < 1e-10,
            "Mean anomaly mismatch"
        );
    }

    #[test]
    fn keplerian_to_state_negative_sma_returns_error() {
        let epoch = test_epoch();
        let ke = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = keplerian_to_state(&ke, epoch);
        assert!(
            matches!(result, Err(ConversionError::InvalidSemiMajorAxis { .. })),
            "Negative SMA should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    #[test]
    fn keplerian_to_state_hyperbolic_returns_error() {
        let epoch = test_epoch();
        let ke = KeplerianElements {
            a_km: 7000.0,
            e: 1.5,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = keplerian_to_state(&ke, epoch);
        assert!(
            matches!(result, Err(ConversionError::InvalidEccentricity { .. })),
            "e >= 1 should return InvalidEccentricity, got {result:?}"
        );
    }

    #[test]
    fn unbound_orbit_returns_error() {
        let epoch = test_epoch();
        // Escape velocity at 7000 km altitude: v_escape = sqrt(2*mu/r) ≈ 10.7 km/s
        let sv = StateVector {
            epoch,
            position_eci_km: nalgebra::Vector3::new(7000.0, 0.0, 0.0),
            velocity_eci_km_s: nalgebra::Vector3::new(0.0, 15.0, 0.0), // well above escape velocity
        };
        let result = state_to_keplerian(&sv);
        assert!(
            matches!(result, Err(ConversionError::UnboundOrbit { .. })),
            "Escape trajectory should return UnboundOrbit, got {result:?}"
        );
    }
}

//! Core domain types: state vectors, Keplerian elements, ROEs, and RIC states.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::constants::{KEPLER_MAX_ITER, KEPLER_TOL, TWO_PI};

/// ECI J2000 state vector (position in km, velocity in km/s)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StateVector {
    /// Epoch of the state
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
    /// Position vector in ECI frame (km)
    pub position: Vector3<f64>,
    /// Velocity vector in ECI frame (km/s)
    pub velocity: Vector3<f64>,
}

/// Classical Keplerian orbital elements
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct KeplerianElements {
    /// Semi-major axis (km)
    pub a: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination (rad)
    pub i: f64,
    /// Right ascension of ascending node (rad)
    pub raan: f64,
    /// Argument of perigee (rad)
    pub aop: f64,
    /// Mean anomaly (rad)
    pub mean_anomaly: f64,
}

impl KeplerianElements {
    /// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E,
    /// then compute true anomaly ν.
    pub fn true_anomaly(&self) -> f64 {
        debug_assert!(self.e >= 0.0 && self.e < 1.0, "eccentricity must be in [0, 1), got {}", self.e);

        let m = self.mean_anomaly.rem_euclid(TWO_PI);
        let e = self.e;

        // Newton's method for Kepler's equation
        let mut ea = if e < 0.8 { m } else { std::f64::consts::PI };
        for _ in 0..KEPLER_MAX_ITER {
            let f = ea - e * ea.sin() - m;
            let fp = 1.0 - e * ea.cos();
            let delta = f / fp;
            ea -= delta;
            if delta.abs() < KEPLER_TOL {
                break;
            }
        }

        // Eccentric anomaly → true anomaly
        let nu = 2.0
            * ((1.0 + e).sqrt() * (ea / 2.0).sin()).atan2((1.0 - e).sqrt() * (ea / 2.0).cos());
        nu.rem_euclid(TWO_PI)
    }

    /// Mean argument of latitude u = ω + M
    pub fn mean_arg_of_lat(&self) -> f64 {
        (self.aop + self.mean_anomaly).rem_euclid(TWO_PI)
    }
}

/// Quasi-nonsingular relative orbital elements (Koenig Eq. 2 / D'Amico Eq. 2.2)
/// All elements are dimensionless (normalized by chief semi-major axis)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct QuasiNonsingularROE {
    /// Relative semi-major axis: `(a_d - a_c) / a_c`
    pub da: f64,
    /// Relative mean longitude
    pub dlambda: f64,
    /// Relative eccentricity vector x-component
    pub dex: f64,
    /// Relative eccentricity vector y-component
    pub dey: f64,
    /// Relative inclination vector x-component
    pub dix: f64,
    /// Relative inclination vector y-component
    pub diy: f64,
}

/// Relative state in the RIC (Radial-In-track-Cross-track) frame
/// Also known as Hill frame or LVLH frame
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RICState {
    /// Relative position in RIC frame (km): [radial, in-track, cross-track]
    pub position: Vector3<f64>,
    /// Relative velocity in RIC frame (km/s): [radial, in-track, cross-track]
    pub velocity: Vector3<f64>,
}

/// Serde support for `hifitime::Epoch` (serialize as ISO 8601 string)
mod epoch_serde {
    use hifitime::Epoch;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(epoch: &Epoch, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let s = format!("{epoch}");
        serializer.serialize_str(&s)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Epoch, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Epoch::from_gregorian_str(&s).map_err(serde::de::Error::custom)
    }
}

//! Physical and numerical constants for orbital mechanics.

/// Earth gravitational parameter (km³/s²)
pub const MU_EARTH: f64 = 398_600.441_8;

/// Earth equatorial radius (km)
pub const R_EARTH: f64 = 6378.137;

/// J2 zonal harmonic coefficient
pub const J2: f64 = 1.082_626_68e-3;

/// Two pi
pub const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Tolerance for near-circular orbit detection
pub const ECC_TOL: f64 = 1e-10;

/// Tolerance for near-equatorial orbit detection
pub const INC_TOL: f64 = 1e-10;

/// Minimum position vector norm (km) — zero-vector guard (0.1 mm).
pub const MIN_POSITION_NORM_KM: f64 = 1e-10;

/// Kepler's equation convergence tolerance
pub const KEPLER_TOL: f64 = 1e-14;

/// Maximum iterations for Kepler's equation
pub const KEPLER_MAX_ITER: usize = 50;

/// Earth J2000 frame for anise orbit construction.
pub const EARTH_J2000: anise::prelude::Frame = anise::prelude::Frame {
    ephemeris_id: 399,
    orientation_id: 1,
    mu_km3_s2: Some(MU_EARTH),
    shape: None,
};

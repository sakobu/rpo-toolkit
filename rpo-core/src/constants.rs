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

/// Minimum angular momentum norm below which the orbital plane is undefined (km²/s).
/// Used by DCM construction to reject rectilinear orbits or collinear position/velocity.
pub const MIN_ANGULAR_MOMENTUM_NORM_KM2_S: f64 = 1e-10;

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

// --- Covariance propagation constants ---

/// Tolerance for covariance matrix symmetry checks.
/// Used to verify P = Pᵀ within floating-point precision.
pub const COVARIANCE_SYMMETRY_TOL: f64 = 1e-12;

/// Default 1-sigma position accuracy per RIC axis (km).
/// 100 m is typical for GPS-based navigation in LEO proximity ops.
pub const DEFAULT_NAV_POSITION_SIGMA_KM: f64 = 0.1;

/// Default 1-sigma velocity accuracy per RIC axis (km/s).
/// 0.1 m/s = 1e-4 km/s, typical for GPS-based navigation.
pub const DEFAULT_NAV_VELOCITY_SIGMA_KM_S: f64 = 1e-4;

/// Default 1-sigma maneuver magnitude error (proportional). 1% = 0.01.
pub const DEFAULT_MANEUVER_MAGNITUDE_SIGMA: f64 = 0.01;

/// Default 1-sigma maneuver pointing error (rad). 1° ≈ 0.01745 rad.
pub const DEFAULT_MANEUVER_POINTING_SIGMA_RAD: f64 = 0.01745;

/// Floor for computed collision probability (avoids exact-zero reporting).
pub const COLLISION_PROBABILITY_FLOOR: f64 = 1e-15;

/// Default collision probability threshold (km) — minimum 3D distance
/// below which a sample is counted as a collision.
pub const MC_DEFAULT_COLLISION_THRESHOLD_KM: f64 = 0.1;

/// Default number of covariance sample points per leg.
pub const DEFAULT_COVARIANCE_SAMPLES_PER_LEG: usize = 100;

/// Singularity threshold for Δv direction vector normalization.
/// Below this norm, the Δv is treated as zero (no maneuver execution error).
pub const DV_NORM_ZERO_THRESHOLD_KM_S: f64 = 1e-15;

// --- Monte Carlo constants ---

/// Singularity threshold for random rotation axis normalization in Rodrigues' formula.
/// If the sampled 3D Gaussian axis has norm below this, a default axis is used.
/// Set to 1e-30 (well below f64 precision) to catch only near-zero vectors.
pub const RODRIGUES_AXIS_NORM_THRESHOLD: f64 = 1e-30;

/// Division-by-zero guard for covariance sigma in MC vs covariance comparison.
/// If predicted 1-sigma is below this, sigma ratio defaults to 1.0.
pub const COVARIANCE_SIGMA_FLOOR: f64 = 1e-15;

/// Minimum spacecraft dry mass (kg) after dispersion.
/// Prevents non-physical zero or negative mass in dispersed samples.
pub const MIN_SPACECRAFT_MASS_KG: f64 = 0.1;

/// Tolerance for floating-point comparison of elapsed time (seconds).
/// Used in test assertions that check whether an elapsed time falls within
/// a mission or trajectory duration. 1e-6 s = 1 µs, well above f64
/// rounding error for orbital-period-scale times (~5500 s).
pub const ELAPSED_TIME_TOL_S: f64 = 1e-6;

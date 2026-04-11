//! Physical and numerical constants for orbital mechanics.

/// Earth gravitational parameter (km³/s²)
pub const MU_EARTH: f64 = 398_600.441_8;

/// Earth equatorial radius (km)
pub const R_EARTH: f64 = 6378.137;

/// J2 zonal harmonic coefficient
pub const J2: f64 = 1.082_626_68e-3;

/// Two pi
pub const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Seconds per solar day (exact).
pub const SECONDS_PER_DAY: f64 = 86_400.0;

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

// --- Eclipse & celestial body constants ---

/// Astronomical Unit in kilometers (IAU 2012 exact definition).
pub const AU_KM: f64 = 149_597_870.700;

/// Mean solar radius in kilometers.
pub const SUN_RADIUS_KM: f64 = 696_000.0;

/// Mean obliquity of the ecliptic at J2000.0 (Meeus Eq. 22.2), in radians.
/// 23 deg 26' 21.448" = 23.4392911 deg = 0.409092804 rad
pub const OBLIQUITY_J2000_RAD: f64 = 0.409_092_804_22;

/// Julian Date of J2000.0 epoch (2000-01-01T12:00:00 TT).
pub const JD_J2000: f64 = 2_451_545.0;

/// Days per Julian century.
pub const DAYS_PER_JULIAN_CENTURY: f64 = 36_525.0;

/// Eclipse percentage difference below which two samples agree on
/// illumination state. Handles penumbra boundary: <1% difference is
/// effectively the same illumination state.
pub const ECLIPSE_PERCENTAGE_AGREEMENT_TOL: f64 = 1.0;

// --- Test tolerances ---
//
// Canonical named tolerances for test assertions across the workspace.
// Per CLAUDE.md tolerance policy: no anonymous thresholds in test code.
// Each constant's physical justification is documented once here,
// not re-justified per call site.

/// f64 round-off noise floor at LEO km-scale (≈ 1 nm).
///
/// Use when asserting that two positions derived from bit-equivalent
/// inputs agree within arithmetic round-off. Consumed by validation
/// tests that compare pre/post-refactor sample positions.
pub const TEST_F64_POSITION_NOISE_KM: f64 = 1.0e-12;

/// f64 epoch round-off budget across a single leg time-of-flight (≈ 1 ns).
///
/// Use when asserting that two epoch computations derived from
/// bit-equivalent inputs agree. Accounts for accumulated round-off
/// over `~5000 s` of `Epoch` arithmetic at f64 precision.
pub const TEST_F64_EPOCH_NOISE_S: f64 = 1.0e-9;

/// f64 identity-arithmetic tolerance (no floating-point ops performed).
///
/// Use when asserting that a value survives a pure move/copy with
/// zero arithmetic operations. Any larger slack indicates a bug.
pub const TEST_F64_EXACT_ARITHMETIC_TOL: f64 = 1.0e-15;

/// Tolerance for a single sqrt on a km-scale sum of squares.
///
/// Use when asserting that a norm computed two ways (e.g. `.norm()`
/// vs. manual `(x*x + y*y + z*z).sqrt()`) agrees. Single-sqrt error
/// accumulates at ~2 ULP over the LEO position magnitude.
pub const TEST_F64_SQRT_ACCUMULATION_TOL: f64 = 1.0e-12;

/// nyx integrator restart tolerance over a one-orbit split-vs-single
/// propagation (≈ 1 m).
///
/// Use when asserting that a segmented propagation (two half-legs
/// with a zero-impulse between them) agrees with a single continuous
/// propagation of the same leg. Reflects RK step-size adaptation and
/// restart overhead, not round-off.
pub const TEST_INTEGRATOR_RESTART_TOL_KM: f64 = 1.0e-3;

/// Safety-reduction residual tolerance for sampling-grid regressions
/// (≈ 1 mm).
///
/// Use when asserting that two sampling strategies yield the same
/// safety metric within mm-scale discretization error.
pub const TEST_SAMPLING_REGRESSION_TOL_KM: f64 = 1.0e-6;

// --- Integer-math helpers ---

/// Round-half-away-from-zero integer-percent multiply: `(x * pct + 50) / 100`.
///
/// Use when computing an integer percent of an integer count and
/// preferring round-half-up semantics (e.g. "30% of 50 samples,
/// rounded to the nearest sample"). Not the same as nearest-rank
/// percentile (which demands `ceil`, not round-half-up); prefer
/// `u32::div_ceil(100)` for percentile rank computations.
#[must_use]
pub const fn round_half_up_percent(x: u32, pct: u32) -> u32 {
    (x * pct + 50) / 100
}

#[cfg(test)]
mod tests {
    use super::round_half_up_percent;

    #[test]
    fn round_half_up_percent_matches_manual_idiom() {
        assert_eq!(round_half_up_percent(50, 30), 15);
        assert_eq!(round_half_up_percent(7, 50), 4);
        assert_eq!(round_half_up_percent(100, 0), 0);
        assert_eq!(round_half_up_percent(0, 100), 0);
        assert_eq!(round_half_up_percent(1, 100), 1);
    }
}

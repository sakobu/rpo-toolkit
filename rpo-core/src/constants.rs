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

/// Minimum position separation (km) between departure and arrival for a
/// valid Lambert problem.
///
/// Below this, the transfer angle is numerically undefined and both
/// Gooding and Izzo return degenerate or NaN solutions. `1e-6 km` (1 mm)
/// is well above f64 arithmetic noise at LEO-scale positions (≈ 7000 km)
/// and well below any physically meaningful separation for proximity ops.
/// Consumed by `rpo_nyx::lambert::validate_inputs` and surfaced to
/// callers via the invariants on `solve_lambert` / `solve_lambert_with_config`.
pub const LAMBERT_MIN_SEPARATION_KM: f64 = 1e-6;

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

/// Maximum expected Sun direction angular error (Meeus vs ANISE DE440s).
///
/// Meeus Ch. 25 with precession correction gives ~0.005 deg at 2024 epoch;
/// 0.02 deg (3.5e-4 rad) provides margin for nutation/perturbation residuals.
/// Consumed by `validate_sun_direction_accuracy` and the shared
/// `assert_eclipse_agreement` helper.
pub const SUN_DIRECTION_VALIDATION_TOL_RAD: f64 = 3.5e-4;

/// Maximum expected Moon direction angular error (Meeus vs ANISE DE440s).
///
/// Meeus Ch. 47 truncated series gives ~0.007 deg with precession correction;
/// 1.0 deg (0.0175 rad) provides generous margin for the truncated series.
pub const MOON_DIRECTION_VALIDATION_TOL_RAD: f64 = 0.0175;

/// Maximum eclipse entry/exit timing error (seconds).
///
/// With linear interpolation of eclipse boundaries between samples, the
/// error is dominated by the non-linearity of the shadow geometry rather
/// than sample-interval quantization.
pub const ECLIPSE_TIMING_VALIDATION_TOL_S: f64 = 120.0;

// --- Earth rotation & reference ellipsoid constants ---
//
// Arcsecond-class Earth rotation math for analytical frame transforms.
// No IERS Earth Orientation Parameters (polar motion, UT1-UTC correction):
// the ceiling reachable from WASM is ~arcsecond (≤ 65 μrad ≈ 13 arcsec),
// which is fine for 3D viewport pixels and acceptable for mission-planner-
// class ground-station predictions. Full-fidelity ITRF with EOP lives in
// rpo-nyx and is exposed via the WebSocket API.

/// Earth rotation rate in revolutions per UT1 day (IERS Conventions 2010, TN 36 Eq. 5.15).
///
/// Coefficient of the linear term in the Earth Rotation Angle formula.
/// Published value is `1.002_737_811_911_354_48`; truncated to f64 precision.
pub const EARTH_REV_PER_UT1_DAY: f64 = 1.002_737_811_911_354_6;

/// Earth Rotation Angle constant term, in revolutions (IERS TN 36 Eq. 5.15).
///
/// ERA at UT1 = 2000-01-01T12:00:00 TT is 2π × this value.
pub const ERA_CONSTANT_TURNS: f64 = 0.779_057_273_264_0;

/// Mean Earth angular velocity (rad/s).
///
/// Derived as `2π · EARTH_REV_PER_UT1_DAY / SECONDS_PER_DAY`. Used for the
/// ω × r correction when transforming ECI velocity to ECEF velocity.
/// LOD (length-of-day) variations are ignored — they contribute <10⁻¹⁰ rad/s.
pub const EARTH_ROTATION_RATE_RAD_S: f64 = 7.292_115_146_706_979e-5;

/// WGS-84 flattening of the reference ellipsoid.
///
/// `f = (a − b) / a`, exact per the WGS-84 defining parameter.
pub const EARTH_FLATTENING_WGS84: f64 = 1.0 / 298.257_223_563;

/// WGS-84 first eccentricity squared.
///
/// Defined as `e² = 2f − f²` (equivalently `1 − (b/a)²`). Expressed here as
/// the const derivation from `EARTH_FLATTENING_WGS84` so transcription errors
/// are impossible; the canonical published value is `6.694_379_990_141_316e-3`.
pub const EARTH_ECCENTRICITY_SQUARED_WGS84: f64 =
    2.0 * EARTH_FLATTENING_WGS84 - EARTH_FLATTENING_WGS84 * EARTH_FLATTENING_WGS84;

/// WGS-84 semi-minor (polar) radius in km.
///
/// Defined as `b = a · (1 − f)` with `a = R_EARTH = 6378.137 km`. Expressed
/// as the const derivation; published value is `6_356.752_314_245_179 km`.
pub const EARTH_POLAR_RADIUS_KM: f64 = R_EARTH * (1.0 - EARTH_FLATTENING_WGS84);

/// Maximum iterations for Bowring's geodetic-latitude iteration.
///
/// Bowring converges quadratically; 2–3 iterations suffice in practice.
/// 10 provides ample margin without ever being reached by well-posed inputs.
pub const GEODETIC_MAX_ITER: u32 = 10;

/// Convergence tolerance on geodetic latitude (rad).
///
/// `1e-12 rad` corresponds to ~6 μm on Earth's surface — orders of magnitude
/// below any rendering or station-placement precision budget.
pub const GEODETIC_LAT_TOL_RAD: f64 = 1e-12;

/// Singularity threshold (km) for the equatorial radius `p = sqrt(x² + y²)`
/// in `ecef_to_geodetic`.
///
/// Below this `p`, the geodetic latitude is undefined (the position is
/// essentially on the rotation axis) and the inversion uses the closed-form
/// polar branch instead of Bowring's iteration. Distinct from
/// [`MIN_POSITION_NORM_KM`] (zero-vector guard, 0.1 mm): a vehicle ~1 mm off
/// the polar axis is well above the zero-vector threshold but still
/// degenerate for latitude computation.
pub const GEODETIC_POLAR_EQUATORIAL_RADIUS_TOL_KM: f64 = 1e-6;

// --- Test tolerances ---
//
// Canonical named tolerances for test assertions across the workspace.
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
    use super::{
        round_half_up_percent, EARTH_ECCENTRICITY_SQUARED_WGS84, EARTH_POLAR_RADIUS_KM,
        EARTH_ROTATION_RATE_RAD_S,
    };

    /// Test tolerance: derived-vs-published comparison for a single-step
    /// f64 computation. ~4 ULP at unit scale; published constants are
    /// truncated to f64 precision so anything tighter is meaningless.
    const PUBLISHED_VALUE_TOL: f64 = 1e-15;

    /// Canonical WGS-84 first eccentricity squared (NIMA TR8350.2 §3.5.3).
    const PUBLISHED_WGS84_E2: f64 = 6.694_379_990_141_316e-3;

    /// Canonical WGS-84 semi-minor axis, km (NIMA TR8350.2 §3.5.3).
    const PUBLISHED_WGS84_B_KM: f64 = 6_356.752_314_245_179;

    /// Canonical Earth angular velocity (rad/s) derived from IERS TN 36
    /// Eq. 5.15: ω = 2π · `1.002_737_811_911_354_48` / 86400.
    const PUBLISHED_EARTH_OMEGA_RAD_S: f64 = 7.292_115_146_706_979e-5;

    #[test]
    fn round_half_up_percent_matches_manual_idiom() {
        assert_eq!(round_half_up_percent(50, 30), 15);
        assert_eq!(round_half_up_percent(7, 50), 4);
        assert_eq!(round_half_up_percent(100, 0), 0);
        assert_eq!(round_half_up_percent(0, 100), 0);
        assert_eq!(round_half_up_percent(1, 100), 1);
    }

    /// Regression: derived `EARTH_ECCENTRICITY_SQUARED_WGS84` matches the
    /// canonical published value (NIMA TR8350.2 §3.5.3).
    #[test]
    fn wgs84_eccentricity_squared_matches_published() {
        let diff = (EARTH_ECCENTRICITY_SQUARED_WGS84 - PUBLISHED_WGS84_E2).abs();
        assert!(
            diff / PUBLISHED_WGS84_E2 < PUBLISHED_VALUE_TOL,
            "WGS-84 e²: derived={EARTH_ECCENTRICITY_SQUARED_WGS84}, published={PUBLISHED_WGS84_E2}, rel diff={}",
            diff / PUBLISHED_WGS84_E2
        );
    }

    /// Regression: derived `EARTH_POLAR_RADIUS_KM` matches the canonical
    /// published value (NIMA TR8350.2 §3.5.3).
    #[test]
    fn wgs84_polar_radius_matches_published() {
        let diff = (EARTH_POLAR_RADIUS_KM - PUBLISHED_WGS84_B_KM).abs();
        assert!(
            diff / PUBLISHED_WGS84_B_KM < PUBLISHED_VALUE_TOL,
            "WGS-84 b: derived={EARTH_POLAR_RADIUS_KM} km, published={PUBLISHED_WGS84_B_KM} km, rel diff={}",
            diff / PUBLISHED_WGS84_B_KM
        );
    }

    /// Regression: `EARTH_ROTATION_RATE_RAD_S` matches the value derived
    /// from IERS TN 36 Eq. 5.15.
    #[test]
    fn earth_rotation_rate_matches_published() {
        let diff = (EARTH_ROTATION_RATE_RAD_S - PUBLISHED_EARTH_OMEGA_RAD_S).abs();
        assert!(
            diff / PUBLISHED_EARTH_OMEGA_RAD_S < PUBLISHED_VALUE_TOL,
            "Earth ω: stored={EARTH_ROTATION_RATE_RAD_S} rad/s, published={PUBLISHED_EARTH_OMEGA_RAD_S} rad/s, rel diff={}",
            diff / PUBLISHED_EARTH_OMEGA_RAD_S
        );
    }

}

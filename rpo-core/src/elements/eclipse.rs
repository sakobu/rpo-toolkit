//! Analytical celestial ephemeris and eclipse geometry.
//!
//! Provides Sun and Moon positions from any `hifitime::Epoch` using
//! low-precision analytical series (Meeus Ch. 25 and Ch. 47). No ephemeris
//! files, no ANISE dependency. Microseconds per call.
//!
//! # References
//!
//! - Meeus, Jean — *Astronomical Algorithms* (2nd ed., 1998), Ch. 25 (Sun), Ch. 47 (Moon)
//! - Montenbruck & Gill — *Satellite Orbits* (2000), Sec. 3.4 (shadow models)

use hifitime::Epoch;
use nalgebra::Vector3;

use crate::constants::{
    AU_KM, DAYS_PER_JULIAN_CENTURY, JD_J2000, OBLIQUITY_J2000_RAD, R_EARTH, SUN_RADIUS_KM,
};
use crate::types::{
    CelestialSnapshot, EclipseInterval, EclipseState, EclipseSummary, KeplerianElements,
    StateVector,
};

use super::keplerian_conversions::{keplerian_to_state, ConversionError};

/// Compute Julian centuries from J2000.0 for a given epoch.
///
/// Uses UTC approximation (TT-UTC ~ 69s). For the angular accuracy
/// required by shadow geometry (~0.01 deg), this is negligible.
fn julian_centuries_from_j2000(epoch: Epoch) -> f64 {
    let jd = epoch.to_jde_utc_days();
    (jd - JD_J2000) / DAYS_PER_JULIAN_CENTURY
}

/// Accumulated general precession in longitude (degrees) at Julian centuries `t`.
///
/// Lieske (1979) / Meeus Ch. 21: `ψ_A` = 5029.0966″ T + 1.1120″ T².
/// Used to convert ecliptic longitudes from mean equinox of date to J2000.
fn precession_in_longitude_deg(t: f64) -> f64 {
    (5029.0966 + 1.1120 * t) / 3600.0 * t
}

/// Compute the Sun's position in ECI J2000 coordinates at the given epoch.
///
/// Uses the low-precision analytical solar ephemeris from Meeus Ch. 25
/// ("Solar Coordinates"). Accuracy: ~0.01 deg in ecliptic longitude, which
/// corresponds to <1 second of eclipse entry/exit timing error for LEO.
///
/// # Arguments
///
/// * `epoch` — Absolute epoch (UTC). TT-UTC difference (~69s) is negligible
///   at this precision level.
///
/// # Returns
///
/// Sun position in ECI J2000 (km). The vector points from Earth center
/// to Sun center.
///
/// # Algorithm (Meeus Ch. 25)
///
/// 1. Compute Julian centuries T from J2000.0
/// 2. Mean longitude L0, mean anomaly M, eccentricity e of Earth's orbit
/// 3. Equation of center C (3-term sine series)
/// 4. Sun's ecliptic longitude lambda = L0 + C
/// 5. Sun-Earth distance R (AU) from orbital equation
/// 6. Rotate from ecliptic to equatorial using mean obliquity epsilon
/// 7. Scale to km
///
/// # Invariants
///
/// - Assumes geocentric perspective (Earth-centered)
/// - Uses mean obliquity at J2000.0 (no nutation or aberration corrections)
/// - UTC approximation for TT (TT-UTC ~ 69s is negligible at 0.01 deg accuracy)
/// - Ecliptic latitude of the Sun is zero (by definition for geocentric coordinates)
///
/// # Validity
///
/// Converges for any epoch within several thousand years of J2000.0.
/// The polynomial coefficients degrade slowly; accuracy remains ~0.01 deg
/// for epochs within +/- 100 years of J2000.0. No singularities — the
/// Keplerian orbital elements of Earth's orbit are well-behaved.
#[must_use]
pub fn sun_position_eci_km(epoch: Epoch) -> Vector3<f64> {
    let t = julian_centuries_from_j2000(epoch);

    // Step 2: Mean longitude, mean anomaly, eccentricity (Meeus Eq. 25.2–25.3)
    // All angles computed in degrees, converted to radians for trig.
    let l0_deg = 280.466_46 + 36_000.769_83 * t + 0.000_303_2 * t * t;
    let m_deg = 357.529_11 + 35_999.050_29 * t - 0.000_153_7 * t * t;
    let e = 0.016_708_634 - 0.000_042_037 * t - 0.000_000_126_7 * t * t;

    let m_rad = m_deg.to_radians();

    // Step 3: Equation of center (Meeus Eq. 25.6, 3-term sine series)
    let c_deg = (1.914_602 - 0.004_817 * t - 0.000_014 * t * t) * m_rad.sin()
        + (0.019_993 - 0.000_101 * t) * (2.0 * m_rad).sin()
        + 0.000_289 * (3.0 * m_rad).sin();

    // Step 4: Sun's true longitude (ecliptic, referred to mean equinox of date)
    let lambda_date_deg = l0_deg + c_deg;

    // Precession correction: Meeus L₀ includes precession (~1.397°/century),
    // so λ is referred to the mean equinox of the date. To get J2000 ecliptic
    // longitude, subtract the accumulated general precession in longitude.
    // Lieske (1979) / Meeus Ch. 21: ψ_A = 5029.0966" T + 1.1120" T²
    let precession_deg = precession_in_longitude_deg(t);
    let lambda_deg = lambda_date_deg - precession_deg;
    let lambda_rad = lambda_deg.to_radians();

    // Step 5: Sun's true anomaly and Sun-Earth distance (Meeus Eq. 25.5)
    let v_rad = m_rad + c_deg.to_radians();
    let r_au = 1.000_001_018 * (1.0 - e * e) / (1.0 + e * v_rad.cos());

    // Step 6: Ecliptic to equatorial rotation using mean obliquity at J2000.0
    // Sun's ecliptic latitude beta = 0 (by definition for geocentric Sun),
    // so the ecliptic coordinates are (R cos(lambda), R sin(lambda), 0).
    let cos_lambda = lambda_rad.cos();
    let sin_lambda = lambda_rad.sin();
    let cos_eps = OBLIQUITY_J2000_RAD.cos();
    let sin_eps = OBLIQUITY_J2000_RAD.sin();

    let x_au = r_au * cos_lambda;
    let y_au = r_au * sin_lambda * cos_eps;
    let z_au = r_au * sin_lambda * sin_eps;

    // Step 7: Scale to km
    Vector3::new(x_au * AU_KM, y_au * AU_KM, z_au * AU_KM)
}

/// Compute the Moon's position in ECI J2000 coordinates at the given epoch.
///
/// Uses a truncated version of the analytical lunar ephemeris from Meeus
/// Ch. 47 ("Position of the Moon"). The full theory has hundreds of periodic
/// terms; this implementation uses the ~20 largest terms for longitude,
/// ~13 for latitude, and ~10 for distance, giving ~0.5 deg accuracy in
/// ecliptic longitude and ~0.3 deg in latitude.
///
/// # Arguments
///
/// * `epoch` — Absolute epoch (UTC). TT-UTC difference (~69s) is negligible
///   at this precision level (~0.5 deg; the Moon moves ~0.5 deg/hour).
///
/// # Returns
///
/// Moon position in ECI J2000 (km). The vector points from Earth center
/// to Moon center.
///
/// # Algorithm (Meeus Ch. 47, truncated)
///
/// 1. Compute Julian centuries T from J2000.0
/// 2. Fundamental arguments: Moon's mean longitude L', mean elongation D,
///    Sun's mean anomaly M, Moon's mean anomaly M', argument of latitude F
/// 3. Sum ~20 largest periodic terms for ecliptic longitude sigma-l
/// 4. Sum ~13 largest periodic terms for ecliptic latitude sigma-b
/// 5. Sum ~10 largest periodic terms for distance sigma-r
/// 6. Compute geocentric ecliptic coordinates (lambda, beta, r)
/// 7. Rotate from ecliptic to equatorial (ECI J2000)
///
/// # Invariants
///
/// - Assumes geocentric perspective (Earth-centered)
/// - Uses mean obliquity at J2000.0 (no nutation or aberration corrections)
/// - UTC approximation for TT (negligible at 0.5 deg accuracy)
/// - Fundamental arguments use linear polynomials only (quadratic and higher
///   terms omitted; contributes <0.01 deg error within +/-100 years of J2000.0)
/// - Meeus E-correction factor for terms involving M (Sun's mean anomaly) is
///   omitted; largest omitted correction is ~0.0005 deg (negligible at 0.5 deg)
///
/// # Validity
///
/// Accuracy remains ~0.5 deg for epochs within +/-100 years of J2000.0.
/// The truncated series omits smaller periodic terms; for high-precision
/// applications (occultation timing, etc.), use a full ephemeris. No
/// singularities — the periodic series converges for all epochs.
#[must_use]
#[allow(clippy::similar_names)] // M (Sun) vs M' (Moon) are distinct standard symbols
pub fn moon_position_eci_km(epoch: Epoch) -> Vector3<f64> {
    let t = julian_centuries_from_j2000(epoch);

    // Step 2: Fundamental arguments (Meeus Ch. 47, all in degrees)
    // L' — Moon's mean longitude, referred to the mean equinox of date
    let lp_deg = 218.316_9 + 481_267.881_3 * t;
    // D — Mean elongation of the Moon
    let d_deg = 297.850_2 + 445_267.111_5 * t;
    // M — Sun's mean anomaly
    let m_deg = 357.529_1 + 35_999.050_3 * t;
    // M' — Moon's mean anomaly
    let mp_deg = 134.963_4 + 477_198.867_6 * t;
    // F — Moon's argument of latitude (mean distance from ascending node)
    let f_deg = 93.272_0 + 483_202.017_5 * t;

    // Convert to radians for trig
    let d = d_deg.to_radians();
    let m = m_deg.to_radians();
    let mp = mp_deg.to_radians();
    let f = f_deg.to_radians();

    // Step 3: Longitude terms sigma-l (units: 10^-6 degrees) — 20 largest from Meeus Table 47.A
    // Each term: coefficient * sin(D_mult*D + M_mult*M + Mp_mult*M' + F_mult*F)
    let sigma_l: f64 = [
        ( 6_288_774.0, 0.0, 0.0, 1.0, 0.0),   // sin(M')
        ( 1_274_027.0, 2.0, 0.0,-1.0, 0.0),    // sin(2D - M')
        (   658_314.0, 2.0, 0.0, 0.0, 0.0),    // sin(2D)
        (   213_618.0, 0.0, 0.0, 2.0, 0.0),    // sin(2M')
        (  -185_116.0, 0.0, 1.0, 0.0, 0.0),    // sin(M)
        (  -114_332.0, 0.0, 0.0, 0.0, 2.0),    // sin(2F)
        (    58_793.0, 2.0, 0.0,-2.0, 0.0),    // sin(2D - 2M')
        (    57_066.0, 2.0,-1.0,-1.0, 0.0),    // sin(2D - M - M')
        (    53_322.0, 2.0, 0.0, 1.0, 0.0),    // sin(2D + M')
        (    45_758.0, 2.0,-1.0, 0.0, 0.0),    // sin(2D - M)
        (   -40_923.0, 0.0, 1.0,-1.0, 0.0),    // sin(M - M')
        (   -34_720.0, 1.0, 0.0, 0.0, 0.0),    // sin(D)
        (   -30_383.0, 0.0, 1.0, 1.0, 0.0),    // sin(M + M')
        (    15_327.0, 2.0, 0.0, 0.0,-2.0),    // sin(2D - 2F)
        (   -12_528.0, 0.0, 0.0, 1.0, 2.0),    // sin(M' + 2F)
        (    10_980.0, 0.0, 0.0, 1.0,-2.0),    // sin(M' - 2F)
        (    10_675.0, 4.0, 0.0,-1.0, 0.0),    // sin(4D - M')
        (    10_034.0, 0.0, 0.0, 3.0, 0.0),    // sin(3M')
        (     8_548.0, 4.0, 0.0,-2.0, 0.0),    // sin(4D - 2M')
        (    -7_888.0, 2.0, 1.0,-1.0, 0.0),    // sin(2D + M - M')
    ].iter().map(|&(coeff, d_m, m_m, mp_m, f_m)| {
        coeff * (d_m * d + m_m * m + mp_m * mp + f_m * f).sin()
    }).sum();

    // Step 4: Latitude terms sigma-b (units: 10^-6 degrees) — 13 largest from Meeus Table 47.B
    let sigma_b: f64 = [
        ( 5_128_122.0, 0.0, 0.0, 0.0, 1.0),   // sin(F)
        (   280_602.0, 0.0, 0.0, 1.0, 1.0),    // sin(M' + F)
        (   277_693.0, 0.0, 0.0, 1.0,-1.0),    // sin(M' - F)
        (   173_237.0, 2.0, 0.0, 0.0,-1.0),    // sin(2D - F)
        (    55_413.0, 2.0, 0.0,-1.0, 1.0),    // sin(2D - M' + F)
        (    46_271.0, 2.0, 0.0,-1.0,-1.0),    // sin(2D - M' - F)
        (    32_573.0, 2.0, 0.0, 0.0, 1.0),    // sin(2D + F)
        (    17_198.0, 0.0, 0.0, 2.0, 1.0),    // sin(2M' + F)
        (     9_266.0, 2.0, 0.0, 1.0,-1.0),    // sin(2D + M' - F)
        (     8_822.0, 0.0, 0.0, 2.0,-1.0),    // sin(2M' - F)
        (     8_216.0, 2.0,-1.0, 0.0,-1.0),    // sin(2D - M - F)
        (     4_324.0, 2.0, 0.0,-2.0,-1.0),    // sin(2D - 2M' - F)
        (     4_200.0, 2.0, 0.0, 1.0, 1.0),    // sin(2D + M' + F)
    ].iter().map(|&(coeff, d_m, m_m, mp_m, f_m)| {
        coeff * (d_m * d + m_m * m + mp_m * mp + f_m * f).sin()
    }).sum();

    // Step 5: Distance terms sigma-r (units: meters) — 10 largest from Meeus Table 47.A
    // Note: distance terms use cosine, not sine.
    let sigma_r: f64 = [
        (-20_905_355.0, 0.0, 0.0, 1.0, 0.0),  // cos(M')
        ( -3_699_111.0, 2.0, 0.0,-1.0, 0.0),   // cos(2D - M')
        ( -2_955_968.0, 2.0, 0.0, 0.0, 0.0),   // cos(2D)
        (   -569_925.0, 0.0, 0.0, 2.0, 0.0),   // cos(2M')
        (     48_888.0, 0.0, 1.0, 0.0, 0.0),   // cos(M)
        (     -3_149.0, 0.0, 0.0, 0.0, 2.0),   // cos(2F)
        (    246_158.0, 2.0, 0.0,-2.0, 0.0),    // cos(2D - 2M')
        (   -152_138.0, 2.0,-1.0,-1.0, 0.0),   // cos(2D - M - M')
        (   -170_733.0, 2.0, 0.0, 1.0, 0.0),   // cos(2D + M')
        (   -204_586.0, 2.0,-1.0, 0.0, 0.0),   // cos(2D - M)
    ].iter().map(|&(coeff, d_m, m_m, mp_m, f_m)| {
        coeff * (d_m * d + m_m * m + mp_m * mp + f_m * f).cos()
    }).sum();

    // Step 6: Geocentric ecliptic coordinates
    // lambda (ecliptic longitude, degrees): L' + sigma_l in 10^-6 degrees
    // L' includes precession, so lambda is referred to the mean equinox of date.
    // Apply the same precession correction as the Sun to get J2000 longitude.
    let precession_deg = precession_in_longitude_deg(t);
    let lambda_deg = lp_deg + sigma_l / 1_000_000.0 - precession_deg;
    // beta (ecliptic latitude, degrees): sigma_b in 10^-6 degrees
    let beta_deg = sigma_b / 1_000_000.0;
    // r (geocentric distance, km): Meeus reference distance + sigma_r (meters -> km)
    let r_km = 385_000.56 + sigma_r / 1000.0;

    let lambda_rad = lambda_deg.to_radians();
    let beta_rad = beta_deg.to_radians();

    // Ecliptic cartesian
    let cos_beta = beta_rad.cos();
    let x_ecl = r_km * cos_beta * lambda_rad.cos();
    let y_ecl = r_km * cos_beta * lambda_rad.sin();
    let z_ecl = r_km * beta_rad.sin();

    // Step 7: Ecliptic to equatorial rotation (same obliquity as Sun ephemeris)
    let cos_eps = OBLIQUITY_J2000_RAD.cos();
    let sin_eps = OBLIQUITY_J2000_RAD.sin();

    let x_eq = x_ecl;
    let y_eq = y_ecl * cos_eps - z_ecl * sin_eps;
    let z_eq = y_ecl * sin_eps + z_ecl * cos_eps;

    Vector3::new(x_eq, y_eq, z_eq)
}

/// Classify the eclipse state of a spacecraft using the conical Earth shadow model.
///
/// Models Earth as a sphere of radius `R_EARTH` and the Sun as a sphere of radius
/// `SUN_RADIUS_KM` at distance `sun_distance_km`. This produces two shadow cones:
///
/// - **Umbra**: full shadow cone (converging behind Earth)
/// - **Penumbra**: partial shadow cone (diverging behind Earth)
///
/// # Arguments
///
/// * `spacecraft_eci_km` - Spacecraft position in ECI J2000 (km)
/// * `sun_eci_km` - Sun position in ECI J2000 (km), from Earth center to Sun center
///
/// # Returns
///
/// `EclipseState::Sunlit`, `Penumbra { shadow_fraction }`, or `Umbra`.
///
/// # Algorithm (Montenbruck & Gill, Sec. 3.4)
///
/// 1. Compute apparent angular radii of Sun and Earth as seen from spacecraft
/// 2. Compute angular separation between Sun center and Earth center as seen from spacecraft
/// 3. Classify based on overlap geometry:
///    - No overlap -> Sunlit
///    - Partial overlap -> Penumbra (`shadow_fraction` from linear interpolation)
///    - Full overlap (Earth covers Sun) -> Umbra
///
/// # Invariants
///
/// - `spacecraft_eci_km` must be non-zero (spacecraft above Earth's surface)
/// - `sun_eci_km` must be non-zero (well-defined Sun position)
/// - Both vectors are in ECI J2000 (km), same frame and units
///
/// # Validity
///
/// - Valid for any Earth-orbiting spacecraft above Earth's surface
/// - Assumes spherical Earth (no oblateness correction — <0.3% shadow boundary shift for LEO)
/// - Ignores atmospheric refraction (adds ~1s uncertainty to eclipse transitions)
/// - Shadow fraction uses linear interpolation between penumbra boundaries,
///   which is acceptable for visualization (exact disk-overlap area formula
///   adds complexity for <1% accuracy gain at LEO)
#[must_use]
pub fn compute_eclipse_state(
    spacecraft_eci_km: &Vector3<f64>,
    sun_eci_km: &Vector3<f64>,
) -> EclipseState {
    // Spacecraft-to-Sun vector and distance
    let sc_to_sun = sun_eci_km - spacecraft_eci_km;
    let sc_to_sun_dist = sc_to_sun.norm();

    // Spacecraft distance from Earth center
    let sc_to_earth_dist = spacecraft_eci_km.norm();

    // Apparent angular radii as seen from spacecraft (rad)
    // Clamp asin arguments for robustness near Earth's surface
    let theta_sun = (SUN_RADIUS_KM / sc_to_sun_dist).clamp(-1.0, 1.0).asin();
    let theta_earth = (R_EARTH / sc_to_earth_dist).clamp(-1.0, 1.0).asin();

    // Angular separation between Sun center and Earth center as seen from spacecraft
    // Sun direction from spacecraft: sc_to_sun / sc_to_sun_dist
    // Earth direction from spacecraft: -spacecraft_eci_km / sc_to_earth_dist
    let cos_theta_sep =
        sc_to_sun.dot(&(-spacecraft_eci_km)) / (sc_to_sun_dist * sc_to_earth_dist);
    let theta_sep = cos_theta_sep.clamp(-1.0, 1.0).acos();

    // Classification based on angular overlap geometry
    let penumbra_boundary = theta_sun + theta_earth;
    let umbra_boundary = theta_earth - theta_sun;

    if theta_sep >= penumbra_boundary {
        // No geometric overlap — fully illuminated
        EclipseState::Sunlit
    } else if theta_earth >= theta_sun && theta_sep <= umbra_boundary {
        // Earth fully covers Sun disk — full shadow
        EclipseState::Umbra
    } else {
        // Partial overlap — linear interpolation for shadow fraction
        // At penumbra entry (theta_sep = penumbra_boundary): shadow_fraction = 0.0
        // At umbra entry (theta_sep = umbra_boundary): shadow_fraction = 1.0
        let shadow_fraction =
            ((penumbra_boundary - theta_sep) / (2.0 * theta_sun)).clamp(0.0, 1.0);
        EclipseState::Penumbra { shadow_fraction }
    }
}

/// Build a single [`CelestialSnapshot`] from a spacecraft ECI position and epoch.
///
/// Computes Sun and Moon positions analytically, evaluates the conical shadow
/// model, and produces direction/distance fields. Shared by both
/// [`compute_celestial_snapshots`] and [`compute_eclipse_from_states`].
pub(crate) fn build_celestial_snapshot(epoch: Epoch, sc_eci_km: &Vector3<f64>) -> CelestialSnapshot {
    let sun_pos = sun_position_eci_km(epoch);
    let moon_pos = moon_position_eci_km(epoch);
    let eclipse_state = compute_eclipse_state(sc_eci_km, &sun_pos);

    let sc_to_sun = sun_pos - sc_eci_km;
    let sc_to_moon = moon_pos - sc_eci_km;
    let sun_dist = sc_to_sun.norm();
    let moon_dist = sc_to_moon.norm();

    CelestialSnapshot {
        epoch,
        sun_direction_eci: sc_to_sun / sun_dist,
        moon_direction_eci: sc_to_moon / moon_dist,
        sun_distance_km: sun_dist,
        moon_distance_km: moon_dist,
        eclipse_state,
    }
}

/// Compute celestial body directions and eclipse state at each trajectory point.
///
/// For each `(Epoch, KeplerianElements)` pair, reconstructs the chief ECI
/// position via [`keplerian_to_state()`], computes Sun and Moon positions
/// analytically, and evaluates the conical shadow model.
///
/// # Arguments
///
/// * `points` — Slice of `(epoch, chief_mean_elements)` pairs. The caller
///   extracts these from `PropagatedState` or any other source — this function
///   has no dependency on propagation types, preserving the module DAG:
///   `constants → types → elements → propagation → mission`.
///
/// # Returns
///
/// One [`CelestialSnapshot`] per input point, in the same order. Each
/// snapshot's `epoch` field is set from the corresponding input pair,
/// making the output slice self-contained for downstream consumers
/// (e.g., [`extract_eclipse_intervals`]).
///
/// Returns an empty `Vec` for empty input.
///
/// # Mean elements approximation
///
/// The input Keplerian elements are typically **mean** elements (from
/// `PropagatedState.chief_mean`), but [`keplerian_to_state()`] treats them
/// as osculating. This introduces a systematic ~1–10 km offset in the
/// reconstructed ECI position due to J2 short-period terms. For shadow
/// geometry at LEO this is negligible: the Sun subtends ~0.5° and Earth
/// subtends ~60° as seen from the spacecraft, so a few km offset barely
/// shifts the shadow boundary. The resulting eclipse timing error is well
/// under 1 second.
///
/// # Performance
///
/// ~350 ns per point (Sun + Moon ephemeris + shadow test + ECI reconstruction).
/// A 7-hour mission at 1-second sampling (25,200 points) completes in ~9 ms.
///
/// # Errors
///
/// Propagates [`ConversionError`] from [`keplerian_to_state()`] if any input
/// element set is degenerate (zero position, unbound orbit).
///
/// # Invariants
///
/// - Each `KeplerianElements` must describe a valid bound orbit (`e < 1`, `a > 0`);
///   degenerate elements propagate as `Err(ConversionError)` from `keplerian_to_state()`
/// - Points should be time-ordered (monotonically increasing epochs) for meaningful
///   downstream interval extraction; this function does not enforce ordering
///
/// # Validity
///
/// - Valid for any Earth-orbiting spacecraft in a bound orbit
/// - No singularities beyond those in [`keplerian_to_state()`] (near-parabolic,
///   zero position)
/// - Accuracy is limited by the analytical ephemeris (~0.01 deg Sun, ~0.5 deg Moon)
///   and the mean-elements approximation (see above)
pub fn compute_celestial_snapshots(
    points: &[(Epoch, KeplerianElements)],
) -> Result<Vec<CelestialSnapshot>, ConversionError> {
    let mut snapshots = Vec::with_capacity(points.len());

    for (epoch, elements) in points {
        let state = keplerian_to_state(elements, *epoch)?;
        snapshots.push(build_celestial_snapshot(*epoch, &state.position_eci_km));
    }

    Ok(snapshots)
}

/// Compute celestial body directions and eclipse state from ECI state vectors.
///
/// Unlike [`compute_celestial_snapshots`] which takes `(Epoch, KeplerianElements)`
/// and reconstructs ECI positions, this function operates directly on ECI state
/// vectors. Suitable for trajectories already expressed in ECI (e.g., Lambert
/// transfer arcs from [`LambertTransfer::densify_arc()`][crate::propagation::lambert::LambertTransfer::densify_arc],
/// Keplerian propagation results).
///
/// # Arguments
///
/// * `states` — ECI state vectors. Each must have a valid `epoch` and
///   non-zero `position_eci_km`.
///
/// # Performance
///
/// ~250 ns per point (Sun + Moon ephemeris + shadow test, no Keplerian conversion).
#[must_use]
pub fn compute_eclipse_from_states(states: &[StateVector]) -> Vec<CelestialSnapshot> {
    states
        .iter()
        .map(|s| build_celestial_snapshot(s.epoch, &s.position_eci_km))
        .collect()
}

/// Extract eclipse intervals from a sequence of celestial snapshots.
///
/// Walks the snapshot array, detects transitions between Sunlit and shadow
/// states (Penumbra or Umbra), and records each contiguous shadow interval
/// with its start/end epochs and duration.
///
/// Transition times are estimated at the sample point where the state
/// changes. For 1-second sampling, this gives sub-second accuracy on
/// entry/exit times.
///
/// # Arguments
///
/// * `snapshots` — Celestial snapshots (from [`compute_celestial_snapshots`]).
///   Each snapshot carries its own `epoch`, so no parallel epoch array is needed.
///
/// # Returns
///
/// [`EclipseSummary`] containing all shadow intervals, total shadow duration,
/// time-in-shadow fraction, and maximum single-interval duration.
/// Returns an empty summary (zero intervals, zero durations) for empty input
/// or single-point input (need >= 2 points for transition detection).
///
/// # Invariants
///
/// - Snapshots should be time-ordered (monotonically increasing epochs);
///   non-monotonic input produces intervals with negative or incorrect durations
/// - Snapshots are assumed well-formed (produced by [`compute_celestial_snapshots`])
///
/// # Validity
///
/// - Eclipse entry/exit timing accuracy is bounded by the sampling interval
///   (e.g., 1-second sampling gives sub-second transition accuracy)
/// - Shadow fraction within intervals uses worst-case (deepest) state, not
///   time-averaged; this is conservative for power budget analysis
/// - No singularities; the function is infallible on well-formed input
#[must_use]
pub fn extract_eclipse_intervals(snapshots: &[CelestialSnapshot]) -> EclipseSummary {
    let empty = EclipseSummary {
        intervals: Vec::new(),
        total_shadow_duration_s: 0.0,
        time_in_shadow_fraction: 0.0,
        max_shadow_duration_s: 0.0,
    };

    if snapshots.len() < 2 {
        return empty;
    }

    let mut intervals: Vec<EclipseInterval> = Vec::new();
    let mut shadow_start: Option<Epoch> = None;
    let mut worst_state = EclipseState::Sunlit;

    for snapshot in snapshots {
        let in_shadow = snapshot.eclipse_state != EclipseState::Sunlit;

        match (in_shadow, shadow_start) {
            // Entering shadow
            (true, None) => {
                shadow_start = Some(snapshot.epoch);
                worst_state = snapshot.eclipse_state;
            }
            // Continuing in shadow — track worst state
            (true, Some(_)) => {
                if is_deeper_shadow(&snapshot.eclipse_state, &worst_state) {
                    worst_state = snapshot.eclipse_state;
                }
            }
            // Exiting shadow — close interval
            (false, Some(start)) => {
                let duration_s = (snapshot.epoch - start).to_seconds();
                intervals.push(EclipseInterval {
                    start,
                    end: snapshot.epoch,
                    duration_s,
                    state: worst_state,
                });
                shadow_start = None;
                worst_state = EclipseState::Sunlit;
            }
            // Remaining sunlit
            (false, None) => {}
        }
    }

    // Close any open interval at the last snapshot
    if let Some(start) = shadow_start
        && let Some(last) = snapshots.last()
    {
        let duration_s = (last.epoch - start).to_seconds();
        intervals.push(EclipseInterval {
            start,
            end: last.epoch,
            duration_s,
            state: worst_state,
        });
    }

    // Compute aggregate metrics
    let total_shadow_duration_s: f64 = intervals.iter().map(|i| i.duration_s).sum();
    let max_shadow_duration_s = intervals
        .iter()
        .map(|i| i.duration_s)
        .fold(0.0_f64, f64::max);

    // Safety: we returned early above when snapshots.len() < 2
    let total_mission_duration_s =
        (snapshots[snapshots.len() - 1].epoch - snapshots[0].epoch).to_seconds();

    let time_in_shadow_fraction = if total_mission_duration_s > 0.0 {
        total_shadow_duration_s / total_mission_duration_s
    } else {
        0.0
    };

    EclipseSummary {
        intervals,
        total_shadow_duration_s,
        time_in_shadow_fraction,
        max_shadow_duration_s,
    }
}

/// Returns `true` if `new` represents a deeper shadow than `current`.
pub(crate) fn is_deeper_shadow(new: &EclipseState, current: &EclipseState) -> bool {
    match (new, current) {
        (EclipseState::Umbra, EclipseState::Penumbra { .. }) => true,
        (
            EclipseState::Penumbra {
                shadow_fraction: f_new,
            },
            EclipseState::Penumbra {
                shadow_fraction: f_cur,
            },
        ) => f_new > f_cur,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hifitime::Epoch;

    use crate::constants::{R_EARTH, SUN_RADIUS_KM, AU_KM, TWO_PI};
    use crate::types::EclipseState;
    use crate::elements::keplerian_conversions::keplerian_to_state;

    /// Direction tolerance: 0.05 deg = 8.73e-4 rad.
    /// Meeus Ch. 25 states ~0.01 deg ecliptic longitude accuracy, but the total
    /// angular separation after ecliptic-to-equatorial rotation accumulates
    /// additional error from the constant obliquity approximation and the lack
    /// of aberration/nutation corrections. At LEO, 0.05 deg shifts the shadow
    /// boundary by ~6 km — still <0.1% of orbit circumference.
    const SUN_DIRECTION_TEST_TOL_RAD: f64 = 8.73e-4;

    /// Distance tolerance: 0.1% relative error.
    /// 0.1% of 1 AU = ~150,000 km, which shifts penumbra cone angle by ~0.0001 deg.
    const SUN_DISTANCE_RELATIVE_TOL: f64 = 0.001;

    /// Equinox longitude tolerance: 1 deg.
    /// At the vernal equinox the Sun crosses 0 deg RA; we verify the ecliptic
    /// longitude is near 0 deg (mod 360 deg) within this tolerance.
    const EQUINOX_LONGITUDE_TOL_RAD: f64 = 0.0175;

    /// Solstice declination tolerance: 0.5 deg.
    /// At summer solstice the Sun's declination peaks at ~+23.44 deg;
    /// we verify the computed declination is within 0.5 deg of this.
    const SOLSTICE_DECLINATION_TOL_RAD: f64 = 0.008_73;

    /// Shadow fraction symmetry: two symmetric positions should produce
    /// identical shadow fractions. 1e-12 covers f64 evaluation of
    /// the conical geometry formula for mirrored inputs.
    const SHADOW_FRACTION_SYMMETRY_TOL: f64 = 1e-12;

    /// Sun direction cross-validation tolerance between `compute_eclipse_from_states`
    /// and `compute_celestial_snapshots`. Both use Meeus Sun, but the slightly
    /// different spacecraft ECI positions (mean vs osculating) shift the
    /// Earth→Sun unit vector by O(1e-4) rad. 1e-3 rad (0.06°) provides margin.
    const SUN_DIRECTION_CROSS_VALIDATION_TOL_RAD: f64 = 1e-3;

    /// Moon direction tolerance: 1.0 deg = 0.01745 rad.
    /// Meeus Ch. 47 truncated to ~20 terms gives ~0.5 deg accuracy in ecliptic
    /// longitude. After ecliptic-to-equatorial rotation, total angular error
    /// can reach ~1 deg. At ~384,400 km distance, 1 deg ~ 6,700 km positional
    /// error — invisible for visualization direction lines from LEO (~6800 km orbit).
    const MOON_DIRECTION_TEST_TOL_RAD: f64 = 0.01745;

    /// Moon distance tolerance: 2% relative error.
    /// 2% of 384,400 km ~ 7,700 km. The truncated series omits smaller
    /// periodic terms that modulate distance. For shadow geometry this is
    /// irrelevant — the Moon's angular size as seen from LEO is ~0.5 deg
    /// regardless of +/-2% distance variation.
    const MOON_DISTANCE_RELATIVE_TOL: f64 = 0.02;

    /// Full Moon elongation tolerance: 5 deg.
    /// At a full moon the Sun-Earth-Moon angle is ~180 deg. We verify
    /// the angle between Sun and Moon directions (as seen from Earth)
    /// is within 5 deg of 180 deg, allowing for orbital inclination
    /// and the truncated series approximation.
    const FULL_MOON_ELONGATION_TOL_RAD: f64 = 0.0873;

    /// Maximum ecliptic latitude bound: 5.3 deg = 0.0925 rad.
    /// The Moon's orbital inclination to the ecliptic is ~5.145 deg.
    /// With perturbations, ecliptic latitude should never exceed ~5.3 deg.
    const MOON_MAX_ECLIPTIC_LATITUDE_RAD: f64 = 0.0925;

    /// Lunar perigee distance bound (km).
    /// True perigee ~356,500 km; generous lower bound accommodates the truncated
    /// Meeus series which omits smaller periodic distance terms.
    const MOON_PERIGEE_BOUND_KM: f64 = 356_000.0;

    /// Lunar apogee distance bound (km).
    /// True apogee ~406,700 km; generous upper bound accommodates the truncated
    /// Meeus series.
    const MOON_APOGEE_BOUND_KM: f64 = 407_000.0;

    // --- Shadow geometry test tolerances ---

    /// Tolerance for penumbra midpoint test (shadow_fraction ~ 0.5).
    /// The test constructs the spacecraft position using small-angle geometry
    /// approximations, which introduces systematic offset from the exact
    /// midpoint. 0.10 tolerance accommodates the approximation error at GEO
    /// distance.
    const PENUMBRA_MIDPOINT_TOL: f64 = 0.10;

    /// Floating-point guard for monotonicity assertions.
    /// Allows shadow_fraction to decrease by at most this amount between
    /// adjacent samples due to floating-point rounding, without flagging
    /// a false monotonicity violation. 1e-10 is well above f64 epsilon
    /// but well below any physically meaningful shadow fraction change.
    const MONOTONICITY_EPSILON: f64 = 1e-10;

    /// ISS eclipse fraction tolerance (absolute).
    /// Published ISS eclipse fraction is ~35-36% of orbit period. We allow
    /// 5% absolute tolerance because: (1) analytical Sun position has ~0.01 deg
    /// error, (2) we sample 360 discrete points around the orbit, (3) the ISS
    /// fixture uses simplified near-circular elements, (4) the beta angle at
    /// the test epoch affects the exact fraction.
    const ISS_ECLIPSE_FRACTION_TOL: f64 = 0.05;

    /// Sun position at J2000.0 (2000-01-01T12:00:00 UTC).
    ///
    /// Reference: JPL Horizons geocentric Sun position at J2000.0.
    /// Ecliptic longitude ~280.5 deg (near winter solstice, ~10 days after).
    /// Distance ~0.9833 AU (near perihelion, ~Jan 3).
    #[test]
    fn sun_position_j2000_epoch() {
        let epoch = Epoch::from_gregorian_utc_hms(2000, 1, 1, 12, 0, 0);
        let pos = sun_position_eci_km(epoch);

        let r_km = pos.norm();
        let r_au = r_km / AU_KM;

        // Distance check: ~0.9833 AU (near perihelion)
        let expected_r_au = 0.9833;
        let rel_err = (r_au - expected_r_au).abs() / expected_r_au;
        assert!(
            rel_err < SUN_DISTANCE_RELATIVE_TOL,
            "J2000 Sun distance: expected ~{expected_r_au} AU, got {r_au} AU (rel err {rel_err})"
        );

        // Direction check: compare unit vector against JPL Horizons.
        // At J2000.0, Sun is at ecliptic longitude ~280.5 deg.
        // JPL Horizons geocentric Sun: RA = 18h 45m 03s = 281.26 deg, Dec = -23.04 deg.
        let expected_ra_rad = 281.26_f64.to_radians();
        let expected_dec_rad = (-23.04_f64).to_radians();
        let expected_dir = Vector3::new(
            expected_dec_rad.cos() * expected_ra_rad.cos(),
            expected_dec_rad.cos() * expected_ra_rad.sin(),
            expected_dec_rad.sin(),
        );

        let computed_dir = pos.normalize();
        let angle_err_rad = computed_dir.dot(&expected_dir).clamp(-1.0, 1.0).acos();
        assert!(
            angle_err_rad < SUN_DIRECTION_TEST_TOL_RAD,
            "J2000 Sun direction error: {:.4} deg (tolerance {:.4} deg)",
            angle_err_rad.to_degrees(),
            SUN_DIRECTION_TEST_TOL_RAD.to_degrees()
        );
    }

    /// Sun near +X ECI at the vernal equinox (2024-03-20).
    ///
    /// At the vernal equinox, the Sun's ecliptic longitude crosses 0 deg,
    /// placing it near the +X ECI axis (RA ~ 0 deg, Dec ~ 0 deg).
    #[test]
    fn sun_position_vernal_equinox_2024() {
        // 2024 vernal equinox: March 20, ~03:06 UTC
        let epoch = Epoch::from_gregorian_utc_hms(2024, 3, 20, 3, 6, 0);
        let pos = sun_position_eci_km(epoch);

        // At vernal equinox, the Sun is at RA ~0 deg, Dec ~0 deg.
        // The ecliptic longitude is 0 deg, so in equatorial:
        // x ~ R, y ~ 0, z ~ 0 (Sun along +X ECI).
        let dir = pos.normalize();

        // Check that x > 0 (Sun in +X hemisphere)
        assert!(
            dir.x > 0.0,
            "Vernal equinox: Sun should be in +X ECI hemisphere, got x={}",
            dir.x
        );

        // Check RA near 0 deg
        let ra_rad = dir.y.atan2(dir.x);
        assert!(
            ra_rad.abs() < EQUINOX_LONGITUDE_TOL_RAD,
            "Vernal equinox RA: {:.4} deg (tolerance {:.4} deg)",
            ra_rad.to_degrees(),
            EQUINOX_LONGITUDE_TOL_RAD.to_degrees()
        );

        // Check Dec near 0 deg
        let dec_rad = dir.z.asin();
        assert!(
            dec_rad.abs() < EQUINOX_LONGITUDE_TOL_RAD,
            "Vernal equinox Dec: {:.4} deg (tolerance {:.4} deg)",
            dec_rad.to_degrees(),
            EQUINOX_LONGITUDE_TOL_RAD.to_degrees()
        );
    }

    /// Sun at max declination ~+23.4 deg at summer solstice 2024.
    ///
    /// At the summer solstice, the Sun reaches maximum positive declination
    /// equal to the obliquity of the ecliptic (~23.44 deg).
    #[test]
    fn sun_position_summer_solstice_2024() {
        // 2024 summer solstice: June 20, ~20:51 UTC
        let epoch = Epoch::from_gregorian_utc_hms(2024, 6, 20, 20, 51, 0);
        let pos = sun_position_eci_km(epoch);

        let dir = pos.normalize();
        let dec_rad = dir.z.asin();

        let expected_dec_rad = OBLIQUITY_J2000_RAD; // ~23.44 deg
        let dec_err = (dec_rad - expected_dec_rad).abs();
        assert!(
            dec_err < SOLSTICE_DECLINATION_TOL_RAD,
            "Summer solstice declination: {:.4} deg, expected ~{:.4} deg (error {:.4} deg, tol {:.4} deg)",
            dec_rad.to_degrees(),
            expected_dec_rad.to_degrees(),
            dec_err.to_degrees(),
            SOLSTICE_DECLINATION_TOL_RAD.to_degrees()
        );
    }

    /// Sun-Earth distance stays within 0.983–1.017 AU for 2020–2030.
    ///
    /// Earth's orbital eccentricity (~0.0167) produces perihelion ~0.983 AU
    /// (early January) and aphelion ~1.017 AU (early July). This test
    /// samples the first of each month over a decade.
    #[test]
    fn sun_distance_range() {
        let min_au = 0.983;
        let max_au = 1.017;

        for year in 2020..=2030 {
            for month in 1..=12 {
                let epoch = Epoch::from_gregorian_utc_hms(year, month, 1, 0, 0, 0);
                let pos = sun_position_eci_km(epoch);
                let r_au = pos.norm() / AU_KM;

                assert!(
                    r_au > min_au && r_au < max_au,
                    "Sun distance out of range at {year}-{month:02}-01: {r_au:.6} AU \
                     (expected {min_au}–{max_au} AU)"
                );
            }
        }
    }

    /// Sun position function is pure, deterministic, and side-effect-free.
    ///
    /// Calling the function twice with the same epoch produces the exact
    /// same result (bitwise identical).
    #[test]
    fn sun_position_is_unit_testable() {
        let epoch = Epoch::from_gregorian_utc_hms(2024, 6, 15, 12, 0, 0);
        let pos1 = sun_position_eci_km(epoch);
        let pos2 = sun_position_eci_km(epoch);

        assert_eq!(pos1, pos2, "sun_position_eci_km must be deterministic");
    }

    /// Moon position at J2000.0 (2000-01-01T12:00:00 UTC).
    ///
    /// Reference: JPL Horizons geocentric Moon position at J2000.0.
    /// RA ~ 222.46 deg, Dec ~ -10.90 deg, distance ~ 402,417 km.
    /// (Moon was waning crescent, ~5 days past last quarter, in Libra.)
    #[test]
    fn moon_position_j2000_epoch() {
        let epoch = Epoch::from_gregorian_utc_hms(2000, 1, 1, 12, 0, 0);
        let pos = moon_position_eci_km(epoch);

        let r_km = pos.norm();

        // Distance check: ~402,417 km (from JPL Horizons)
        let expected_r_km = 402_417.0;
        let rel_err = (r_km - expected_r_km).abs() / expected_r_km;
        assert!(
            rel_err < MOON_DISTANCE_RELATIVE_TOL,
            "J2000 Moon distance: expected ~{expected_r_km} km, got {r_km:.0} km \
             (rel err {rel_err:.4})"
        );

        // Direction check: compare unit vector against JPL Horizons.
        // JPL Horizons geocentric Moon at J2000.0 (ICRF/J2000):
        // RA = 222.459 deg (14h 49.84m), Dec = -10.903 deg
        let expected_ra_rad = 222.459_f64.to_radians();
        let expected_dec_rad = (-10.903_f64).to_radians();
        let expected_dir = Vector3::new(
            expected_dec_rad.cos() * expected_ra_rad.cos(),
            expected_dec_rad.cos() * expected_ra_rad.sin(),
            expected_dec_rad.sin(),
        );

        let computed_dir = pos.normalize();
        let angle_err_rad = computed_dir.dot(&expected_dir).clamp(-1.0, 1.0).acos();
        assert!(
            angle_err_rad < MOON_DIRECTION_TEST_TOL_RAD,
            "J2000 Moon direction error: {:.4} deg (tolerance {:.4} deg)",
            angle_err_rad.to_degrees(),
            MOON_DIRECTION_TEST_TOL_RAD.to_degrees()
        );
    }

    /// Moon distance stays within perigee-apogee band for 2020-2030.
    ///
    /// Perigee ~356,500 km, apogee ~406,700 km. We use generous bounds
    /// (356,000-407,000) to accommodate the truncated series, sampling
    /// the 1st and 15th of each month over a decade.
    #[test]
    fn moon_distance_range() {
        for year in 2020..=2030 {
            for month in 1..=12 {
                for day in &[1, 15] {
                    let epoch = Epoch::from_gregorian_utc_hms(year, month, *day, 0, 0, 0);
                    let pos = moon_position_eci_km(epoch);
                    let r_km = pos.norm();

                    assert!(
                        r_km > MOON_PERIGEE_BOUND_KM && r_km < MOON_APOGEE_BOUND_KM,
                        "Moon distance out of range at {year}-{month:02}-{day:02}: \
                         {r_km:.0} km (expected {MOON_PERIGEE_BOUND_KM}-{MOON_APOGEE_BOUND_KM} km)"
                    );
                }
            }
        }
    }

    /// Moon approximately opposite Sun at a known full moon.
    ///
    /// Full Moon on 2024-02-24 ~12:30 UTC. The Sun-Earth-Moon angle
    /// (elongation) should be near 180 deg.
    #[test]
    fn moon_position_known_full_moon() {
        let epoch = Epoch::from_gregorian_utc_hms(2024, 2, 24, 12, 30, 0);
        let sun_pos = sun_position_eci_km(epoch);
        let moon_pos = moon_position_eci_km(epoch);

        let sun_dir = sun_pos.normalize();
        let moon_dir = moon_pos.normalize();

        // Elongation: angle between Sun and Moon as seen from Earth center
        let elongation_rad = sun_dir.dot(&moon_dir).clamp(-1.0, 1.0).acos();

        // At full moon, elongation should be near 180 deg (pi rad)
        let error_from_opposition = (elongation_rad - std::f64::consts::PI).abs();
        assert!(
            error_from_opposition < FULL_MOON_ELONGATION_TOL_RAD,
            "Full moon elongation: {:.2} deg (expected ~180 deg, error {:.2} deg, \
             tolerance {:.2} deg)",
            elongation_rad.to_degrees(),
            error_from_opposition.to_degrees(),
            FULL_MOON_ELONGATION_TOL_RAD.to_degrees()
        );
    }

    /// Moon's ecliptic latitude stays within orbital inclination bound.
    ///
    /// The Moon's orbit is inclined ~5.145 deg to the ecliptic. With
    /// perturbations, the ecliptic latitude |beta| should never exceed ~5.3 deg.
    /// We sample every 3 days over a full year to cover all orbital phases.
    #[test]
    fn moon_ecliptic_latitude_bounded() {
        let cos_eps = OBLIQUITY_J2000_RAD.cos();
        let sin_eps = OBLIQUITY_J2000_RAD.sin();

        for day_offset in (0..365).step_by(3) {
            let epoch = Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0)
                + hifitime::Duration::from_seconds(f64::from(day_offset) * 86400.0);
            let pos = moon_position_eci_km(epoch);

            // Rotate ECI back to ecliptic to extract ecliptic latitude
            let x_ecl = pos.x;
            let y_ecl = pos.y * cos_eps + pos.z * sin_eps;
            let z_ecl = -pos.y * sin_eps + pos.z * cos_eps;
            let r = (x_ecl * x_ecl + y_ecl * y_ecl + z_ecl * z_ecl).sqrt();
            let beta_rad = (z_ecl / r).asin();

            assert!(
                beta_rad.abs() < MOON_MAX_ECLIPTIC_LATITUDE_RAD,
                "Moon ecliptic latitude {:.3} deg exceeds {:.1} deg bound at day offset {day_offset}",
                beta_rad.to_degrees(),
                MOON_MAX_ECLIPTIC_LATITUDE_RAD.to_degrees()
            );
        }
    }

    /// Spacecraft on the Sun-side of Earth is Sunlit.
    ///
    /// Places the spacecraft between Earth and Sun at ISS-like altitude
    /// (400 km). The Sun and Earth are in opposite directions from the
    /// spacecraft, so no shadow overlap is possible.
    #[test]
    fn sunlit_when_between_earth_and_sun() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        // Directly toward Sun at LEO altitude
        let sc_direct = Vector3::new(R_EARTH + 400.0, 0.0, 0.0);
        assert_eq!(
            compute_eclipse_state(&sc_direct, &sun_eci_km),
            EclipseState::Sunlit,
            "Spacecraft directly between Earth and Sun should be Sunlit"
        );

        // Offset laterally — still on Sun-side
        let sc_offset = Vector3::new(R_EARTH + 400.0, 1000.0, 500.0);
        assert_eq!(
            compute_eclipse_state(&sc_offset, &sun_eci_km),
            EclipseState::Sunlit,
            "Spacecraft on Sun-side with lateral offset should be Sunlit"
        );
    }

    /// Spacecraft directly behind Earth relative to Sun is in Umbra.
    ///
    /// At LEO behind Earth, the spacecraft is well inside the umbral cone.
    /// Earth subtends ~60 deg apparent radius while the Sun subtends ~0.27 deg,
    /// both in the same direction from the spacecraft (through Earth toward Sun).
    #[test]
    fn umbra_when_behind_earth() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        // Directly behind Earth at LEO
        let sc = Vector3::new(-(R_EARTH + 400.0), 0.0, 0.0);
        assert_eq!(
            compute_eclipse_state(&sc, &sun_eci_km),
            EclipseState::Umbra,
            "Spacecraft directly behind Earth should be in Umbra"
        );

        // Behind Earth at higher altitude (MEO-like, ~20,000 km)
        let sc_meo = Vector3::new(-(R_EARTH + 13_600.0), 0.0, 0.0);
        assert_eq!(
            compute_eclipse_state(&sc_meo, &sun_eci_km),
            EclipseState::Umbra,
            "Spacecraft behind Earth at MEO altitude should be in Umbra"
        );
    }

    /// GEO-altitude spacecraft is Sunlit at most orbital positions.
    ///
    /// At GEO (~42,164 km), Earth subtends ~8.7 deg and the shadow cone
    /// covers only ~4.8% of the orbit. Sampling 72 positions (5 deg steps),
    /// the vast majority should be Sunlit.
    #[test]
    fn sunlit_at_high_altitude() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);
        let geo_radius_km = 42_164.0;
        let n_samples = 72;
        let mut sunlit_count = 0;

        for k in 0..n_samples {
            let theta = f64::from(k) / f64::from(n_samples) * TWO_PI;
            let sc = Vector3::new(
                geo_radius_km * theta.cos(),
                geo_radius_km * theta.sin(),
                0.0,
            );
            if compute_eclipse_state(&sc, &sun_eci_km) == EclipseState::Sunlit {
                sunlit_count += 1;
            }
        }

        let sunlit_fraction = f64::from(sunlit_count) / f64::from(n_samples);
        assert!(
            sunlit_fraction > 0.90,
            "GEO spacecraft should be Sunlit >90% of orbit, got {:.1}%",
            sunlit_fraction * 100.0
        );
    }

    /// Spacecraft at the geometric edge of the shadow cone yields Penumbra.
    ///
    /// Places the spacecraft at GEO distance behind Earth, offset laterally
    /// so it sits in the penumbra region. At GEO, penumbra spans ~0.53 deg
    /// (= 2 * theta_sun). Computes the exact offset for shadow_fraction ~ 0.5.
    #[test]
    fn penumbra_at_shadow_boundary() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);
        let geo_r = 42_164.0;

        // Compute angular geometry at GEO behind Earth
        let theta_earth = (R_EARTH / geo_r).asin();
        let sc_to_sun_approx = AU_KM + geo_r; // approximate, Sun is far
        let _theta_sun = (SUN_RADIUS_KM / sc_to_sun_approx).asin();

        // For shadow_fraction ~ 0.5, we need theta_sep = theta_earth
        // (midpoint of the penumbra band). Place spacecraft behind Earth
        // with lateral offset y such that the angle from the anti-Sun axis
        // (as seen from spacecraft looking toward Earth) equals theta_earth.
        //
        // Spacecraft at (-geo_r * cos(offset_angle), geo_r * sin(offset_angle), 0)
        // where offset_angle produces theta_sep ~ theta_earth.
        // For small angles, offset_angle ~ theta_earth works as initial estimate.
        // Use a position slightly inside penumbra for robustness.
        let offset_angle = theta_earth;
        let sc = Vector3::new(
            -geo_r * offset_angle.cos(),
            geo_r * offset_angle.sin(),
            0.0,
        );

        let state = compute_eclipse_state(&sc, &sun_eci_km);
        match state {
            EclipseState::Penumbra { shadow_fraction } => {
                assert!(
                    (shadow_fraction - 0.5).abs() < PENUMBRA_MIDPOINT_TOL,
                    "Expected shadow_fraction near 0.5 at penumbra midpoint, got {shadow_fraction}"
                );
            }
            other => panic!(
                "Expected Penumbra at shadow boundary, got {other:?}"
            ),
        }
    }

    /// Shadow fraction increases monotonically from sunlit through penumbra to umbra.
    ///
    /// Sweeps a spacecraft along an arc at GEO distance from a sunlit position
    /// toward the anti-Sun axis. Extracts an effective shadow value (0.0 for Sunlit,
    /// shadow_fraction for Penumbra, 1.0 for Umbra) and verifies monotonicity.
    #[test]
    fn shadow_fraction_monotonic() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);
        let geo_r = 42_164.0;
        let n = 200;

        let mut prev_shadow = 0.0_f64;
        let mut entered_shadow = false;

        // Sweep from theta = 170 deg (sunlit, near the limb) to theta = 180 deg (anti-Sun)
        // in the X-Y plane. theta is measured from +X axis.
        for k in 0..=n {
            let theta_deg = 170.0 + 10.0 * f64::from(k) / f64::from(n);
            let theta_rad = theta_deg.to_radians();
            let sc = Vector3::new(
                geo_r * theta_rad.cos(),
                geo_r * theta_rad.sin(),
                0.0,
            );

            let state = compute_eclipse_state(&sc, &sun_eci_km);
            let shadow_val = match state {
                EclipseState::Sunlit => 0.0,
                EclipseState::Penumbra { shadow_fraction } => shadow_fraction,
                EclipseState::Umbra => 1.0,
            };

            if entered_shadow {
                assert!(
                    shadow_val >= prev_shadow - MONOTONICITY_EPSILON,
                    "Shadow fraction not monotonic at step {k}: prev={prev_shadow}, curr={shadow_val}"
                );
            }
            if shadow_val > 0.0 {
                entered_shadow = true;
            }
            prev_shadow = shadow_val;
        }

        // Verify we actually reached Umbra
        assert!(
            prev_shadow > 0.99,
            "Expected to reach Umbra at anti-Sun position, final shadow={prev_shadow}"
        );
    }

    /// Shadow fraction is always in [0.0, 1.0] for Penumbra cases.
    ///
    /// Samples many positions at various altitudes (LEO, MEO, GEO) around
    /// the shadow boundary region and verifies that any Penumbra result has
    /// a valid shadow_fraction.
    #[test]
    fn penumbra_shadow_fraction_range() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        for &radius in &[R_EARTH + 400.0, R_EARTH + 2000.0, 20_200.0, 42_164.0] {
            for k in 0..360 {
                let theta = f64::from(k) * std::f64::consts::PI / 180.0;
                let sc = Vector3::new(
                    radius * theta.cos(),
                    radius * theta.sin(),
                    0.0,
                );

                if let EclipseState::Penumbra { shadow_fraction } =
                    compute_eclipse_state(&sc, &sun_eci_km)
                {
                    assert!(
                        (0.0..=1.0).contains(&shadow_fraction),
                        "Invalid shadow_fraction {shadow_fraction} at radius={radius} km, \
                         theta={:.1} deg",
                        theta.to_degrees()
                    );
                }
            }
        }
    }

    /// Symmetric positions relative to the Sun-Earth line produce identical eclipse states.
    ///
    /// The shadow geometry is axially symmetric about the Earth-Sun line.
    /// Mirror-image positions across any plane containing that axis must
    /// produce the same eclipse classification and shadow fraction.
    #[test]
    fn symmetry_about_shadow_axis() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        // Test at multiple distances behind Earth with lateral offsets
        for &x in &[-(R_EARTH + 400.0), -20_200.0, -42_164.0] {
            for &offset in &[100.0, 500.0, 2000.0, 5000.0] {
                // Mirror across X-Z plane (Y -> -Y)
                let sc_a = Vector3::new(x, offset, 300.0);
                let sc_b = Vector3::new(x, -offset, 300.0);
                let state_a = compute_eclipse_state(&sc_a, &sun_eci_km);
                let state_b = compute_eclipse_state(&sc_b, &sun_eci_km);

                match (&state_a, &state_b) {
                    (EclipseState::Sunlit, EclipseState::Sunlit)
                    | (EclipseState::Umbra, EclipseState::Umbra) => {}
                    (
                        EclipseState::Penumbra {
                            shadow_fraction: fa,
                        },
                        EclipseState::Penumbra {
                            shadow_fraction: fb,
                        },
                    ) => {
                        assert!(
                            (fa - fb).abs() < SHADOW_FRACTION_SYMMETRY_TOL,
                            "Symmetric positions should have equal shadow_fraction: \
                             {fa} vs {fb} at x={x}, y_offset={offset}"
                        );
                    }
                    _ => panic!(
                        "Symmetric positions should have same state: {state_a:?} vs {state_b:?} \
                         at x={x}, y_offset={offset}"
                    ),
                }

                // Mirror across X-Y plane (Z -> -Z)
                let sc_c = Vector3::new(x, offset, 300.0);
                let sc_d = Vector3::new(x, offset, -300.0);
                let state_c = compute_eclipse_state(&sc_c, &sun_eci_km);
                let state_d = compute_eclipse_state(&sc_d, &sun_eci_km);

                match (&state_c, &state_d) {
                    (EclipseState::Sunlit, EclipseState::Sunlit)
                    | (EclipseState::Umbra, EclipseState::Umbra) => {}
                    (
                        EclipseState::Penumbra {
                            shadow_fraction: fc,
                        },
                        EclipseState::Penumbra {
                            shadow_fraction: fd,
                        },
                    ) => {
                        assert!(
                            (fc - fd).abs() < SHADOW_FRACTION_SYMMETRY_TOL,
                            "Z-symmetric positions should have equal shadow_fraction: \
                             {fc} vs {fd} at x={x}, y_offset={offset}"
                        );
                    }
                    _ => panic!(
                        "Z-symmetric positions should have same state: {state_c:?} vs {state_d:?} \
                         at x={x}, y_offset={offset}"
                    ),
                }
            }
        }
    }

    /// Conical model produces Umbra for positions well inside the geometric shadow cylinder.
    ///
    /// The cylindrical shadow model (Task 4.3, deferred) would classify any position
    /// behind Earth within Earth's cross-section as shadowed. The conical model should
    /// agree for positions well inside the cone — i.e., on the anti-Sun axis at
    /// various altitudes below the umbral cone apex.
    #[test]
    fn cylindrical_agrees_with_conical_deep_shadow() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        // Multiple altitudes directly behind Earth on the anti-Sun axis
        for &alt_km in &[400.0, 1_000.0, 5_000.0, R_EARTH] {
            let sc = Vector3::new(-(R_EARTH + alt_km), 0.0, 0.0);
            let state = compute_eclipse_state(&sc, &sun_eci_km);
            assert_eq!(
                state,
                EclipseState::Umbra,
                "On anti-Sun axis at alt={alt_km} km: expected Umbra, got {state:?}"
            );
        }
    }

    /// ISS-like orbit has ~35% eclipse fraction per orbit.
    ///
    /// Cross-phase test: uses `keplerian_to_state()` (elements) and
    /// `sun_position_eci_km()` (Phase 2) with `compute_eclipse_state()` (Phase 4).
    /// Samples 360 points around one orbit (1 deg mean anomaly steps).
    ///
    /// Published value: ISS eclipse fraction is ~36% at low beta angles.
    /// At the test epoch (2024-01-01), the beta angle depends on the Sun position
    /// and ISS RAAN; we allow 5% tolerance to accommodate this variation.
    #[test]
    fn iss_eclipse_fraction() {
        let mut elements = crate::test_helpers::iss_like_elements();
        let epoch = crate::test_helpers::test_epoch();

        // Sun position at test epoch (barely moves over one ~92 min orbit)
        let sun_pos = sun_position_eci_km(epoch);

        let n_samples: u32 = 360;
        let mut shadow_count: u32 = 0;

        for k in 0..n_samples {
            elements.mean_anomaly_rad = f64::from(k) / f64::from(n_samples) * TWO_PI;
            let state_vec =
                keplerian_to_state(&elements, epoch).expect("keplerian_to_state failed");
            let eclipse = compute_eclipse_state(&state_vec.position_eci_km, &sun_pos);

            if eclipse != EclipseState::Sunlit {
                shadow_count += 1;
            }
        }

        let eclipse_fraction = f64::from(shadow_count) / f64::from(n_samples);
        let expected = 0.35;
        assert!(
            (eclipse_fraction - expected).abs() < ISS_ECLIPSE_FRACTION_TOL,
            "ISS eclipse fraction: {:.1}% (expected ~{:.0}%, tolerance {:.0}%)",
            eclipse_fraction * 100.0,
            expected * 100.0,
            ISS_ECLIPSE_FRACTION_TOL * 100.0
        );
    }

    // =========================================================================
    // Phase 5: Trajectory Eclipse Computation
    // =========================================================================

    /// ISS orbital period ~92.4 min = 5544 s. Published ~15.5 orbits/day.
    const ISS_PERIOD_S: f64 = 5544.0;

    /// Expected ISS eclipse duration per orbit (~35 min = 2100 s).
    /// Published: ~36 min typical for low beta angle.
    const ISS_ECLIPSE_DURATION_EXPECTED_S: f64 = 2100.0;

    /// Tolerance on ISS eclipse duration (±5 min = 300 s).
    /// Accounts for Sun position uncertainty (~0.01 deg), beta angle variation,
    /// sampling discretization (1 deg M steps), and mean-vs-osculating offset.
    const ISS_ECLIPSE_DURATION_TOL_S: f64 = 300.0;

    /// Expected ~15 eclipses in 24 hours for ISS (15.5 orbits/day, each with one eclipse).
    const ISS_24H_ECLIPSE_COUNT_EXPECTED: usize = 15;

    /// Tolerance on 24h eclipse count (±2). Accounts for partial eclipses at
    /// start/end of 24h window and beta angle variation.
    const ISS_24H_ECLIPSE_COUNT_TOL: usize = 2;

    /// Build an ISS-like trajectory as (Epoch, KeplerianElements) pairs.
    ///
    /// Generates `n_orbits` orbits at 360 points per orbit (1 deg mean anomaly steps).
    /// Each point advances both the epoch and mean anomaly uniformly.
    fn iss_trajectory_points(n_orbits: u32) -> Vec<(Epoch, crate::types::KeplerianElements)> {
        let base_elements = crate::test_helpers::iss_like_elements();
        let epoch = crate::test_helpers::test_epoch();
        let base_m = base_elements.mean_anomaly_rad;
        let n_steps_per_orbit: u32 = 360;
        let total_steps = n_orbits * n_steps_per_orbit;
        let dt_per_step = ISS_PERIOD_S / f64::from(n_steps_per_orbit);

        let mut points = Vec::with_capacity(total_steps as usize);
        for k in 0..total_steps {
            let t = f64::from(k) * dt_per_step;
            let m = base_m + f64::from(k) * TWO_PI / f64::from(n_steps_per_orbit);
            let mut elements = base_elements.clone();
            elements.mean_anomaly_rad = m.rem_euclid(TWO_PI);
            points.push((
                epoch + hifitime::Duration::from_seconds(t),
                elements,
            ));
        }
        points
    }

    /// ~15 eclipses in a 24-hour ISS orbit propagation.
    ///
    /// Published: ISS completes ~15.5 orbits/day, each with one eclipse passage.
    /// We generate 16 orbits (~24.6 hours) and verify the eclipse count.
    #[test]
    fn iss_orbit_eclipse_count() {
        let points = iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        let count = summary.intervals.len();
        let lo = ISS_24H_ECLIPSE_COUNT_EXPECTED - ISS_24H_ECLIPSE_COUNT_TOL;
        let hi = ISS_24H_ECLIPSE_COUNT_EXPECTED + ISS_24H_ECLIPSE_COUNT_TOL;
        assert!(
            (lo..=hi).contains(&count),
            "Expected {lo}–{hi} eclipses in ~24h ISS orbit, got {count}"
        );
    }

    /// Each ISS eclipse lasts ~35 minutes.
    ///
    /// Published: ~36 min typical for low beta angle. We propagate 3 orbits
    /// and check only the middle interval(s), which are guaranteed not to be
    /// truncated by the trajectory boundary.
    #[test]
    fn iss_eclipse_duration() {
        let points = iss_trajectory_points(3);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        assert!(
            summary.intervals.len() >= 2,
            "Expected at least 2 eclipses in 3 ISS orbits, got {}",
            summary.intervals.len()
        );

        // Skip first and last intervals (may be truncated at trajectory boundary)
        let interior = &summary.intervals[1..summary.intervals.len() - 1];
        assert!(
            !interior.is_empty(),
            "Expected at least one interior eclipse interval in 3 orbits"
        );

        for interval in interior {
            assert!(
                (interval.duration_s - ISS_ECLIPSE_DURATION_EXPECTED_S).abs()
                    < ISS_ECLIPSE_DURATION_TOL_S,
                "Eclipse duration {:.0}s outside expected {:.0}±{:.0}s",
                interval.duration_s,
                ISS_ECLIPSE_DURATION_EXPECTED_S,
                ISS_ECLIPSE_DURATION_TOL_S
            );
        }
    }

    /// Dawn-dusk SSO at ~98° inclination has minimal or no eclipses.
    ///
    /// A dawn-dusk sun-synchronous orbit has its orbital plane aligned with
    /// the terminator, so the spacecraft stays in near-permanent sunlight.
    /// The RAAN is set 90° ahead of the Sun's right ascension at the test epoch.
    #[test]
    fn sun_sync_dawn_dusk_no_eclipse() {
        let epoch = crate::test_helpers::test_epoch();

        // Compute Sun RA at test epoch to set the dawn-dusk RAAN
        let sun_pos = sun_position_eci_km(epoch);
        let sun_ra = sun_pos.y.atan2(sun_pos.x);
        let dawn_dusk_raan = sun_ra + std::f64::consts::FRAC_PI_2;

        let base = crate::types::KeplerianElements {
            a_km: 7078.135,
            e: 0.001,
            i_rad: 98.19_f64.to_radians(),
            raan_rad: dawn_dusk_raan,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };

        // Generate 1-orbit trajectory (360 points)
        let n_steps: u32 = 360;
        let period_s = base.period().expect("period computation");
        let dt = period_s / f64::from(n_steps);
        let mut points = Vec::with_capacity(n_steps as usize);
        for k in 0..n_steps {
            let t = f64::from(k) * dt;
            let m = f64::from(k) * TWO_PI / f64::from(n_steps);
            let mut elements = base.clone();
            elements.mean_anomaly_rad = m.rem_euclid(TWO_PI);
            points.push((
                epoch + hifitime::Duration::from_seconds(t),
                elements,
            ));
        }

        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        assert!(
            summary.time_in_shadow_fraction < 0.02,
            "Dawn-dusk SSO eclipse fraction {:.1}% exceeds 2% threshold",
            summary.time_in_shadow_fraction * 100.0
        );
    }

    /// Sum of all interval durations equals `total_shadow_duration_s`.
    #[test]
    fn interval_continuity() {
        let points = iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        let manual_sum: f64 = summary.intervals.iter().map(|i| i.duration_s).sum();
        let n = u32::try_from(summary.intervals.len()).expect("interval count fits u32");
        let tol = f64::EPSILON * f64::from(n) * summary.total_shadow_duration_s.max(1.0);
        assert!(
            (manual_sum - summary.total_shadow_duration_s).abs() < tol,
            "Interval sum {manual_sum} != total_shadow_duration_s {}",
            summary.total_shadow_duration_s
        );
    }

    /// Output snapshot count equals input point count.
    #[test]
    fn snapshot_count_matches_trajectory() {
        let points = iss_trajectory_points(1);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert_eq!(
            snapshots.len(),
            points.len(),
            "Snapshot count {} != input point count {}",
            snapshots.len(),
            points.len()
        );
    }

    /// Zero points produce empty snapshots and empty summary.
    #[test]
    fn empty_trajectory_returns_empty() {
        let points: Vec<(Epoch, crate::types::KeplerianElements)> = vec![];
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert!(snapshots.is_empty(), "Expected empty snapshots for empty input");

        let summary = extract_eclipse_intervals(&snapshots);
        assert!(summary.intervals.is_empty());
        assert!((summary.total_shadow_duration_s).abs() < f64::EPSILON);
        assert!((summary.time_in_shadow_fraction).abs() < f64::EPSILON);
        assert!((summary.max_shadow_duration_s).abs() < f64::EPSILON);
    }

    /// One point produces one snapshot but no intervals (need >= 2 for transitions).
    #[test]
    fn single_point_no_intervals() {
        let points = vec![(
            crate::test_helpers::test_epoch(),
            crate::test_helpers::iss_like_elements(),
        )];
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert_eq!(snapshots.len(), 1);
        assert!(snapshots[0].sun_distance_km > 0.0);
        assert!(snapshots[0].moon_distance_km > 0.0);

        let summary = extract_eclipse_intervals(&snapshots);
        assert!(
            summary.intervals.is_empty(),
            "Single-point input should produce no intervals"
        );
    }

    /// Eclipse intervals are sorted by start epoch and non-overlapping.
    #[test]
    fn monotonic_epochs() {
        let points = iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        for interval in &summary.intervals {
            assert!(
                interval.start < interval.end,
                "Interval start {:?} >= end {:?}",
                interval.start,
                interval.end
            );
            assert!(
                interval.duration_s > 0.0,
                "Interval duration must be positive, got {}",
                interval.duration_s
            );
        }

        for pair in summary.intervals.windows(2) {
            assert!(
                pair[0].end <= pair[1].start,
                "Intervals overlap: first ends {:?}, next starts {:?}",
                pair[0].end,
                pair[1].start
            );
        }
    }

    // =======================================================================
    // compute_eclipse_from_states tests
    // =======================================================================

    /// `compute_eclipse_from_states` returns empty vec for empty input.
    #[test]
    fn eclipse_from_states_empty_input() {
        let result = compute_eclipse_from_states(&[]);
        assert!(result.is_empty());
    }

    /// `compute_eclipse_from_states` produces equivalent results to
    /// `compute_celestial_snapshots` for the same ECI positions.
    ///
    /// Tolerance: exact agreement — both functions compute the same
    /// Sun/Moon ephemeris and shadow model from identical ECI positions.
    #[test]
    fn eclipse_from_states_matches_snapshots() {
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let elements = iss_like_elements();
        let epoch = test_epoch();

        // Generate a short trajectory via keplerian_to_state
        let points: Vec<_> = (0..10)
            .map(|i| {
                let dt_s = f64::from(i) * 60.0;
                let n = elements.mean_motion().unwrap();
                let advanced = KeplerianElements {
                    mean_anomaly_rad: (elements.mean_anomaly_rad + n * dt_s)
                        .rem_euclid(std::f64::consts::TAU),
                    ..elements
                };
                let ep = epoch + hifitime::Duration::from_seconds(dt_s);
                (ep, advanced)
            })
            .collect();

        // Convert to StateVectors
        let states: Vec<StateVector> = points
            .iter()
            .map(|(ep, ke)| crate::elements::keplerian_conversions::keplerian_to_state(ke, *ep).unwrap())
            .collect();

        let from_keplerian = compute_celestial_snapshots(&points).unwrap();
        let from_states = compute_eclipse_from_states(&states);

        assert_eq!(from_keplerian.len(), from_states.len());

        for (a, b) in from_keplerian.iter().zip(from_states.iter()) {
            assert_eq!(a.eclipse_state, b.eclipse_state, "eclipse state mismatch");
            // Sun direction should be very close (not exact due to slightly
            // different spacecraft positions from mean vs osculating)
            let angle = a
                .sun_direction_eci
                .dot(&b.sun_direction_eci)
                .clamp(-1.0, 1.0)
                .acos();
            assert!(
                angle < SUN_DIRECTION_CROSS_VALIDATION_TOL_RAD,
                "Sun direction diverges by {:.4} rad",
                angle
            );
        }
    }

    /// ISS-like orbit via `compute_eclipse_from_states` shows ~35% shadow.
    #[test]
    fn eclipse_from_states_iss_orbit() {
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let elements = iss_like_elements();
        let epoch = test_epoch();
        let period_s = elements.period().unwrap();

        // Propagate one full orbit
        let states: Vec<StateVector> =
            crate::propagation::keplerian::propagate_keplerian(
                &crate::elements::keplerian_conversions::keplerian_to_state(&elements, epoch).unwrap(),
                period_s,
                360,
            )
            .unwrap();

        let snapshots = compute_eclipse_from_states(&states);
        let summary = extract_eclipse_intervals(&snapshots);

        // ISS-like orbit: ~30-40% in shadow
        assert!(
            summary.time_in_shadow_fraction > 0.25,
            "ISS shadow fraction should be >25%, got {:.1}%",
            summary.time_in_shadow_fraction * 100.0
        );
        assert!(
            summary.time_in_shadow_fraction < 0.45,
            "ISS shadow fraction should be <45%, got {:.1}%",
            summary.time_in_shadow_fraction * 100.0
        );
    }
}

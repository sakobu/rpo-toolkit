//! Analytical celestial ephemeris: Sun and Moon positions.
//!
//! Provides Sun and Moon positions from any `hifitime::Epoch` using
//! low-precision analytical series (Meeus Ch. 25 and Ch. 47). No ephemeris
//! files, no ANISE dependency. Microseconds per call.
//!
//! # References
//!
//! - Meeus, Jean — *Astronomical Algorithms* (2nd ed., 1998), Ch. 25 (Sun), Ch. 47 (Moon)

use hifitime::Epoch;
use nalgebra::Vector3;

use crate::constants::{AU_KM, DAYS_PER_JULIAN_CENTURY, JD_J2000, OBLIQUITY_J2000_RAD};

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

#[cfg(test)]
mod tests {
    use super::*;

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
}

//! Earth Rotation Angle — arcsecond-class display / mission-planner math.
//!
//! Implements the IERS Conventions 2010 (TN 36) Eq. 5.15 Earth Rotation
//! Angle (ERA) formula for converting between ECI (GCRS) and ECEF (ITRS)
//! frames. Pure analytical, no ephemeris data, WASM-safe, microseconds
//! per call.
//!
//! # Precision regime
//!
//! **Target: arcsecond-class.** ERA is formally defined at UT1, but
//! without access to IERS Earth Orientation Parameter tables we cannot
//! compute the DUT1 = UT1 − UTC offset. This module treats UT1 ≈ UTC,
//! which introduces an error of `|DUT1|/86400 × 2π × 1.00274 rev/day`.
//! Because `|DUT1| < 0.9 s` by construction (leap seconds keep it bounded),
//! the ERA error is ≤ 65 μrad ≈ 13 arcsec. At LEO altitude that is
//! ~500 m of sub-satellite longitude shift — fine for viewport rendering
//! and mission-planner-class ground-station access windows (typical
//! elevation masks are 5–10°).
//!
//! Polar motion, nutation, and precession since J2000 are **not** applied.
//! Full-fidelity ITRF conversions (sub-arcsec) require EOP tables and live
//! in `rpo-nyx` / the WebSocket API, not in WASM.
//!
//! # References
//!
//! - Petit, G. & Luzum, B. (eds.), *IERS Conventions (2010)*, IERS
//!   Technical Note 36, §5.5, Eq. 5.15.
//! - Vallado, D. A., *Fundamentals of Astrodynamics and Applications*,
//!   4e, Ch. 3 (frame transformations).

use hifitime::Epoch;

use crate::constants::{EARTH_REV_PER_UT1_DAY, ERA_CONSTANT_TURNS, JD_J2000, TWO_PI};

/// Earth Rotation Angle at the given epoch, in radians `[0, 2π)`.
///
/// Computes IERS TN 36 Eq. 5.15:
///
/// ```text
/// θ(Tu) = 2π · frac(ERA_CONSTANT_TURNS + EARTH_REV_PER_UT1_DAY · Tu)
/// ```
///
/// where `Tu` is the UT1 day count since J2000.0. UT1 is approximated by
/// UTC (error ≤ 0.9 s ≈ 13 arcsec of ERA — see module-level docs).
///
/// # Arguments
/// * `epoch` — absolute epoch (any hifitime time scale; converted to UTC).
///
/// # Invariants
/// - Return value is in `[0, 2π)` (modular reduction via fractional part).
/// - Monotonically increasing in wall-clock time, modulo `2π` wrap.
/// - Periodic with period one sidereal day ≈ `SECONDS_PER_DAY / EARTH_REV_PER_UT1_DAY ≈ 86164.0905 s`.
///
/// # Precision
/// Arcsecond-class (~13 arcsec worst case). Suitable for viewport rendering
/// and mission-planner-class frame transforms. Do **not** use for operational
/// ITRF products — for that, go through `rpo-nyx` with EOP corrections.
#[must_use]
pub fn earth_rotation_angle_rad(epoch: Epoch) -> f64 {
    let jd_utc_days_since_j2000 = epoch.to_jde_utc_days() - JD_J2000;
    let turns = ERA_CONSTANT_TURNS + EARTH_REV_PER_UT1_DAY * jd_utc_days_since_j2000;
    let frac = turns - turns.floor();
    TWO_PI * frac
}

#[cfg(test)]
mod tests {
    use hifitime::Duration;

    use super::earth_rotation_angle_rad;
    use crate::constants::{EARTH_REV_PER_UT1_DAY, ERA_CONSTANT_TURNS, SECONDS_PER_DAY, TWO_PI};
    use crate::test_helpers::j2000_epoch;

    // ---------------------------------------------------------------------
    // Named tolerance / fixture constants
    // ---------------------------------------------------------------------

    /// Test tolerance: ERA at J2000 vs IERS TN 36 reference value. Pure f64
    /// polynomial; 1e-10 rad is tight (~20 microarcsec).
    const ERA_REFERENCE_TOL_RAD: f64 = 1e-10;

    /// Test tolerance: sidereal-day periodicity. After one sidereal day, ERA
    /// returns to its starting value mod 2π; the residual is f64 round-off
    /// accumulated over `JD_days · rev_per_day − floor(...)`. Empirically
    /// ~1e-9 rad (0.2 milliarcsec).
    const ERA_SIDEREAL_PERIOD_TOL_RAD: f64 = 1e-8;

    /// Test tolerance: ERA advance over a 6-h window matches the analytical
    /// formula. Single polynomial evaluation; 1e-10 rad is tight.
    const ERA_QUARTER_TURN_TOL_RAD: f64 = 1e-10;

    /// Test bound: per-step ERA advance over the monotonicity loop must be
    /// strictly positive and below this bound. Each step is
    /// `WINDOW_S / STEPS ≈ 216 s × ω ≈ 0.016 rad`; 0.5 rad provides ample
    /// slack and is well below π so the wrapped delta is unambiguously
    /// positive (no 2π aliasing).
    const ERA_MONOTONIC_PER_STEP_BOUND_RAD: f64 = 0.5;

    /// Test fixture: monotonicity test window length (s). 6 h is short enough
    /// to avoid a full ERA wrap, long enough to exercise many integration steps.
    const ERA_MONOTONICITY_TEST_WINDOW_S: f64 = 6.0 * 3600.0;

    /// Test fixture: number of steps inside the monotonicity test window.
    const ERA_MONOTONICITY_TEST_STEPS: u32 = 100;

    /// Expected ERA value at 2000-01-01T12:00:00 UTC (Tu = 0), per IERS TN 36
    /// Eq. 5.15: `θ = 2π × 0.7790572732640 = 4.894_961_212_735_792 rad`.
    const ERA_AT_J2000_UTC_RAD: f64 = TWO_PI * ERA_CONSTANT_TURNS;

    /// ERA at J2000 UTC must equal `2π × ERA_CONSTANT_TURNS` from IERS TN 36.
    #[test]
    fn era_at_j2000_matches_iers_reference() {
        let era = earth_rotation_angle_rad(j2000_epoch());
        let diff = (era - ERA_AT_J2000_UTC_RAD).abs();
        assert!(
            diff < ERA_REFERENCE_TOL_RAD,
            "ERA at J2000 UTC = {era} rad, expected {ERA_AT_J2000_UTC_RAD} rad, diff = {diff}"
        );
    }

    /// ERA must always be in `[0, 2π)`.
    #[test]
    fn era_is_in_principal_range() {
        let base = j2000_epoch();
        for dt_days in [-365.0_f64, -1.0, 0.0, 1.0, 100.0, 365.0 * 25.0] {
            let epoch = base + Duration::from_seconds(dt_days * SECONDS_PER_DAY);
            let era = earth_rotation_angle_rad(epoch);
            assert!(
                (0.0..TWO_PI).contains(&era),
                "ERA {era} at dt_days={dt_days} outside [0, 2π)"
            );
        }
    }

    /// Over one sidereal day, ERA advances by exactly 2π — i.e., the ERA
    /// value returns to its starting point modulo the principal range.
    #[test]
    fn era_is_periodic_over_one_sidereal_day() {
        let sidereal_day_s = SECONDS_PER_DAY / EARTH_REV_PER_UT1_DAY;
        let epoch0 = j2000_epoch();
        let epoch1 = epoch0 + Duration::from_seconds(sidereal_day_s);
        let era0 = earth_rotation_angle_rad(epoch0);
        let era1 = earth_rotation_angle_rad(epoch1);
        let diff = (era1 - era0).abs();
        assert!(
            diff < ERA_SIDEREAL_PERIOD_TOL_RAD,
            "ERA changed by {diff} rad over one sidereal day (should be ~0 mod 2π)"
        );
    }

    /// ERA advances monotonically on intervals much shorter than one sidereal
    /// day (ignoring 2π wrap). Tests `STEPS` points over `WINDOW_S` seconds.
    #[test]
    fn era_is_monotonic_over_six_hours() {
        let epoch0 = j2000_epoch();
        let mut prev = earth_rotation_angle_rad(epoch0);
        let step_s = ERA_MONOTONICITY_TEST_WINDOW_S / f64::from(ERA_MONOTONICITY_TEST_STEPS);
        for i in 1..=ERA_MONOTONICITY_TEST_STEPS {
            let dt_s = f64::from(i) * step_s;
            let epoch = epoch0 + Duration::from_seconds(dt_s);
            let era = earth_rotation_angle_rad(epoch);
            let delta = (era - prev).rem_euclid(TWO_PI);
            assert!(
                delta > 0.0 && delta < ERA_MONOTONIC_PER_STEP_BOUND_RAD,
                "ERA not monotonic at step {i}: prev={prev}, era={era}, delta={delta}"
            );
            prev = era;
        }
    }

    /// ERA changes by approximately `2π × 6/24 × 1.00274 ≈ π/2` over 6 hours.
    /// This is a sanity check on the rotation-rate coefficient magnitude.
    #[test]
    fn era_advances_by_quarter_turn_in_six_hours() {
        let epoch0 = j2000_epoch();
        let epoch1 = epoch0 + Duration::from_seconds(ERA_MONOTONICITY_TEST_WINDOW_S);
        let delta = (earth_rotation_angle_rad(epoch1) - earth_rotation_angle_rad(epoch0))
            .rem_euclid(TWO_PI);
        let expected = TWO_PI * 6.0 / 24.0 * EARTH_REV_PER_UT1_DAY;
        let diff = (delta - expected).abs();
        assert!(
            diff < ERA_QUARTER_TURN_TOL_RAD,
            "ERA advance over 6 h = {delta} rad, expected {expected} rad, diff = {diff}"
        );
    }
}

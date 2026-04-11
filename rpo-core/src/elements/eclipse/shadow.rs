//! Conical Earth shadow geometry and celestial snapshot construction.
//!
//! Models Earth as a sphere and uses the Montenbruck & Gill (Sec. 3.4)
//! conical shadow model to classify spacecraft illumination state.

use hifitime::Epoch;
use nalgebra::Vector3;

use crate::constants::{R_EARTH, SUN_RADIUS_KM};
use crate::types::{CelestialSnapshot, EclipseState};

use super::ephemeris::{moon_position_eci_km, sun_position_eci_km};

/// Shadow geometry intermediates reused by [`build_celestial_snapshot`].
///
/// Contains the eclipse state plus the spacecraft-to-Sun vector and distance
/// so that callers needing both can avoid redundant computation.
struct ShadowResult {
    eclipse_state: EclipseState,
    sc_to_sun: Vector3<f64>,
    sc_to_sun_dist: f64,
}

/// Core conical shadow geometry (Montenbruck & Gill, Sec. 3.4).
///
/// Returns the eclipse state together with the spacecraft-to-Sun vector and
/// distance so callers that need both can avoid recomputing them.
fn shadow_geometry(
    spacecraft_eci_km: &Vector3<f64>,
    sun_eci_km: &Vector3<f64>,
) -> ShadowResult {
    let sc_to_sun = sun_eci_km - spacecraft_eci_km;
    let sc_to_sun_dist = sc_to_sun.norm();

    let sc_to_earth_dist = spacecraft_eci_km.norm();

    // Apparent angular radii as seen from spacecraft (rad)
    // Clamp asin arguments for robustness near Earth's surface
    let theta_sun = (SUN_RADIUS_KM / sc_to_sun_dist).clamp(-1.0, 1.0).asin();
    let theta_earth = (R_EARTH / sc_to_earth_dist).clamp(-1.0, 1.0).asin();

    // Angular separation between Sun center and Earth center as seen from spacecraft
    let cos_theta_sep =
        sc_to_sun.dot(&(-spacecraft_eci_km)) / (sc_to_sun_dist * sc_to_earth_dist);
    let theta_sep = cos_theta_sep.clamp(-1.0, 1.0).acos();

    // Classification based on angular overlap geometry
    let penumbra_boundary = theta_sun + theta_earth;
    let umbra_boundary = theta_earth - theta_sun;

    let eclipse_state = if theta_sep >= penumbra_boundary {
        EclipseState::Sunlit
    } else if theta_earth >= theta_sun && theta_sep <= umbra_boundary {
        EclipseState::Umbra
    } else {
        // Linear interpolation for shadow fraction
        let shadow_fraction =
            ((penumbra_boundary - theta_sep) / (2.0 * theta_sun)).clamp(0.0, 1.0);
        EclipseState::Penumbra { shadow_fraction }
    };

    ShadowResult {
        eclipse_state,
        sc_to_sun,
        sc_to_sun_dist,
    }
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
    shadow_geometry(spacecraft_eci_km, sun_eci_km).eclipse_state
}

/// Build a single [`CelestialSnapshot`] from a spacecraft ECI position and epoch.
///
/// Computes Sun and Moon positions analytically, evaluates the conical shadow
/// model, and produces direction/distance fields. Reuses the spacecraft-to-Sun
/// vector from shadow geometry to avoid redundant computation.
///
/// Shared by both [`super::compute_celestial_snapshots`] and
/// [`super::compute_eclipse_from_states`].
pub(crate) fn build_celestial_snapshot(epoch: Epoch, sc_eci_km: &Vector3<f64>) -> CelestialSnapshot {
    let sun_pos = sun_position_eci_km(epoch);
    let moon_pos = moon_position_eci_km(epoch);

    let shadow = shadow_geometry(sc_eci_km, &sun_pos);

    let sc_to_moon = moon_pos - sc_eci_km;
    let moon_dist = sc_to_moon.norm();

    CelestialSnapshot {
        epoch,
        sun_direction_eci: shadow.sc_to_sun / shadow.sc_to_sun_dist,
        moon_direction_eci: sc_to_moon / moon_dist,
        sun_distance_km: shadow.sc_to_sun_dist,
        moon_distance_km: moon_dist,
        eclipse_state: shadow.eclipse_state,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{AU_KM, TWO_PI};

    /// ISS-like LEO altitude (km). Typical low Earth orbit test altitude.
    const LEO_ALTITUDE_KM: f64 = 400.0;

    /// Higher LEO altitude (km) for testing altitude-dependent shadow behavior.
    const HIGH_LEO_ALTITUDE_KM: f64 = 2000.0;

    /// GPS/MEO orbit altitude above Earth's surface (km).
    const MEO_ALTITUDE_KM: f64 = 13_600.0;

    /// GPS/MEO orbital radius from Earth center (km).
    const MEO_RADIUS_KM: f64 = 20_200.0;

    /// GEO orbital radius from Earth center (km). ~42,164 km = 35,786 km altitude.
    const GEO_RADIUS_KM: f64 = 42_164.0;

    /// Shadow fraction symmetry: two symmetric positions should produce
    /// identical shadow fractions. 1e-12 covers f64 evaluation of
    /// the conical geometry formula for mirrored inputs.
    const SHADOW_FRACTION_SYMMETRY_TOL: f64 = 1e-12;

    /// Tolerance for penumbra midpoint test (`shadow_fraction` ~ 0.5).
    /// The test constructs the spacecraft position using small-angle geometry
    /// approximations, which introduces systematic offset from the exact
    /// midpoint. 0.10 tolerance accommodates the approximation error at GEO
    /// distance.
    const PENUMBRA_MIDPOINT_TOL: f64 = 0.10;

    /// Floating-point guard for monotonicity assertions.
    /// Allows `shadow_fraction` to decrease by at most this amount between
    /// adjacent samples due to floating-point rounding, without flagging
    /// a false monotonicity violation. 1e-10 is well above f64 epsilon
    /// but well below any physically meaningful shadow fraction change.
    const MONOTONICITY_EPSILON: f64 = 1e-10;

    /// Spacecraft on the Sun-side of Earth is Sunlit.
    ///
    /// Places the spacecraft between Earth and Sun at ISS-like altitude
    /// (400 km). The Sun and Earth are in opposite directions from the
    /// spacecraft, so no shadow overlap is possible.
    #[test]
    fn sunlit_when_between_earth_and_sun() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        // Directly toward Sun at LEO altitude
        let sc_direct = Vector3::new(R_EARTH + LEO_ALTITUDE_KM, 0.0, 0.0);
        assert_eq!(
            compute_eclipse_state(&sc_direct, &sun_eci_km),
            EclipseState::Sunlit,
            "Spacecraft directly between Earth and Sun should be Sunlit"
        );

        // Offset laterally — still on Sun-side
        let sc_offset = Vector3::new(R_EARTH + LEO_ALTITUDE_KM, 1000.0, 500.0);
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
        let sc = Vector3::new(-(R_EARTH + LEO_ALTITUDE_KM), 0.0, 0.0);
        assert_eq!(
            compute_eclipse_state(&sc, &sun_eci_km),
            EclipseState::Umbra,
            "Spacecraft directly behind Earth should be in Umbra"
        );

        // Behind Earth at higher altitude (MEO-like, ~20,000 km)
        let sc_meo = Vector3::new(-(R_EARTH + MEO_ALTITUDE_KM), 0.0, 0.0);
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
        let n_samples = 72;
        let mut sunlit_count = 0;

        for k in 0..n_samples {
            let theta = f64::from(k) / f64::from(n_samples) * TWO_PI;
            let sc = Vector3::new(
                GEO_RADIUS_KM * theta.cos(),
                GEO_RADIUS_KM * theta.sin(),
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
    /// (= 2 * `theta_sun`). Computes the exact offset for `shadow_fraction` ~ 0.5.
    #[test]
    fn penumbra_at_shadow_boundary() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);
        let geo_r = GEO_RADIUS_KM;

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
    /// `shadow_fraction` for Penumbra, 1.0 for Umbra) and verifies monotonicity.
    #[test]
    fn shadow_fraction_monotonic() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);
        let geo_r = GEO_RADIUS_KM;
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
    /// a valid `shadow_fraction`.
    #[test]
    fn penumbra_shadow_fraction_range() {
        let sun_eci_km = Vector3::new(AU_KM, 0.0, 0.0);

        for &radius in &[R_EARTH + LEO_ALTITUDE_KM, R_EARTH + HIGH_LEO_ALTITUDE_KM, MEO_RADIUS_KM, GEO_RADIUS_KM] {
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
        for &x in &[-(R_EARTH + LEO_ALTITUDE_KM), -MEO_RADIUS_KM, -GEO_RADIUS_KM] {
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
        for &alt_km in &[LEO_ALTITUDE_KM, 1_000.0, 5_000.0, R_EARTH] {
            let sc = Vector3::new(-(R_EARTH + alt_km), 0.0, 0.0);
            let state = compute_eclipse_state(&sc, &sun_eci_km);
            assert_eq!(
                state,
                EclipseState::Umbra,
                "On anti-Sun axis at alt={alt_km} km: expected Umbra, got {state:?}"
            );
        }
    }
}

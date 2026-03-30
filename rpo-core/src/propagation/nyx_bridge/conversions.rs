//! Type conversions between ROE-RUST types and nyx-space types.
//!
//! All unit and frame assumptions are documented at each conversion boundary.

use anise::prelude::Orbit;
use nalgebra::Vector3;
use nyx_space::cosmic::Spacecraft;

use crate::constants::EARTH_J2000;
use crate::types::{SpacecraftConfig, StateVector};

/// Convert a [`StateVector`] to an ANISE [`Orbit`] for ephemeris queries.
///
/// Uses the project's `EARTH_J2000` frame (with μ = 398600.4418 km³/s²).
/// Position in km, velocity in km/s, epoch from the state vector.
pub(crate) fn state_to_orbit(sv: &StateVector) -> Orbit {
    Orbit::new(
        sv.position_eci_km.x,
        sv.position_eci_km.y,
        sv.position_eci_km.z,
        sv.velocity_eci_km_s.x,
        sv.velocity_eci_km_s.y,
        sv.velocity_eci_km_s.z,
        sv.epoch,
        EARTH_J2000,
    )
}

/// Build a nyx [`Spacecraft`] from our [`StateVector`] and [`SpacecraftConfig`].
///
/// # Boundary assumptions
/// - Position/velocity in ECI J2000, km and km/s
/// - Epoch from `StateVector.epoch` (hifitime `Epoch`)
/// - Mass, areas, coefficients from `SpacecraftConfig`
pub(crate) fn config_to_spacecraft(sv: &StateVector, config: &SpacecraftConfig) -> Spacecraft {
    let orbit = state_to_orbit(sv);
    Spacecraft::from_srp_defaults(orbit, config.dry_mass_kg, config.srp_area_m2)
        .with_drag(config.drag_area_m2, config.coeff_drag)
        .with_cr(config.coeff_reflectivity)
}

/// Convert a nyx [`Spacecraft`] back to a ROE-RUST [`StateVector`].
///
/// Extracts ECI position (km) and velocity (km/s) from the spacecraft's orbit.
pub(crate) fn spacecraft_to_state(sc: &Spacecraft) -> StateVector {
    StateVector {
        epoch: sc.orbit.epoch,
        position_eci_km: Vector3::new(
            sc.orbit.radius_km.x,
            sc.orbit.radius_km.y,
            sc.orbit.radius_km.z,
        ),
        velocity_eci_km_s: Vector3::new(
            sc.orbit.velocity_km_s.x,
            sc.orbit.velocity_km_s.y,
            sc.orbit.velocity_km_s.z,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::test_helpers::{iss_like_elements, test_epoch};

    /// Spacecraft↔StateVector roundtrip tolerance. Nyx stores position
    /// and velocity as separate vectors; conversion introduces no error
    /// beyond floating-point representation.
    const SPACECRAFT_ROUNDTRIP_TOL: f64 = 1e-12;

    /// `config_to_spacecraft` → `spacecraft_to_state` roundtrip preserves
    /// position and velocity to machine precision.
    #[test]
    fn config_spacecraft_roundtrip() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let config = SpacecraftConfig::SERVICER_500KG;

        let sc = config_to_spacecraft(&sv, &config);
        let recovered = spacecraft_to_state(&sc);

        let pos_err = (recovered.position_eci_km - sv.position_eci_km).norm();
        let vel_err = (recovered.velocity_eci_km_s - sv.velocity_eci_km_s).norm();

        assert!(
            pos_err < SPACECRAFT_ROUNDTRIP_TOL,
            "position roundtrip error = {pos_err} km"
        );
        assert!(
            vel_err < SPACECRAFT_ROUNDTRIP_TOL,
            "velocity roundtrip error = {vel_err} km/s"
        );
        assert_eq!(recovered.epoch, sv.epoch, "epoch should be preserved");
    }

    /// Verify that `state_to_orbit` preserves position, velocity, and epoch.
    #[test]
    fn state_to_orbit_preserves_state() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let orbit = state_to_orbit(&sv);

        let pos_err = (Vector3::new(
            orbit.radius_km.x,
            orbit.radius_km.y,
            orbit.radius_km.z,
        ) - sv.position_eci_km)
            .norm();
        let vel_err = (Vector3::new(
            orbit.velocity_km_s.x,
            orbit.velocity_km_s.y,
            orbit.velocity_km_s.z,
        ) - sv.velocity_eci_km_s)
            .norm();

        assert!(
            pos_err < SPACECRAFT_ROUNDTRIP_TOL,
            "position error = {pos_err}"
        );
        assert!(
            vel_err < SPACECRAFT_ROUNDTRIP_TOL,
            "velocity error = {vel_err}"
        );
        assert_eq!(orbit.epoch, sv.epoch, "epoch should be preserved");
    }
}

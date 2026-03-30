//! Nyx-space propagation wrappers, impulse application, and safety state construction.

use std::sync::Arc;

use anise::constants::frames::{SUN_J2000, EARTH_J2000 as ANISE_EARTH_J2000};
use anise::prelude::{Almanac, Frame, Orbit};
use hifitime::Duration;
use nalgebra::Vector3;
use nyx_space::md::prelude::{Propagator, SpacecraftDynamics};
use rayon::prelude::*;

use serde::Serialize;

use crate::elements::keplerian_conversions::state_to_keplerian;
use crate::elements::eci_ric_dcm::{eci_to_ric_relative, ric_to_eci_dv, DcmError};
use crate::elements::roe::compute_roe;
use crate::propagation::propagator::PropagatedState;
use crate::types::{SpacecraftConfig, StateVector};

use super::errors::NyxBridgeError;
use super::conversions::{config_to_spacecraft, spacecraft_to_state};

/// An ECI state vector at a specific elapsed time from propagation start.
///
/// Replaces bare `(f64, StateVector)` tuples to satisfy the naming
/// rule: public/crate-visible quantities with physical units must carry unit
/// suffixes in their field names.
#[derive(Debug, Clone)]
pub(crate) struct TimedState {
    /// Elapsed time since propagation start (seconds).
    pub elapsed_s: f64,
    /// ECI state at this time.
    pub state: StateVector,
}

/// A chief/deputy ECI state pair at a specific elapsed time.
///
/// Used to pass time-tagged chief-deputy snapshots to safety analysis
/// routines where both states share the same elapsed-time reference.
#[derive(Debug, Clone, Serialize)]
pub struct ChiefDeputySnapshot {
    /// Elapsed time since mission/leg start (seconds).
    pub elapsed_s: f64,
    /// Chief ECI state at this time.
    pub chief: StateVector,
    /// Deputy ECI state at this time.
    pub deputy: StateVector,
}

/// Query ANISE for the solar eclipse percentage and Sun position at a given epoch.
///
/// Uses `almanac.solar_eclipsing()` with Earth as the eclipsing body for eclipse
/// percentage, and `almanac.translate(SUN_J2000, ...)` for the Sun position vector.
///
/// # Arguments
///
/// * `spacecraft_orbit` — Spacecraft as an ANISE `Orbit` in J2000
/// * `earth_frame` — Earth frame with radius info (from `almanac.frame_info(...)`)
/// * `almanac` — Full-physics ANISE almanac (from `load_full_almanac`)
///
/// # Returns
///
/// `(eclipse_percentage, sun_position_eci_km)` where `eclipse_percentage` is 0–100
/// and `sun_position_eci_km` is the Sun's geocentric ECI J2000 position in km.
///
/// # Errors
///
/// Returns [`NyxBridgeError`] if the eclipse or ephemeris query fails.
pub(crate) fn query_anise_eclipse(
    spacecraft_orbit: Orbit,
    earth_frame: Frame,
    almanac: &Arc<Almanac>,
) -> Result<(f64, Vector3<f64>), NyxBridgeError> {
    // Eclipse percentage via ANISE conical shadow model
    let occultation = almanac
        .solar_eclipsing(earth_frame, spacecraft_orbit, None)
        .map_err(|e| NyxBridgeError::AlmanacLoad {
            source: Box::new(e),
        })?;

    // Sun position in ECI J2000 via DE440s ephemeris
    let sun_state = almanac
        .translate(SUN_J2000, ANISE_EARTH_J2000, spacecraft_orbit.epoch, None)
        .map_err(|e| NyxBridgeError::EphemerisQuery {
            source: Box::new(e),
        })?;
    let sun_eci = Vector3::new(
        sun_state.radius_km.x,
        sun_state.radius_km.y,
        sun_state.radius_km.z,
    );

    Ok((occultation.percentage, sun_eci))
}

/// Propagate a spacecraft through nyx for a given duration, returning
/// the final state. Optionally returns intermediate states at evenly-spaced
/// sample times.
///
/// When `n_samples` is 0, returns only the final state.
/// When `n_samples` > 0, returns `n_samples + 1` points (initial + samples).
///
/// # Invariants
/// - `sv` must represent a bound orbit (`e < 1`, `a > 0`)
/// - `duration_s` must be finite and positive
///
/// # Errors
/// Returns [`NyxBridgeError::Propagation`] if nyx propagation fails.
pub(crate) fn nyx_propagate_segment(
    sv: &StateVector,
    duration_s: f64,
    n_samples: u32,
    config: &SpacecraftConfig,
    dynamics: SpacecraftDynamics,
    almanac: &Arc<Almanac>,
) -> Result<Vec<TimedState>, NyxBridgeError> {
    let propagator = Propagator::default(dynamics);

    if n_samples == 0 {
        let sc = config_to_spacecraft(sv, config);
        let final_sc = propagator
            .with(sc, almanac.clone())
            .for_duration(Duration::from_seconds(duration_s))?;
        Ok(vec![TimedState { elapsed_s: duration_s, state: spacecraft_to_state(&final_sc) }])
    } else {
        let dt = duration_s / f64::from(n_samples);
        let mut results = Vec::with_capacity(n_samples as usize + 1); // u32 → usize: always safe (usize ≥ 32 bits)
        results.push(TimedState { elapsed_s: 0.0, state: sv.clone() });
        let mut current_sv = sv.clone();

        for i in 1..=n_samples {
            let sc = config_to_spacecraft(&current_sv, config);
            let final_sc = propagator
                .with(sc, almanac.clone())
                .for_duration(Duration::from_seconds(dt))?;
            current_sv = spacecraft_to_state(&final_sc);
            let elapsed = f64::from(i) * dt;
            results.push(TimedState { elapsed_s: elapsed, state: current_sv.clone() });
        }

        Ok(results)
    }
}

/// Apply an impulsive Δv (in RIC frame) to the deputy spacecraft.
///
/// Converts the Δv from the chief-centered RIC frame to ECI via [`ric_to_eci_dv`]
/// and adds it to the deputy's velocity. Position and epoch are unchanged.
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero (for RIC frame definition)
/// - `chief` angular momentum must be non-zero
pub(crate) fn apply_impulse(
    deputy: &StateVector,
    chief: &StateVector,
    dv_ric_km_s: &Vector3<f64>,
) -> Result<StateVector, DcmError> {
    let dv_eci = ric_to_eci_dv(dv_ric_km_s, chief)?;
    Ok(StateVector {
        epoch: deputy.epoch,
        position_eci_km: deputy.position_eci_km,
        velocity_eci_km_s: deputy.velocity_eci_km_s + dv_eci,
    })
}

/// Build [`PropagatedState`] entries from chief/deputy ECI pairs for safety analysis.
///
/// For each pair, converts ECI states to Keplerian elements, computes QNS ROE,
/// and computes the RIC relative state. The `chief_mean` field contains osculating
/// elements (acceptable for validation — safety uses ROE/RIC, not the Keplerian
/// field except for `a_km` scaling, where osculating ≈ mean for near-circular orbits).
///
/// # Invariants
/// - Each `StateVector` in the input must represent a valid ECI state with
///   non-zero position (required for Keplerian element extraction and RIC
///   frame construction)
/// - Each pair must represent a bound orbit (`e < 1`, `a > 0`)
///
/// # Errors
/// Returns [`NyxBridgeError::Conversion`] if Keplerian element extraction fails.
pub(crate) fn build_nyx_safety_states(
    chief_deputy_pairs: &[ChiefDeputySnapshot],
) -> Result<Vec<PropagatedState>, NyxBridgeError> {
    chief_deputy_pairs
        .par_iter()
        .map(|snapshot| {
            let chief_ke = state_to_keplerian(&snapshot.chief)?;
            let deputy_ke = state_to_keplerian(&snapshot.deputy)?;
            let roe = compute_roe(&chief_ke, &deputy_ke)?;
            let ric = eci_to_ric_relative(&snapshot.chief, &snapshot.deputy)?;
            Ok(PropagatedState {
                epoch: snapshot.chief.epoch,
                roe,
                chief_mean: chief_ke,
                ric,
                elapsed_s: snapshot.elapsed_s,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::eci_ric_dcm::ric_to_eci_dv;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::test_helpers::{iss_like_elements, test_epoch};

    /// Position must be unchanged after impulse (exact identity operation).
    /// 1e-15 is at machine epsilon level for km-scale positions.
    const IMPULSE_POSITION_TOL: f64 = 1e-15;

    /// Velocity error from RIC→ECI DCM transformation and addition.
    /// Machine-precision for km/s-scale velocities.
    const IMPULSE_VELOCITY_TOL: f64 = 1e-15;

    /// DCM orthogonality preserves Δv magnitude. 1e-14 accounts for
    /// two matrix-vector multiplications at machine precision.
    const DV_MAGNITUDE_PRESERVATION_TOL: f64 = 1e-14;

    /// Verify that `apply_impulse` correctly transforms RIC Δv to ECI and applies it.
    ///
    /// Checks: position unchanged, epoch unchanged, velocity changed by
    /// ECI-transformed Δv, Δv magnitude preserved (DCM orthogonality),
    /// zero Δv returns identical state.
    #[test]
    fn apply_impulse_preserves_position_and_epoch() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        // Apply known RIC Δv
        let dv_ric = Vector3::new(0.001, -0.002, 0.0005);
        let result = apply_impulse(&deputy_sv, &chief_sv, &dv_ric).unwrap();

        // Position unchanged
        let pos_err = (result.position_eci_km - deputy_sv.position_eci_km).norm();
        assert!(pos_err < IMPULSE_POSITION_TOL, "position should be unchanged, error = {pos_err}");

        // Epoch unchanged
        assert_eq!(result.epoch, deputy_sv.epoch, "epoch should be unchanged");

        // Velocity changed by ECI-transformed Δv
        let dv_eci = ric_to_eci_dv(&dv_ric, &chief_sv).unwrap();
        let expected_vel = deputy_sv.velocity_eci_km_s + dv_eci;
        let vel_err = (result.velocity_eci_km_s - expected_vel).norm();
        assert!(vel_err < IMPULSE_VELOCITY_TOL, "velocity error = {vel_err}");

        // Δv magnitude preserved (DCM is orthogonal)
        let applied_dv_mag = (result.velocity_eci_km_s - deputy_sv.velocity_eci_km_s).norm();
        assert!(
            (applied_dv_mag - dv_ric.norm()).abs() < DV_MAGNITUDE_PRESERVATION_TOL,
            "Δv magnitude not preserved: {applied_dv_mag} vs {}",
            dv_ric.norm()
        );
    }

    /// Zero Δv should return an identical state (identity operation).
    #[test]
    fn apply_impulse_zero_dv_identity() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let result = apply_impulse(&deputy_sv, &chief_sv, &Vector3::zeros()).unwrap();
        let vel_err = (result.velocity_eci_km_s - deputy_sv.velocity_eci_km_s).norm();
        assert!(vel_err < IMPULSE_VELOCITY_TOL, "zero Δv should not change velocity");
    }
}

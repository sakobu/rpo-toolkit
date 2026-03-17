//! Nyx-space integration bridge: almanac loading, dynamics construction,
//! propagation helpers, and DMF rate extraction.
//!
//! All adapters between ROE-RUST types and nyx-space types live here.
//! Unit and frame assumptions are documented at each conversion boundary.

use std::sync::Arc;

use anise::almanac::planetary::PlanetaryDataError;
use anise::constants::celestial_objects::{MOON, SUN};
use anise::constants::frames::IAU_EARTH_FRAME;
use anise::errors::AlmanacError;
use anise::prelude::{Almanac, MetaAlmanac, Orbit};
use hifitime::Duration;
use nalgebra::Vector3;
use nyx_space::cosmic::Spacecraft;
use nyx_space::dynamics::{DynamicsError, Drag, Harmonics, PointMasses, SolarPressure};
use nyx_space::io::gravity::HarmonicsMem;
use nyx_space::md::prelude::{OrbitalDynamics, Propagator, SpacecraftDynamics};
use nyx_space::propagators::PropagationError as NyxPropagationError;

use crate::constants::EARTH_J2000;
use crate::elements::keplerian_conversions::{state_to_keplerian, ConversionError};
use crate::elements::eci_ric_dcm::{eci_to_ric_relative, ric_to_eci_dv, DcmError};
use crate::elements::roe::compute_roe;
use crate::propagation::propagator::{DragConfig, PropagatedState};
use crate::types::{KeplerError, SpacecraftConfig, StateVector};

/// Errors from the nyx-space integration bridge.
#[derive(Debug)]
pub enum NyxBridgeError {
    /// `MetaAlmanac` / ANISE kernel loading failure.
    AlmanacLoad {
        /// The underlying almanac error.
        source: Box<AlmanacError>,
    },
    /// Frame information retrieval failure (`IAU_EARTH`, etc.)
    FrameLookup {
        /// The underlying planetary data error.
        source: PlanetaryDataError,
    },
    /// Force model initialization failure (drag, SRP).
    DynamicsSetup {
        /// The underlying dynamics error.
        source: DynamicsError,
    },
    /// Nyx propagation failure.
    Propagation {
        /// The underlying nyx propagation error.
        source: NyxPropagationError,
    },
    /// ECI → Keplerian or ROE conversion failure.
    Conversion {
        /// The underlying conversion error.
        source: ConversionError,
    },
    /// Nyx propagation returned no data points.
    EmptyResult,
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
}

impl std::fmt::Display for NyxBridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AlmanacLoad { source } => {
                write!(f, "almanac loading failed: {source}")
            }
            Self::FrameLookup { source } => {
                write!(f, "frame lookup failed: {source}")
            }
            Self::DynamicsSetup { source } => {
                write!(f, "dynamics setup failed: {source}")
            }
            Self::Propagation { source } => {
                write!(f, "nyx propagation failed: {source}")
            }
            Self::Conversion { source } => {
                write!(f, "conversion failed: {source}")
            }
            Self::EmptyResult => {
                write!(f, "nyx propagation returned no data points")
            }
            Self::DcmFailure(e) => {
                write!(f, "frame conversion failed: {e}")
            }
        }
    }
}

impl std::error::Error for NyxBridgeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::AlmanacLoad { source } => Some(source.as_ref()),
            Self::FrameLookup { source } => Some(source),
            Self::DynamicsSetup { source } => Some(source),
            Self::Propagation { source } => Some(source),
            Self::Conversion { source } => Some(source),
            Self::DcmFailure(e) => Some(e),
            Self::EmptyResult => None,
        }
    }
}

impl From<DcmError> for NyxBridgeError {
    fn from(e: DcmError) -> Self {
        Self::DcmFailure(e)
    }
}

impl From<KeplerError> for NyxBridgeError {
    fn from(e: KeplerError) -> Self {
        Self::Conversion { source: ConversionError::from(e) }
    }
}

impl From<ConversionError> for NyxBridgeError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion { source: e }
    }
}

impl From<NyxPropagationError> for NyxBridgeError {
    fn from(e: NyxPropagationError) -> Self {
        Self::Propagation { source: e }
    }
}

impl From<DynamicsError> for NyxBridgeError {
    fn from(e: DynamicsError) -> Self {
        Self::DynamicsSetup { source: e }
    }
}

impl From<PlanetaryDataError> for NyxBridgeError {
    fn from(e: PlanetaryDataError) -> Self {
        Self::FrameLookup { source: e }
    }
}

/// Threshold below which extracted DMF rates are treated as numerical noise
/// and replaced with zero. For ISS-like orbits with identical ballistic
/// coefficients, residual rates from numerical integration are O(1e-16).
const DMF_NOISE_THRESHOLD: f64 = 1.0e-15;

/// Create a default ANISE almanac for two-body propagation.
///
/// Two-body dynamics only need the frame's `mu` value (baked into `EARTH_J2000`),
/// so an empty `Almanac::default()` suffices — no network download required.
#[must_use]
pub fn load_default_almanac() -> Arc<Almanac> {
    Arc::new(Almanac::default())
}

/// Load a full-physics almanac with all ANISE kernel data.
///
/// Downloads and caches to `~/.local/share/nyx-space/anise/`:
/// - `de440s.bsp` (planetary ephemerides, ~32 MB)
/// - `pck11.pca` (planetary constants)
/// - `earth_latest_high_prec.bpc` (Earth orientation from JPL)
/// - `moon_pa_de440_200625.bpc` (Moon orientation)
///
/// # Errors
/// Returns [`NyxBridgeError::AlmanacLoad`] if kernel download or parsing fails.
pub fn load_full_almanac() -> Result<Arc<Almanac>, NyxBridgeError> {
    let almanac = MetaAlmanac::latest()
        .map_err(|e| NyxBridgeError::AlmanacLoad { source: Box::new(e) })?;
    Ok(Arc::new(almanac))
}

/// Extract density-model-free (DMF) drag rates by running a short nyx simulation.
///
/// Implements the DMF formulation from Koenig Sec. VIII (Eqs. 73–77): the
/// augmented 9×9 STM assumes constant differential drag rates, which this
/// function estimates via a secular fit over a short full-physics arc.
///
/// Algorithm:
/// 1. Build full-physics dynamics for both chief and deputy
/// 2. Propagate both for 2 orbital periods under full physics
/// 3. Convert initial and final states to Keplerian elements
/// 4. Compute QNS ROE at t=0 and t=T
/// 5. Fit secular drift: rate = ΔROE / T
/// 6. Return `DragConfig { da_dot, dex_dot, dey_dot }`
///
/// Takes separate configs because differential drag arises from different
/// ballistic coefficients between chief and deputy.
///
/// # Near-zero guard
/// If chief and deputy have identical ballistic coefficients, all rates
/// will be near-zero. Returns `DragConfig::zero()` when extracted rates
/// are below `DMF_NOISE_THRESHOLD` (1e-15).
///
/// # Errors
/// Returns [`NyxBridgeError`] if almanac frame lookup, dynamics setup,
/// propagation, or Keplerian conversion fails.
#[allow(clippy::similar_names)] // rate_ex / rate_ey are standard eccentricity vector components
pub fn extract_dmf_rates(
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<DragConfig, NyxBridgeError> {
    // Compute initial ROE
    let chief_ke_0 = state_to_keplerian(chief_initial)?;
    let deputy_ke_0 = state_to_keplerian(deputy_initial)?;
    let roe_0 = compute_roe(&chief_ke_0, &deputy_ke_0)?;

    // Propagation duration: 2 orbital periods
    let period = chief_ke_0.period()?;
    let duration = 2.0 * period;

    // Build dynamics and propagate both spacecraft
    let chief_dynamics = build_full_physics_dynamics(almanac)?;
    let chief_results = nyx_propagate_segment(
        chief_initial, duration, 0, chief_config, chief_dynamics, almanac,
    )?;
    let chief_final = &chief_results.last().ok_or(NyxBridgeError::EmptyResult)?.1;

    let deputy_dynamics = build_full_physics_dynamics(almanac)?;
    let deputy_results = nyx_propagate_segment(
        deputy_initial, duration, 0, deputy_config, deputy_dynamics, almanac,
    )?;
    let deputy_final = &deputy_results.last().ok_or(NyxBridgeError::EmptyResult)?.1;

    // Compute final ROE
    let chief_ke_f = state_to_keplerian(chief_final)?;
    let deputy_ke_f = state_to_keplerian(deputy_final)?;
    let roe_f = compute_roe(&chief_ke_f, &deputy_ke_f)?;

    // Fit secular drift rates
    let rate_da = (roe_f.da - roe_0.da) / duration;
    let rate_ex = (roe_f.dex - roe_0.dex) / duration;
    let rate_ey = (roe_f.dey - roe_0.dey) / duration;

    // Near-zero guard: treat sub-threshold rates as noise
    if rate_da.abs() < DMF_NOISE_THRESHOLD
        && rate_ex.abs() < DMF_NOISE_THRESHOLD
        && rate_ey.abs() < DMF_NOISE_THRESHOLD
    {
        return Ok(DragConfig::zero());
    }

    Ok(DragConfig {
        da_dot: rate_da,
        dex_dot: rate_ex,
        dey_dot: rate_ey,
    })
}

/// Build full-physics spacecraft dynamics for nyx propagation.
///
/// Force model stack:
/// 1. J2 gravity (Harmonics with JGM3 J2 coefficient, body-fixed frame)
/// 2. US Standard Atmosphere 1976 drag model
/// 3. Solar radiation pressure with eclipse detection
/// 4. Sun + Moon third-body gravitational perturbations
///
/// Spacecraft-specific properties (Cd, drag area, Cr, SRP area) are set on
/// the `Spacecraft` object via [`config_to_spacecraft`], not here.
///
/// # Errors
/// Returns [`NyxBridgeError::FrameLookup`] if `IAU_EARTH` frame data is unavailable,
/// or [`NyxBridgeError::DynamicsSetup`] if drag/SRP initialization fails.
pub(crate) fn build_full_physics_dynamics(
    almanac: &Arc<Almanac>,
) -> Result<SpacecraftDynamics, NyxBridgeError> {
    let iau_earth = almanac
        .frame_info(IAU_EARTH_FRAME)
        .map_err(|e| NyxBridgeError::FrameLookup { source: e })?;

    // J2 harmonics (body-fixed frame)
    let j2_harmonics = Harmonics::from_stor(iau_earth, HarmonicsMem::j2_jgm3());
    // Third-body perturbations
    let third_body = PointMasses::new(vec![MOON, SUN]);
    let orbital_dyn = OrbitalDynamics::new(vec![j2_harmonics, third_body]);

    // Spacecraft force models
    let drag = Drag::std_atm1976(almanac.clone())
        .map_err(|e| NyxBridgeError::DynamicsSetup { source: e })?;
    let srp = SolarPressure::default(iau_earth, almanac.clone())
        .map_err(|e| NyxBridgeError::DynamicsSetup { source: e })?;

    Ok(SpacecraftDynamics::from_models(orbital_dyn, vec![drag, srp]))
}

/// Build a nyx [`Spacecraft`] from our [`StateVector`] and [`SpacecraftConfig`].
///
/// # Boundary assumptions
/// - Position/velocity in ECI J2000, km and km/s
/// - Epoch from `StateVector.epoch` (hifitime `Epoch`)
/// - Mass, areas, coefficients from `SpacecraftConfig`
pub(crate) fn config_to_spacecraft(sv: &StateVector, config: &SpacecraftConfig) -> Spacecraft {
    let orbit = Orbit::new(
        sv.position_eci_km.x,
        sv.position_eci_km.y,
        sv.position_eci_km.z,
        sv.velocity_eci_km_s.x,
        sv.velocity_eci_km_s.y,
        sv.velocity_eci_km_s.z,
        sv.epoch,
        EARTH_J2000,
    );
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

/// Propagate a spacecraft through nyx for a given duration, returning
/// the final state. Optionally returns intermediate states at evenly-spaced
/// sample times.
///
/// When `n_samples` is 0, returns only the final state.
/// When `n_samples` > 0, returns `n_samples + 1` points (initial + samples).
///
/// Returns `Vec<(elapsed_s, state)>` — each tuple contains the elapsed
/// time in seconds since propagation start and the ECI state at that time.
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
) -> Result<Vec<(f64, StateVector)>, NyxBridgeError> {
    if n_samples == 0 {
        let sc = config_to_spacecraft(sv, config);
        let propagator = Propagator::default(dynamics);
        let final_sc = propagator
            .with(sc, almanac.clone())
            .for_duration(Duration::from_seconds(duration_s))?;
        Ok(vec![(duration_s, spacecraft_to_state(&final_sc))])
    } else {
        let dt = duration_s / f64::from(n_samples);
        let mut results = Vec::with_capacity(n_samples as usize + 1);
        results.push((0.0, sv.clone()));
        let mut current_sv = sv.clone();

        for i in 1..=n_samples {
            let sc = config_to_spacecraft(&current_sv, config);
            let propagator = Propagator::default(dynamics.clone());
            let final_sc = propagator
                .with(sc, almanac.clone())
                .for_duration(Duration::from_seconds(dt))?;
            current_sv = spacecraft_to_state(&final_sc);
            let elapsed = f64::from(i) * dt;
            results.push((elapsed, current_sv.clone()));
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
/// # Errors
/// Returns [`NyxBridgeError::Conversion`] if Keplerian element extraction fails.
pub(crate) fn build_nyx_safety_states(
    chief_deputy_pairs: &[(f64, StateVector, StateVector)],
) -> Result<Vec<PropagatedState>, NyxBridgeError> {
    let mut states = Vec::with_capacity(chief_deputy_pairs.len());
    for (elapsed_s, chief, deputy) in chief_deputy_pairs {
        let chief_ke = state_to_keplerian(chief)?;
        let deputy_ke = state_to_keplerian(deputy)?;
        let roe = compute_roe(&chief_ke, &deputy_ke)?;
        let ric = eci_to_ric_relative(chief, deputy)?;
        states.push(PropagatedState {
            epoch: chief.epoch,
            roe,
            chief_mean: chief_ke,
            ric,
            elapsed_s: *elapsed_s,
        });
    }
    Ok(states)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::eci_ric_dcm::ric_to_eci_dv;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::test_helpers::{iss_like_elements, test_epoch};

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
        assert!(pos_err < 1e-15, "position should be unchanged, error = {pos_err}");

        // Epoch unchanged
        assert_eq!(result.epoch, deputy_sv.epoch, "epoch should be unchanged");

        // Velocity changed by ECI-transformed Δv
        let dv_eci = ric_to_eci_dv(&dv_ric, &chief_sv).unwrap();
        let expected_vel = deputy_sv.velocity_eci_km_s + dv_eci;
        let vel_err = (result.velocity_eci_km_s - expected_vel).norm();
        assert!(vel_err < 1e-15, "velocity error = {vel_err}");

        // Δv magnitude preserved (DCM is orthogonal)
        let applied_dv_mag = (result.velocity_eci_km_s - deputy_sv.velocity_eci_km_s).norm();
        assert!(
            (applied_dv_mag - dv_ric.norm()).abs() < 1e-14,
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
        assert!(vel_err < 1e-15, "zero Δv should not change velocity");
    }

    /// `config_to_spacecraft` → `spacecraft_to_state` roundtrip preserves
    /// position and velocity to machine precision.
    #[test]
    fn config_spacecraft_roundtrip() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let config = SpacecraftConfig::default();

        let sc = config_to_spacecraft(&sv, &config);
        let recovered = spacecraft_to_state(&sc);

        let pos_err = (recovered.position_eci_km - sv.position_eci_km).norm();
        let vel_err = (recovered.velocity_eci_km_s - sv.velocity_eci_km_s).norm();

        assert!(
            pos_err < 1e-12,
            "position roundtrip error = {pos_err} km"
        );
        assert!(
            vel_err < 1e-12,
            "velocity roundtrip error = {vel_err} km/s"
        );
        assert_eq!(recovered.epoch, sv.epoch, "epoch should be preserved");
    }

    /// `load_default_almanac` returns a valid almanac without panicking.
    #[test]
    fn load_default_almanac_succeeds() {
        let almanac = load_default_almanac();
        // Verify the Arc is non-null (Almanac::default() always succeeds)
        assert!(Arc::strong_count(&almanac) >= 1);
    }
}

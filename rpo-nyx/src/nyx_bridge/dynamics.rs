//! Force model construction and density-model-free (DMF) drag rate extraction.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anise::constants::celestial_objects::{MOON, SUN};
use anise::constants::frames::IAU_EARTH_FRAME;
use anise::prelude::Almanac;
use nyx_space::dynamics::{Drag, Harmonics, PointMasses, SolarPressure};
use nyx_space::io::gravity::HarmonicsMem;
use nyx_space::md::prelude::{OrbitalDynamics, SpacecraftDynamics};

use rpo_core::elements::keplerian_conversions::state_to_keplerian;
use rpo_core::elements::roe::compute_roe;
use rpo_core::propagation::propagator::DragConfig;
use rpo_core::types::{SpacecraftConfig, StateVector};

use super::errors::NyxBridgeError;
use super::propagate::nyx_propagate_segment;

/// Threshold below which extracted DMF rates are treated as numerical noise
/// and replaced with zero. For ISS-like orbits with identical ballistic
/// coefficients, residual rates from numerical integration are O(1e-16).
const DMF_NOISE_THRESHOLD: f64 = 1.0e-15;
/// Fixed propagation chunk for cooperative cancellation checks.
const DMF_CANCEL_CHECK_STEP_S: f64 = 120.0;

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
/// # Invariants
/// - `chief_initial` and `deputy_initial` must represent bound orbits (`e < 1`, `a > 0`)
/// - `chief_config` and `deputy_config` must have positive `mass_kg`,
///   `drag_area_m2`, and `srp_area_m2`
/// - `almanac` must contain Earth frame data (EME2000/J2000) for dynamics setup
///
/// # Near-zero guard
/// If chief and deputy have identical ballistic coefficients, all rates
/// will be near-zero. Returns `DragConfig::zero()` when extracted rates
/// are below `DMF_NOISE_THRESHOLD` (1e-15).
///
/// # Singularities and regime boundaries
/// - **Near-parabolic orbits:** Keplerian conversion at arc endpoints may
///   lose precision for high-eccentricity orbits.
/// - **Very small separation:** If chief and deputy are nearly co-located,
///   the ROE difference is dominated by numerical noise, making the secular
///   fit unreliable. The near-zero guard mitigates this for drag rates.
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

    // Build dynamics once; clone is cheap (Arc ref-count bumps).
    let dynamics = build_full_physics_dynamics(almanac)?;

    // Propagate chief and deputy in parallel (independent full-physics propagations).
    let (chief_result, deputy_result) = rayon::join(
        || nyx_propagate_segment(chief_initial, duration, 0, chief_config, dynamics.clone(), almanac),
        || nyx_propagate_segment(deputy_initial, duration, 0, deputy_config, dynamics.clone(), almanac),
    );
    let chief_states = chief_result?;
    let deputy_states = deputy_result?;
    let chief_final = &chief_states.last().ok_or(NyxBridgeError::EmptyResult)?.state;
    let deputy_final = &deputy_states.last().ok_or(NyxBridgeError::EmptyResult)?.state;

    compute_drag_rates(&roe_0, chief_final, deputy_final, duration)
}

/// Extract DMF drag rates with cooperative cancellation support.
///
/// Identical algorithm to [`extract_dmf_rates`], but chief and deputy are
/// propagated sequentially in `DMF_CANCEL_CHECK_STEP_S` chunks instead of
/// in parallel via `rayon::join`. This trades ~2× wall-clock time for
/// responsive cancellation — the flag is checked between every chunk.
///
/// # Errors
/// Returns [`NyxBridgeError::Cancelled`] if `cancel` is set during execution,
/// otherwise forwards the same error cases as [`extract_dmf_rates`].
#[allow(clippy::similar_names)]
pub fn extract_dmf_rates_with_cancel(
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    cancel: &AtomicBool,
) -> Result<DragConfig, NyxBridgeError> {
    if cancel.load(Ordering::Relaxed) {
        return Err(NyxBridgeError::Cancelled);
    }

    // Compute initial ROE
    let chief_ke_0 = state_to_keplerian(chief_initial)?;
    let deputy_ke_0 = state_to_keplerian(deputy_initial)?;
    let roe_0 = compute_roe(&chief_ke_0, &deputy_ke_0)?;

    // Propagation duration: 2 orbital periods
    let period = chief_ke_0.period()?;
    let duration = 2.0 * period;

    // Build dynamics once; clone is cheap (Arc ref-count bumps).
    let dynamics = build_full_physics_dynamics(almanac)?;

    // Sequential chunked propagation for responsive cancellation.
    let chief_final = propagate_final_state_with_cancel(
        chief_initial,
        duration,
        chief_config,
        &dynamics,
        almanac,
        cancel,
    )?;
    let deputy_final = propagate_final_state_with_cancel(
        deputy_initial,
        duration,
        deputy_config,
        &dynamics,
        almanac,
        cancel,
    )?;

    if cancel.load(Ordering::Relaxed) {
        return Err(NyxBridgeError::Cancelled);
    }

    compute_drag_rates(&roe_0, &chief_final, &deputy_final, duration)
}

fn propagate_final_state_with_cancel(
    initial: &StateVector,
    duration_s: f64,
    config: &SpacecraftConfig,
    dynamics: &SpacecraftDynamics,
    almanac: &Arc<Almanac>,
    cancel: &AtomicBool,
) -> Result<StateVector, NyxBridgeError> {
    let mut current = *initial;
    let mut elapsed_s = 0.0_f64;

    while elapsed_s < duration_s {
        if cancel.load(Ordering::Relaxed) {
            return Err(NyxBridgeError::Cancelled);
        }

        let remaining_s = duration_s - elapsed_s;
        let step_s = remaining_s.min(DMF_CANCEL_CHECK_STEP_S);
        let result = nyx_propagate_segment(&current, step_s, 0, config, dynamics.clone(), almanac)?;
        let final_state = result.last().ok_or(NyxBridgeError::EmptyResult)?;
        current = final_state.state;
        elapsed_s += step_s;
    }

    Ok(current)
}

/// Compute drag rates from initial ROE and propagated final states.
///
/// Shared by [`extract_dmf_rates`] (parallel) and
/// [`extract_dmf_rates_with_cancel`] (sequential/cancellable).
#[allow(clippy::similar_names)]
fn compute_drag_rates(
    roe_0: &rpo_core::types::roe::QuasiNonsingularROE,
    chief_final: &StateVector,
    deputy_final: &StateVector,
    duration: f64,
) -> Result<DragConfig, NyxBridgeError> {
    let chief_ke_f = state_to_keplerian(chief_final)?;
    let deputy_ke_f = state_to_keplerian(deputy_final)?;
    let roe_f = compute_roe(&chief_ke_f, &deputy_ke_f)?;

    let rate_da = (roe_f.da - roe_0.da) / duration;
    let rate_ex = (roe_f.dex - roe_0.dex) / duration;
    let rate_ey = (roe_f.dey - roe_0.dey) / duration;

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
/// the `Spacecraft` object via `config_to_spacecraft`, not here.
///
/// # Invariants
/// - `almanac` must contain Earth frame data (`IAU_EARTH`) for the
///   body-fixed gravity and drag models
///
/// # Errors
/// Returns [`NyxBridgeError::FrameLookup`] if `IAU_EARTH` frame data is unavailable,
/// or [`NyxBridgeError::DynamicsSetup`] if drag/SRP initialization fails.
pub fn build_full_physics_dynamics(
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

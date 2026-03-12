//! Validation helpers and integration tests for nyx numerical propagation comparisons.
//!
//! Provides utility functions for converting between ROE-RUST types and nyx-space types,
//! loading the ANISE almanac, and running nyx two-body numerical propagation.

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
use crate::elements::conversions::{state_to_keplerian, ConversionError};
use crate::elements::frames::{eci_to_ric_relative, ric_to_eci_dv};
use crate::elements::roe::compute_roe;
use crate::mission::safety::{analyze_trajectory_safety, SafetyError};
use crate::propagation::propagator::PropagatedState;
use crate::types::{
    DragConfig, RICState, SpacecraftConfig, StateVector, ValidationPoint,
    ValidationReport, WaypointMission,
};

/// Errors from nyx high-fidelity validation.
#[derive(Debug)]
pub enum ValidationError {
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
    /// Safety analysis failure.
    Safety {
        /// The underlying safety error.
        source: SafetyError,
    },
    /// No trajectory points to analyze.
    EmptyTrajectory,
}

impl std::fmt::Display for ValidationError {
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
            Self::Safety { source } => {
                write!(f, "safety analysis failed: {source}")
            }
            Self::EmptyTrajectory => {
                write!(f, "no trajectory points to analyze")
            }
        }
    }
}

impl std::error::Error for ValidationError {}

impl From<ConversionError> for ValidationError {
    fn from(e: ConversionError) -> Self {
        Self::Conversion { source: e }
    }
}

impl From<SafetyError> for ValidationError {
    fn from(e: SafetyError) -> Self {
        Self::Safety { source: e }
    }
}

impl From<NyxPropagationError> for ValidationError {
    fn from(e: NyxPropagationError) -> Self {
        Self::Propagation { source: e }
    }
}

impl From<DynamicsError> for ValidationError {
    fn from(e: DynamicsError) -> Self {
        Self::DynamicsSetup { source: e }
    }
}

impl From<PlanetaryDataError> for ValidationError {
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
/// Returns [`ValidationError::AlmanacLoad`] if kernel download or parsing fails.
pub fn load_full_almanac() -> Result<Arc<Almanac>, ValidationError> {
    let almanac = MetaAlmanac::latest()
        .map_err(|e| ValidationError::AlmanacLoad { source: Box::new(e) })?;
    Ok(Arc::new(almanac))
}

/// Extract density-model-free (DMF) drag rates by running a short nyx simulation.
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
/// # Panics
/// Panics if `nyx_propagate_segment` returns an empty result vector, which
/// cannot happen when `n_samples = 0` (always returns exactly one element).
///
/// # Errors
/// Returns [`ValidationError`] if almanac frame lookup, dynamics setup,
/// propagation, or Keplerian conversion fails.
#[allow(clippy::similar_names)] // rate_ex / rate_ey are standard eccentricity vector components
pub fn extract_dmf_rates(
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<DragConfig, ValidationError> {
    // Compute initial ROE
    let chief_ke_0 = state_to_keplerian(chief_initial)?;
    let deputy_ke_0 = state_to_keplerian(deputy_initial)?;
    let roe_0 = compute_roe(&chief_ke_0, &deputy_ke_0)?;

    // Propagation duration: 2 orbital periods
    let period = chief_ke_0.period();
    let duration = 2.0 * period;

    // Build dynamics and propagate both spacecraft
    let chief_dynamics = build_full_physics_dynamics(almanac)?;
    let chief_results = nyx_propagate_segment(
        chief_initial, duration, 0, chief_config, chief_dynamics, almanac,
    )?;
    let chief_final = &chief_results.last().unwrap().1;

    let deputy_dynamics = build_full_physics_dynamics(almanac)?;
    let deputy_results = nyx_propagate_segment(
        deputy_initial, duration, 0, deputy_config, deputy_dynamics, almanac,
    )?;
    let deputy_final = &deputy_results.last().unwrap().1;

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
/// Returns [`ValidationError::FrameLookup`] if `IAU_EARTH` frame data is unavailable,
/// or [`ValidationError::DynamicsSetup`] if drag/SRP initialization fails.
fn build_full_physics_dynamics(
    almanac: &Arc<Almanac>,
) -> Result<SpacecraftDynamics, ValidationError> {
    let iau_earth = almanac
        .frame_info(IAU_EARTH_FRAME)
        .map_err(|e| ValidationError::FrameLookup { source: e })?;

    // J2 harmonics (body-fixed frame)
    let j2_harmonics = Harmonics::from_stor(iau_earth, HarmonicsMem::j2_jgm3());
    // Third-body perturbations
    let third_body = PointMasses::new(vec![MOON, SUN]);
    let orbital_dyn = OrbitalDynamics::new(vec![j2_harmonics, third_body]);

    // Spacecraft force models
    let drag = Drag::std_atm1976(almanac.clone())
        .map_err(|e| ValidationError::DynamicsSetup { source: e })?;
    let srp = SolarPressure::default(iau_earth, almanac.clone())
        .map_err(|e| ValidationError::DynamicsSetup { source: e })?;

    Ok(SpacecraftDynamics::from_models(orbital_dyn, vec![drag, srp]))
}

/// Build a nyx [`Spacecraft`] from our [`StateVector`] and [`SpacecraftConfig`].
///
/// # Boundary assumptions
/// - Position/velocity in ECI J2000, km and km/s
/// - Epoch from `StateVector.epoch` (hifitime `Epoch`)
/// - Mass, areas, coefficients from `SpacecraftConfig`
fn config_to_spacecraft(sv: &StateVector, config: &SpacecraftConfig) -> Spacecraft {
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
fn spacecraft_to_state(sc: &Spacecraft) -> StateVector {
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
/// Returns [`ValidationError::Propagation`] if nyx propagation fails.
fn nyx_propagate_segment(
    sv: &StateVector,
    duration_s: f64,
    n_samples: u32,
    config: &SpacecraftConfig,
    dynamics: SpacecraftDynamics,
    almanac: &Arc<Almanac>,
) -> Result<Vec<(f64, StateVector)>, ValidationError> {
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

/// Find the closest analytical RIC state by elapsed time (nearest-neighbor).
///
/// Falls back to zero RIC if the trajectory is empty. Interpolation error
/// from nearest-neighbor lookup is ~14m for a 200-step trajectory, well below
/// expected analytical-vs-numerical divergence.
fn find_closest_analytical_ric(trajectory: &[PropagatedState], elapsed_s: f64) -> RICState {
    trajectory
        .iter()
        .min_by(|a, b| {
            (a.elapsed_s - elapsed_s)
                .abs()
                .partial_cmp(&(b.elapsed_s - elapsed_s).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map_or_else(
            || RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::zeros(),
            },
            |s| s.ric.clone(),
        )
}

/// Apply an impulsive Δv (in RIC frame) to the deputy spacecraft.
///
/// Converts the Δv from the chief-centered RIC frame to ECI via [`ric_to_eci_dv`]
/// and adds it to the deputy's velocity. Position and epoch are unchanged.
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero (for RIC frame definition)
/// - `chief` angular momentum must be non-zero
fn apply_impulse(
    deputy: &StateVector,
    chief: &StateVector,
    dv_ric_km_s: &Vector3<f64>,
) -> StateVector {
    let dv_eci = ric_to_eci_dv(dv_ric_km_s, chief);
    StateVector {
        epoch: deputy.epoch,
        position_eci_km: deputy.position_eci_km,
        velocity_eci_km_s: deputy.velocity_eci_km_s + dv_eci,
    }
}

/// Compute aggregate report statistics from per-leg validation points.
///
/// Returns `(max_position_error_km, mean_position_error_km, rms_position_error_km,
/// max_velocity_error_km_s)`. Returns `(0, 0, 0, 0)` for empty input.
fn compute_report_statistics(leg_points: &[Vec<ValidationPoint>]) -> (f64, f64, f64, f64) {
    let mut max_pos = 0.0_f64;
    let mut max_vel = 0.0_f64;
    let mut sum_pos = 0.0_f64;
    let mut sum_pos_sq = 0.0_f64;
    let mut count = 0_u32;

    for points in leg_points {
        for p in points {
            max_pos = max_pos.max(p.position_error_km);
            max_vel = max_vel.max(p.velocity_error_km_s);
            sum_pos += p.position_error_km;
            sum_pos_sq += p.position_error_km * p.position_error_km;
            count += 1;
        }
    }

    if count == 0 {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let mean_pos = sum_pos / f64::from(count);
    let rms_pos = (sum_pos_sq / f64::from(count)).sqrt();
    (max_pos, mean_pos, rms_pos, max_vel)
}

/// Build [`PropagatedState`] entries from chief/deputy ECI pairs for safety analysis.
///
/// For each pair, converts ECI states to Keplerian elements, computes QNS ROE,
/// and computes the RIC relative state. The `chief_mean` field contains osculating
/// elements (acceptable for validation — safety uses ROE/RIC, not the Keplerian
/// field except for `a_km` scaling, where osculating ≈ mean for near-circular orbits).
///
/// # Errors
/// Returns [`ValidationError::Conversion`] if Keplerian element extraction fails.
fn build_nyx_safety_states(
    chief_deputy_pairs: &[(f64, StateVector, StateVector)],
) -> Result<Vec<PropagatedState>, ValidationError> {
    let mut states = Vec::with_capacity(chief_deputy_pairs.len());
    for (elapsed_s, chief, deputy) in chief_deputy_pairs {
        let chief_ke = state_to_keplerian(chief)?;
        let deputy_ke = state_to_keplerian(deputy)?;
        let roe = compute_roe(&chief_ke, &deputy_ke)?;
        let ric = eci_to_ric_relative(chief, deputy);
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

/// Validate a waypoint mission against nyx full-physics propagation.
///
/// Propagates chief and deputy through each mission leg using nyx with full
/// force models (J2 harmonics, drag, SRP, Sun/Moon third-body), applies
/// impulsive Δv maneuvers at burn epochs, and compares the resulting RIC
/// trajectory against the analytical trajectory from the mission planner.
///
/// # Algorithm
/// 1. For each leg: propagate chief through nyx, apply departure Δv to deputy,
///    propagate deputy through nyx, sample and compare RIC states.
/// 2. At leg boundaries: apply arrival Δv, advance to next leg.
/// 3. Build safety states from accumulated chief/deputy pairs.
/// 4. Compute aggregate statistics and return report.
///
/// # Arguments
/// * `mission` — Analytical waypoint mission (from `plan_waypoint_mission`)
/// * `chief_initial` — Chief ECI state at mission start
/// * `deputy_initial` — Deputy ECI state at mission start
/// * `samples_per_leg` — Number of intermediate samples per leg (0 = final only)
/// * `chief_config` — Chief spacecraft properties (mass, drag, SRP)
/// * `deputy_config` — Deputy spacecraft properties
/// * `almanac` — Full-physics ANISE almanac (from `load_full_almanac`)
///
/// # Errors
/// Returns [`ValidationError`] if the mission has no legs, almanac frame lookup
/// fails, dynamics setup fails, propagation fails, or safety analysis fails.
#[allow(clippy::similar_names)]
pub fn validate_mission_nyx(
    mission: &WaypointMission,
    chief_initial: &StateVector,
    deputy_initial: &StateVector,
    samples_per_leg: u32,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<ValidationReport, ValidationError> {
    if mission.legs.is_empty() {
        return Err(ValidationError::EmptyTrajectory);
    }

    let mut chief_state = chief_initial.clone();
    let mut deputy_state = deputy_initial.clone();
    let mut cumulative_time = 0.0_f64;
    let mut leg_points = Vec::with_capacity(mission.legs.len());
    let mut safety_pairs: Vec<(f64, StateVector, StateVector)> = Vec::new();

    for leg in &mission.legs {
        let tof = leg.tof_s;

        // Propagate chief for this leg
        let chief_dynamics = build_full_physics_dynamics(almanac)?;
        let chief_results = nyx_propagate_segment(
            &chief_state,
            tof,
            samples_per_leg,
            chief_config,
            chief_dynamics,
            almanac,
        )?;

        // Apply departure Δv to deputy
        let deputy_post_burn =
            apply_impulse(&deputy_state, &chief_state, &leg.departure_maneuver.dv_ric_km_s);

        // Propagate deputy for this leg
        let deputy_dynamics = build_full_physics_dynamics(almanac)?;
        let deputy_results = nyx_propagate_segment(
            &deputy_post_burn,
            tof,
            samples_per_leg,
            deputy_config,
            deputy_dynamics,
            almanac,
        )?;

        // Build comparison points for this leg
        let mut points = Vec::with_capacity(chief_results.len());
        for (chief_sample, deputy_sample) in chief_results.iter().zip(deputy_results.iter()) {
            let numerical_ric = eci_to_ric_relative(&chief_sample.1, &deputy_sample.1);
            let elapsed = cumulative_time + chief_sample.0;
            let analytical_ric = find_closest_analytical_ric(&leg.trajectory, chief_sample.0);

            let pos_err =
                (numerical_ric.position_ric_km - analytical_ric.position_ric_km).norm();
            let vel_err =
                (numerical_ric.velocity_ric_km_s - analytical_ric.velocity_ric_km_s).norm();

            safety_pairs.push((elapsed, chief_sample.1.clone(), deputy_sample.1.clone()));
            points.push(ValidationPoint {
                elapsed_s: elapsed,
                analytical_ric,
                numerical_ric,
                position_error_km: pos_err,
                velocity_error_km_s: vel_err,
            });
        }
        leg_points.push(points);

        // Advance states for next leg
        chief_state = chief_results
            .last()
            .ok_or(ValidationError::EmptyTrajectory)?
            .1
            .clone();
        let deputy_coast_end = deputy_results
            .last()
            .ok_or(ValidationError::EmptyTrajectory)?
            .1
            .clone();
        deputy_state =
            apply_impulse(&deputy_coast_end, &chief_state, &leg.arrival_maneuver.dv_ric_km_s);
        cumulative_time += tof;
    }

    // Compute safety from nyx trajectory
    let nyx_safety_states = build_nyx_safety_states(&safety_pairs)?;
    let numerical_safety = analyze_trajectory_safety(&nyx_safety_states)?;

    // Compute statistics
    let (max_pos, mean_pos, rms_pos, max_vel) = compute_report_statistics(&leg_points);

    Ok(ValidationReport {
        leg_points,
        max_position_error_km: max_pos,
        mean_position_error_km: mean_pos,
        rms_position_error_km: rms_pos,
        max_velocity_error_km_s: max_vel,
        analytical_safety: mission.safety,
        numerical_safety,
        chief_config: *chief_config,
        deputy_config: *deputy_config,
    })
}

#[cfg(test)]
mod tests {
    use nyx_space::md::prelude::{OrbitalDynamics, SpacecraftDynamics};

    use crate::elements::conversions::{keplerian_to_state, state_to_keplerian};
    use crate::elements::roe::compute_roe;
    use crate::mission::lambert::LambertConfig;
    use crate::mission::planning::{classify_separation, plan_mission};
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{
        iss_like_elements, leo_400km_elements, leo_800km_target_elements, test_drag_config,
        test_epoch,
    };
    use crate::types::{
        KeplerianElements, MissionPhase, PerchGeometry, ProximityConfig, RICState,
        SpacecraftConfig, ValidationPoint,
    };

    // =========================================================================
    // RK4 J2 Numerical Integrator (test-only, independent truth source)
    // =========================================================================

    use nalgebra::Vector3;
    use crate::constants::{J2, MU_EARTH, R_EARTH};
    use crate::types::StateVector;

    /// ECI acceleration: two-body + J2 zonal harmonic.
    #[allow(clippy::doc_markdown)]
    fn j2_acceleration(pos: &Vector3<f64>) -> Vector3<f64> {
        let r2 = pos.dot(pos);
        let r = r2.sqrt();
        let r3 = r * r2;
        let r5 = r3 * r2;
        let z2 = pos.z * pos.z;
        let z2_over_r2 = z2 / r2;

        // Two-body
        let a_2body = -MU_EARTH / r3 * pos;

        // J2 perturbation
        let j2_coeff = -1.5 * J2 * MU_EARTH * R_EARTH * R_EARTH / r5;
        let a_j2 = Vector3::new(
            j2_coeff * pos.x * (1.0 - 5.0 * z2_over_r2),
            j2_coeff * pos.y * (1.0 - 5.0 * z2_over_r2),
            j2_coeff * pos.z * (3.0 - 5.0 * z2_over_r2),
        );

        a_2body + a_j2
    }

    /// Propagate an ECI state forward using RK4 with J2 gravity.
    ///
    /// Fixed time step `dt` (seconds). State = [pos; vel] in ECI.
    /// This is a test-only utility providing an independent numerical truth source
    /// with no shared code paths with the analytical STM.
    fn rk4_j2_propagate(sv: &StateVector, duration_s: f64, dt: f64) -> StateVector {
        debug_assert!(duration_s >= 0.0 && dt > 0.0, "duration and dt must be positive");
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_steps = (duration_s / dt).ceil() as u32;
        let actual_dt = duration_s / f64::from(n_steps);

        let mut pos = sv.position_eci_km;
        let mut vel = sv.velocity_eci_km_s;

        for _ in 0..n_steps {
            // RK4 for d(pos)/dt = vel, d(vel)/dt = accel(pos)
            let k1v = j2_acceleration(&pos);
            let k1r = vel;

            let pos2 = pos + 0.5 * actual_dt * k1r;
            let vel2 = vel + 0.5 * actual_dt * k1v;
            let k2v = j2_acceleration(&pos2);
            let k2r = vel2;

            let pos3 = pos + 0.5 * actual_dt * k2r;
            let vel3 = vel + 0.5 * actual_dt * k2v;
            let k3v = j2_acceleration(&pos3);
            let k3r = vel3;

            let pos4 = pos + actual_dt * k3r;
            let vel4 = vel + actual_dt * k3v;
            let k4v = j2_acceleration(&pos4);
            let k4r = vel4;

            pos += actual_dt / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
            vel += actual_dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        }

        StateVector {
            epoch: sv.epoch + hifitime::Duration::from_seconds(duration_s),
            position_eci_km: pos,
            velocity_eci_km_s: vel,
        }
    }

    /// Self-validate the RK4 J2 integrator: specific orbital energy should be
    /// approximately conserved (slowly varying due to J2's non-conservative-like effects
    /// on osculating elements, but not blowing up).
    #[test]
    fn rk4_j2_integrator_self_validation() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

        let period = chief_ke.period();
        let sv_1orbit = rk4_j2_propagate(&sv, period, 10.0);

        // Specific energy: E = v²/2 - μ/r
        let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv.position_eci_km.norm();
        let energy_1 = sv_1orbit.velocity_eci_km_s.norm_squared() / 2.0
            - MU_EARTH / sv_1orbit.position_eci_km.norm();

        // J2 causes osculating energy to oscillate (short-period terms) but not drift secularly.
        // Over 1 orbit, relative change should be < 1e-5 (J2 oscillation amplitude).
        let rel_energy_change = ((energy_1 - energy_0) / energy_0).abs();
        assert!(
            rel_energy_change < 1e-5,
            "Energy change over 1 orbit = {rel_energy_change:.2e} (should be < 1e-5)"
        );

        // Position magnitude should remain roughly the same (bound orbit)
        let r0 = sv.position_eci_km.norm();
        let r1 = sv_1orbit.position_eci_km.norm();
        assert!(
            (r1 - r0).abs() / r0 < 0.01,
            "Radius changed by {:.4}% over 1 orbit — integrator may be unstable",
            (r1 - r0).abs() / r0 * 100.0
        );
    }

    /// End-to-end mission pipeline: far-field classification → Lambert transfer →
    /// perch orbit → proximity propagation.
    #[test]
    fn full_mission_scenario() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 200.0,
            e: 0.005,
            i_rad: chief_ke.i_rad + 0.05,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        // Phase classification
        let phase = classify_separation(&chief, &deputy, &config).unwrap();
        assert!(
            matches!(phase, MissionPhase::FarField { .. }),
            "200 km + inclination offset should be FarField, got {phase:?}"
        );

        // Full mission plan
        let perch = PerchGeometry::VBar {
            along_track_km: 5.0,
        };

        let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0, &LambertConfig::default())
            .expect("mission plan should succeed");

        // Lambert transfer assertions
        let transfer = plan.transfer.as_ref().expect("should have Lambert transfer");
        assert!(
            transfer.total_dv_km_s > 0.1 && transfer.total_dv_km_s < 10.0,
            "total Δv = {} km/s unreasonable",
            transfer.total_dv_km_s
        );
        assert!(
            transfer.departure_dv_eci_km_s.norm() > 0.0,
            "departure Δv should be nonzero"
        );
        assert!(
            transfer.arrival_dv_eci_km_s.norm() > 0.0,
            "arrival Δv should be nonzero"
        );

        // Perch ROE assertions
        assert!(
            plan.perch_roe.dlambda.abs() > 1e-6,
            "V-bar perch should have nonzero dlambda"
        );
        assert!(
            plan.perch_roe.da.abs() < 1e-10,
            "V-bar perch da should be near-zero"
        );
        assert!(
            plan.perch_roe.dex.abs() < 1e-10,
            "V-bar perch dex should be near-zero"
        );
        assert!(
            plan.perch_roe.dey.abs() < 1e-10,
            "V-bar perch dey should be near-zero"
        );
    }

    /// Verify DMF drag model self-consistency: zero-ROE propagation with drag
    /// should produce drift proportional to `da_dot`, with quadratic along-track growth.
    #[test]
    fn drag_stm_self_consistency() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let zero_roe = crate::types::QuasiNonsingularROE::default();
        let drag_config = test_drag_config();
        let period = std::f64::consts::TAU / chief.mean_motion();

        // J2-only propagator (zero ROE should stay near zero)
        let j2_prop = PropagationModel::J2Stm;
        let j2_5 = j2_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period).unwrap();
        assert!(
            j2_5.roe.da.abs() < 1e-12,
            "J2-only da from zero ROE should stay near zero, got {}",
            j2_5.roe.da
        );

        // J2+drag propagator
        let drag_prop = PropagationModel::J2DragStm {
            drag: drag_config,
        };
        let drag_5 = drag_prop.propagate(&zero_roe, &chief, epoch, 5.0 * period).unwrap();
        let drag_10 = drag_prop.propagate(&zero_roe, &chief, epoch, 10.0 * period).unwrap();

        // da drift should be approximately da_dot * t
        let t5 = 5.0 * period;
        let t10 = 10.0 * period;
        let expected_da_5 = drag_config.da_dot * t5;
        let expected_da_10 = drag_config.da_dot * t10;
        let rel_err_5 = ((drag_5.roe.da - expected_da_5) / expected_da_5).abs();
        let rel_err_10 = ((drag_10.roe.da - expected_da_10) / expected_da_10).abs();
        assert!(
            rel_err_5 < 0.1,
            "da drift at 5 orbits: relative error {rel_err_5:.4} > 10%"
        );
        assert!(
            rel_err_10 < 0.1,
            "da drift at 10 orbits: relative error {rel_err_10:.4} > 10%"
        );

        // Along-track (dlambda) grows quadratically: ratio at 10/5 orbits ≈ 4
        let ratio = drag_10.roe.dlambda / drag_5.roe.dlambda;
        assert!(
            (ratio - 4.0).abs() < 1.0,
            "dlambda(10)/dlambda(5) = {ratio:.2}, expected ≈ 4.0"
        );
    }

    /// Verify J2 secular rates produce physically reasonable effects:
    /// constant da, RAAN regression, and mean longitude drift.
    #[test]
    fn j2_effect_physical_reasonableness() {
        let chief = iss_like_elements();
        let epoch = test_epoch();

        // Deputy with 1 km SMA offset and 0.001 rad inclination offset
        let mut deputy = chief;
        deputy.a_km += 1.0;
        deputy.i_rad += 0.001;
        let roe_0 = compute_roe(&chief, &deputy).unwrap();
        let period = std::f64::consts::TAU / chief.mean_motion();

        let prop = PropagationModel::J2Stm;
        let state_10 = prop.propagate(&roe_0, &chief, epoch, 10.0 * period).unwrap();

        // da should remain constant under J2 (no secular da drift)
        let da_change = (state_10.roe.da - roe_0.da).abs();
        assert!(
            da_change < 1e-8,
            "da should be constant under J2, changed by {da_change}"
        );

        // diy should drift due to differential RAAN regression
        // ISS RAAN regression rate is ~5 deg/day ≈ 8.7e-5 rad/s × orbital period
        // With small inclination offset, differential RAAN produces diy drift
        let diy_drift = (state_10.roe.diy - roe_0.diy).abs();
        assert!(
            diy_drift > 1e-10,
            "diy should drift under J2 differential RAAN, got {diy_drift}"
        );
        // Physically: 10 orbits ≈ 15 hours for ISS, RAAN drift ~3 deg
        // differential diy should be order 1e-5 to 1e-3
        assert!(
            diy_drift < 0.1,
            "diy drift {diy_drift} seems too large for 10 ISS orbits"
        );

        // dlambda should drift from J2 mean motion coupling with da offset
        let dlambda_drift = (state_10.roe.dlambda - roe_0.dlambda).abs();
        assert!(
            dlambda_drift > 1e-8,
            "dlambda should drift from da + J2, got {dlambda_drift}"
        );
    }

    /// Verify Lambert solution by propagating departure state with nyx two-body
    /// and comparing against expected arrival position.
    #[test]
    fn lambert_two_body_verification() {
        let epoch = test_epoch();
        let tof = 2400.0;

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + hifitime::Duration::from_seconds(tof),
        ).unwrap();

        // Solve Lambert
        let transfer =
            crate::mission::lambert::solve_lambert(&dep, &arr).expect("Lambert should converge");

        // Propagate departure state (with Lambert velocity) using nyx two-body
        let almanac = super::load_default_almanac();
        let dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
        let results = super::nyx_propagate_segment(
            &transfer.departure_state, tof, 0,
            &SpacecraftConfig::default(), dynamics, &almanac,
        ).expect("nyx propagation should succeed");
        let propagated = results.last().unwrap().1.clone();

        // Position at arrival should match Lambert's arrival position
        let pos_err = (propagated.position_eci_km - arr.position_eci_km).norm();
        assert!(
            pos_err < 1.0,
            "position error = {pos_err:.4} km (expected < 1.0 km)"
        );

        let vel_err = (propagated.velocity_eci_km_s - transfer.arrival_state.velocity_eci_km_s).norm();
        assert!(
            vel_err < 0.01,
            "velocity error = {vel_err:.6} km/s (expected < 0.01 km/s)"
        );
    }

    /// Verify non-coplanar Lambert solution against nyx two-body propagation.
    #[test]
    fn lambert_non_coplanar_verification() {
        let epoch = test_epoch();
        let tof = 3600.0;

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr_ke = KeplerianElements {
            a_km: 6378.137 + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 10.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + hifitime::Duration::from_seconds(tof)).unwrap();

        let transfer =
            crate::mission::lambert::solve_lambert(&dep, &arr).expect("Lambert should converge");

        let almanac = super::load_default_almanac();
        let dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
        let results = super::nyx_propagate_segment(
            &transfer.departure_state, tof, 0,
            &SpacecraftConfig::default(), dynamics, &almanac,
        ).expect("nyx propagation should succeed");
        let propagated = results.last().unwrap().1.clone();

        let pos_err = (propagated.position_eci_km - arr.position_eci_km).norm();
        assert!(
            pos_err < 1.0,
            "position error = {pos_err:.4} km (expected < 1.0 km)"
        );

        let vel_err = (propagated.velocity_eci_km_s - transfer.arrival_state.velocity_eci_km_s).norm();
        assert!(
            vel_err < 0.01,
            "velocity error = {vel_err:.6} km/s (expected < 0.01 km/s)"
        );
    }

    // =========================================================================
    // Phase 2.1–2.3: J2 STM vs. RK4 J2 Numerical Integrator Comparisons
    // =========================================================================

    /// Compare STM-propagated ROE against RK4 J2 numerical integration.
    ///
    /// Returns physical errors `a * |ROE_stm - ROE_numerical|` in meters per element.
    fn compare_stm_vs_rk4(
        chief: &KeplerianElements,
        roe: &crate::types::QuasiNonsingularROE,
        n_orbits: f64,
    ) -> [f64; 6] {
        use crate::test_helpers::deputy_from_roe;
        use crate::propagation::stm::propagate_roe_stm;

        let epoch = test_epoch();
        let deputy = deputy_from_roe(chief, roe);
        let period = chief.period();
        let duration = n_orbits * period;

        // Convert to ECI
        let chief_sv = keplerian_to_state(chief, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy, epoch).unwrap();

        // RK4 J2 numerical propagation (dt=10s for all cases)
        let chief_final = rk4_j2_propagate(&chief_sv, duration, 10.0);
        let deputy_final = rk4_j2_propagate(&deputy_sv, duration, 10.0);

        // Convert back to Keplerian and compute numerical ROE
        let chief_ke_final = state_to_keplerian(&chief_final).unwrap();
        let deputy_ke_final = state_to_keplerian(&deputy_final).unwrap();
        let numerical_roe = compute_roe(&chief_ke_final, &deputy_ke_final).unwrap();

        // STM propagation
        let (stm_roe, _) = propagate_roe_stm(roe, chief, duration).unwrap();

        // Compute errors in meters: a * |ROE_stm - ROE_numerical|
        let a = chief.a_km * 1000.0; // convert to meters
        [
            a * (stm_roe.da - numerical_roe.da).abs(),
            a * (crate::elements::roe::wrap_angle(stm_roe.dlambda - numerical_roe.dlambda)).abs(),
            a * (stm_roe.dex - numerical_roe.dex).abs(),
            a * (stm_roe.dey - numerical_roe.dey).abs(),
            a * (stm_roe.dix - numerical_roe.dix).abs(),
            a * (stm_roe.diy - numerical_roe.diy).abs(),
        ]
    }

    /// Koenig Table 4 Case 1: J2 STM accuracy over 10 orbits vs. RK4 J2 numerical integrator.
    ///
    /// Chief: a=6812 km, e=0.005, i=30°. ROE: 200m physical separation in each component.
    /// Koenig Table 4 reports QNS J2 STM errors vs. a 20×20 gravity model:
    ///   aδa=38.5m, aδλ=1808.8m, aδex'=13.5m, aδey'=11.3m, aδix=0.9m, aδiy=2.5m.
    /// Our comparison is J2 STM vs. J2-only RK4, so errors should be dominated by
    /// linearization rather than unmodeled harmonics, yielding much smaller values.
    /// We use 10× Table 4 as generous bounds.
    #[test]
    fn koenig_table4_j2_stm_accuracy_case1() {
        use crate::test_helpers::{koenig_table2_case1, koenig_table2_case1_roe};

        let chief = koenig_table2_case1();
        let roe = koenig_table2_case1_roe();
        let errors = compare_stm_vs_rk4(&chief, &roe, 10.0);

        // Bounds: 10× Koenig Table 4 QNS J2 values (Harris-Priester)
        assert!(errors[0] < 385.0,   "aδa  = {:.1}m (bound 385m)", errors[0]);
        assert!(errors[1] < 18088.0, "aδλ  = {:.1}m (bound 18088m)", errors[1]);
        assert!(errors[2] < 135.0,   "aδex = {:.1}m (bound 135m)", errors[2]);
        assert!(errors[3] < 113.0,   "aδey = {:.1}m (bound 113m)", errors[3]);
        assert!(errors[4] < 9.0,     "aδix = {:.1}m (bound 9m)", errors[4]);
        assert!(errors[5] < 25.0,    "aδiy = {:.1}m (bound 25m)", errors[5]);

        // Print actual errors for diagnostic purposes
        eprintln!("Case 1 STM vs RK4 J2 errors (m): da={:.2}, dλ={:.2}, dex={:.2}, dey={:.2}, dix={:.2}, diy={:.2}",
            errors[0], errors[1], errors[2], errors[3], errors[4], errors[5]);
    }

    /// Koenig Table 2 Cases 2-3: verify RK4 J2 integrator correctly propagates
    /// eccentric orbits (energy conservation, orbit stability).
    ///
    /// Direct ROE comparison between the mean-element STM and osculating-element
    /// numerical integration is not meaningful for eccentric orbits (e=0.2, 0.5)
    /// without osculating-to-mean averaging (Koenig Fig. 4). Instead, we verify:
    /// 1. The RK4 integrator maintains energy conservation for eccentric orbits
    /// 2. The orbit remains bound after 10 orbits
    /// 3. The STM preserves δa and eccentricity vector magnitude (structural)
    #[test]
    fn rk4_j2_eccentric_orbit_stability() {
        use crate::test_helpers::{koenig_table2_case2, koenig_table2_case3};

        for (label, chief) in &[
            ("Case 2 (e=0.2)", koenig_table2_case2()),
            ("Case 3 (e=0.5)", koenig_table2_case3()),
        ] {
            let epoch = test_epoch();
            let sv = keplerian_to_state(chief, epoch).unwrap();
            let duration = 10.0 * chief.period();

            let sv_final = rk4_j2_propagate(&sv, duration, 10.0);

            // Energy should be approximately conserved
            let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
                - MU_EARTH / sv.position_eci_km.norm();
            let energy_f = sv_final.velocity_eci_km_s.norm_squared() / 2.0
                - MU_EARTH / sv_final.position_eci_km.norm();
            let rel_energy = ((energy_f - energy_0) / energy_0).abs();
            assert!(
                rel_energy < 1e-5,
                "{label}: energy change = {rel_energy:.2e} over 10 orbits (should be < 1e-5)"
            );

            // Orbit should remain bound (can convert back to Keplerian)
            let ke_final = state_to_keplerian(&sv_final).unwrap();
            assert!(
                ke_final.e < 1.0 && ke_final.a_km > 0.0,
                "{label}: orbit became unbound (a={}, e={})", ke_final.a_km, ke_final.e
            );

            // SMA should be approximately conserved (J2 doesn't cause secular SMA drift)
            let rel_sma = ((ke_final.a_km - chief.a_km) / chief.a_km).abs();
            assert!(
                rel_sma < 0.001,
                "{label}: SMA changed by {:.4}% over 10 orbits", rel_sma * 100.0
            );
        }
    }

    /// Koenig Table 2 Cases 2-3: verify STM structural properties for eccentric orbits.
    ///
    /// The STM preserves δa exactly and rotates the eccentricity vector by ω̇τ.
    /// Over 10 orbits, verify:
    /// 1. δa is exactly conserved (row 0 of STM)
    /// 2. δix is exactly conserved (row 4 of STM)
    /// 3. Eccentricity vector magnitude |δe| is approximately conserved (rotation)
    /// 4. Propagation completes without error
    #[test]
    fn stm_structural_properties_eccentric() {
        use crate::test_helpers::{
            koenig_table2_case2, koenig_table2_case2_roe,
            koenig_table2_case3, koenig_table2_case3_roe,
        };
        use crate::propagation::stm::propagate_roe_stm;

        let cases: [(&str, KeplerianElements, crate::types::QuasiNonsingularROE); 2] = [
            ("Case 2 (e=0.2)", koenig_table2_case2(), koenig_table2_case2_roe()),
            ("Case 3 (e=0.5)", koenig_table2_case3(), koenig_table2_case3_roe()),
        ];

        for (label, chief, roe) in &cases {
            let duration = 10.0 * chief.period();
            let (roe_prop, _) = propagate_roe_stm(roe, chief, duration).unwrap();

            // δa must be exactly conserved
            assert!(
                (roe_prop.da - roe.da).abs() < 1e-8,
                "{label}: δa not conserved: {} → {}", roe.da, roe_prop.da
            );

            // δix must be exactly conserved
            assert!(
                (roe_prop.dix - roe.dix).abs() < 1e-8,
                "{label}: δix not conserved: {} → {}", roe.dix, roe_prop.dix
            );

            // |δe| approximately conserved (rotation, not growth)
            let de_0 = (roe.dex.powi(2) + roe.dey.powi(2)).sqrt();
            let de_f = (roe_prop.dex.powi(2) + roe_prop.dey.powi(2)).sqrt();
            if de_0 > 1e-10 {
                let rel_de = (de_f - de_0).abs() / de_0;
                assert!(
                    rel_de < 0.05,
                    "{label}: |δe| changed by {:.2}% over 10 orbits", rel_de * 100.0
                );
            }
        }
    }

    /// D'Amico ROE→RIC trajectory accuracy: compare analytical ROE→RIC mapping
    /// against numerical ECI→RIC conversion over 1 orbit.
    ///
    /// Chief = D'Amico Table 2.1, ROE = Case 1. Propagate both chief and deputy
    /// numerically with RK4 J2 for 1 orbit, sampling every ~30s. At each sample,
    /// compare "true" RIC (from ECI difference via DCM) against predicted RIC
    /// (from STM-propagated ROE via `roe_to_ric`). RMS position error should be < 50m.
    #[test]
    fn damico_roe_to_ric_trajectory_accuracy() {
        use crate::elements::frames::eci_to_ric_relative;
        use crate::elements::ric::roe_to_ric;
        use crate::propagation::stm::propagate_roe_stm;
        use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief, deputy_from_roe};

        let epoch = test_epoch();
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let deputy = deputy_from_roe(&chief, &roe);

        let chief_sv = keplerian_to_state(&chief, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy, epoch).unwrap();

        let period = chief.period();
        let dt_sample = 30.0; // sample every 30 seconds
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_samples = (period / dt_sample).floor() as u32;

        let mut sum_sq_err = 0.0;
        let mut count = 0;

        for i in 0..=n_samples {
            let t = f64::from(i) * dt_sample;

            // "True" RIC from numerical ECI propagation
            let chief_t = rk4_j2_propagate(&chief_sv, t, 10.0);
            let deputy_t = rk4_j2_propagate(&deputy_sv, t, 10.0);
            let ric_true = eci_to_ric_relative(&chief_t, &deputy_t);

            // Predicted RIC from STM-propagated ROE
            let (roe_t, chief_mean_t) = propagate_roe_stm(&roe, &chief, t).unwrap();
            let ric_pred = roe_to_ric(&roe_t, &chief_mean_t).unwrap();

            let pos_err = (ric_true.position_ric_km - ric_pred.position_ric_km).norm();
            sum_sq_err += pos_err * pos_err;
            count += 1;
        }

        let rms_position_m = (sum_sq_err / f64::from(count)).sqrt() * 1000.0;

        // D'Amico Fig. 2.8 shows <3m for two-body. J2 adds secular drift over
        // 1 orbit plus modeling differences (mean vs osculating). 50m is generous
        // enough to catch bugs without false positives.
        assert!(
            rms_position_m < 50.0,
            "RMS RIC position error = {rms_position_m:.2}m (expected < 50m)"
        );

        eprintln!(
            "ROE→RIC trajectory RMS position error: {rms_position_m:.2}m over {count} samples"
        );
    }

    /// Compare analytical J2 STM propagation against nyx Keplerian (two-body) baseline.
    ///
    /// Since J2 STM includes perturbations that nyx two-body does not,
    /// this test verifies that the J2 corrections are physically reasonable
    /// rather than expecting exact agreement.
    #[test]
    fn j2_stm_vs_nyx_two_body() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;
        deputy_ke.i_rad += 0.001;

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = std::f64::consts::TAU / chief_ke.mean_motion();
        let duration = 10.0 * period;

        // Propagate both with nyx two-body
        let almanac = super::load_default_almanac();
        let chief_dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
        let chief_results = super::nyx_propagate_segment(
            &chief_sv, duration, 0,
            &SpacecraftConfig::default(), chief_dynamics, &almanac,
        ).expect("chief nyx propagation failed");
        let chief_nyx = chief_results.last().unwrap().1.clone();

        let deputy_dynamics = SpacecraftDynamics::new(OrbitalDynamics::two_body());
        let deputy_results = super::nyx_propagate_segment(
            &deputy_sv, duration, 0,
            &SpacecraftConfig::default(), deputy_dynamics, &almanac,
        ).expect("deputy nyx propagation failed");
        let deputy_nyx = deputy_results.last().unwrap().1.clone();

        // Compute ROEs from nyx final states
        let chief_ke_final = state_to_keplerian(&chief_nyx).unwrap();
        let deputy_ke_final = state_to_keplerian(&deputy_nyx).unwrap();
        let nyx_roe = compute_roe(&chief_ke_final, &deputy_ke_final).unwrap();

        // Propagate with J2 STM
        let roe_0 = compute_roe(&chief_ke, &deputy_ke).unwrap();
        let prop = PropagationModel::J2Stm;
        let stm_state = prop.propagate(&roe_0, &chief_ke, epoch, duration).unwrap();

        // da should be constant in both (Keplerian preserves SMA)
        assert!(
            (nyx_roe.da - roe_0.da).abs() < 1e-6,
            "nyx da should be approximately constant: initial={}, final={}",
            roe_0.da,
            nyx_roe.da
        );
        assert!(
            (stm_state.roe.da - roe_0.da).abs() < 1e-6,
            "STM da should be approximately constant: initial={}, final={}",
            roe_0.da,
            stm_state.roe.da
        );

        // The J2-induced diy difference should be physically correct
        // Over 10 ISS orbits (~15 hours), differential RAAN regression
        // for 0.001 rad inclination offset produces diy drift of ~1e-5 to 1e-3 rad
        let stm_diy_drift = (stm_state.roe.diy - roe_0.diy).abs();
        assert!(
            stm_diy_drift > 1e-8 && stm_diy_drift < 0.2,
            "J2 STM diy drift = {stm_diy_drift} should be in [1e-8, 0.2] rad"
        );

        // nyx two-body should have minimal diy drift (no J2)
        let nyx_diy_drift = (nyx_roe.diy - roe_0.diy).abs();
        assert!(
            nyx_diy_drift < stm_diy_drift * 10.0 || nyx_diy_drift < 1e-4,
            "nyx two-body diy drift = {nyx_diy_drift} should be small compared to J2 drift"
        );
    }

    // =========================================================================
    // Phase 3: Full-Physics Dynamics Stack Tests
    // =========================================================================

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn full_almanac_loads() {
        let almanac =
            super::load_full_almanac().expect("MetaAlmanac::latest() should succeed");
        // Verify Earth frame data is available
        let earth = almanac
            .frame_info(anise::constants::frames::IAU_EARTH_FRAME)
            .expect("IAU_EARTH frame should be available");
        assert!(
            earth.mu_km3_s2().unwrap() > 0.0,
            "Earth mu should be positive"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn full_physics_propagate_one_orbit() {
        let almanac =
            super::load_full_almanac().expect("full almanac should load");
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let period = chief_ke.period();

        let dynamics = super::build_full_physics_dynamics(&almanac)
            .expect("full physics dynamics should build");
        let results = super::nyx_propagate_segment(
            &sv,
            period,
            0,
            &SpacecraftConfig::default(),
            dynamics,
            &almanac,
        )
        .expect("full physics propagation should succeed");

        let final_state = &results.last().unwrap().1;
        let r = final_state.position_eci_km.norm();
        let v = final_state.velocity_eci_km_s.norm();

        // ISS-like orbit: ~6800 km altitude, ~7.5 km/s
        assert!(
            r > 6000.0 && r < 7500.0,
            "final radius = {r:.1} km (expected 6000-7500)"
        );
        assert!(
            v > 6.5 && v < 8.5,
            "final velocity = {v:.4} km/s (expected 6.5-8.5)"
        );

        // Energy should be approximately conserved
        let energy_0 = sv.velocity_eci_km_s.norm_squared() / 2.0
            - crate::constants::MU_EARTH / sv.position_eci_km.norm();
        let energy_f = final_state.velocity_eci_km_s.norm_squared() / 2.0
            - crate::constants::MU_EARTH / final_state.position_eci_km.norm();
        let rel_energy = ((energy_f - energy_0) / energy_0).abs();
        // Full physics (drag, SRP) will cause some energy change, but not catastrophic
        assert!(
            rel_energy < 1e-4,
            "energy change = {rel_energy:.2e} (expected < 1e-4)"
        );
    }

    // =========================================================================
    // Phase 4: DMF Rate Extraction Tests
    // =========================================================================

    /// Two spacecraft with different ballistic coefficients should produce
    /// nonzero differential drag rates. Chief: 500 kg / 1.0 m² (default),
    /// Deputy: 200 kg / 2.0 m² (higher B* = Cd·A/m).
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn extract_dmf_rates_nonzero() {
        let almanac = super::load_full_almanac().expect("full almanac should load");
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

        // Deputy on same orbit — differential drag comes from different B*
        let chief_config = SpacecraftConfig::default();
        let deputy_config = SpacecraftConfig {
            dry_mass_kg: 200.0,
            drag_area_m2: 2.0,
            ..SpacecraftConfig::default()
        };

        let drag = super::extract_dmf_rates(
            &sv, &sv, &chief_config, &deputy_config, &almanac,
        )
        .expect("DMF extraction should succeed");

        // da_dot should be nonzero: higher B* deputy decays faster
        assert!(
            drag.da_dot.abs() > 1e-16,
            "da_dot should be nonzero for different B*, got {:.2e}",
            drag.da_dot
        );

        // da_dot should be negative: deputy has higher drag, so relative SMA decreases
        assert!(
            drag.da_dot < 0.0,
            "da_dot should be negative (deputy decays faster), got {:.2e}",
            drag.da_dot
        );

        // Physical reasonableness: for ISS-like orbit, differential da_dot
        // should be order 1e-12 to 1e-9 (dimensionless, per second)
        assert!(
            drag.da_dot.abs() < 1e-6,
            "da_dot = {:.2e} seems unreasonably large",
            drag.da_dot
        );

        eprintln!(
            "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
    }

    /// Identical spacecraft configs should produce zero (or sub-threshold) drag rates.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn extract_dmf_rates_identical() {
        let almanac = super::load_full_almanac().expect("full almanac should load");
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let sv = keplerian_to_state(&chief_ke, epoch).unwrap();

        let config = SpacecraftConfig::default();
        let drag = super::extract_dmf_rates(
            &sv, &sv, &config, &config, &almanac,
        )
        .expect("DMF extraction should succeed");

        // Identical B* → zero differential drag
        assert!(
            drag.da_dot.abs() < f64::EPSILON,
            "da_dot should be zero for identical configs, got {:.2e}",
            drag.da_dot
        );
        assert!(
            drag.dex_dot.abs() < f64::EPSILON,
            "dex_dot should be zero for identical configs, got {:.2e}",
            drag.dex_dot
        );
        assert!(
            drag.dey_dot.abs() < f64::EPSILON,
            "dey_dot should be zero for identical configs, got {:.2e}",
            drag.dey_dot
        );
    }

    // =========================================================================
    // Phase 5: Mission Validation Pipeline Tests
    // =========================================================================

    /// Verify that `apply_impulse` correctly transforms RIC Δv to ECI and applies it.
    ///
    /// Checks: position unchanged, epoch unchanged, velocity changed by
    /// ECI-transformed Δv, Δv magnitude preserved (DCM orthogonality),
    /// zero Δv returns identical state.
    #[test]
    fn validate_impulse_application() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        // Apply known RIC Δv
        let dv_ric = Vector3::new(0.001, -0.002, 0.0005);
        let result = super::apply_impulse(&deputy_sv, &chief_sv, &dv_ric);

        // Position unchanged
        let pos_err = (result.position_eci_km - deputy_sv.position_eci_km).norm();
        assert!(pos_err < 1e-15, "position should be unchanged, error = {pos_err}");

        // Epoch unchanged
        assert_eq!(result.epoch, deputy_sv.epoch, "epoch should be unchanged");

        // Velocity changed by ECI-transformed Δv
        let dv_eci = crate::elements::frames::ric_to_eci_dv(&dv_ric, &chief_sv);
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

        // Zero Δv returns identical state
        let zero_result = super::apply_impulse(&deputy_sv, &chief_sv, &Vector3::zeros());
        let zero_vel_err = (zero_result.velocity_eci_km_s - deputy_sv.velocity_eci_km_s).norm();
        assert!(zero_vel_err < 1e-15, "zero Δv should not change velocity");
    }

    /// Verify `compute_report_statistics` with known error values.
    ///
    /// Three points with position errors [1.0, 3.0, 2.0] km:
    /// max=3.0, mean=2.0, rms=sqrt(14/3)≈2.160, max_vel=0.03.
    /// Also verifies empty input returns zeros.
    #[test]
    fn validation_report_statistics() {
        let make_point = |pos_err: f64, vel_err: f64| ValidationPoint {
            elapsed_s: 0.0,
            analytical_ric: RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::zeros(),
            },
            numerical_ric: RICState {
                position_ric_km: Vector3::zeros(),
                velocity_ric_km_s: Vector3::zeros(),
            },
            position_error_km: pos_err,
            velocity_error_km_s: vel_err,
        };

        let leg = vec![
            make_point(1.0, 0.01),
            make_point(3.0, 0.03),
            make_point(2.0, 0.02),
        ];
        let (max_pos, mean_pos, rms_pos, max_vel) = super::compute_report_statistics(&[leg]);

        assert!(
            (max_pos - 3.0).abs() < 1e-15,
            "max_pos = {max_pos}, expected 3.0"
        );
        assert!(
            (mean_pos - 2.0).abs() < 1e-15,
            "mean_pos = {mean_pos}, expected 2.0"
        );
        // rms = sqrt((1 + 9 + 4) / 3) = sqrt(14/3) ≈ 2.160
        let expected_rms = (14.0_f64 / 3.0).sqrt();
        assert!(
            (rms_pos - expected_rms).abs() < 1e-12,
            "rms_pos = {rms_pos}, expected {expected_rms}"
        );
        assert!(
            (max_vel - 0.03).abs() < 1e-15,
            "max_vel = {max_vel}, expected 0.03"
        );

        // Empty input returns zeros
        let (mp, meanp, rmsp, mv) = super::compute_report_statistics(&[]);
        assert_eq!(mp, 0.0, "empty max_pos");
        assert_eq!(meanp, 0.0, "empty mean_pos");
        assert_eq!(rmsp, 0.0, "empty rms_pos");
        assert_eq!(mv, 0.0, "empty max_vel");
    }

    /// Single V-bar transfer validated against nyx full-physics propagation.
    ///
    /// ISS-like orbit, deputy starts colocated, single waypoint at [0,5,0] RIC km,
    /// TOF = 1 orbital period. Max position error should be < 1.0 km.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_single_leg() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::types::{DepartureState, MissionConfig, QuasiNonsingularROE, Waypoint};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let period = chief_ke.period();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        };

        let departure = DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &[waypoint], &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = super::load_full_almanac().expect("full almanac should load");
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig::SERVICER_500KG;

        let report = super::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            20,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("validation should succeed");

        eprintln!(
            "Single-leg validation: max_pos={:.4} km, mean_pos={:.4} km, rms_pos={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert!(
            report.max_position_error_km < 1.0,
            "max position error = {:.4} km (expected < 1.0)",
            report.max_position_error_km,
        );
        assert_eq!(report.leg_points.len(), 1, "should have 1 leg");
    }

    /// Three-waypoint V-bar mission validated against nyx full-physics propagation.
    ///
    /// ISS-like orbit, 3 V-bar waypoints at [0,2,0], [0,5,0], [0,1,0] km,
    /// each with 0.75-period TOF. Errors should remain bounded across legs.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_three_waypoint() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::types::{DepartureState, MissionConfig, QuasiNonsingularROE, Waypoint};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let period = chief_ke.period();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 2.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = super::load_full_almanac().expect("full almanac should load");
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig::SERVICER_500KG;

        let report = super::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            20,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("validation should succeed");

        eprintln!(
            "Three-waypoint validation: max_pos={:.4} km, mean_pos={:.4} km, rms_pos={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert_eq!(report.leg_points.len(), 3, "should have 3 legs");

        // Errors should be bounded (< 5 km for 3 legs with full physics)
        assert!(
            report.max_position_error_km < 5.0,
            "max position error = {:.4} km (expected < 5.0)",
            report.max_position_error_km,
        );

        // Safety metrics should be populated
        assert!(
            report.numerical_safety.min_distance_3d_km > 0.0,
            "numerical safety 3D distance should be positive"
        );
    }

    // =========================================================================
    // Phase 6: Full-Physics Integration Tests
    // =========================================================================

    /// Position tolerance for a single-leg transfer (~1 orbit).
    /// Unmodeled perturbations (drag, SRP, 3rd-body) contribute ~50m total;
    /// 10× margin gives 0.5 km.
    const FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM: f64 = 0.5;

    /// Position tolerance for a multi-leg mission (~3 legs × 0.75 period each).
    /// ~3× single-leg tolerance plus maneuver state mismatch across legs.
    const FULL_PHYSICS_MULTI_LEG_POS_TOL_KM: f64 = 3.0;

    /// Position tolerance for drag STM vs nyx comparison.
    /// DMF linear fit error + unmodeled SRP/3rd-body over 1 orbit.
    const DRAG_STM_VS_NYX_POS_TOL_KM: f64 = 1.0;

    /// Relative tolerance for R/C separation comparison (analytical vs numerical).
    /// Different sampling density + mean/osculating offset justify 50%.
    const SAFETY_RC_RELATIVE_TOL: f64 = 0.50;

    /// Absolute tolerance for 3D distance comparison (km).
    /// Same effects as R/C; absolute because 3D distance can be small.
    const SAFETY_3D_ABSOLUTE_TOL_KM: f64 = 0.5;

    /// Relative tolerance for e/i vector separation comparison.
    /// ROE-level drift O(1e-5) over 1–3 orbits.
    const SAFETY_EI_RELATIVE_TOL: f64 = 0.50;

    /// Below this threshold (km), R/C separations are operationally "at V-bar"
    /// and relative-error comparison is not meaningful.
    const SAFETY_RC_NEAR_ZERO_KM: f64 = 0.01;

    /// Single-leg transfer with nonzero initial ROE and mixed-axis waypoint,
    /// validated against nyx full-physics propagation.
    ///
    /// Chief: ISS-like orbit. Deputy: ~300m-scale formation (nonzero dex, dey, dix).
    /// Waypoint: [0.5, 3.0, 1.0] RIC km, TOF = 0.8 orbital periods.
    /// Non-integer period avoids CW singularity at nt = 2π (rank-1 Φ_rv).
    /// 50 samples for detailed error characterization.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_full_physics_single_leg() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::types::{DepartureState, MissionConfig, QuasiNonsingularROE, Waypoint};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        // ~300m-scale formation: nonzero dex, dey, dix
        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.3 / a,
            dey: -0.2 / a,
            dix: 0.2 / a,
            diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.5, 3.0, 1.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &[waypoint], &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = super::load_full_almanac().expect("full almanac should load");
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig::SERVICER_500KG;

        let report = super::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            50,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Per-point error logging (every 10th sample)
        for (leg_idx, points) in report.leg_points.iter().enumerate() {
            for (i, p) in points.iter().enumerate() {
                if i % 10 == 0 || i == points.len() - 1 {
                    eprintln!(
                        "  leg {leg_idx} sample {i:>3}: t={:.0}s  pos_err={:.4} km",
                        p.elapsed_s, p.position_error_km,
                    );
                }
            }
        }

        eprintln!(
            "Full-physics single-leg: max={:.4} km, mean={:.4} km, rms={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert_eq!(report.leg_points.len(), 1, "should have 1 leg");
        assert!(
            report.max_position_error_km < FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM,
            "max position error = {:.4} km (expected < {FULL_PHYSICS_SINGLE_LEG_POS_TOL_KM})",
            report.max_position_error_km,
        );
        assert!(
            report.rms_position_error_km <= report.max_position_error_km,
            "RMS ({:.4}) should be <= max ({:.4})",
            report.rms_position_error_km,
            report.max_position_error_km,
        );
    }

    /// Multi-waypoint mission with nonzero initial ROE and mixed-axis waypoints,
    /// validated against nyx full-physics propagation.
    ///
    /// Chief: ISS-like orbit. Deputy: ~300m-scale formation.
    /// 3 waypoints with 0.75-period TOF each, spanning R/I/C axes.
    /// Per-leg error characterization and error-growth logging.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_full_physics_multi_waypoint() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::types::{DepartureState, MissionConfig, QuasiNonsingularROE, Waypoint};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.3 / a,
            dey: -0.2 / a,
            dix: 0.2 / a,
            diy: 0.0,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.5, 3.0, 0.5),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(1.0, -2.0, 1.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 1.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = super::load_full_almanac().expect("full almanac should load");
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig::SERVICER_500KG;

        let report = super::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            50,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Per-leg error characterization
        for (leg_idx, points) in report.leg_points.iter().enumerate() {
            let leg_max = points
                .iter()
                .map(|p| p.position_error_km)
                .fold(0.0_f64, f64::max);
            eprintln!("  leg {leg_idx}: max_pos_err = {leg_max:.4} km ({} samples)", points.len());
        }

        eprintln!(
            "Full-physics multi-waypoint: max={:.4} km, mean={:.4} km, rms={:.4} km",
            report.max_position_error_km,
            report.mean_position_error_km,
            report.rms_position_error_km,
        );

        assert_eq!(report.leg_points.len(), 3, "should have 3 legs");
        assert!(
            report.max_position_error_km < FULL_PHYSICS_MULTI_LEG_POS_TOL_KM,
            "max position error = {:.4} km (expected < {FULL_PHYSICS_MULTI_LEG_POS_TOL_KM})",
            report.max_position_error_km,
        );
    }

    /// End-to-end drag STM validation: extract DMF rates from nyx, plan with
    /// J2+drag STM, validate against nyx full-physics propagation.
    ///
    /// 1. ISS-like orbit, deputy colocated (zero ROE)
    /// 2. Chief: SERVICER_500KG (B*=0.0044), Deputy: 200kg/2m² (B*=0.022, ~5× higher)
    /// 3. Extract DMF rates → DragConfig
    /// 4. Plan with J2DragStm: single V-bar waypoint [0,5,0], 0.8 periods
    ///    (non-integer period avoids CW singularity at nt = 2π)
    /// 5. Plan same with J2Stm for comparison
    /// 6. Validate both against nyx
    /// 7. Assert drag-aware error < tolerance; log improvement ratio
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_drag_stm_vs_nyx_drag() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::types::{DepartureState, MissionConfig, QuasiNonsingularROE, Waypoint};

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone(); // colocated start

        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig {
            dry_mass_kg: 200.0,
            drag_area_m2: 2.0,
            ..SpacecraftConfig::default()
        };

        // Step 1: Extract DMF rates
        let almanac = super::load_full_almanac().expect("full almanac should load");
        let drag = super::extract_dmf_rates(
            &chief_sv, &deputy_sv, &chief_config, &deputy_config, &almanac,
        )
        .expect("DMF extraction should succeed");

        eprintln!(
            "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot,
        );

        // Sanity: da_dot should be nonzero, negative, and physically reasonable
        assert!(
            drag.da_dot.abs() > 1e-16,
            "da_dot should be nonzero, got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot < 0.0,
            "da_dot should be negative (deputy decays faster), got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot.abs() < 1e-6,
            "da_dot = {:.2e} seems unreasonably large",
            drag.da_dot,
        );

        // Step 2: Plan missions with both propagators
        let period = chief_ke.period();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(0.8 * period),
        };

        let departure = DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig::default();

        // J2+drag mission
        let drag_propagator = PropagationModel::J2DragStm { drag };
        let drag_mission =
            plan_waypoint_mission(&departure, &[waypoint.clone()], &config, &drag_propagator)
                .expect("drag mission planning should succeed");

        // J2-only mission (comparison baseline)
        let j2_propagator = PropagationModel::J2Stm;
        let j2_mission =
            plan_waypoint_mission(&departure, &[waypoint], &config, &j2_propagator)
                .expect("J2 mission planning should succeed");

        // Step 3: Validate both against nyx
        let drag_report = super::validate_mission_nyx(
            &drag_mission,
            &chief_sv,
            &deputy_sv,
            50,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("drag validation should succeed");

        let j2_report = super::validate_mission_nyx(
            &j2_mission,
            &chief_sv,
            &deputy_sv,
            50,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("J2 validation should succeed");

        eprintln!(
            "Drag STM vs nyx: max={:.4} km, mean={:.4} km, rms={:.4} km",
            drag_report.max_position_error_km,
            drag_report.mean_position_error_km,
            drag_report.rms_position_error_km,
        );
        eprintln!(
            "J2-only vs nyx:  max={:.4} km, mean={:.4} km, rms={:.4} km",
            j2_report.max_position_error_km,
            j2_report.mean_position_error_km,
            j2_report.rms_position_error_km,
        );

        // Diagnostic: improvement ratio (not hard-asserted — SRP/3rd-body may dominate)
        if j2_report.max_position_error_km > 1e-10 {
            let improvement = j2_report.max_position_error_km / drag_report.max_position_error_km;
            eprintln!("Drag/J2 improvement ratio: {improvement:.2}×");
            if improvement < 1.0 {
                eprintln!(
                    "  Note: drag STM did not improve over J2-only for this scenario. \
                     For short single-orbit transfers with colocated start, differential \
                     drag effect may be negligible vs unmodeled perturbations (SRP, 3rd-body)."
                );
            }
        }

        // Core assertion: drag-aware propagation should match nyx within tolerance
        assert!(
            drag_report.max_position_error_km < DRAG_STM_VS_NYX_POS_TOL_KM,
            "drag STM max error = {:.4} km (expected < {DRAG_STM_VS_NYX_POS_TOL_KM})",
            drag_report.max_position_error_km,
        );
    }

    /// Safety metrics comparison: analytical (ROE-based) vs numerical (nyx) safety.
    ///
    /// Chief: ISS-like. Deputy: formation with perpendicular e/i vectors for
    /// meaningful e/i separation (dex=0.5/a, diy=0.5/a).
    /// 3 waypoints with 0.75-period TOF, safety analysis enabled.
    /// Compares R/C separation, 3D distance, and e/i separation between
    /// analytical and numerical trajectories.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_safety_full_physics() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::types::{
            DepartureState, MissionConfig, QuasiNonsingularROE, SafetyConfig, Waypoint,
        };

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let a = chief_ke.a_km;

        // Perpendicular e/i vectors: dex along ex, diy along iy → meaningful separation
        let formation_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.5 / a,
            dey: 0.0,
            dix: 0.0,
            diy: 0.5 / a,
        };
        let deputy_ke = deputy_from_roe(&chief_ke, &formation_roe);

        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let period = chief_ke.period();
        let tof = 0.75 * period;
        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.5, 5.0, 0.5),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(0.0, 2.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let departure = DepartureState {
            roe: formation_roe,
            chief: chief_ke,
            epoch,
        };
        let config = MissionConfig {
            safety: Some(SafetyConfig::default()),
            ..MissionConfig::default()
        };
        let propagator = PropagationModel::J2Stm;

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("mission planning should succeed");

        let almanac = super::load_full_almanac().expect("full almanac should load");
        let chief_config = SpacecraftConfig::SERVICER_500KG;
        let deputy_config = SpacecraftConfig::SERVICER_500KG;

        let report = super::validate_mission_nyx(
            &mission,
            &chief_sv,
            &deputy_sv,
            50,
            &chief_config,
            &deputy_config,
            &almanac,
        )
        .expect("validation should succeed");

        // Analytical safety must be present (we enabled it)
        let analytical = report
            .analytical_safety
            .expect("analytical safety should be present (SafetyConfig enabled)");
        let numerical = &report.numerical_safety;

        eprintln!("Safety comparison (analytical vs numerical):");
        eprintln!(
            "  R/C separation:  {:.4} km vs {:.4} km",
            analytical.min_rc_separation_km, numerical.min_rc_separation_km,
        );
        eprintln!(
            "  3D distance:     {:.4} km vs {:.4} km",
            analytical.min_distance_3d_km, numerical.min_distance_3d_km,
        );
        eprintln!(
            "  e/i separation:  {:.4} km vs {:.4} km",
            analytical.min_ei_separation_km, numerical.min_ei_separation_km,
        );
        eprintln!(
            "  |δe|:            {:.6} vs {:.6}",
            analytical.de_magnitude, numerical.de_magnitude,
        );
        eprintln!(
            "  |δi|:            {:.6} vs {:.6}",
            analytical.di_magnitude, numerical.di_magnitude,
        );
        eprintln!(
            "  e/i phase angle: {:.4} rad vs {:.4} rad",
            analytical.ei_phase_angle_rad, numerical.ei_phase_angle_rad,
        );

        // R/C separation: relative agreement within 50%
        // When both values are near zero (V-bar configuration), relative comparison
        // is meaningless — skip it.
        let rc_ref = analytical.min_rc_separation_km.max(numerical.min_rc_separation_km);
        if rc_ref > SAFETY_RC_NEAR_ZERO_KM {
            let rc_rel_err =
                (analytical.min_rc_separation_km - numerical.min_rc_separation_km).abs() / rc_ref;
            eprintln!("  R/C relative error: {rc_rel_err:.2}");
            assert!(
                rc_rel_err < SAFETY_RC_RELATIVE_TOL,
                "R/C separation relative error = {rc_rel_err:.2} (expected < {SAFETY_RC_RELATIVE_TOL})",
            );
        } else {
            eprintln!(
                "  R/C near-zero ({rc_ref:.4} km < {SAFETY_RC_NEAR_ZERO_KM}): \
                 skipping relative comparison"
            );
        }

        // 3D distance: absolute agreement within 0.5 km
        let dist_3d_err =
            (analytical.min_distance_3d_km - numerical.min_distance_3d_km).abs();
        eprintln!("  3D distance absolute error: {dist_3d_err:.4} km");
        assert!(
            dist_3d_err < SAFETY_3D_ABSOLUTE_TOL_KM,
            "3D distance error = {dist_3d_err:.4} km (expected < {SAFETY_3D_ABSOLUTE_TOL_KM})",
        );

        // e/i separation: relative agreement within 50%
        let ei_ref = analytical.min_ei_separation_km.max(numerical.min_ei_separation_km);
        if ei_ref > 1e-6 {
            let ei_rel_err =
                (analytical.min_ei_separation_km - numerical.min_ei_separation_km).abs() / ei_ref;
            eprintln!("  e/i relative error: {ei_rel_err:.2}");
            assert!(
                ei_rel_err < SAFETY_EI_RELATIVE_TOL,
                "e/i separation relative error = {ei_rel_err:.2} (expected < {SAFETY_EI_RELATIVE_TOL})",
            );
        }
    }
}

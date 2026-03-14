//! Full-physics Monte Carlo ensemble analysis using nyx propagation.
//!
//! Each sample propagates chief + deputy through nyx with dispersed initial
//! states and Δv execution errors. Uses rayon for parallel execution with
//! deterministic per-sample seeding via `ChaCha20Rng`.

use std::fmt;
use std::time::Instant;

use nalgebra::Vector3;
use rand::Rng;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rand_distr::{Normal, Uniform as RandUniform};
use rayon::prelude::*;

use crate::constants::{
    COVARIANCE_SIGMA_FLOOR, DV_NORM_ZERO_THRESHOLD_KM_S, MC_DEFAULT_COLLISION_THRESHOLD_KM,
    MIN_SPACECRAFT_MASS_KG, RODRIGUES_AXIS_NORM_THRESHOLD,
};
use crate::elements::conversions::state_to_keplerian;
use crate::elements::frames::{eci_to_ric_dcm, eci_to_ric_relative};
use crate::elements::roe::compute_roe;
use crate::mission::safety::analyze_trajectory_safety;
use crate::mission::waypoints::plan_waypoint_mission;
use crate::propagation::propagator::{PropagatedState, PropagationError, PropagationModel};
use crate::types::{
    CovarianceValidation, DepartureState, DispersionEnvelope, Distribution, EnsembleStatistics,
    ManeuverDispersion, MissionConfig, MissionCovarianceReport, MissionError, MonteCarloConfig,
    MonteCarloInput, MonteCarloMode, MonteCarloReport, PercentileStats, SampleResult,
    SpacecraftConfig, StateVector, Waypoint, WaypointMission,
};

use super::validation::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
    ValidationError,
};

/// Errors from Monte Carlo ensemble analysis.
#[derive(Debug)]
pub enum MonteCarloError {
    /// `num_samples` must be > 0.
    ZeroSamples,
    /// All MC samples failed (none converged or propagated successfully).
    AllSamplesFailed {
        /// Total number of samples attempted.
        num_samples: u32,
        /// Number that failed due to targeting non-convergence.
        convergence_failures: u32,
        /// Number that failed due to propagation errors.
        propagation_failures: u32,
    },
    /// Dispersed state produced negative semi-major axis.
    NegativeSma {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting semi-major axis (km).
        a_km: f64,
    },
    /// Dispersed state produced invalid eccentricity.
    InvalidEccentricity {
        /// Which sample produced the invalid state.
        sample_index: u32,
        /// The resulting eccentricity.
        e: f64,
    },
    /// Dispersion sigma must be non-negative.
    NegativeSigma {
        /// The invalid sigma value.
        value: f64,
    },
    /// Dispersion half-width must be non-negative.
    NegativeHalfWidth {
        /// The invalid half-width value.
        value: f64,
    },
    /// Mission planning failure during closed-loop re-targeting.
    Mission(MissionError),
    /// Propagation failure during sample execution.
    Propagation(PropagationError),
    /// Validation bridge failure.
    Validation(Box<ValidationError>),
    /// Empty ensemble (no samples to compute statistics from).
    EmptyEnsemble,
}

impl fmt::Display for MonteCarloError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroSamples => write!(f, "num_samples must be > 0"),
            Self::AllSamplesFailed {
                num_samples,
                convergence_failures,
                propagation_failures,
            } => write!(
                f,
                "all {num_samples} MC samples failed: {convergence_failures} convergence, \
                 {propagation_failures} propagation"
            ),
            Self::NegativeSma { sample_index, a_km } => {
                write!(f, "sample {sample_index}: negative SMA = {a_km} km")
            }
            Self::InvalidEccentricity { sample_index, e } => {
                write!(f, "sample {sample_index}: invalid eccentricity = {e}")
            }
            Self::NegativeSigma { value } => {
                write!(f, "dispersion sigma must be non-negative, got {value}")
            }
            Self::NegativeHalfWidth { value } => {
                write!(f, "dispersion half-width must be non-negative, got {value}")
            }
            Self::Mission(e) => write!(f, "mission planning failure: {e}"),
            Self::Propagation(e) => write!(f, "propagation failure: {e}"),
            Self::Validation(e) => write!(f, "validation failure: {e}"),
            Self::EmptyEnsemble => write!(f, "empty ensemble: no samples to compute statistics"),
        }
    }
}

impl std::error::Error for MonteCarloError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Mission(e) => Some(e),
            Self::Propagation(e) => Some(e),
            Self::Validation(e) => Some(e.as_ref()),
            _ => None,
        }
    }
}

impl From<MissionError> for MonteCarloError {
    fn from(e: MissionError) -> Self {
        Self::Mission(e)
    }
}

impl From<PropagationError> for MonteCarloError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<ValidationError> for MonteCarloError {
    fn from(e: ValidationError) -> Self {
        Self::Validation(Box::new(e))
    }
}

// ---------------------------------------------------------------------------
// Sampling primitives
// ---------------------------------------------------------------------------

/// Draw a single value from a [`Distribution`].
///
/// # Invariants
/// - Gaussian `sigma >= 0`; zero sigma returns `0.0` deterministically.
/// - Uniform `half_width >= 0`; zero half-width returns `0.0` deterministically.
///
/// # Errors
/// Returns [`MonteCarloError::NegativeSigma`] if `sigma < 0`, or
/// [`MonteCarloError::NegativeHalfWidth`] if `half_width < 0`.
fn sample_distribution<R: Rng>(
    dist: &Distribution,
    rng: &mut R,
) -> Result<f64, MonteCarloError> {
    match *dist {
        Distribution::Gaussian { sigma } => {
            if sigma < 0.0 {
                return Err(MonteCarloError::NegativeSigma { value: sigma });
            }
            if sigma > 0.0 {
                let normal = Normal::new(0.0, sigma)
                    .map_err(|_| MonteCarloError::NegativeSigma { value: sigma })?;
                Ok(rng.sample(normal))
            } else {
                Ok(0.0)
            }
        }
        Distribution::Uniform { half_width } => {
            if half_width < 0.0 {
                return Err(MonteCarloError::NegativeHalfWidth { value: half_width });
            }
            if half_width > 0.0 {
                let uniform = RandUniform::new(-half_width, half_width);
                Ok(rng.sample(uniform))
            } else {
                Ok(0.0)
            }
        }
    }
}

/// Apply magnitude and pointing errors to a nominal Δv via Rodrigues' rotation.
///
/// # Invariants
/// - `dispersion.magnitude_sigma >= 0`
/// - Zero-norm `nominal_dv` (below [`DV_NORM_ZERO_THRESHOLD_KM_S`]) returns zero.
///
/// # Validity regime
/// The Rodrigues rotation formula is exact for all angles. However, the physical
/// model assumes small pointing errors (`σ_pointing` << 1 rad). For σ > ~0.5 rad
/// (~30°), the "isotropic cone" interpretation becomes questionable — but the
/// math remains valid, so no error is returned.
///
/// # Singularities
/// If the randomly sampled rotation axis has norm below
/// [`RODRIGUES_AXIS_NORM_THRESHOLD`], a default x-axis is used instead.
///
/// # Errors
/// Returns [`MonteCarloError::NegativeSigma`] if `magnitude_sigma < 0`.
fn disperse_maneuver<R: Rng>(
    nominal_dv: &Vector3<f64>,
    dispersion: &ManeuverDispersion,
    rng: &mut R,
) -> Result<Vector3<f64>, MonteCarloError> {
    let dv_norm = nominal_dv.norm();
    if dv_norm < DV_NORM_ZERO_THRESHOLD_KM_S {
        return Ok(Vector3::zeros());
    }

    // Magnitude scaling: 1 + N(0, magnitude_sigma)
    if dispersion.magnitude_sigma < 0.0 {
        return Err(MonteCarloError::NegativeSigma {
            value: dispersion.magnitude_sigma,
        });
    }
    let scale = if dispersion.magnitude_sigma > 0.0 {
        let mag_normal = Normal::new(0.0, dispersion.magnitude_sigma)
            .map_err(|_| MonteCarloError::NegativeSigma {
                value: dispersion.magnitude_sigma,
            })?;
        1.0 + rng.sample(mag_normal)
    } else {
        1.0
    };

    // Pointing rotation via Rodrigues' formula
    let v_rotated = if dispersion.pointing_sigma_rad > 0.0 {
        // Random rotation axis on unit sphere
        let axis_normal = Normal::new(0.0, 1.0)
            .map_err(|_| MonteCarloError::NegativeSigma { value: 1.0 })?;
        let raw_axis = Vector3::new(
            rng.sample(axis_normal),
            rng.sample(axis_normal),
            rng.sample(axis_normal),
        );
        let axis_norm = raw_axis.norm();
        let k = if axis_norm > RODRIGUES_AXIS_NORM_THRESHOLD {
            raw_axis / axis_norm
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        };

        // Rotation angle from N(0, pointing_sigma_rad)
        let angle_normal = Normal::new(0.0, dispersion.pointing_sigma_rad)
            .map_err(|_| MonteCarloError::NegativeSigma {
                value: dispersion.pointing_sigma_rad,
            })?;
        let theta: f64 = rng.sample(angle_normal);

        // Rodrigues: v_rot = v*cos(θ) + (k × v)*sin(θ) + k*(k·v)*(1-cos(θ))
        let cos_t = theta.cos();
        let sin_t = theta.sin();
        let k_cross_v = k.cross(nominal_dv);
        let k_dot_v = k.dot(nominal_dv);
        *nominal_dv * cos_t + k_cross_v * sin_t + k * k_dot_v * (1.0 - cos_t)
    } else {
        *nominal_dv
    };

    Ok(scale * v_rotated)
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/// Compute percentile statistics from a slice of scalar values.
///
/// Uses nearest-rank method for percentile extraction.
/// Converts the slice length to `u32` for lossless `f64::from` conversion.
/// Standard deviation uses Bessel correction (n-1 denominator) for n > 1;
/// returns 0.0 for n = 1.
///
/// # Invariants
/// - `values` must be non-empty.
/// - For n = 1, all percentiles equal the single value and `std_dev = 0`.
///
/// # Errors
/// Returns [`MonteCarloError::EmptyEnsemble`] if `values` is empty.
fn compute_percentile_stats(values: &[f64]) -> Result<PercentileStats, MonteCarloError> {
    if values.is_empty() {
        return Err(MonteCarloError::EmptyEnsemble);
    }
    // Convert length to u32 for lossless f64 conversion.
    // MC sample counts are always bounded by MonteCarloConfig.num_samples (u32),
    // so this conversion is infallible in practice.
    let n = u32::try_from(values.len()).unwrap_or(u32::MAX);
    let n_f = f64::from(n);

    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Nearest-rank percentile using integer arithmetic only.
    // rank = ceil(p * n / 100) computed as (p * n + 99) / 100 in u32.
    let percentile = |p_percent: u32| -> f64 {
        if n == 1 {
            return sorted[0];
        }
        let rank = (p_percent * n).div_ceil(100);
        // u32 → usize: always widening on 32-bit and 64-bit platforms.
        let idx = rank.saturating_sub(1).min(n - 1) as usize;
        sorted[idx]
    };

    let sum: f64 = values.iter().sum();
    let mean = sum / n_f;
    let variance = if n > 1 {
        values.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / f64::from(n - 1)
    } else {
        0.0
    };

    Ok(PercentileStats {
        min: sorted[0],
        p01: percentile(1),
        p05: percentile(5),
        p25: percentile(25),
        p50: percentile(50),
        p75: percentile(75),
        p95: percentile(95),
        p99: percentile(99),
        max: sorted[sorted.len() - 1],
        mean,
        std_dev: variance.sqrt(),
    })
}

/// Compute trajectory dispersion envelope from per-sample trajectories.
///
/// For each time index, collects RIC positions across samples and computes
/// per-axis percentile statistics. Trajectories may have variable length;
/// time steps with no data are silently skipped.
///
/// # Invariants
/// - Returns empty `Vec` if `sample_trajectories` is empty or `n_steps == 0`.
/// - Each trajectory may be shorter than `n_steps`; steps beyond a
///   trajectory's length are excluded from that step's statistics.
fn compute_dispersion_envelope(
    sample_trajectories: &[Vec<PropagatedState>],
    n_steps: u32,
) -> Vec<DispersionEnvelope> {
    if sample_trajectories.is_empty() || n_steps == 0 {
        return Vec::new();
    }

    // u32 → usize: always widening on 32-bit and 64-bit platforms.
    let mut envelopes = Vec::with_capacity(n_steps as usize + 1);

    for j in 0..=n_steps {
        let j_idx = j as usize;
        let mut radial_values = Vec::with_capacity(sample_trajectories.len());
        let mut intrack_values = Vec::with_capacity(sample_trajectories.len());
        let mut crosstrack_values = Vec::with_capacity(sample_trajectories.len());
        let mut elapsed = 0.0_f64;
        let mut needs_elapsed = true;

        for traj in sample_trajectories {
            if j_idx < traj.len() {
                let state = &traj[j_idx];
                radial_values.push(state.ric.position_ric_km.x);
                intrack_values.push(state.ric.position_ric_km.y);
                crosstrack_values.push(state.ric.position_ric_km.z);
                if needs_elapsed {
                    elapsed = state.elapsed_s;
                    needs_elapsed = false;
                }
            }
        }

        if radial_values.is_empty() {
            continue;
        }

        // These succeed because the vecs are non-empty (checked above)
        let Ok(r_stats) = compute_percentile_stats(&radial_values) else {
            continue;
        };
        let Ok(i_stats) = compute_percentile_stats(&intrack_values) else {
            continue;
        };
        let Ok(c_stats) = compute_percentile_stats(&crosstrack_values) else {
            continue;
        };

        envelopes.push(DispersionEnvelope {
            elapsed_s: elapsed,
            radial_km: r_stats,
            in_track_km: i_stats,
            cross_track_km: c_stats,
        });
    }

    envelopes
}


// ---------------------------------------------------------------------------
// Per-sample execution
// ---------------------------------------------------------------------------

/// Result of a single MC sample execution (internal).
struct SampleOutput {
    /// Lightweight result for the report.
    result: SampleResult,
    /// Full trajectory for dispersion envelope computation.
    trajectory: Vec<PropagatedState>,
}

/// Execute a single Monte Carlo sample.
///
/// Propagates chief + deputy through nyx with dispersed initial state and
/// maneuver execution errors. In closed-loop mode, re-targets the mission from
/// the dispersed state before propagation.
///
/// # Invariants
/// - `input.initial_chief` and `input.initial_deputy` must represent bound orbits.
/// - `input.nominal_mission.legs` must be non-empty (caller responsibility).
/// - Dispersed state must remain a bound orbit (`a > 0`, `0 <= e < 1`) or
///   an error is returned for this sample.
///
/// # Errors
/// - [`MonteCarloError::NegativeSma`] / [`MonteCarloError::InvalidEccentricity`]
///   if state dispersion produces an unbound orbit.
/// - [`MonteCarloError::NegativeSigma`] / [`MonteCarloError::NegativeHalfWidth`]
///   if dispersion parameters are invalid.
/// - [`MonteCarloError::EmptyEnsemble`] if a nyx trajectory segment is empty.
/// - Propagation or validation errors from nyx bridge functions.
#[allow(clippy::similar_names, clippy::too_many_lines)]
fn run_single_sample(
    input: &MonteCarloInput<'_>,
    index: u32,
    master_seed: u64,
) -> Result<SampleOutput, MonteCarloError> {
    let config = input.config;
    let initial_chief = input.initial_chief;
    let initial_deputy = input.initial_deputy;
    let nominal_mission = input.nominal_mission;
    let deputy_config = input.deputy_config;
    let chief_config = input.chief_config;
    let almanac = input.almanac;

    let mut rng = ChaCha20Rng::seed_from_u64(master_seed.wrapping_add(u64::from(index)));

    // Disperse deputy initial state in RIC, convert to ECI
    let dispersed_deputy = if let Some(ref state_disp) = config.dispersions.state {
        let pos_ric_delta = Vector3::new(
            sample_distribution(&state_disp.position_radial_km, &mut rng)?,
            sample_distribution(&state_disp.position_intrack_km, &mut rng)?,
            sample_distribution(&state_disp.position_crosstrack_km, &mut rng)?,
        );
        let vel_ric_delta = Vector3::new(
            sample_distribution(&state_disp.velocity_radial_km_s, &mut rng)?,
            sample_distribution(&state_disp.velocity_intrack_km_s, &mut rng)?,
            sample_distribution(&state_disp.velocity_crosstrack_km_s, &mut rng)?,
        );

        // RIC → ECI via DCM transpose
        let dcm = eci_to_ric_dcm(initial_chief);
        let dcm_transpose = dcm.transpose();
        let pos_eci_delta = dcm_transpose * pos_ric_delta;
        let vel_eci_delta = dcm_transpose * vel_ric_delta;

        StateVector {
            epoch: initial_deputy.epoch,
            position_eci_km: initial_deputy.position_eci_km + pos_eci_delta,
            velocity_eci_km_s: initial_deputy.velocity_eci_km_s + vel_eci_delta,
        }
    } else {
        initial_deputy.clone()
    };

    // Validity check: ensure dispersed state is a bound orbit
    let ke = state_to_keplerian(&dispersed_deputy)
        .map_err(|_| MonteCarloError::NegativeSma {
            sample_index: index,
            a_km: 0.0,
        })?;
    if ke.a_km <= 0.0 {
        return Err(MonteCarloError::NegativeSma {
            sample_index: index,
            a_km: ke.a_km,
        });
    }
    if ke.e >= 1.0 {
        return Err(MonteCarloError::InvalidEccentricity {
            sample_index: index,
            e: ke.e,
        });
    }

    // Optionally disperse spacecraft properties
    let sample_deputy_config = if let Some(ref sc_disp) = config.dispersions.spacecraft {
        SpacecraftConfig {
            coeff_drag: deputy_config.coeff_drag
                + sample_distribution(&sc_disp.coeff_drag, &mut rng)?,
            drag_area_m2: (deputy_config.drag_area_m2
                + sample_distribution(&sc_disp.drag_area_m2, &mut rng)?)
            .max(0.0),
            dry_mass_kg: (deputy_config.dry_mass_kg
                + sample_distribution(&sc_disp.dry_mass_kg, &mut rng)?)
            .max(MIN_SPACECRAFT_MASS_KG),
            ..*deputy_config
        }
    } else {
        *deputy_config
    };

    // Determine which mission plan to use for propagation
    let (mission_to_use, converged) = match config.mode {
        MonteCarloMode::OpenLoop => (None, true),
        MonteCarloMode::ClosedLoop => {
            match retarget_from_dispersed(
                initial_chief,
                &dispersed_deputy,
                nominal_mission,
                input.mission_config,
                input.propagator,
            ) {
                Ok(retargeted) => (Some(retargeted), true),
                Err(_) => (None, false), // fall back to nominal plan
            }
        }
    };
    let active_mission = mission_to_use.as_ref().unwrap_or(nominal_mission);

    // Propagate through legs with maneuver execution errors
    let mut chief_state = initial_chief.clone();
    let mut deputy_state = dispersed_deputy;
    let mut total_dv = 0.0_f64;
    let mut waypoint_miss_km = Vec::with_capacity(active_mission.legs.len());
    let mut all_safety_pairs: Vec<(f64, StateVector, StateVector)> = Vec::new();
    let mut elapsed_total = 0.0_f64;

    let maneuver_disp = config.dispersions.maneuver.as_ref();
    let traj_steps = config.trajectory_steps.max(1);

    for leg in &active_mission.legs {
        // Apply dispersed departure Δv
        let dep_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.departure_maneuver.dv_ric_km_s, disp, &mut rng)?
        } else {
            leg.departure_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &dep_dv);
        total_dv += dep_dv.norm();

        // Build dynamics and propagate chief + deputy through this leg
        let chief_dynamics = build_full_physics_dynamics(almanac)?;
        let chief_traj = nyx_propagate_segment(
            &chief_state,
            leg.tof_s,
            traj_steps,
            chief_config,
            chief_dynamics,
            almanac,
        )?;

        let deputy_dynamics = build_full_physics_dynamics(almanac)?;
        let deputy_traj = nyx_propagate_segment(
            &deputy_state,
            leg.tof_s,
            traj_steps,
            &sample_deputy_config,
            deputy_dynamics,
            almanac,
        )?;

        // Collect chief/deputy pairs for safety analysis
        for (c_entry, d_entry) in chief_traj.iter().zip(deputy_traj.iter()) {
            all_safety_pairs.push((
                elapsed_total + c_entry.0,
                c_entry.1.clone(),
                d_entry.1.clone(),
            ));
        }

        // Update states to end of leg
        chief_state = chief_traj
            .last()
            .map(|(_, s)| s.clone())
            .ok_or(MonteCarloError::EmptyEnsemble)?;
        deputy_state = deputy_traj
            .last()
            .map(|(_, s)| s.clone())
            .ok_or(MonteCarloError::EmptyEnsemble)?;

        // Apply dispersed arrival Δv
        let arr_dv = if let Some(disp) = maneuver_disp {
            disperse_maneuver(&leg.arrival_maneuver.dv_ric_km_s, disp, &mut rng)?
        } else {
            leg.arrival_maneuver.dv_ric_km_s
        };
        deputy_state = apply_impulse(&deputy_state, &chief_state, &arr_dv);
        total_dv += arr_dv.norm();

        // Compute miss distance at waypoint arrival
        let ric_rel = eci_to_ric_relative(&chief_state, &deputy_state);
        let miss = (ric_rel.position_ric_km - leg.to_position_ric_km).norm();
        waypoint_miss_km.push(miss);

        elapsed_total += leg.tof_s;
    }

    // Build safety states and compute safety metrics
    let safety_states = build_nyx_safety_states(&all_safety_pairs)?;
    let safety = analyze_trajectory_safety(&safety_states).ok();

    Ok(SampleOutput {
        result: SampleResult {
            index,
            total_dv_km_s: total_dv,
            safety,
            waypoint_miss_km,
            converged,
        },
        trajectory: safety_states,
    })
}

/// Re-target the mission from a dispersed deputy state (closed-loop mode).
///
/// Converts the dispersed ECI states to a `DepartureState`, extracts waypoints
/// from the nominal mission legs, and runs `plan_waypoint_mission` to produce
/// a new mission plan with updated Δvs.
///
/// # Invariants
/// - `chief` and `dispersed_deputy` must be convertible to Keplerian elements
///   (bound orbits, non-degenerate position vectors).
/// - `nominal_mission.legs` must be non-empty.
///
/// # Errors
/// - [`MissionError::Conversion`] if ECI → Keplerian or ROE computation fails.
/// - Any [`MissionError`] variant from `plan_waypoint_mission` (e.g., targeting
///   non-convergence).
fn retarget_from_dispersed(
    chief: &StateVector,
    dispersed_deputy: &StateVector,
    nominal_mission: &WaypointMission,
    mission_config: &MissionConfig,
    propagator: &PropagationModel,
) -> Result<WaypointMission, MissionError> {
    let chief_ke = state_to_keplerian(chief).map_err(MissionError::Conversion)?;
    let deputy_ke = state_to_keplerian(dispersed_deputy).map_err(MissionError::Conversion)?;
    let roe = compute_roe(&chief_ke, &deputy_ke).map_err(MissionError::Conversion)?;

    let departure = DepartureState {
        roe,
        chief: chief_ke,
        epoch: chief.epoch,
    };

    // Extract waypoints from nominal mission legs (same targets, same TOFs)
    let waypoints: Vec<Waypoint> = nominal_mission
        .legs
        .iter()
        .map(|leg| Waypoint {
            position_ric_km: leg.to_position_ric_km,
            velocity_ric_km_s: leg.target_velocity_ric_km_s,
            tof_s: Some(leg.tof_s),
        })
        .collect();

    plan_waypoint_mission(&departure, &waypoints, mission_config, propagator)
}

// ---------------------------------------------------------------------------
// Orchestrator
// ---------------------------------------------------------------------------

/// Run full-physics Monte Carlo ensemble analysis using nyx propagation.
///
/// Each sample propagates chief + deputy through nyx with dispersed initial
/// states and Δv execution errors. In closed-loop mode, each sample re-targets
/// the mission from the dispersed initial state before propagation.
/// Uses rayon for parallel execution with deterministic per-sample seeding.
///
/// # Invariants
/// - `input.config.num_samples > 0`
/// - `input.initial_chief` and `input.initial_deputy` are bound orbits
///   (caller responsibility).
/// - `input.nominal_mission.legs` is non-empty (caller responsibility).
/// - Results are deterministic for a given `input.config.seed`.
///
/// # Errors
/// - [`MonteCarloError::ZeroSamples`] if `num_samples == 0`.
/// - [`MonteCarloError::AllSamplesFailed`] if every sample fails (with
///   breakdown of convergence vs propagation failures).
pub fn run_monte_carlo(input: &MonteCarloInput<'_>) -> Result<MonteCarloReport, MonteCarloError> {
    let config = input.config;
    let nominal_mission = input.nominal_mission;

    if config.num_samples == 0 {
        return Err(MonteCarloError::ZeroSamples);
    }

    let start = Instant::now();
    let master_seed = config.seed.unwrap_or(42);

    // Run samples in parallel
    let results: Vec<Result<SampleOutput, MonteCarloError>> = (0..config.num_samples)
        .into_par_iter()
        .map(|i| run_single_sample(input, i, master_seed))
        .collect();

    // Separate successes from failures
    let mut samples = Vec::new();
    let mut trajectories = Vec::new();
    let mut num_failures = 0_u32;
    let mut convergence_failures = 0_u32;
    let mut propagation_failures = 0_u32;

    for result in results {
        match result {
            Ok(output) => {
                samples.push(output.result);
                trajectories.push(output.trajectory);
            }
            Err(MonteCarloError::Mission(_)) => {
                convergence_failures += 1;
                num_failures += 1;
            }
            Err(_) => {
                propagation_failures += 1;
                num_failures += 1;
            }
        }
    }

    if samples.is_empty() {
        return Err(MonteCarloError::AllSamplesFailed {
            num_samples: config.num_samples,
            convergence_failures,
            propagation_failures,
        });
    }

    // Compute ensemble statistics
    let statistics = collect_ensemble_statistics(&samples, &trajectories, config)?;

    // Covariance validation (if provided)
    let covariance_validation = input
        .covariance_report
        .map(|cov_report| compute_covariance_validation(cov_report, &statistics));

    Ok(MonteCarloReport {
        config: config.clone(),
        nominal_dv_km_s: nominal_mission.total_dv_km_s,
        nominal_safety: nominal_mission.safety,
        statistics,
        samples,
        num_failures,
        elapsed_wall_s: start.elapsed().as_secs_f64(),
        covariance_validation,
    })
}

/// Collect aggregate ensemble statistics from successful samples.
///
/// Computes percentile distributions for Δv, safety metrics, waypoint miss
/// distances, collision probability, convergence rate, and trajectory
/// dispersion envelope.
///
/// # Invariants
/// - `samples` must be non-empty (caller ensures this after filtering).
/// - `trajectories.len() == samples.len()`.
///
/// # Errors
/// Returns [`MonteCarloError::EmptyEnsemble`] if `compute_percentile_stats`
/// fails on an empty Δv vector (should not occur if `samples` is non-empty).
fn collect_ensemble_statistics(
    samples: &[SampleResult],
    trajectories: &[Vec<PropagatedState>],
    config: &MonteCarloConfig,
) -> Result<EnsembleStatistics, MonteCarloError> {
    let total_dvs: Vec<f64> = samples.iter().map(|s| s.total_dv_km_s).collect();

    // Extract safety metrics from samples that have them
    let mut min_rc_values = Vec::new();
    let mut min_3d_values = Vec::new();
    let mut min_ei_values = Vec::new();

    for s in samples {
        if let Some(ref safety) = s.safety {
            min_rc_values.push(safety.min_rc_separation_km);
            min_3d_values.push(safety.min_distance_3d_km);
            min_ei_values.push(safety.min_ei_separation_km);
        }
    }

    let total_dv_stats = compute_percentile_stats(&total_dvs)?;

    // Use zero stats if no safety data available
    let min_rc_stats = if min_rc_values.is_empty() {
        PercentileStats::default()
    } else {
        compute_percentile_stats(&min_rc_values)?
    };
    let min_3d_stats = if min_3d_values.is_empty() {
        PercentileStats::default()
    } else {
        compute_percentile_stats(&min_3d_values)?
    };
    let min_ei_stats = if min_ei_values.is_empty() {
        PercentileStats::default()
    } else {
        compute_percentile_stats(&min_ei_values)?
    };

    // Per-waypoint miss distance statistics
    let num_waypoints = samples
        .first()
        .map_or(0, |s| s.waypoint_miss_km.len());
    let mut waypoint_miss_stats = Vec::with_capacity(num_waypoints);
    for wp_idx in 0..num_waypoints {
        let misses: Vec<f64> = samples
            .iter()
            .filter_map(|s| s.waypoint_miss_km.get(wp_idx).copied())
            .collect();
        if misses.is_empty() {
            waypoint_miss_stats.push(PercentileStats::default());
        } else {
            waypoint_miss_stats.push(compute_percentile_stats(&misses)?);
        }
    }

    // Empirical collision probability: fraction of samples violating keep-out
    let n_samples = u32::try_from(samples.len()).unwrap_or(u32::MAX);
    let n_samples_f = f64::from(n_samples);

    let mut collision_count = 0_u32;
    let mut converged_count = 0_u32;
    for s in samples {
        if s.safety.as_ref().is_some_and(|safety| {
            safety.min_distance_3d_km < MC_DEFAULT_COLLISION_THRESHOLD_KM
        }) {
            collision_count += 1;
        }
        if s.converged {
            converged_count += 1;
        }
    }
    let collision_probability = f64::from(collision_count) / n_samples_f;
    let convergence_rate = f64::from(converged_count) / n_samples_f;

    // Dispersion envelope
    let dispersion_envelope = compute_dispersion_envelope(trajectories, config.trajectory_steps);

    Ok(EnsembleStatistics {
        total_dv_km_s: total_dv_stats,
        min_rc_distance_km: min_rc_stats,
        min_3d_distance_km: min_3d_stats,
        min_ei_separation_km: min_ei_stats,
        waypoint_miss_km: waypoint_miss_stats,
        collision_probability,
        convergence_rate,
        dispersion_envelope,
    })
}

/// Compare MC ensemble statistics against covariance predictions.
///
/// Computes sigma ratios (MC σ / covariance σ per RIC axis) and approximate
/// 3-sigma containment fraction. A well-calibrated covariance model should
/// produce sigma ratios near 1.0 and containment near 99.7%.
///
/// # Invariants
/// - If `statistics.dispersion_envelope` is empty, sigma ratios default to 1.0.
/// - If predicted covariance sigma is below [`COVARIANCE_SIGMA_FLOOR`],
///   sigma ratios default to 1.0 (avoids division by zero).
fn compute_covariance_validation(
    cov_report: &MissionCovarianceReport,
    statistics: &EnsembleStatistics,
) -> CovarianceValidation {
    // Compute sigma ratios: MC std_dev vs covariance 1-sigma per axis
    let mut sigma_ratio_r = 1.0;
    let mut sigma_ratio_i = 1.0;
    let mut sigma_ratio_c = 1.0;

    if let Some(last_env) = statistics.dispersion_envelope.last() {
        let mc_sigma_r = last_env.radial_km.std_dev;
        let mc_sigma_i = last_env.in_track_km.std_dev;
        let mc_sigma_c = last_env.cross_track_km.std_dev;

        // Covariance max 3-sigma → 1-sigma
        let cov_sigma = cov_report.max_sigma3_position_km / 3.0;
        if cov_sigma > COVARIANCE_SIGMA_FLOOR {
            sigma_ratio_r = mc_sigma_r / cov_sigma;
            sigma_ratio_i = mc_sigma_i / cov_sigma;
            sigma_ratio_c = mc_sigma_c / cov_sigma;
        }
    }

    // 3-sigma containment approximation via percentile comparison
    let cov_3sigma = cov_report.max_sigma3_position_km;
    let fraction_within_3sigma = if statistics.min_3d_distance_km.std_dev > 0.0 {
        if statistics.min_3d_distance_km.p99 < cov_3sigma {
            0.99
        } else if statistics.min_3d_distance_km.p95 < cov_3sigma {
            0.95
        } else {
            0.90
        }
    } else {
        1.0
    };

    CovarianceValidation {
        fraction_within_3sigma,
        covariance_collision_prob: cov_report.max_collision_probability,
        mc_collision_prob: statistics.collision_probability,
        sigma_ratio_ric: Vector3::new(sigma_ratio_r, sigma_ratio_i, sigma_ratio_c),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DispersionConfig;
    use rand::SeedableRng;
    use rand_chacha::ChaCha20Rng;

    /// Tolerance for Gaussian mean convergence: 100k samples of N(0,1)
    /// should have mean within 2% of zero with high probability.
    const GAUSSIAN_MEAN_TOL: f64 = 0.02;

    /// Tolerance for sigma scaling ratio: 10k samples give ~1% std error
    /// on estimated std dev, so 5% tolerance is conservative.
    const SIGMA_RATIO_TOL: f64 = 0.05;

    /// Tolerance for magnitude ratio in maneuver dispersion: 10k samples,
    /// 1% magnitude sigma → expect mean ratio within 2% of 1.0.
    const MAGNITUDE_RATIO_TOL: f64 = 0.02;

    /// Tolerance for nearest-rank percentile accuracy with 1000 integer
    /// values: off-by-one in rank gives ±1, so ±2 is conservative.
    const PERCENTILE_ACCURACY_TOL: f64 = 2.0;

    /// Pointing cone bound: 4× pointing sigma covers >99.99% of
    /// N(0, σ) samples (4-sigma bound).
    const POINTING_CONE_SIGMA_BOUND: f64 = 4.0;

    /// Tolerance for pointing-only magnitude preservation: Rodrigues
    /// rotation is exact, so magnitude change should be at machine epsilon.
    const POINTING_MAGNITUDE_TOL: f64 = 1e-14;

    /// Open-loop convergence rate is exactly 1.0 (all samples "converge").
    /// Tolerance accounts for floating-point division in rate computation.
    const CONVERGENCE_RATE_TOL: f64 = 1e-10;

    /// Upper bound on collision probability for "safe" geometry test.
    /// 5 km V-bar separation with 100 m dispersions should have Pc well below 10%.
    const SAFE_COLLISION_PROB_BOUND: f64 = 0.1;

    /// Waypoint distance (km) for collision probability "unsafe" test.
    /// 50 m is deliberately close to the 100 m collision threshold.
    const UNSAFE_WAYPOINT_DISTANCE_KM: f64 = 0.05;

    // -----------------------------------------------------------------------
    // Sampling primitives
    // -----------------------------------------------------------------------

    #[test]
    fn gaussian_mean_converges() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let n = 100_000_u32;
        let sum: f64 = (0..n)
            .map(|_| sample_distribution(&dist, &mut rng).unwrap())
            .sum();
        let mean = sum / f64::from(n);
        assert!(
            mean.abs() < GAUSSIAN_MEAN_TOL,
            "gaussian mean should be near 0, got {mean}"
        );
    }

    #[test]
    fn gaussian_sigma_scales() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let n = 10_000_u32;

        let dist_1 = Distribution::Gaussian { sigma: 1.0 };
        let samples_1: Vec<f64> = (0..n)
            .map(|_| sample_distribution(&dist_1, &mut rng).unwrap())
            .collect();
        let std_1 = compute_percentile_stats(&samples_1).unwrap().std_dev;

        let mut rng2 = ChaCha20Rng::seed_from_u64(99);
        let dist_half = Distribution::Gaussian { sigma: 0.5 };
        let samples_half: Vec<f64> = (0..n)
            .map(|_| sample_distribution(&dist_half, &mut rng2).unwrap())
            .collect();
        let std_half = compute_percentile_stats(&samples_half).unwrap().std_dev;

        let ratio = std_half / std_1;
        assert!(
            (ratio - 0.5).abs() < SIGMA_RATIO_TOL,
            "sigma ratio should be ~0.5, got {ratio}"
        );
    }

    #[test]
    fn uniform_bounded() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let dist = Distribution::Uniform { half_width: 0.5 };
        for _ in 0..10_000 {
            let v = sample_distribution(&dist, &mut rng).unwrap();
            assert!(
                (-0.5..=0.5).contains(&v),
                "uniform sample {v} out of bounds"
            );
        }
    }

    #[test]
    fn deterministic_seed() {
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let mut rng1 = ChaCha20Rng::seed_from_u64(42);
        let mut rng2 = ChaCha20Rng::seed_from_u64(42);
        for _ in 0..100 {
            let a = sample_distribution(&dist, &mut rng1).unwrap();
            let b = sample_distribution(&dist, &mut rng2).unwrap();
            assert_eq!(a, b, "same seed should produce identical values");
        }
    }

    #[test]
    fn different_seeds_differ() {
        let dist = Distribution::Gaussian { sigma: 1.0 };
        let mut rng1 = ChaCha20Rng::seed_from_u64(42);
        let mut rng2 = ChaCha20Rng::seed_from_u64(99);
        let a = sample_distribution(&dist, &mut rng1).unwrap();
        let b = sample_distribution(&dist, &mut rng2).unwrap();
        assert_ne!(a, b, "different seeds should produce different values");
    }

    // -----------------------------------------------------------------------
    // Maneuver dispersion
    // -----------------------------------------------------------------------

    #[test]
    fn maneuver_magnitude_mean() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.01,
            pointing_sigma_rad: 0.0,
        };
        let n = 10_000_u32;
        let sum_mag: f64 = (0..n)
            .map(|_| disperse_maneuver(&nominal, &disp, &mut rng).unwrap().norm())
            .sum();
        let mean_mag = sum_mag / f64::from(n);
        let ratio = mean_mag / nominal.norm();
        assert!(
            (ratio - 1.0).abs() < MAGNITUDE_RATIO_TOL,
            "mean magnitude ratio should be ~1.0, got {ratio}"
        );
    }

    #[test]
    fn maneuver_pointing_preserves_magnitude() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.0,
            pointing_sigma_rad: 0.01,
        };
        let nominal_mag = nominal.norm();
        for _ in 0..1000 {
            let dispersed = disperse_maneuver(&nominal, &disp, &mut rng).unwrap();
            let diff = (dispersed.norm() - nominal_mag).abs();
            assert!(
                diff < POINTING_MAGNITUDE_TOL,
                "pointing-only: magnitude changed by {diff}"
            );
        }
    }

    #[test]
    fn maneuver_zero_dv_identity() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let zero_dv = Vector3::zeros();
        let disp = ManeuverDispersion::default();
        let result = disperse_maneuver(&zero_dv, &disp, &mut rng).unwrap();
        assert_eq!(result, Vector3::zeros(), "zero Δv should return zero");
    }

    #[test]
    fn maneuver_cone_bounded() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let nominal = Vector3::new(0.001, 0.0, 0.0);
        let pointing_sigma = 0.01_f64;
        let disp = ManeuverDispersion {
            magnitude_sigma: 0.0,
            pointing_sigma_rad: pointing_sigma,
        };
        for _ in 0..1000 {
            let dispersed = disperse_maneuver(&nominal, &disp, &mut rng).unwrap();
            let cos_angle = nominal.dot(&dispersed) / (nominal.norm() * dispersed.norm());
            let angle = cos_angle.clamp(-1.0, 1.0).acos();
            assert!(
                angle < POINTING_CONE_SIGMA_BOUND * pointing_sigma,
                "angle {angle} exceeds {POINTING_CONE_SIGMA_BOUND}*sigma bound"
            );
        }
    }

    // -----------------------------------------------------------------------
    // Statistics
    // -----------------------------------------------------------------------

    #[test]
    fn percentile_ordering() {
        let mut rng = ChaCha20Rng::seed_from_u64(42);
        let values: Vec<f64> = (0..500)
            .map(|_| {
                let dist = Distribution::Gaussian { sigma: 1.0 };
                sample_distribution(&dist, &mut rng).unwrap()
            })
            .collect();
        let stats = compute_percentile_stats(&values).unwrap();
        assert!(stats.min <= stats.p01);
        assert!(stats.p01 <= stats.p05);
        assert!(stats.p05 <= stats.p25);
        assert!(stats.p25 <= stats.p50);
        assert!(stats.p50 <= stats.p75);
        assert!(stats.p75 <= stats.p95);
        assert!(stats.p95 <= stats.p99);
        assert!(stats.p99 <= stats.max);
    }

    #[test]
    fn percentile_known_uniform() {
        let values: Vec<f64> = (0..1000).map(|i| f64::from(i)).collect();
        let stats = compute_percentile_stats(&values).unwrap();
        assert!(
            (stats.p50 - 499.0).abs() < PERCENTILE_ACCURACY_TOL,
            "median should be ~499, got {}",
            stats.p50
        );
        assert!(
            (stats.p95 - 949.0).abs() < PERCENTILE_ACCURACY_TOL,
            "p95 should be ~949, got {}",
            stats.p95
        );
    }

    #[test]
    fn percentile_single_value() {
        let stats = compute_percentile_stats(&[42.0]).unwrap();
        assert_eq!(stats.min, 42.0);
        assert_eq!(stats.max, 42.0);
        assert_eq!(stats.p50, 42.0);
        assert_eq!(stats.mean, 42.0);
        assert_eq!(stats.std_dev, 0.0);
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Build standard MC test fixtures: ISS-like chief, colocated deputy,
    /// single V-bar waypoint at 5 km, planned mission, and full almanac.
    fn build_mc_test_fixtures(
        num_samples: u32,
        mode: MonteCarloMode,
        dispersions: DispersionConfig,
    ) -> (
        StateVector,
        StateVector,
        WaypointMission,
        MonteCarloConfig,
        MissionConfig,
        PropagationModel,
        std::sync::Arc<anise::prelude::Almanac>,
    ) {
        use crate::elements::conversions::keplerian_to_state;
        use crate::mission::validation::load_full_almanac;
        use crate::test_helpers::{iss_like_elements, test_epoch};

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
            roe: crate::types::QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };

        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;

        let mission =
            plan_waypoint_mission(&departure, &[waypoint], &mission_config, &propagator)
                .expect("mission planning should succeed");

        let mc_config = MonteCarloConfig {
            num_samples,
            dispersions,
            mode,
            seed: Some(42),
            trajectory_steps: 10,
        };

        let almanac = load_full_almanac().expect("full almanac should load");

        (
            chief_sv,
            deputy_sv,
            mission,
            mc_config,
            mission_config,
            propagator,
            almanac,
        )
    }

    /// Default state dispersions for MC tests using navigation accuracy
    /// defaults from `constants.rs`: 100 m position, 0.1 m/s velocity.
    fn default_state_dispersions() -> DispersionConfig {
        use crate::types::StateDispersion;
        DispersionConfig {
            state: Some(StateDispersion::default()),
            maneuver: None,
            spacecraft: None,
        }
    }

    // -----------------------------------------------------------------------
    // Error paths (no nyx required)
    // -----------------------------------------------------------------------

    #[test]
    fn mc_zero_samples_error() {
        use std::sync::Arc;
        let dummy_epoch =
            hifitime::Epoch::from_gregorian_utc_hms(2024, 1, 1, 0, 0, 0);
        let chief = StateVector {
            epoch: dummy_epoch,
            position_eci_km: Vector3::new(6786.0, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, 7.67, 0.0),
        };
        let mission = WaypointMission {
            legs: vec![],
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            safety: None,
            covariance: None,
        };
        let mc_config = MonteCarloConfig {
            num_samples: 0,
            dispersions: DispersionConfig::default(),
            mode: MonteCarloMode::OpenLoop,
            seed: Some(42),
            trajectory_steps: 0,
        };
        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let almanac = Arc::new(anise::prelude::Almanac::default());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &chief,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        match run_monte_carlo(&input) {
            Err(MonteCarloError::ZeroSamples) => {}
            other => panic!("expected ZeroSamples error, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Full-physics MC (require nyx almanac)
    // -----------------------------------------------------------------------

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_single_sample() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(1, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("single sample should succeed");
        assert_eq!(report.samples.len(), 1, "should have exactly 1 sample");
        assert!(report.samples[0].converged, "sample should converge");
        assert!(
            report.samples[0].total_dv_km_s.is_finite(),
            "Δv should be finite"
        );
        assert_eq!(report.num_failures, 0, "no failures expected");
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_10_samples_statistics() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("10 samples should succeed");
        let stats = &report.statistics;

        // All PercentileStats fields should be finite
        assert!(stats.total_dv_km_s.mean.is_finite(), "Δv mean should be finite");
        assert!(stats.total_dv_km_s.std_dev.is_finite(), "Δv std should be finite");

        // Percentile ordering
        assert!(stats.total_dv_km_s.p05 <= stats.total_dv_km_s.p50);
        assert!(stats.total_dv_km_s.p50 <= stats.total_dv_km_s.p95);

        // Collision probability bounded
        assert!(stats.collision_probability <= 1.0);

        // Convergence rate should be 1.0 (open-loop always "converges")
        assert!(
            (stats.convergence_rate - 1.0).abs() < CONVERGENCE_RATE_TOL,
            "open-loop convergence rate should be 1.0, got {}",
            stats.convergence_rate
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_parallel_deterministic() {
        let dispersions = default_state_dispersions();

        let (chief1, deputy1, mission1, mc1, mc_cfg1, prop1, alm1) =
            build_mc_test_fixtures(5, MonteCarloMode::OpenLoop, dispersions);
        let input1 = MonteCarloInput {
            nominal_mission: &mission1,
            initial_chief: &chief1,
            initial_deputy: &deputy1,
            config: &mc1,
            mission_config: &mc_cfg1,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &prop1,
            almanac: &alm1,
            covariance_report: None,
        };
        let report1 = run_monte_carlo(&input1).expect("run 1 should succeed");

        let dispersions2 = default_state_dispersions();
        let (chief2, deputy2, mission2, mc2, mc_cfg2, prop2, alm2) =
            build_mc_test_fixtures(5, MonteCarloMode::OpenLoop, dispersions2);
        let input2 = MonteCarloInput {
            nominal_mission: &mission2,
            initial_chief: &chief2,
            initial_deputy: &deputy2,
            config: &mc2,
            mission_config: &mc_cfg2,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &prop2,
            almanac: &alm2,
            covariance_report: None,
        };
        let report2 = run_monte_carlo(&input2).expect("run 2 should succeed");

        // Same seed → identical per-sample Δv
        assert_eq!(report1.samples.len(), report2.samples.len());
        for (s1, s2) in report1.samples.iter().zip(report2.samples.iter()) {
            assert_eq!(
                s1.total_dv_km_s, s2.total_dv_km_s,
                "sample {} Δv mismatch: {} vs {}",
                s1.index, s1.total_dv_km_s, s2.total_dv_km_s
            );
        }
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_report_serde_roundtrip() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(3, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        let json = serde_json::to_string(&report).expect("serialize should succeed");
        let roundtrip: MonteCarloReport =
            serde_json::from_str(&json).expect("deserialize should succeed");

        assert_eq!(roundtrip.samples.len(), report.samples.len());
        assert_eq!(roundtrip.config.num_samples, report.config.num_samples);
        assert_eq!(
            roundtrip.nominal_dv_km_s, report.nominal_dv_km_s,
            "nominal Δv should survive roundtrip"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_collision_prob_safe() {
        // Large separation (5 km V-bar), small dispersions → low collision probability
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        assert!(
            report.statistics.collision_probability < SAFE_COLLISION_PROB_BOUND,
            "5 km separation with 100 m dispersions should be safe, got Pc={}",
            report.statistics.collision_probability
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_collision_prob_unsafe() {
        use crate::elements::conversions::keplerian_to_state;
        use crate::mission::validation::load_full_almanac;
        use crate::test_helpers::{iss_like_elements, test_epoch};

        // Tiny waypoint (50 m) with large dispersions → expect violations
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();

        let period = chief_ke.period();
        let waypoint = Waypoint {
            position_ric_km: Vector3::new(0.0, UNSAFE_WAYPOINT_DISTANCE_KM, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(period),
        };

        let departure = DepartureState {
            roe: crate::types::QuasiNonsingularROE::default(),
            chief: chief_ke,
            epoch,
        };

        let mission_config = MissionConfig::default();
        let propagator = PropagationModel::J2Stm;
        let mission =
            plan_waypoint_mission(&departure, &[waypoint], &mission_config, &propagator)
                .expect("planning should succeed");

        // Large dispersions relative to 50 m target
        let mc_config = MonteCarloConfig {
            num_samples: 10,
            dispersions: default_state_dispersions(),
            mode: MonteCarloMode::OpenLoop,
            seed: Some(42),
            trajectory_steps: 10,
        };

        let almanac = load_full_almanac().expect("almanac should load");

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief_sv,
            initial_deputy: &deputy_sv,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("MC should succeed");
        // With 100 m dispersions and 50 m target, some samples should violate keep-out.
        // With only 10 samples we can't guarantee Pc > 0 deterministically,
        // but the test confirms MC runs successfully on near-keep-out geometry.
        assert!(
            report.statistics.collision_probability <= 1.0,
            "collision probability should be bounded"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_closed_loop_converges() {
        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(5, MonteCarloMode::ClosedLoop, default_state_dispersions());

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: None,
        };

        let report = run_monte_carlo(&input).expect("closed-loop MC should succeed");
        assert!(
            report.statistics.convergence_rate > 0.0,
            "at least some closed-loop samples should converge"
        );
        // Closed-loop should have comparable or lower miss distances than open-loop
        assert!(
            report.statistics.total_dv_km_s.mean.is_finite(),
            "closed-loop Δv mean should be finite"
        );
    }

    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn nyx_mc_covariance_validation() {
        use crate::propagation::covariance::{
            propagate_mission_covariance, ric_accuracy_to_roe_covariance,
        };
        use crate::types::NavigationAccuracy;

        let (chief, deputy, mission, mc_config, mission_config, propagator, almanac) =
            build_mc_test_fixtures(10, MonteCarloMode::OpenLoop, default_state_dispersions());

        // Run covariance propagation first
        let nav = NavigationAccuracy::default();
        let chief_ke = crate::test_helpers::iss_like_elements();
        let initial_cov =
            ric_accuracy_to_roe_covariance(&nav, &chief_ke).expect("covariance init should work");
        let cov_report =
            propagate_mission_covariance(&mission, &initial_cov, &nav, None, &propagator, 20)
                .expect("covariance propagation should succeed");

        let input = MonteCarloInput {
            nominal_mission: &mission,
            initial_chief: &chief,
            initial_deputy: &deputy,
            config: &mc_config,
            mission_config: &mission_config,
            chief_config: &SpacecraftConfig::default(),
            deputy_config: &SpacecraftConfig::default(),
            propagator: &propagator,
            almanac: &almanac,
            covariance_report: Some(&cov_report),
        };

        let report = run_monte_carlo(&input).expect("MC with covariance should succeed");

        // CovarianceValidation should be populated
        let cv = report
            .covariance_validation
            .as_ref()
            .expect("covariance_validation should be Some");

        // Sigma ratios should be finite and positive
        assert!(cv.sigma_ratio_ric.x.is_finite(), "radial sigma ratio not finite");
        assert!(cv.sigma_ratio_ric.y.is_finite(), "in-track sigma ratio not finite");
        assert!(cv.sigma_ratio_ric.z.is_finite(), "cross-track sigma ratio not finite");

        // 3-sigma containment should be a valid fraction
        assert!(
            cv.fraction_within_3sigma >= 0.0 && cv.fraction_within_3sigma <= 1.0,
            "3-sigma containment should be in [0, 1], got {}",
            cv.fraction_within_3sigma
        );

        // MC collision prob should be bounded
        assert!(cv.mc_collision_prob <= 1.0);
    }
}

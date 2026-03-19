//! Validation helpers and integration tests for nyx numerical propagation comparisons.
//!
//! Uses the nyx bridge (`propagation::nyx_bridge`) for almanac loading, dynamics
//! construction, and segment propagation.  This module owns the high-level
//! `validate_mission_nyx` orchestrator and its associated error type.

use std::sync::Arc;

use anise::constants::frames::EARTH_J2000 as ANISE_EARTH_J2000;
use anise::prelude::Almanac;
use hifitime::Epoch;
use nalgebra::Vector3;
use serde::Serialize;

use crate::constants::ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
use crate::elements::eci_ric_dcm::{eci_to_ric_relative, DcmError};
use crate::elements::eclipse::{compute_eclipse_state, sun_position_eci_km};
use crate::mission::safety::{analyze_trajectory_safety, SafetyError};
use crate::propagation::nyx_bridge::{
    apply_impulse, build_full_physics_dynamics, build_nyx_safety_states, nyx_propagate_segment,
    query_anise_eclipse, state_to_orbit, ChiefDeputySnapshot, NyxBridgeError, TimedState,
};
use crate::propagation::propagator::PropagatedState;
use crate::types::{EclipseInterval, EclipseState, EclipseSummary, RICState, SpacecraftConfig, StateVector};

use super::types::{
    EclipseIntervalComparison, EclipseValidation, EclipseValidationPoint, ManeuverLeg,
    ValidationPoint, ValidationReport, WaypointMission,
};

/// Errors from nyx high-fidelity validation.
#[derive(Debug)]
pub enum ValidationError {
    /// Nyx bridge failure (almanac, dynamics, propagation, conversion).
    NyxBridge(Box<NyxBridgeError>),
    /// Safety analysis failure.
    Safety {
        /// The underlying safety error.
        source: SafetyError,
    },
    /// No trajectory points to analyze.
    EmptyTrajectory,
    /// ECI↔RIC frame conversion failed.
    DcmFailure(DcmError),
    /// ANISE eclipse query failed at a specific epoch.
    EclipseQuery {
        /// The epoch at which the query failed.
        epoch: Epoch,
        /// The underlying error message.
        message: String,
    },
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NyxBridge(e) => write!(f, "nyx bridge: {e}"),
            Self::Safety { source } => {
                write!(f, "safety analysis failed: {source}")
            }
            Self::EmptyTrajectory => {
                write!(f, "no trajectory points to analyze")
            }
            Self::DcmFailure(e) => write!(f, "frame conversion failed: {e}"),
            Self::EclipseQuery { epoch, message } => {
                write!(f, "eclipse query at {epoch} failed: {message}")
            }
        }
    }
}

impl std::error::Error for ValidationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::NyxBridge(e) => Some(e.as_ref()),
            Self::Safety { source } => Some(source),
            Self::DcmFailure(e) => Some(e),
            Self::EmptyTrajectory | Self::EclipseQuery { .. } => None,
        }
    }
}

impl From<NyxBridgeError> for ValidationError {
    fn from(e: NyxBridgeError) -> Self {
        Self::NyxBridge(Box::new(e))
    }
}

impl From<DcmError> for ValidationError {
    fn from(e: DcmError) -> Self {
        Self::DcmFailure(e)
    }
}

impl From<SafetyError> for ValidationError {
    fn from(e: SafetyError) -> Self {
        Self::Safety { source: e }
    }
}

/// Output from validating a single mission leg against nyx full-physics.
///
/// Contains per-sample comparison points and updated chief/deputy states
/// for threading into the next leg. Used by the API server for per-leg
/// progress streaming during validation.
#[derive(Debug, Clone, Serialize)]
pub struct LegValidationOutput {
    /// Per-sample analytical vs numerical RIC comparison points.
    pub points: Vec<ValidationPoint>,
    /// Chief/deputy ECI snapshot pairs for safety analysis.
    pub safety_pairs: Vec<ChiefDeputySnapshot>,
    /// Updated chief ECI state at end of this leg.
    pub chief_state_after: StateVector,
    /// Updated deputy ECI state at end of this leg (arrival delta-v applied).
    pub deputy_state_after: StateVector,
}

/// Validate a single mission leg against nyx full-physics propagation.
///
/// Propagates chief and deputy through the leg using nyx full-physics dynamics,
/// compares against the analytical trajectory, and returns comparison points plus
/// updated ECI states for threading to the next leg.
///
/// This function does **not** collect eclipse samples; eclipse validation is only
/// available through the mission-level [`validate_mission_nyx`] orchestrator.
///
/// # Invariants
/// - `chief_state` and `deputy_state` must be valid bound ECI states
/// - `almanac` must contain Earth frame data and planetary ephemerides
///
/// # Errors
/// Returns [`ValidationError`] if propagation, impulse application, or frame conversion fails.
#[allow(clippy::too_many_arguments)]
pub fn validate_leg_nyx(
    leg: &ManeuverLeg,
    chief_state: &StateVector,
    deputy_state: &StateVector,
    samples_per_leg: u32,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    cumulative_time: f64,
) -> Result<LegValidationOutput, ValidationError> {
    let (chief_results, deputy_results) = propagate_leg_parallel(
        chief_state,
        deputy_state,
        leg,
        samples_per_leg,
        chief_config,
        deputy_config,
        almanac,
    )?;

    // Build comparison points (no eclipse — that stays in validate_mission_nyx)
    let leg_output = build_leg_comparison_points(
        &chief_results,
        &deputy_results,
        &leg.trajectory,
        cumulative_time,
        None, // no eclipse frame for per-leg API
        almanac,
    )?;

    // Advance states
    let chief_after = chief_results
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();
    let deputy_coast_end = deputy_results
        .last()
        .ok_or(ValidationError::EmptyTrajectory)?
        .state
        .clone();
    let deputy_after =
        apply_impulse(&deputy_coast_end, &chief_after, &leg.arrival_maneuver.dv_ric_km_s)?;

    Ok(LegValidationOutput {
        points: leg_output.points,
        safety_pairs: leg_output.safety_pairs,
        chief_state_after: chief_after,
        deputy_state_after: deputy_after,
    })
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

/// Guard threshold for flat eclipse percentage segments in [`interpolate_crossing`].
/// When |pct₁ − pct₀| < machine epsilon, the segment is flat and interpolation
/// is undefined; fall back to the current sample epoch to avoid division by zero.
const ECLIPSE_INTERPOLATION_FLAT_GUARD: f64 = f64::EPSILON;

/// Shadow percentage at or above which the eclipse state is classified as full umbra
/// rather than penumbra. Based on physical convention: >99% shadow is effectively total.
const UMBRA_PERCENTAGE_THRESHOLD: f64 = 99.0;

/// Compute eclipse intervals from per-sample ANISE eclipse percentages.
///
/// Walks the sample array with a sliding window, detects transitions between
/// sunlit (<1% eclipse) and shadow (>=1% eclipse) states, and records
/// contiguous shadow intervals with worst-case shadow state.
///
/// When a transition is detected between adjacent samples `(t₀, pct₀)` and
/// `(t₁, pct₁)`, linear interpolation finds the crossing epoch:
///
/// ```text
/// t_cross = t₀ + (t₁ - t₀) × (threshold - pct₀) / (pct₁ - pct₀)
/// ```
///
/// This reduces eclipse boundary error from ±(`sample_interval`/2) to the
/// non-linearity of the shadow geometry between samples.
///
/// # Arguments
/// * `samples` — Time-ordered pairs of `(epoch, eclipse_percentage)` where
///   0.0 = fully sunlit and 100.0 = full umbra.
///
/// # Returns
/// A vector of [`EclipseInterval`] for each detected shadow interval.
/// Returns an empty vector if fewer than 2 samples are provided.
fn compute_numerical_eclipse_intervals(
    samples: &[(Epoch, f64)],
) -> Vec<EclipseInterval> {
    if samples.len() < 2 {
        return Vec::new();
    }

    let threshold = ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
    let mut intervals = Vec::new();
    let mut in_shadow = false;
    let mut shadow_start = samples[0].0;
    let mut max_pct = 0.0_f64;

    // Check if first sample is already in shadow (no interpolation possible)
    let (_, first_pct) = samples[0];
    if first_pct >= threshold {
        in_shadow = true;
        shadow_start = samples[0].0;
        max_pct = first_pct;
    }

    for i in 1..samples.len() {
        let (prev_epoch, prev_pct) = samples[i - 1];
        let (curr_epoch, curr_pct) = samples[i];

        if curr_pct >= threshold {
            if in_shadow {
                max_pct = max_pct.max(curr_pct);
            } else {
                // Entering shadow: interpolate crossing epoch
                in_shadow = true;
                max_pct = curr_pct;
                shadow_start = interpolate_crossing(
                    prev_epoch, prev_pct, curr_epoch, curr_pct, threshold,
                );
            }
        } else if in_shadow {
            // Exiting shadow: interpolate crossing epoch
            in_shadow = false;
            let shadow_end = interpolate_crossing(
                prev_epoch, prev_pct, curr_epoch, curr_pct, threshold,
            );
            let duration_s = (shadow_end - shadow_start).to_seconds();
            let state = if max_pct >= UMBRA_PERCENTAGE_THRESHOLD {
                EclipseState::Umbra
            } else {
                EclipseState::Penumbra {
                    shadow_fraction: max_pct / 100.0,
                }
            };
            intervals.push(EclipseInterval {
                start: shadow_start,
                end: shadow_end,
                duration_s,
                state,
            });
        }
    }

    // Close any open interval at end of samples
    if in_shadow {
        let last_epoch = samples[samples.len() - 1].0;
        let duration_s = (last_epoch - shadow_start).to_seconds();
        let state = if max_pct >= UMBRA_PERCENTAGE_THRESHOLD {
            EclipseState::Umbra
        } else {
            EclipseState::Penumbra {
                shadow_fraction: max_pct / 100.0,
            }
        };
        intervals.push(EclipseInterval {
            start: shadow_start,
            end: last_epoch,
            duration_s,
            state,
        });
    }

    intervals
}

/// Linearly interpolate the epoch at which the eclipse percentage crosses a threshold.
///
/// Given two adjacent samples `(t₀, pct₀)` and `(t₁, pct₁)` that straddle
/// `threshold`, returns the interpolated crossing epoch. Falls back to `t₁`
/// if `pct₁ == pct₀` (flat segment, avoids division by zero).
fn interpolate_crossing(
    t0: Epoch,
    pct0: f64,
    t1: Epoch,
    pct1: f64,
    threshold: f64,
) -> Epoch {
    let dpct = pct1 - pct0;
    if dpct.abs() < ECLIPSE_INTERPOLATION_FLAT_GUARD {
        return t1;
    }
    let dt_s = (t1 - t0).to_seconds();
    let frac = (threshold - pct0) / dpct;
    // Clamp to [0, 1] to stay within the sample interval
    let frac = frac.clamp(0.0, 1.0);
    t0 + hifitime::Duration::from_seconds(frac * dt_s)
}

/// Match analytical and numerical eclipse intervals by closest start epoch.
///
/// Greedy matching: for each analytical interval, find the numerical interval
/// with the closest start epoch that has not already been matched. Unmatched
/// intervals are counted separately.
///
/// # Arguments
/// * `analytical` — The analytical eclipse summary containing intervals to match.
/// * `numerical` — The numerical eclipse intervals to match against.
///
/// # Returns
/// A tuple of `(comparisons, unmatched_count)` where `comparisons` contains
/// one entry per matched pair and `unmatched_count` is the number of intervals
/// that could not be paired.
fn match_eclipse_intervals(
    analytical: &EclipseSummary,
    numerical: &[EclipseInterval],
) -> (Vec<EclipseIntervalComparison>, usize) {
    let mut comparisons = Vec::new();
    let mut used = vec![false; numerical.len()];
    let mut matched_analytical = 0_usize;

    for a_interval in &analytical.intervals {
        // Find the closest unused numerical interval by start epoch
        let mut best_idx: Option<usize> = None;
        let mut best_diff = f64::INFINITY;

        for (j, n_interval) in numerical.iter().enumerate() {
            if used[j] {
                continue;
            }
            let diff = (a_interval.start - n_interval.start).to_seconds().abs();
            if diff < best_diff {
                best_diff = diff;
                best_idx = Some(j);
            }
        }

        if let Some(idx) = best_idx {
            used[idx] = true;
            matched_analytical += 1;

            let n_interval = &numerical[idx];
            let entry_error_s = (a_interval.start - n_interval.start).to_seconds();
            let exit_error_s = (a_interval.end - n_interval.end).to_seconds();
            let duration_error_s = a_interval.duration_s - n_interval.duration_s;

            comparisons.push(EclipseIntervalComparison {
                analytical_start: a_interval.start,
                numerical_start: n_interval.start,
                entry_error_s,
                analytical_end: a_interval.end,
                numerical_end: n_interval.end,
                exit_error_s,
                duration_error_s,
            });
        }
    }

    let unmatched_analytical = analytical.intervals.len() - matched_analytical;
    let unmatched_numerical = numerical.len() - used.iter().filter(|&&u| u).count();
    let unmatched_count = unmatched_analytical + unmatched_numerical;

    (comparisons, unmatched_count)
}

/// Per-sample eclipse data collected during nyx propagation for later comparison.
struct EclipseSample {
    /// Elapsed time since mission start (seconds).
    elapsed_s: f64,
    /// Epoch at this sample.
    epoch: Epoch,
    /// ANISE eclipse percentage (0–100).
    numerical_pct: f64,
    /// Sun position in ECI J2000 (km) from ANISE.
    anise_sun_eci_km: Vector3<f64>,
    /// Chief position in ECI J2000 (km) at this sample.
    chief_eci_km: Vector3<f64>,
}

/// Output from comparing nyx-propagated states against analytical trajectory for one leg.
struct LegComparisonOutput {
    /// Per-sample validation comparison points.
    points: Vec<ValidationPoint>,
    /// Chief/deputy ECI snapshot pairs for safety analysis.
    safety_pairs: Vec<ChiefDeputySnapshot>,
    /// Eclipse samples for validation (empty if eclipse validation disabled).
    eclipse_samples: Vec<EclipseSample>,
}

/// Build comparison points for a single leg of the mission.
///
/// Compares nyx-propagated chief/deputy states against analytical trajectory,
/// collects safety pairs and eclipse samples for downstream analysis.
fn build_leg_comparison_points(
    chief_results: &[TimedState],
    deputy_results: &[TimedState],
    trajectory: &[PropagatedState],
    cumulative_time: f64,
    earth_frame: Option<anise::prelude::Frame>,
    almanac: &Arc<Almanac>,
) -> Result<LegComparisonOutput, ValidationError> {
    let mut points = Vec::with_capacity(chief_results.len());
    let mut safety_pairs = Vec::with_capacity(chief_results.len());
    let mut eclipse_samples = Vec::new();

    for (idx, (chief_sample, deputy_sample)) in
        chief_results.iter().zip(deputy_results.iter()).enumerate()
    {
        let numerical_ric = eci_to_ric_relative(&chief_sample.state, &deputy_sample.state)?;
        let elapsed = cumulative_time + chief_sample.elapsed_s;
        let analytical_ric = find_closest_analytical_ric(trajectory, chief_sample.elapsed_s);

        let pos_err = (numerical_ric.position_ric_km - analytical_ric.position_ric_km).norm();
        let vel_err =
            (numerical_ric.velocity_ric_km_s - analytical_ric.velocity_ric_km_s).norm();

        // Skip t=0 sample from safety: at the maneuver instant, positions
        // haven't separated yet — distance is physically meaningless
        // (consistent with monte_carlo/execution.rs).
        if idx > 0 {
            safety_pairs.push(ChiefDeputySnapshot {
                elapsed_s: elapsed,
                chief: chief_sample.state.clone(),
                deputy: deputy_sample.state.clone(),
            });
        }
        points.push(ValidationPoint {
            elapsed_s: elapsed,
            analytical_ric,
            numerical_ric,
            position_error_km: pos_err,
            velocity_error_km_s: vel_err,
        });

        // Collect eclipse samples for validation
        if let Some(ef) = earth_frame {
            let chief_orbit = state_to_orbit(&chief_sample.state);
            if let Ok((eclipse_pct, sun_eci)) = query_anise_eclipse(chief_orbit, ef, almanac) {
                eclipse_samples.push(EclipseSample {
                    elapsed_s: elapsed,
                    epoch: chief_sample.state.epoch,
                    numerical_pct: eclipse_pct,
                    anise_sun_eci_km: sun_eci,
                    chief_eci_km: chief_sample.state.position_eci_km,
                });
            }
        }
    }
    Ok(LegComparisonOutput {
        points,
        safety_pairs,
        eclipse_samples,
    })
}

/// Build eclipse validation from collected ANISE samples and analytical eclipse data.
///
/// Compares per-point analytical (Meeus) vs numerical (ANISE) Sun direction and
/// eclipse state, matches eclipse intervals, and computes aggregate statistics.
///
/// Returns `None` if there are no eclipse samples.
fn build_eclipse_validation(
    eclipse_data: &crate::types::MissionEclipseData,
    eclipse_samples: &[EclipseSample],
) -> Option<EclipseValidation> {
    if eclipse_samples.is_empty() {
        return None;
    }

    // Build per-point comparison
    let mut ev_points = Vec::with_capacity(eclipse_samples.len());
    let mut sum_sun_err = 0.0_f64;
    let mut max_sun_err = 0.0_f64;

    for sample in eclipse_samples {
        // Analytical Sun position at this epoch (Meeus Ch. 25)
        let meeus_sun = sun_position_eci_km(sample.epoch);
        let meeus_sun_dir = meeus_sun.normalize();
        let anise_sun_dir = sample.anise_sun_eci_km.normalize();
        let sun_err = meeus_sun_dir
            .dot(&anise_sun_dir)
            .clamp(-1.0, 1.0)
            .acos();

        // Analytical eclipse state at nyx chief position using Meeus Sun
        let analytical_state = compute_eclipse_state(&sample.chief_eci_km, &meeus_sun);
        let analytical_pct = match analytical_state {
            EclipseState::Sunlit => 0.0,
            EclipseState::Penumbra { shadow_fraction } => shadow_fraction * 100.0,
            EclipseState::Umbra => 100.0,
        };

        let pct_err = (analytical_pct - sample.numerical_pct).abs();
        max_sun_err = max_sun_err.max(sun_err);
        sum_sun_err += sun_err;

        ev_points.push(EclipseValidationPoint {
            elapsed_s: sample.elapsed_s,
            analytical_eclipse_pct: analytical_pct,
            numerical_eclipse_pct: sample.numerical_pct,
            eclipse_pct_error: pct_err,
            sun_direction_error_rad: sun_err,
        });
    }

    // Extract numerical intervals from ANISE eclipse samples
    let numerical_interval_samples: Vec<(Epoch, f64)> = eclipse_samples
        .iter()
        .map(|s| (s.epoch, s.numerical_pct))
        .collect();
    let numerical_intervals =
        compute_numerical_eclipse_intervals(&numerical_interval_samples);

    // Match analytical vs numerical intervals
    let (comparisons, unmatched) =
        match_eclipse_intervals(&eclipse_data.summary, &numerical_intervals);

    // Compute aggregate statistics
    let n = ev_points.len();
    let mean_sun_err = if n == 0 {
        0.0
    } else {
        sum_sun_err / f64::from(u32::try_from(n).ok()?)
    };
    let max_timing = comparisons
        .iter()
        .flat_map(|c| [c.entry_error_s.abs(), c.exit_error_s.abs()])
        .fold(0.0_f64, f64::max);
    let mean_timing = if comparisons.is_empty() {
        0.0
    } else {
        let sum: f64 = comparisons
            .iter()
            .flat_map(|c| [c.entry_error_s.abs(), c.exit_error_s.abs()])
            .sum();
        let denom = 2.0 * f64::from(u32::try_from(comparisons.len()).ok()?);
        sum / denom
    };
    let matched_count = comparisons.len();

    Some(EclipseValidation {
        points: ev_points,
        interval_comparisons: comparisons,
        max_sun_direction_error_rad: max_sun_err,
        mean_sun_direction_error_rad: mean_sun_err,
        max_timing_error_s: max_timing,
        mean_timing_error_s: mean_timing,
        analytical_interval_count: eclipse_data.summary.intervals.len(),
        numerical_interval_count: numerical_intervals.len(),
        matched_interval_count: matched_count,
        unmatched_interval_count: unmatched,
    })
}

/// Propagate chief and deputy through a single leg in parallel via rayon.
///
/// Applies the leg's departure impulse to the deputy, then propagates both
/// vehicles through nyx full-physics dynamics concurrently using [`rayon::join`].
///
/// # Invariants
/// - `chief_state` and `deputy_state` must represent valid orbits.
/// - `almanac` must contain required frames and force models.
///
/// # Errors
/// - [`ValidationError`] if impulse application, dynamics setup, or propagation fails.
fn propagate_leg_parallel(
    chief_state: &StateVector,
    deputy_state: &StateVector,
    leg: &ManeuverLeg,
    samples_per_leg: u32,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
) -> Result<(Vec<TimedState>, Vec<TimedState>), ValidationError> {
    // Build dynamics once; clone is cheap (Arc ref-count bumps).
    let dynamics = build_full_physics_dynamics(almanac)?;

    let (chief_result, deputy_result) = rayon::join(
        || -> Result<Vec<TimedState>, ValidationError> {
            Ok(nyx_propagate_segment(
                chief_state,
                leg.tof_s,
                samples_per_leg,
                chief_config,
                dynamics.clone(),
                almanac,
            )?)
        },
        || -> Result<Vec<TimedState>, ValidationError> {
            let deputy_post_burn =
                apply_impulse(deputy_state, chief_state, &leg.departure_maneuver.dv_ric_km_s)?;
            Ok(nyx_propagate_segment(
                &deputy_post_burn,
                leg.tof_s,
                samples_per_leg,
                deputy_config,
                dynamics.clone(),
                almanac,
            )?)
        },
    );
    Ok((chief_result?, deputy_result?))
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
/// # Invariants
/// - `mission.legs` must be non-empty (at least one maneuver leg)
/// - `chief_initial` and `deputy_initial` must be valid bound ECI states at mission start epoch
/// - `almanac` must contain Earth frame data (`IAU_EARTH`) and planetary ephemerides
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
    let estimated_total_samples = usize::try_from(samples_per_leg).unwrap_or(50)
        * mission.legs.len();
    let mut safety_pairs: Vec<ChiefDeputySnapshot> =
        Vec::with_capacity(estimated_total_samples);
    let mut eclipse_samples: Vec<EclipseSample> =
        Vec::with_capacity(estimated_total_samples);

    // Get Earth frame with radius info for ANISE eclipse queries
    let earth_frame = if mission.eclipse.is_some() {
        Some(
            almanac
                .frame_info(ANISE_EARTH_J2000)
                .map_err(|e| {
                    ValidationError::NyxBridge(Box::new(NyxBridgeError::FrameLookup {
                        source: e,
                    }))
                })?,
        )
    } else {
        None
    };

    for leg in &mission.legs {
        let (chief_results, deputy_results) = propagate_leg_parallel(
            &chief_state,
            &deputy_state,
            leg,
            samples_per_leg,
            chief_config,
            deputy_config,
            almanac,
        )?;

        // Build comparison points for this leg
        let leg_output = build_leg_comparison_points(
            &chief_results,
            &deputy_results,
            &leg.trajectory,
            cumulative_time,
            earth_frame,
            almanac,
        )?;
        safety_pairs.extend(leg_output.safety_pairs);
        eclipse_samples.extend(leg_output.eclipse_samples);
        leg_points.push(leg_output.points);

        // Advance states for next leg
        chief_state = chief_results
            .last()
            .ok_or(ValidationError::EmptyTrajectory)?
            .state
            .clone();
        let deputy_coast_end = deputy_results
            .last()
            .ok_or(ValidationError::EmptyTrajectory)?
            .state
            .clone();
        deputy_state =
            apply_impulse(&deputy_coast_end, &chief_state, &leg.arrival_maneuver.dv_ric_km_s)?;
        cumulative_time += leg.tof_s;
    }

    // Compute safety from nyx trajectory
    let nyx_safety_states = build_nyx_safety_states(&safety_pairs)?;
    let numerical_safety = analyze_trajectory_safety(&nyx_safety_states)?;

    // Compute eclipse validation if eclipse data is present
    let eclipse_validation = mission.eclipse.as_ref().and_then(|eclipse_data| {
        build_eclipse_validation(eclipse_data, &eclipse_samples)
    });

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
        eclipse_validation,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::propagation::nyx_bridge;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{
        iss_like_elements, test_epoch, DMF_RATE_NONZERO_LOWER_BOUND, DMF_RATE_UPPER_BOUND,
    };
    use crate::mission::types::ValidationPoint;
    use crate::types::{RICState, SpacecraftConfig};

    // =========================================================================
    // Mission Validation Pipeline Tests
    // =========================================================================

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
            (max_pos - 3.0).abs() < EXACT_ARITHMETIC_TOL,
            "max_pos = {max_pos}, expected 3.0"
        );
        assert!(
            (mean_pos - 2.0).abs() < EXACT_ARITHMETIC_TOL,
            "mean_pos = {mean_pos}, expected 2.0"
        );
        // rms = sqrt((1 + 9 + 4) / 3) = sqrt(14/3) ≈ 2.160
        let expected_rms = (14.0_f64 / 3.0).sqrt();
        assert!(
            (rms_pos - expected_rms).abs() < RMS_COMPUTATION_TOL,
            "rms_pos = {rms_pos}, expected {expected_rms}"
        );
        assert!(
            (max_vel - 0.03).abs() < EXACT_ARITHMETIC_TOL,
            "max_vel = {max_vel}, expected 0.03"
        );

        // Empty input returns zeros
        let (mp, meanp, rmsp, mv) = super::compute_report_statistics(&[]);
        assert_eq!(mp, 0.0, "empty max_pos");
        assert_eq!(meanp, 0.0, "empty mean_pos");
        assert_eq!(rmsp, 0.0, "empty rms_pos");
        assert_eq!(mv, 0.0, "empty max_vel");
    }


    // =========================================================================
    // Full-Physics Integration Tests
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

    /// Tolerance for exact-arithmetic statistics (max, mean, sum).
    /// These involve only addition/comparison of representable f64 values,
    /// so agreement to machine epsilon is expected.
    const EXACT_ARITHMETIC_TOL: f64 = 1e-15;

    /// Tolerance for RMS computation.
    /// `sqrt` introduces ~1 ULP of floating-point error beyond the exact sum,
    /// so we allow a slightly wider tolerance than exact arithmetic.
    const RMS_COMPUTATION_TOL: f64 = 1e-12;

    /// Guard threshold for improvement ratio computation (km).
    /// When the J2-only error is below this threshold, the improvement ratio
    /// is numerically meaningless (division by near-zero). Skip the diagnostic.
    const IMPROVEMENT_RATIO_GUARD_KM: f64 = 1e-10;

    /// Below this threshold (km), e/i separation values are too small
    /// for a meaningful relative-error comparison.
    const SAFETY_EI_NEAR_ZERO_KM: f64 = 1e-6;

    // Named tolerance constants for eclipse validation tests

    /// Serde roundtrip precision for f64 fields. JSON serialization
    /// preserves full f64 precision; 1e-15 is at machine epsilon.
    const SERDE_ROUNDTRIP_TOL: f64 = 1e-15;

    /// Eclipse interval matching: entry/exit/duration error for
    /// exact-match test cases. 1e-10 s for identical intervals.
    const INTERVAL_EXACT_MATCH_TOL_S: f64 = 1e-10;

    /// Eclipse numerical interval duration tolerance.
    /// Sample-spaced intervals have ±1 sample quantization;
    /// at 1s spacing, 2s covers worst case.
    const INTERVAL_DURATION_TOL_S: f64 = 2.0;

    /// Maximum expected Sun direction angular error (Meeus vs ANISE DE440s).
    /// Meeus Ch. 25 with precession correction gives ~0.005° at 2024 epoch.
    /// 0.02° (3.5e-4 rad) provides margin for nutation/perturbation residuals.
    const SUN_DIRECTION_VALIDATION_TOL_RAD: f64 = 3.5e-4;

    /// Maximum expected Moon direction angular error (Meeus vs ANISE DE440s).
    /// Meeus Ch. 47 truncated series gives ~0.007° with precession correction.
    /// 1.0° (0.0175 rad) provides generous margin for the truncated series.
    const MOON_DIRECTION_VALIDATION_TOL_RAD: f64 = 0.0175;

    /// Maximum eclipse entry/exit timing error (seconds).
    /// With linear interpolation of eclipse boundaries between samples,
    /// the error is dominated by the non-linearity of the shadow geometry
    /// rather than sample-interval quantization.
    const ECLIPSE_TIMING_VALIDATION_TOL_S: f64 = 120.0;

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
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

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

        let period = chief_ke.period().unwrap();
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

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
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
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

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

        let period = chief_ke.period().unwrap();
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

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
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
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

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
        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let drag = nyx_bridge::extract_dmf_rates(
            &chief_sv, &deputy_sv, &chief_config, &deputy_config, &almanac,
        )
        .expect("DMF extraction should succeed");

        eprintln!(
            "DMF rates: da_dot={:.4e}, dex_dot={:.4e}, dey_dot={:.4e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot,
        );

        // Sanity: da_dot should be nonzero, negative, and physically reasonable
        assert!(
            drag.da_dot.abs() > DMF_RATE_NONZERO_LOWER_BOUND,
            "da_dot should be nonzero, got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot < 0.0,
            "da_dot should be negative (deputy decays faster), got {:.2e}",
            drag.da_dot,
        );
        assert!(
            drag.da_dot.abs() < DMF_RATE_UPPER_BOUND,
            "da_dot = {:.2e} seems unreasonably large",
            drag.da_dot,
        );

        // Step 2: Plan missions with both propagators
        let period = chief_ke.period().unwrap();
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
        if j2_report.max_position_error_km > IMPROVEMENT_RATIO_GUARD_KM {
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
        use crate::mission::config::{MissionConfig, SafetyConfig};
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

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

        let period = chief_ke.period().unwrap();
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

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
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
            analytical.operational.min_rc_separation_km, numerical.operational.min_rc_separation_km,
        );
        eprintln!(
            "  3D distance:     {:.4} km vs {:.4} km",
            analytical.operational.min_distance_3d_km, numerical.operational.min_distance_3d_km,
        );
        eprintln!(
            "  e/i separation:  {:.4} km vs {:.4} km",
            analytical.passive.min_ei_separation_km, numerical.passive.min_ei_separation_km,
        );
        eprintln!(
            "  |δe|:            {:.6} vs {:.6}",
            analytical.passive.de_magnitude, numerical.passive.de_magnitude,
        );
        eprintln!(
            "  |δi|:            {:.6} vs {:.6}",
            analytical.passive.di_magnitude, numerical.passive.di_magnitude,
        );
        eprintln!(
            "  e/i phase angle: {:.4} rad vs {:.4} rad",
            analytical.passive.ei_phase_angle_rad, numerical.passive.ei_phase_angle_rad,
        );

        // R/C separation: relative agreement within 50%
        // When both values are near zero (V-bar configuration), relative comparison
        // is meaningless — skip it.
        let rc_ref = analytical.operational.min_rc_separation_km.max(numerical.operational.min_rc_separation_km);
        if rc_ref > SAFETY_RC_NEAR_ZERO_KM {
            let rc_rel_err =
                (analytical.operational.min_rc_separation_km - numerical.operational.min_rc_separation_km).abs() / rc_ref;
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
            (analytical.operational.min_distance_3d_km - numerical.operational.min_distance_3d_km).abs();
        eprintln!("  3D distance absolute error: {dist_3d_err:.4} km");
        assert!(
            dist_3d_err < SAFETY_3D_ABSOLUTE_TOL_KM,
            "3D distance error = {dist_3d_err:.4} km (expected < {SAFETY_3D_ABSOLUTE_TOL_KM})",
        );

        // e/i separation: relative agreement within 50%
        let ei_ref = analytical.passive.min_ei_separation_km.max(numerical.passive.min_ei_separation_km);
        if ei_ref > SAFETY_EI_NEAR_ZERO_KM {
            let ei_rel_err =
                (analytical.passive.min_ei_separation_km - numerical.passive.min_ei_separation_km).abs() / ei_ref;
            eprintln!("  e/i relative error: {ei_rel_err:.2}");
            assert!(
                ei_rel_err < SAFETY_EI_RELATIVE_TOL,
                "e/i separation relative error = {ei_rel_err:.2} (expected < {SAFETY_EI_RELATIVE_TOL})",
            );
        }
    }

    #[test]
    fn eclipse_validation_serde_roundtrip() {
        use crate::mission::types::EclipseValidationPoint;

        let point = EclipseValidationPoint {
            elapsed_s: 100.0,
            analytical_eclipse_pct: 0.0,
            numerical_eclipse_pct: 0.5,
            eclipse_pct_error: 0.5,
            sun_direction_error_rad: 1e-4,
        };
        let json = serde_json::to_string(&point).unwrap();
        let recovered: EclipseValidationPoint = serde_json::from_str(&json).unwrap();
        assert!(
            (recovered.elapsed_s - 100.0).abs() < SERDE_ROUNDTRIP_TOL,
            "elapsed_s roundtrip"
        );
        assert!(
            (recovered.eclipse_pct_error - 0.5).abs() < SERDE_ROUNDTRIP_TOL,
            "eclipse_pct_error roundtrip"
        );
    }

    #[test]
    fn eclipse_validation_tolerances_are_reasonable() {
        use crate::constants::ECLIPSE_PERCENTAGE_AGREEMENT_TOL;
        // Sun direction: 0.02° = 3.5e-4 rad (Meeus with precession correction)
        assert!(SUN_DIRECTION_VALIDATION_TOL_RAD > 0.0);
        assert!(SUN_DIRECTION_VALIDATION_TOL_RAD < 0.01); // < 0.57°
        // Moon direction: 1.0° = 0.0175 rad (Meeus truncated ~0.5°)
        assert!(MOON_DIRECTION_VALIDATION_TOL_RAD > SUN_DIRECTION_VALIDATION_TOL_RAD);
        assert!(MOON_DIRECTION_VALIDATION_TOL_RAD < 0.1); // < 5.7°
        // Eclipse timing: <120s (dominated by sample-interval quantization)
        assert!(ECLIPSE_TIMING_VALIDATION_TOL_S > 0.0);
        assert!(ECLIPSE_TIMING_VALIDATION_TOL_S < 300.0);
        // Eclipse percentage agreement: <1% difference
        assert!(ECLIPSE_PERCENTAGE_AGREEMENT_TOL > 0.0);
        assert!(ECLIPSE_PERCENTAGE_AGREEMENT_TOL < 10.0);
    }

    // =========================================================================
    // Eclipse Interval Matching Tests
    // =========================================================================

    /// Helper to create an EclipseInterval for testing.
    fn make_eclipse_interval(start_s: f64, end_s: f64) -> crate::types::EclipseInterval {
        use crate::types::{EclipseInterval, EclipseState};
        let base = crate::test_helpers::test_epoch();
        EclipseInterval {
            start: base + hifitime::Duration::from_seconds(start_s),
            end: base + hifitime::Duration::from_seconds(end_s),
            duration_s: end_s - start_s,
            state: EclipseState::Umbra,
        }
    }

    /// Helper to create an EclipseSummary from intervals.
    fn make_eclipse_summary(
        intervals: Vec<crate::types::EclipseInterval>,
    ) -> crate::types::EclipseSummary {
        let total = intervals.iter().map(|i| i.duration_s).sum::<f64>();
        crate::types::EclipseSummary {
            intervals,
            total_shadow_duration_s: total,
            time_in_shadow_fraction: 0.0,
            max_shadow_duration_s: 0.0,
        }
    }

    #[test]
    fn eclipse_interval_matching_exact() {
        let intervals = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
        ];
        let summary = make_eclipse_summary(intervals.clone());

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &intervals);

        assert_eq!(comparisons.len(), 2, "should match both intervals");
        assert_eq!(unmatched, 0, "no unmatched intervals");
        for c in &comparisons {
            assert!(
                c.entry_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S,
                "entry error should be zero"
            );
            assert!(c.exit_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S, "exit error should be zero");
            assert!(
                c.duration_error_s.abs() < INTERVAL_EXACT_MATCH_TOL_S,
                "duration error should be zero"
            );
        }
    }

    #[test]
    fn eclipse_interval_matching_shifted() {
        let analytical = vec![make_eclipse_interval(100.0, 200.0)];
        let numerical = vec![make_eclipse_interval(102.0, 198.0)];
        let summary = make_eclipse_summary(analytical);

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &numerical);

        assert_eq!(comparisons.len(), 1);
        assert_eq!(unmatched, 0);
        // analytical starts 2s before numerical -> entry_error = 100 - 102 = -2.0
        assert!(
            (comparisons[0].entry_error_s - (-2.0)).abs() < INTERVAL_EXACT_MATCH_TOL_S,
            "entry_error = {}, expected -2.0",
            comparisons[0].entry_error_s
        );
        // analytical ends 2s after numerical -> exit_error = 200 - 198 = 2.0
        assert!(
            (comparisons[0].exit_error_s - 2.0).abs() < INTERVAL_EXACT_MATCH_TOL_S,
            "exit_error = {}, expected 2.0",
            comparisons[0].exit_error_s
        );
    }

    #[test]
    fn eclipse_interval_unmatched() {
        let analytical = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
            make_eclipse_interval(900.0, 1000.0),
        ];
        let numerical = vec![
            make_eclipse_interval(100.0, 200.0),
            make_eclipse_interval(500.0, 600.0),
        ];
        let summary = make_eclipse_summary(analytical);

        let (comparisons, unmatched) = super::match_eclipse_intervals(&summary, &numerical);

        assert_eq!(comparisons.len(), 2, "should match 2 of 3");
        assert!(unmatched > 0, "should have unmatched intervals");
    }

    #[test]
    fn eclipse_comparison_point_construction() {
        use crate::mission::types::EclipseValidationPoint;

        let point = EclipseValidationPoint {
            elapsed_s: 100.0,
            analytical_eclipse_pct: 0.0,
            numerical_eclipse_pct: 50.0,
            eclipse_pct_error: 50.0,
            sun_direction_error_rad: 1e-4,
        };
        assert!((point.eclipse_pct_error - 50.0).abs() < SERDE_ROUNDTRIP_TOL);
        assert!((point.sun_direction_error_rad - 1e-4).abs() < SERDE_ROUNDTRIP_TOL);
    }

    #[test]
    fn compute_numerical_intervals_from_samples() {
        let base = crate::test_helpers::test_epoch();
        let samples: Vec<(hifitime::Epoch, f64)> = (0..100)
            .map(|i| {
                let t = f64::from(i);
                let epoch = base + hifitime::Duration::from_seconds(t);
                // Eclipse from t=20 to t=40 and t=60 to t=80
                let pct = if (20.0..=40.0).contains(&t) || (60.0..=80.0).contains(&t) {
                    100.0
                } else {
                    0.0
                };
                (epoch, pct)
            })
            .collect();

        let intervals = super::compute_numerical_eclipse_intervals(&samples);

        assert_eq!(
            intervals.len(),
            2,
            "should detect 2 eclipse intervals, got {}",
            intervals.len()
        );
        // First interval should be ~20s duration
        assert!(
            (intervals[0].duration_s - 20.0).abs() < INTERVAL_DURATION_TOL_S,
            "first interval duration = {}, expected ~20",
            intervals[0].duration_s
        );
    }

    /// Interpolation accuracy tolerance for eclipse boundary tests.
    /// Linear interpolation of a linear ramp should be exact to floating-point
    /// precision; 1e-9 s provides margin for epoch arithmetic rounding.
    const INTERPOLATION_ACCURACY_TOL_S: f64 = 1e-9;

    /// Verify that eclipse boundary interpolation accurately locates the
    /// threshold crossing for a linear ramp in eclipse percentage.
    ///
    /// Constructs samples with known linear transitions:
    /// - t=0..90: sunlit (0%)
    /// - t=90..110: linear ramp 0% → 100% (threshold 1% crossed at t≈90.2)
    /// - t=110..190: full umbra (100%)
    /// - t=190..210: linear ramp 100% → 0% (threshold 1% crossed at t≈209.8)
    /// - t=210..300: sunlit (0%)
    ///
    /// With linear interpolation, the entry/exit epochs should be accurate
    /// to floating-point precision (not quantized to sample boundaries).
    #[test]
    fn eclipse_boundary_interpolation_accuracy() {
        let base = crate::test_helpers::test_epoch();

        // 10s sample spacing, 31 samples from t=0 to t=300
        let samples: Vec<(hifitime::Epoch, f64)> = (0..=30)
            .map(|i| {
                let t = f64::from(i) * 10.0;
                let epoch = base + hifitime::Duration::from_seconds(t);
                let pct = if t <= 90.0 {
                    0.0
                } else if t <= 110.0 {
                    // Linear ramp: 0% at t=90, 100% at t=110
                    (t - 90.0) / 20.0 * 100.0
                } else if t <= 190.0 {
                    100.0
                } else if t <= 210.0 {
                    // Linear ramp: 100% at t=190, 0% at t=210
                    (210.0 - t) / 20.0 * 100.0
                } else {
                    0.0
                };
                (epoch, pct)
            })
            .collect();

        let intervals = super::compute_numerical_eclipse_intervals(&samples);

        assert_eq!(intervals.len(), 1, "should detect 1 eclipse interval");

        // Entry: threshold (1%) crossing on 0%→50% ramp between t=90 and t=100
        // Interpolation: t_cross = 90 + 10 × (1 - 0) / (50 - 0) = 90.2
        let expected_entry_s = 90.2;
        let actual_entry_s = (intervals[0].start - base).to_seconds();
        let entry_err = (actual_entry_s - expected_entry_s).abs();
        assert!(
            entry_err < INTERPOLATION_ACCURACY_TOL_S,
            "entry at {actual_entry_s:.6}s, expected {expected_entry_s:.6}s, error {entry_err:.2e}s"
        );

        // Exit: threshold (1%) crossing on 50%→0% ramp between t=200 and t=210
        // Interpolation: t_cross = 200 + 10 × (1 - 50) / (0 - 50) = 200 + 10 × 49/50 = 209.8
        let expected_exit_s = 209.8;
        let actual_exit_s = (intervals[0].end - base).to_seconds();
        let exit_err = (actual_exit_s - expected_exit_s).abs();
        assert!(
            exit_err < INTERPOLATION_ACCURACY_TOL_S,
            "exit at {actual_exit_s:.6}s, expected {expected_exit_s:.6}s, error {exit_err:.2e}s"
        );

        // Duration should match
        let expected_duration = expected_exit_s - expected_entry_s;
        let duration_err = (intervals[0].duration_s - expected_duration).abs();
        assert!(
            duration_err < INTERPOLATION_ACCURACY_TOL_S,
            "duration {:.6}s, expected {expected_duration:.6}s, error {duration_err:.2e}s",
            intervals[0].duration_s,
        );
    }

    // =========================================================================
    // Eclipse Validation Integration Tests (require MetaAlmanac)
    // =========================================================================

    /// Full multi-waypoint mission with eclipse validation.
    /// Verifies that `validate_mission_nyx()` produces eclipse validation data
    /// and that Sun direction and timing errors are within tolerances.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_eclipse_full_mission() {
        use crate::mission::waypoints::plan_waypoint_mission;
        use crate::test_helpers::deputy_from_roe;
        use crate::mission::config::MissionConfig;
        use crate::mission::types::Waypoint;
        use crate::types::{DepartureState, QuasiNonsingularROE};

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

        let period = chief_ke.period().unwrap();
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

        // Verify eclipse data was computed
        assert!(
            mission.eclipse.is_some(),
            "mission should have eclipse data"
        );

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
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

        // Eclipse validation should be present
        let ev = report
            .eclipse_validation
            .expect("eclipse validation should be present");

        eprintln!("Eclipse validation results:");
        eprintln!(
            "  Sun direction error: max {:.6} deg mean {:.6} deg",
            ev.max_sun_direction_error_rad.to_degrees(),
            ev.mean_sun_direction_error_rad.to_degrees(),
        );
        eprintln!(
            "  Intervals: {} analytical, {} numerical, {} matched, {} unmatched",
            ev.analytical_interval_count,
            ev.numerical_interval_count,
            ev.matched_interval_count,
            ev.unmatched_interval_count,
        );
        if !ev.interval_comparisons.is_empty() {
            eprintln!(
                "  Timing error: max {:.2} s, mean {:.2} s",
                ev.max_timing_error_s, ev.mean_timing_error_s,
            );
        }

        // Sun direction should be within Meeus accuracy
        assert!(
            ev.max_sun_direction_error_rad < SUN_DIRECTION_VALIDATION_TOL_RAD,
            "max Sun direction error = {:.6} deg (expected < {:.4} deg)",
            ev.max_sun_direction_error_rad.to_degrees(),
            SUN_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );

        // Timing error should be within tolerance (if intervals were matched)
        if !ev.interval_comparisons.is_empty() {
            assert!(
                ev.max_timing_error_s < ECLIPSE_TIMING_VALIDATION_TOL_S,
                "max timing error = {:.2} s (expected < {:.1} s)",
                ev.max_timing_error_s,
                ECLIPSE_TIMING_VALIDATION_TOL_S,
            );
        }

        // Should have some eclipse points
        assert!(
            !ev.points.is_empty(),
            "should have eclipse validation points"
        );
    }

    /// Meeus Sun position vs ANISE DE440s over 24 hours.
    /// Validates that the analytical Sun ephemeris (Meeus Ch. 25) agrees
    /// with the high-fidelity ANISE ephemeris within the stated accuracy.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_sun_direction_accuracy() {
        use crate::elements::eclipse::sun_position_eci_km;
        use anise::constants::frames::{SUN_J2000, EARTH_J2000 as ANISE_EARTH_J2000};

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let base_epoch = test_epoch();

        let mut max_err = 0.0_f64;
        let mut sum_err = 0.0_f64;
        let n_samples = 864; // 24 hours at 100s intervals

        for i in 0..n_samples {
            let dt = f64::from(i) * 100.0;
            let epoch = base_epoch + hifitime::Duration::from_seconds(dt);

            // Analytical (Meeus)
            let meeus_sun = sun_position_eci_km(epoch);
            let meeus_dir = meeus_sun.normalize();

            // Numerical (ANISE DE440s)
            let sun_state = almanac
                .translate(SUN_J2000, ANISE_EARTH_J2000, epoch, None)
                .expect("Sun translate should succeed");
            let anise_dir = Vector3::new(
                sun_state.radius_km.x,
                sun_state.radius_km.y,
                sun_state.radius_km.z,
            )
            .normalize();

            let err = meeus_dir.dot(&anise_dir).clamp(-1.0, 1.0).acos();
            max_err = max_err.max(err);
            sum_err += err;
        }

        let mean_err = sum_err / f64::from(n_samples);
        eprintln!(
            "Sun direction accuracy (24h, 100s intervals):"
        );
        eprintln!("  max: {:.6} deg", max_err.to_degrees());
        eprintln!("  mean: {:.6} deg", mean_err.to_degrees());

        assert!(
            max_err < SUN_DIRECTION_VALIDATION_TOL_RAD,
            "max Sun direction error = {:.6} deg (expected < {:.4} deg)",
            max_err.to_degrees(),
            SUN_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );
    }

    /// Meeus Moon position vs ANISE DE440s over 24 hours.
    /// Validates that the analytical Moon ephemeris (Meeus Ch. 47, truncated)
    /// agrees with the high-fidelity ANISE ephemeris within the stated accuracy.
    #[test]
    #[ignore] // Requires MetaAlmanac (network on first run)
    fn validate_moon_direction_accuracy() {
        use crate::elements::eclipse::moon_position_eci_km;
        use anise::constants::frames::{MOON_J2000, EARTH_J2000 as ANISE_EARTH_J2000};

        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        let base_epoch = test_epoch();

        let mut max_err = 0.0_f64;
        let mut sum_err = 0.0_f64;
        let n_samples = 864; // 24 hours at 100s intervals

        for i in 0..n_samples {
            let dt = f64::from(i) * 100.0;
            let epoch = base_epoch + hifitime::Duration::from_seconds(dt);

            // Analytical (Meeus Ch. 47 truncated)
            let meeus_moon = moon_position_eci_km(epoch);
            let meeus_dir = meeus_moon.normalize();

            // Numerical (ANISE DE440s)
            let moon_state = almanac
                .translate(MOON_J2000, ANISE_EARTH_J2000, epoch, None)
                .expect("Moon translate should succeed");
            let anise_dir = Vector3::new(
                moon_state.radius_km.x,
                moon_state.radius_km.y,
                moon_state.radius_km.z,
            )
            .normalize();

            let err = meeus_dir.dot(&anise_dir).clamp(-1.0, 1.0).acos();
            max_err = max_err.max(err);
            sum_err += err;
        }

        let mean_err = sum_err / f64::from(n_samples);
        eprintln!(
            "Moon direction accuracy (24h, 100s intervals):"
        );
        eprintln!("  max: {:.6} deg", max_err.to_degrees());
        eprintln!("  mean: {:.6} deg", mean_err.to_degrees());

        assert!(
            max_err < MOON_DIRECTION_VALIDATION_TOL_RAD,
            "max Moon direction error = {:.6} deg (expected < {:.4} deg)",
            max_err.to_degrees(),
            MOON_DIRECTION_VALIDATION_TOL_RAD.to_degrees(),
        );
    }
}

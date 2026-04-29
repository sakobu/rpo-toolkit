//! Lambert-driven transfer planning + visualization-arc densification.
//!
//! Promoted from `rpo-nyx::pipeline` after the Izzo solver moved into
//! `rpo-core`. The functions here are now WASM-eligible — no nyx-space,
//! anise, rayon, or filesystem deps.

use nalgebra::Vector3;

use crate::constants::{INC_TOL, LAMBERT_ARC_SAMPLES, TWO_PI};
use crate::elements::keplerian_conversions::{keplerian_to_state, ConversionError};
use crate::elements::state_to_keplerian;
use crate::mission::config::ProximityConfig;
use crate::mission::errors::MissionError;
use crate::mission::formation::{EnrichmentSuggestion, SafetyRequirements};
use crate::mission::planning::{classify_separation, perch_to_roe};
use crate::mission::types::{MissionPhase, MissionPlan, PerchGeometry};
use crate::types::elements::KeplerError;
use crate::propagation::keplerian::{propagate_keplerian, propagate_keplerian_from_elements};
use crate::propagation::lambert::{solve_lambert_with_config, LambertConfig, LambertTransfer};
use crate::propagation::propagator::PropagationError;
use crate::types::{KeplerianElements, QuasiNonsingularROE, StateVector};

use super::errors::PipelineError;
use super::execute::{execute_mission_from_transfer, suggest_enrichment_from_parts};
use super::types::{
    PipelineInput, PipelineOutput, TransferComputationInput, TransferResult,
};

/// Plan a complete mission with Lambert transfer support.
///
/// If the spacecraft are in proximity, computes the perch ROE directly. If
/// far-field, advances the chief to the arrival epoch, derives a target
/// Keplerian orbit from the perch geometry, and solves the Lambert problem
/// from the deputy to that target.
///
/// # Caller must ensure
///
/// - Both states represent bound orbits (`e < 1`, `a > 0`); violations
///   surface from downstream [`state_to_keplerian`] /
///   [`KeplerianElements::mean_motion`] as
///   [`ConversionError::UnboundOrbit`].
/// - `lambert_tof_s > 0` when used in the far-field regime; non-positive
///   TOF is rejected by [`solve_lambert_with_config`] as
///   [`crate::propagation::lambert::LambertError::NonPositiveTimeOfFlight`].
/// - For multi-rev (`lambert_config.revolutions > 0`), `lambert_tof_s` is
///   long enough to admit the requested rev count (rejected at the Lambert
///   layer as [`crate::propagation::lambert::LambertError::NoSolutionForRevolutions`]).
///
/// # Errors
///
/// Returns [`MissionError`] on classification, propagation, perch
/// conversion, or Lambert failure. Lambert errors propagate via
/// [`MissionError::Lambert`] (auto-`#[from]`).
pub fn plan_mission(
    chief: &StateVector,
    deputy: &StateVector,
    perch: &PerchGeometry,
    config: ProximityConfig,
    lambert_tof_s: f64,
    lambert_config: &LambertConfig,
) -> Result<MissionPlan, MissionError> {
    let phase = classify_separation(chief, deputy, &config)?;

    match phase {
        MissionPhase::Proximity { ref chief_elements, .. } => {
            let chief_at_arrival = *chief_elements;
            let perch_roe = perch_to_roe(perch, chief_elements)?;

            Ok(MissionPlan {
                phase,
                transfer: None,
                perch_roe,
                chief_at_arrival,
            })
        }
        MissionPhase::FarField { ref chief_elements, .. } => {
            let chief_ke = *chief_elements;

            // Advance chief mean anomaly to arrival epoch (two-body).
            let n = chief_ke.mean_motion()?;
            let chief_ke_arrival = KeplerianElements {
                mean_anomaly_rad: (chief_ke.mean_anomaly_rad + n * lambert_tof_s)
                    .rem_euclid(TWO_PI),
                ..chief_ke
            };

            let perch_roe = perch_to_roe(perch, &chief_ke_arrival)?;

            // Convert perch ROE to a target Keplerian orbit for Lambert.
            let target_ke = perch_roe_to_keplerian(&perch_roe, &chief_ke_arrival);
            let arrival_epoch =
                deputy.epoch + hifitime::Duration::from_seconds(lambert_tof_s);
            let target_state = keplerian_to_state(&target_ke, arrival_epoch)?;

            // Solve Lambert: deputy → perch.
            let transfer = solve_lambert_with_config(deputy, &target_state, lambert_config)?;

            Ok(MissionPlan {
                phase,
                transfer: Some(transfer),
                perch_roe,
                chief_at_arrival: chief_ke_arrival,
            })
        }
    }
}

/// Convert a perch ROE state back to absolute Keplerian elements.
///
/// Inverts the QNS ROE definition (Koenig Eq. 2 / D'Amico Eq. 2.2).
///
/// # Singularities
///
/// For near-equatorial chief orbits (`|sin(i_c)| < INC_TOL`) the RAAN
/// inversion `δΩ = δiy / sin(i_c)` is undefined; this function falls back
/// to `chief.raan_rad`, which **silently drops the deputy's `δiy`
/// component**. The lossy fallback is acceptable for the targeting
/// pipeline because near-equatorial chiefs are explicitly out of the
/// supported regime (mission planner targets LEO+); the fallback produces
/// a valid Keplerian state instead of NaN, which keeps Lambert inputs
/// well-posed.
#[allow(clippy::similar_names)]
fn perch_roe_to_keplerian(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
) -> KeplerianElements {
    let a = chief.a_km * (1.0 + roe.da);
    let i = chief.i_rad + roe.dix;

    let (sin_aop, cos_aop) = chief.aop_rad.sin_cos();
    let ecc_x_chief = chief.e * cos_aop;
    let ecc_y_chief = chief.e * sin_aop;
    let ecc_x_dep = ecc_x_chief + roe.dex;
    let ecc_y_dep = ecc_y_chief + roe.dey;
    let e = (ecc_x_dep * ecc_x_dep + ecc_y_dep * ecc_y_dep).sqrt();
    let aop = ecc_y_dep.atan2(ecc_x_dep).rem_euclid(TWO_PI);

    let raan = if chief.i_rad.sin().abs() > INC_TOL {
        chief.raan_rad + roe.diy / chief.i_rad.sin()
    } else {
        chief.raan_rad
    };

    let d_raan = raan - chief.raan_rad;
    let lambda_c = chief.mean_anomaly_rad + chief.aop_rad;
    let lambda_d = roe.dlambda + lambda_c - d_raan * chief.i_rad.cos();
    let mean_anomaly = (lambda_d - aop).rem_euclid(TWO_PI);

    KeplerianElements {
        a_km: a,
        e,
        i_rad: i,
        raan_rad: raan,
        aop_rad: aop,
        mean_anomaly_rad: mean_anomaly,
    }
}

/// Classify separation, solve Lambert if far-field, compute perch ECI states.
///
/// First phase of the mission pipeline. Promoted from `rpo-nyx` since the
/// Lambert solver now lives in `rpo-core`.
///
/// # Errors
///
/// Returns [`PipelineError`] if classification, Lambert solving, propagation,
/// or arc densification fails, or if the chief trajectory is empty after
/// Lambert propagation.
pub fn compute_transfer(
    input: &TransferComputationInput,
) -> Result<TransferResult, PipelineError> {
    let plan = plan_mission(
        &input.chief,
        &input.deputy,
        &input.perch,
        input.proximity,
        input.lambert_tof_s,
        &input.lambert_config,
    )?;

    let lambert_dv_km_s = plan.transfer.as_ref().map_or(0.0, |t| t.total_dv_km_s);

    let outputs = if let Some(ref transfer) = plan.transfer {
        build_far_field_outputs(input, transfer)?
    } else {
        FarFieldOutputs {
            chief_at_arrival: input.chief,
            deputy_at_perch: input.deputy,
            arrival_epoch: input.chief.epoch,
            arc_samples_eci_km: Vec::new(),
            arc_sampling_error: None,
        }
    };

    Ok(TransferResult {
        plan,
        perch_chief: outputs.chief_at_arrival,
        perch_deputy: outputs.deputy_at_perch,
        arrival_epoch: outputs.arrival_epoch,
        lambert_dv_km_s,
        arc_samples_eci_km: outputs.arc_samples_eci_km,
        arc_sampling_error: outputs.arc_sampling_error,
    })
}

/// Bundle of computed values from [`build_far_field_outputs`]. Replaces a
/// 5-tuple return that triggered `clippy::type_complexity`.
struct FarFieldOutputs {
    chief_at_arrival: StateVector,
    deputy_at_perch: StateVector,
    arrival_epoch: hifitime::Epoch,
    arc_samples_eci_km: Vec<Vector3<f64>>,
    arc_sampling_error: Option<ArcDensificationError>,
}

/// Far-field outputs: chief ECI state at arrival, deputy ECI state at perch,
/// densified visualization arc.
fn build_far_field_outputs(
    input: &TransferComputationInput,
    transfer: &LambertTransfer,
) -> Result<FarFieldOutputs, PipelineError> {
    let arrival_epoch =
        input.chief.epoch + hifitime::Duration::from_seconds(input.lambert_tof_s);
    let chief_traj = propagate_keplerian(&input.chief, input.lambert_tof_s, 1)?;
    let chief_at_arrival = *chief_traj.last().ok_or(PipelineError::EmptyTrajectory)?;

    let deputy_at_perch = StateVector {
        epoch: arrival_epoch,
        position_eci_km: transfer.arrival_state.position_eci_km,
        velocity_eci_km_s: transfer.arrival_state.velocity_eci_km_s
            + transfer.arrival_dv_eci_km_s,
    };

    let (arc_samples_eci_km, arc_sampling_error) =
        match densify_visualization_arc(transfer, input.lambert_config.revolutions) {
            Ok(arc) => (arc, None),
            Err(e) => (Vec::new(), Some(e)),
        };

    Ok(FarFieldOutputs {
        chief_at_arrival,
        deputy_at_perch,
        arrival_epoch,
        arc_samples_eci_km,
        arc_sampling_error,
    })
}

/// Compute a transfer end-to-end together with its perch enrichment outcome.
///
/// Always returns an [`EnrichmentSuggestion`]:
/// - `Some(reqs)` → enrichment is attempted; on success the returned
///   suggestion is [`EnrichmentSuggestion::Enriched`]; on failure the
///   underlying [`crate::mission::formation::FormationDesignError`]
///   propagates via [`PipelineError`].
/// - `None` → suggestion is [`EnrichmentSuggestion::Baseline`] carrying
///   the geometric perch ROE so callers (CLI report, WASM consumer) can
///   render the "no requirements requested" state without nullable
///   plumbing.
///
/// # Invariants
///
/// - `input.chief_state.epoch == input.deputy_state.epoch`
/// - `input.lambert_tof_s > 0` (when single-burn classification produces a
///   transfer)
///
/// # Validity regime
///
/// When `safety_requirements` is supplied, the requested
/// `min_separation_km` must lie within the linearization regime — see
/// [`super::execute::suggest_enrichment_from_parts`] for the exact bound.
///
/// # Errors
///
/// Returns [`PipelineError`] on any classification, Lambert, propagation,
/// arc-densification, or perch-enrichment failure.
pub fn compute_transfer_with_enrichment(
    input: &TransferComputationInput,
    safety_requirements: Option<&SafetyRequirements>,
) -> Result<(TransferResult, EnrichmentSuggestion), PipelineError> {
    let transfer = compute_transfer(input)?;
    let enrichment = match safety_requirements {
        Some(reqs) => {
            suggest_enrichment_from_parts(&input.perch, &transfer.plan.chief_at_arrival, reqs)?
        }
        None => EnrichmentSuggestion::Baseline {
            perch_roe: transfer.plan.perch_roe,
        },
    };
    Ok((transfer, enrichment))
}

/// Densify the Lambert transfer ellipse for visualization.
///
/// Returns exactly [`LAMBERT_ARC_SAMPLES`] ECI positions: a partial arc r1→r2
/// over the time of flight for single-rev (open polyline), or one full
/// ellipse over a single orbital period for multi-rev (closed by physics).
///
/// `densify_arc(N - 1)` returns `N` states (start point + `N-1` propagated
/// steps), matching [`LAMBERT_ARC_SAMPLES`] exactly.
fn densify_visualization_arc(
    transfer: &LambertTransfer,
    revolutions: u8,
) -> Result<Vec<Vector3<f64>>, ArcDensificationError> {
    // Single Cartesian → Keplerian conversion for both regimes; both paths
    // then call the rotation-hoisted propagator. Eliminates the double
    // `state_to_keplerian` the multi-rev branch used to perform.
    let ke = state_to_keplerian(&transfer.departure_state)?;
    let span_s = if revolutions == 0 {
        transfer.tof_s
    } else {
        ke.period()?
    };
    let traj = propagate_keplerian_from_elements(
        &ke,
        transfer.departure_state.epoch,
        span_s,
        LAMBERT_ARC_SAMPLES - 1,
    )?;
    Ok(traj.into_iter().map(|s| s.position_eci_km).collect())
}

/// Errors from [`densify_visualization_arc`].
///
/// Each variant preserves the underlying typed error so the WASM /
/// frontend layer can dispatch on the actual failure mode (hyperbolic
/// transfer arc, mean-motion underflow, two-body propagation breakdown)
/// instead of pattern-matching on a stringified message.
#[derive(Debug, Clone, thiserror::Error)]
pub enum ArcDensificationError {
    /// Departure state could not be converted to Keplerian elements.
    /// Notably triggered by [`ConversionError::UnboundOrbit`] when a
    /// hyperbolic Lambert transfer is densified.
    #[error(transparent)]
    Conversion(#[from] ConversionError),
    /// Period could not be derived from the converted Keplerian elements
    /// (mean-motion / SMA singular).
    #[error(transparent)]
    KeplerPeriod(#[from] KeplerError),
    /// Two-body propagation failed along the densified arc.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
}

/// End-to-end mission pipeline: compute Lambert transfer, then run waypoint
/// planning, safety, covariance, and reporting.
///
/// Promoted from `rpo-nyx` together with [`compute_transfer`].
///
/// # Errors
///
/// Returns [`PipelineError`] if Lambert solving or any pipeline phase fails.
pub fn execute_mission(input: &PipelineInput) -> Result<PipelineOutput, PipelineError> {
    let transfer_input = TransferComputationInput::from(input);
    let mut transfer = compute_transfer(&transfer_input)?;
    execute_mission_from_transfer(&mut transfer, &input.base)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::elements::roe::compute_roe;
    use crate::mission::config::MissionConfig;
    use crate::mission::errors::MissionError;
    use crate::mission::formation::safety_envelope::enrich_waypoint;
    use crate::mission::formation::types::EnrichmentMode;
    use crate::mission::formation::{
        EiAlignment, EnrichmentSuggestion, SafetyRequirements,
    };
    use crate::mission::planning::compute_transfer_eclipse;
    use crate::mission::safety::compute_ei_separation;
    use crate::pipeline::{
        accept_waypoint_enrichment, apply_perch_enrichment, compute_safety_analysis,
        default_perch, plan_waypoints_from_transfer, suggest_enrichment, to_propagation_model,
        MissionInput, PropagatorChoice, WaypointInput, DEFAULT_LAMBERT_TOF_S,
    };
    use crate::propagation::lambert::TransferDirection;
    use crate::propagation::stm::propagate_roe_stm;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use hifitime::Epoch;

    /// Tolerance for the perch ROE→Keplerian→ROE roundtrip.
    const PERCH_ROE_ROUNDTRIP_TOL: f64 = 1e-10;
    /// Tolerance for structural zeros (no arithmetic).
    const STRUCTURAL_ZERO_TOL: f64 = 1e-15;
    /// Tolerance for perch ROE expected-value comparison (single division).
    const PERCH_ROE_EXPECTED_TOL: f64 = 1e-12;
    /// Tolerance for serde JSON roundtrip of ROE values.
    const SERDE_ROUNDTRIP_TOL: f64 = 1e-12;
    /// Minimum Δv difference (km/s) between 0-rev and 1-rev solutions.
    const MULTI_REV_DV_DIFFERENCE_MIN: f64 = 1e-6;
    /// Time-of-flight for single-revolution Lambert test cases (seconds).
    const TEST_LAMBERT_TOF_S: f64 = 3600.0;
    /// Time-of-flight for multi-revolution Lambert test cases (seconds).
    const MULTI_REV_LAMBERT_TOF_S: f64 = 12_000.0;
    /// Pure data-copy fidelity tolerance.
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;
    /// Minimum e/i magnitude expected after enrichment.
    const ENRICHMENT_NONZERO_TOL: f64 = 1e-10;
    /// Exact-equality tolerance for ROE that should be bitwise identical.
    const ROE_IDENTITY_TOL: f64 = 1e-15;
    /// Drift-prediction phase residual upper bound (rad).
    const PIPELINE_DRIFT_PHASE_TOL_RAD: f64 = 1.0e-3;
    /// Pipeline-roundtrip enrichment phase tolerance (rad).
    const PIPELINE_ENRICHMENT_PHASE_TOL_RAD: f64 = 0.05;
    /// |`delta_da`| upper bound after enrichment (dimensionless).
    const ENRICHMENT_DA_DRIFT_TOL: f64 = 1e-3;
    /// Default test waypoint TOF (seconds).
    const TEST_WAYPOINT_TOF_S: f64 = 4200.0;
    /// Arc endpoint agreement with Lambert departure/arrival positions (km).
    const ARC_ENDPOINT_AGREEMENT_TOL_KM: f64 = 1e-3;

    /// Far-field SMA offset placing deputy 200 km above chief — large
    /// enough to cross the proximity classification threshold and make
    /// the e/i separation nontrivial.
    const FAR_FIELD_SMA_OFFSET_KM: f64 = 200.0;
    /// Far-field eccentricity for the deputy fixture.
    const FAR_FIELD_ECCENTRICITY: f64 = 0.005;
    /// Far-field inclination offset for the deputy fixture (rad).
    const FAR_FIELD_INC_OFFSET_RAD: f64 = 0.05;
    /// Far-field deputy initial mean anomaly (rad).
    const FAR_FIELD_MEAN_ANOMALY_RAD: f64 = 2.0;
    /// V-bar perch along-track distance for far-field test fixtures (km).
    const FAR_FIELD_PERCH_ALONG_TRACK_KM: f64 = 5.0;

    /// SMA offset placing deputy 50 km above chief for the
    /// `far_field_input` builder — small enough to avoid surface intersection
    /// for short Lambert TOFs.
    const FAR_FIELD_INPUT_SMA_OFFSET_KM: f64 = 50.0;
    /// Phase offset (rad) along the chief orbit for the deputy in
    /// `far_field_input`.
    const FAR_FIELD_INPUT_PHASE_OFFSET_RAD: f64 = 0.2;

    /// Proximity SMA offset placing deputy 1 km above chief.
    const PROXIMITY_SMA_OFFSET_KM: f64 = 1.0;
    /// Proximity in-track phase offset (rad).
    const PROXIMITY_PHASE_OFFSET_RAD: f64 = 0.01;

    /// Number of transfer-eclipse samples.
    const TRANSFER_ECLIPSE_SAMPLES: u32 = 200;

    /// Expected number of distinct top-level keys after `PipelineInput`'s
    /// `serde(flatten)` collapses `MissionInput` into the parent object.
    /// Sum: 10 `MissionInput` keys (`chief`, `deputy`, `waypoints`, `config`,
    /// `propagator`, `perch`, `cola`, `navigation_accuracy`,
    /// `maneuver_uncertainty`, `safety_requirements`) + 6 `PipelineInput`-only
    /// keys (`lambert_tof_s`, `lambert_config`, `proximity`, `chief_config`,
    /// `deputy_config`, `monte_carlo`).
    const PIPELINE_INPUT_FLATTENED_KEY_COUNT: usize = 16;

    /// Minimum e/i separation requirement for enrichment fixtures (km).
    const ENRICHMENT_SAFETY_MIN_SEP_KM: f64 = 0.15;
    /// Larger separation requirement used by the formation-design fixture.
    const ENRICHMENT_SAFETY_MIN_SEP_LOOSE_KM: f64 = 0.10;
    /// Out-of-bounds waypoint index for invalid-replan-index test.
    const INVALID_REPLAN_INDEX: usize = 99;

    const TEST_WP1_POSITION_RIC_KM: [f64; 3] = [0.5, 2.0, 0.5];
    const TEST_WP2_POSITION_RIC_KM: [f64; 3] = [0.5, 0.5, 0.5];
    const TEST_WP2_VELOCITY_RIC_KM_S: [f64; 3] = [0.0, 0.001, 0.0];
    const TEST_WP_EXTRA_POSITION_RIC_KM: [f64; 3] = [0.3, 1.5, 0.2];
    const CASCADE_DV_CHANGE_TOL_KM_S: f64 = 1e-10;

    fn wrap_mission(base: MissionInput) -> PipelineInput {
        PipelineInput {
            base,
            lambert_tof_s: DEFAULT_LAMBERT_TOF_S,
            lambert_config: LambertConfig::default(),
            proximity: ProximityConfig::default(),
            chief_config: None,
            deputy_config: None,
            monte_carlo: None,
        }
    }

    fn far_field_input() -> PipelineInput {
        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += FAR_FIELD_INPUT_SMA_OFFSET_KM;
        deputy_ke.mean_anomaly_rad += FAR_FIELD_INPUT_PHASE_OFFSET_RAD;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        wrap_mission(MissionInput {
            chief,
            deputy,
            perch: default_perch(),
            waypoints: vec![
                WaypointInput {
                    position_ric_km: TEST_WP1_POSITION_RIC_KM,
                    velocity_ric_km_s: None,
                    tof_s: Some(TEST_WAYPOINT_TOF_S),
                    label: Some("WP1".into()),
                },
                WaypointInput {
                    position_ric_km: TEST_WP2_POSITION_RIC_KM,
                    velocity_ric_km_s: Some(TEST_WP2_VELOCITY_RIC_KM_S),
                    tof_s: Some(TEST_WAYPOINT_TOF_S),
                    label: Some("WP2".into()),
                },
            ],
            config: MissionConfig::default(),
            propagator: PropagatorChoice::J2,
            cola: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            safety_requirements: None,
        })
    }

    fn proximity_input() -> PipelineInput {
        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += PROXIMITY_SMA_OFFSET_KM;
        deputy_ke.mean_anomaly_rad += PROXIMITY_PHASE_OFFSET_RAD;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        wrap_mission(MissionInput {
            chief,
            deputy,
            perch: default_perch(),
            waypoints: vec![WaypointInput {
                position_ric_km: TEST_WP1_POSITION_RIC_KM,
                velocity_ric_km_s: None,
                tof_s: Some(TEST_WAYPOINT_TOF_S),
                label: Some("WP1".into()),
            }],
            config: MissionConfig::default(),
            propagator: PropagatorChoice::J2,
            cola: None,
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            safety_requirements: None,
        })
    }

    fn proximity_input_with_enrichment() -> PipelineInput {
        let mut input = proximity_input();
        input.base.safety_requirements = Some(SafetyRequirements {
            min_separation_km: ENRICHMENT_SAFETY_MIN_SEP_KM,
            alignment: EiAlignment::Parallel,
        });
        input
    }

    fn far_field_deputy_ke(chief_ke: KeplerianElements) -> KeplerianElements {
        KeplerianElements {
            a_km: chief_ke.a_km + FAR_FIELD_SMA_OFFSET_KM,
            e: FAR_FIELD_ECCENTRICITY,
            i_rad: chief_ke.i_rad + FAR_FIELD_INC_OFFSET_RAD,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: FAR_FIELD_MEAN_ANOMALY_RAD,
        }
    }

    fn compute_transfer_from_pipeline(
        input: &PipelineInput,
    ) -> Result<TransferResult, PipelineError> {
        let transfer_input = TransferComputationInput::from(input);
        compute_transfer(&transfer_input)
    }

    // ---- plan_mission tests (moved from rpo-nyx::pipeline::planning) ----

    #[test]
    fn perch_roe_to_keplerian_roundtrip() {
        let chief = iss_like_elements();
        let original_roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0002,
            dix: 0.0005,
            diy: 0.0003,
        };

        let deputy_ke = perch_roe_to_keplerian(&original_roe, &chief);
        let recovered = compute_roe(&chief, &deputy_ke).unwrap();

        for (label, l, r) in [
            ("da", original_roe.da, recovered.da),
            ("dlambda", original_roe.dlambda, recovered.dlambda),
            ("dex", original_roe.dex, recovered.dex),
            ("dey", original_roe.dey, recovered.dey),
            ("dix", original_roe.dix, recovered.dix),
            ("diy", original_roe.diy, recovered.diy),
        ] {
            assert!(
                (l - r).abs() < PERCH_ROE_ROUNDTRIP_TOL,
                "{label} roundtrip: {l} vs {r}"
            );
        }
    }

    #[test]
    fn farfield_mission_with_lambert() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = far_field_deputy_ke(chief_ke);

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };

        let plan = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            TEST_LAMBERT_TOF_S,
            &LambertConfig::default(),
        )
        .expect("far-field mission should succeed");

        assert!(matches!(plan.phase, MissionPhase::FarField { .. }));
        assert!(plan.transfer.is_some(), "should have Lambert transfer");
        assert!(
            plan.transfer.as_ref().unwrap().total_dv_km_s > 0.0,
            "Lambert Δv should be positive"
        );
    }

    fn run_proximity_plan_mission(perch: &PerchGeometry) -> (MissionPlan, KeplerianElements) {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += PROXIMITY_SMA_OFFSET_KM;
        deputy_ke.mean_anomaly_rad += PROXIMITY_PHASE_OFFSET_RAD;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        let plan = plan_mission(
            &chief,
            &deputy,
            perch,
            ProximityConfig::default(),
            TEST_LAMBERT_TOF_S,
            &LambertConfig::default(),
        )
        .expect("proximity mission should succeed");
        (plan, chief_ke)
    }

    #[test]
    fn proximity_mission_uses_perch_roe() {
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };
        let (plan, chief_ke) = run_proximity_plan_mission(&perch);

        assert!(matches!(plan.phase, MissionPhase::Proximity { .. }));
        assert!(plan.transfer.is_none());

        let expected_dlambda = FAR_FIELD_PERCH_ALONG_TRACK_KM / chief_ke.a_km;
        assert!(plan.perch_roe.da.abs() < STRUCTURAL_ZERO_TOL);
        assert!((plan.perch_roe.dlambda - expected_dlambda).abs() < PERCH_ROE_EXPECTED_TOL);
    }

    #[test]
    fn proximity_mission_uses_rbar_perch() {
        let perch = PerchGeometry::RBar { radial_km: 2.0 };
        let (plan, chief_ke) = run_proximity_plan_mission(&perch);

        assert!(matches!(plan.phase, MissionPhase::Proximity { .. }));
        assert!(plan.transfer.is_none());

        let expected_da = 2.0 / chief_ke.a_km;
        assert!((plan.perch_roe.da - expected_da).abs() < PERCH_ROE_EXPECTED_TOL);
        assert!(plan.perch_roe.dlambda.abs() < STRUCTURAL_ZERO_TOL);
    }

    #[test]
    fn mission_plan_serde_roundtrip() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += PROXIMITY_SMA_OFFSET_KM;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };

        let plan = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            TEST_LAMBERT_TOF_S,
            &LambertConfig::default(),
        )
        .expect("mission should succeed");

        let json = serde_json::to_string(&plan).unwrap();
        let deserialized: MissionPlan = serde_json::from_str(&json).unwrap();

        let expected_dlambda = FAR_FIELD_PERCH_ALONG_TRACK_KM / chief_ke.a_km;
        assert!(
            (deserialized.perch_roe.dlambda - expected_dlambda).abs() < SERDE_ROUNDTRIP_TOL
        );
    }

    #[test]
    fn farfield_mission_multi_rev_lambert() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = far_field_deputy_ke(chief_ke);

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };
        let tof_s = MULTI_REV_LAMBERT_TOF_S;

        let config_0rev = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 0,
        };
        let config_1rev = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 1,
        };

        let plan_0 = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            tof_s,
            &config_0rev,
        )
        .unwrap();
        let plan_1 = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            tof_s,
            &config_1rev,
        )
        .unwrap();

        assert!(matches!(plan_1.phase, MissionPhase::FarField { .. }));
        let dv_0 = plan_0.transfer.as_ref().unwrap().total_dv_km_s;
        let dv_1 = plan_1.transfer.as_ref().unwrap().total_dv_km_s;
        assert!(
            (dv_0 - dv_1).abs() > MULTI_REV_DV_DIFFERENCE_MIN,
            "Multi-rev should produce different Δv: 0-rev={dv_0:.6}, 1-rev={dv_1:.6}"
        );
    }

    #[test]
    fn transfer_eclipse_far_field() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = far_field_deputy_ke(chief_ke);

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };

        let plan = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            TEST_LAMBERT_TOF_S,
            &LambertConfig::default(),
        )
        .unwrap();

        let transfer = plan.transfer.as_ref().expect("Lambert transfer");
        let eclipse = compute_transfer_eclipse(transfer, &chief, TRANSFER_ECLIPSE_SAMPLES).unwrap();
        let expected_samples = TRANSFER_ECLIPSE_SAMPLES as usize + 1;
        assert_eq!(eclipse.deputy_celestial.len(), expected_samples);
        assert_eq!(eclipse.chief_eclipse.len(), expected_samples);
        assert!(
            eclipse.summary.time_in_shadow_fraction >= 0.0
                && eclipse.summary.time_in_shadow_fraction <= 1.0
        );
    }

    #[test]
    fn transfer_eclipse_proximity_returns_none() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += PROXIMITY_SMA_OFFSET_KM;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let perch = PerchGeometry::VBar { along_track_km: FAR_FIELD_PERCH_ALONG_TRACK_KM };

        let plan = plan_mission(
            &chief,
            &deputy,
            &perch,
            ProximityConfig::default(),
            TEST_LAMBERT_TOF_S,
            &LambertConfig::default(),
        )
        .unwrap();
        assert!(plan.transfer.is_none());
    }

    // ---- compute_transfer / execute_mission tests (moved from rpo-nyx::pipeline::mod) ----

    #[test]
    fn test_compute_transfer_far_field() {
        let input = far_field_input();
        let result = compute_transfer_from_pipeline(&input).expect("compute_transfer");

        assert!(result.plan.transfer.is_some());
        assert!(result.lambert_dv_km_s > 0.0);
        assert!(result.arc_sampling_error.is_none());
        assert_eq!(
            result.arc_samples_eci_km.len(),
            LAMBERT_ARC_SAMPLES as usize
        );
        let transfer = result.plan.transfer.as_ref().unwrap();
        let head = result.arc_samples_eci_km.first().unwrap();
        let tail = result.arc_samples_eci_km.last().unwrap();
        assert!(
            (head - transfer.departure_state.position_eci_km).norm()
                < ARC_ENDPOINT_AGREEMENT_TOL_KM
        );
        assert!(
            (tail - transfer.arrival_state.position_eci_km).norm()
                < ARC_ENDPOINT_AGREEMENT_TOL_KM
        );
    }

    #[test]
    fn test_compute_transfer_multi_rev_renders_closed_ellipse() {
        let mut input = far_field_input();
        input.lambert_tof_s = MULTI_REV_LAMBERT_TOF_S;
        input.lambert_config = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 1,
        };

        let result = compute_transfer_from_pipeline(&input).expect("multi-rev compute_transfer");
        assert!(result.plan.transfer.is_some());
        assert!(result.arc_sampling_error.is_none());
        assert_eq!(
            result.arc_samples_eci_km.len(),
            LAMBERT_ARC_SAMPLES as usize
        );
        let head = result.arc_samples_eci_km.first().unwrap();
        let tail = result.arc_samples_eci_km.last().unwrap();
        assert!(
            (head - tail).norm() < ARC_ENDPOINT_AGREEMENT_TOL_KM,
            "multi-rev arc must close (first == last within tol)"
        );
    }

    #[test]
    fn test_execute_mission_far_field() {
        let input = far_field_input();
        let output = execute_mission(&input).expect("execute_mission");
        assert!(output.transfer.is_some());
        assert!(!output.mission.legs.is_empty());
        assert!(output.total_dv_km_s > 0.0);
        assert!(output.total_duration_s > 0.0);
    }

    #[test]
    fn test_compute_transfer_proximity_arrival_epoch() {
        let input = proximity_input();
        let result = compute_transfer_from_pipeline(&input).expect("compute_transfer");

        assert!(result.plan.transfer.is_none());
        assert!(result.lambert_dv_km_s.abs() < f64::EPSILON);
        assert_eq!(result.arrival_epoch, input.base.chief.epoch);
        assert!(result.arc_samples_eci_km.is_empty());
        assert!(result.arc_sampling_error.is_none());
    }

    #[test]
    fn test_pipeline_input_serde_roundtrip() {
        let input = far_field_input();
        let json = serde_json::to_string(&input).unwrap();
        let roundtrip: PipelineInput = serde_json::from_str(&json).unwrap();
        assert_eq!(input.base.chief.epoch, roundtrip.base.chief.epoch);
        assert_eq!(input.base.waypoints.len(), roundtrip.base.waypoints.len());
        assert!(
            (input.lambert_tof_s - roundtrip.lambert_tof_s).abs() < COPY_FIDELITY_TOL,
        );
    }

    #[test]
    fn pipeline_input_flatten_no_field_collision() {
        let input = far_field_input();
        let value = serde_json::to_value(&input).unwrap();
        let map = value.as_object().unwrap();
        assert_eq!(
            map.len(),
            PIPELINE_INPUT_FLATTENED_KEY_COUNT,
            "flatten collision: expected {PIPELINE_INPUT_FLATTENED_KEY_COUNT} distinct keys, got {}",
            map.len(),
        );
    }

    #[test]
    fn test_transfer_result_serde_roundtrip() {
        let input = far_field_input();
        let result = compute_transfer_from_pipeline(&input).unwrap();
        let json = serde_json::to_string(&result).unwrap();
        let roundtrip: TransferResult = serde_json::from_str(&json).unwrap();
        assert_eq!(result.arrival_epoch, roundtrip.arrival_epoch);
        assert!(
            (result.lambert_dv_km_s - roundtrip.lambert_dv_km_s).abs() < COPY_FIDELITY_TOL,
        );
        assert!(result.plan.transfer.is_some() == roundtrip.plan.transfer.is_some());
    }

    #[test]
    fn test_execute_mission_with_formation_design() {
        let mut input = far_field_input();
        input.base.safety_requirements = Some(SafetyRequirements {
            min_separation_km: ENRICHMENT_SAFETY_MIN_SEP_LOOSE_KM,
            alignment: EiAlignment::Parallel,
        });
        let output = execute_mission(&input).unwrap();

        let report = output
            .formation_design
            .as_ref()
            .expect("formation_design should be Some");

        assert!(matches!(
            report.perch,
            EnrichmentSuggestion::Enriched { .. },
        ));

        let roe = &output.perch_roe;
        let ei_magnitude = (roe.dex * roe.dex
            + roe.dey * roe.dey
            + roe.dix * roe.dix
            + roe.diy * roe.diy)
            .sqrt();
        assert!(ei_magnitude > ENRICHMENT_NONZERO_TOL);

        if let EnrichmentSuggestion::Enriched { ref safe_perch, .. } = report.perch {
            let diff = (safe_perch.roe.to_vector() - roe.to_vector()).norm();
            assert!(diff < ROE_IDENTITY_TOL);
        }

        assert_eq!(report.waypoints.len(), output.mission.legs.len());
        assert_eq!(report.transit_safety.len(), output.mission.legs.len());
        assert!(report.waypoints.iter().all(Option::is_some));
        assert!(report.transit_safety.iter().all(Option::is_some));

        let mission_min = report
            .mission_min_ei_separation_km
            .expect("mission_min should be Some");
        assert!(mission_min > 0.0);
    }

    #[test]
    fn formation_report_has_drift_prediction() {
        let mut input = proximity_input();
        input.base.safety_requirements = Some(SafetyRequirements {
            min_separation_km: ENRICHMENT_SAFETY_MIN_SEP_LOOSE_KM,
            alignment: EiAlignment::Parallel,
        });

        let output = execute_mission(&input).unwrap();
        let report = output.formation_design.as_ref().unwrap();
        assert!(report.drift_prediction.is_some());
        let pred = report.drift_prediction.as_ref().unwrap();
        assert!(pred.predicted_min_ei_km > 0.0);
        assert!(pred.predicted_phase_angle_rad.abs() < PIPELINE_DRIFT_PHASE_TOL_RAD);
    }

    #[test]
    fn test_suggest_enrichment_does_not_mutate() {
        let input = proximity_input_with_enrichment();
        let transfer = compute_transfer_from_pipeline(&input).unwrap();

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input.base).unwrap();

        assert!(suggestion.is_some());
        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
        );
    }

    #[test]
    fn test_suggest_enrichment_none_without_requirements() {
        let input = far_field_input();
        let transfer = compute_transfer_from_pipeline(&input).unwrap();
        let suggestion = suggest_enrichment(&transfer, &input.base).unwrap();
        assert!(suggestion.is_none());
    }

    #[test]
    fn test_apply_perch_enrichment_mutates() {
        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer_from_pipeline(&input).unwrap();

        let original_perch_roe = transfer.plan.perch_roe;
        let suggestion = suggest_enrichment(&transfer, &input.base).unwrap().unwrap();

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.dey - original_perch_roe.dey).abs()
                > ENRICHMENT_NONZERO_TOL,
        );
    }

    #[test]
    fn test_apply_perch_enrichment_baseline_no_mutate() {
        let input = proximity_input_with_enrichment();
        let mut transfer = compute_transfer_from_pipeline(&input).unwrap();

        let original_perch_roe = transfer.plan.perch_roe;

        let suggestion = EnrichmentSuggestion::Baseline {
            perch_roe: original_perch_roe,
        };

        apply_perch_enrichment(&mut transfer, &suggestion);

        assert!(
            (transfer.plan.perch_roe.to_vector() - original_perch_roe.to_vector()).norm()
                < COPY_FIDELITY_TOL,
        );
    }

    #[test]
    fn test_compute_safety_analysis_matches_build_output() {
        use crate::mission::config::SafetyConfig;

        let mut input = far_field_input();
        input.base.config.safety = Some(SafetyConfig {
            min_ei_separation_km: ENRICHMENT_SAFETY_MIN_SEP_LOOSE_KM,
            min_distance_3d_km: 0.05,
        });

        let output = execute_mission(&input).unwrap();
        let transfer = compute_transfer_from_pipeline(&input).unwrap();
        let propagator = to_propagation_model(&input.base.propagator);
        let mut transfer2 = transfer;
        let suggestion = suggest_enrichment(&transfer2, &input.base).unwrap();
        if let Some(ref s) = suggestion {
            apply_perch_enrichment(&mut transfer2, s);
        }
        let wp_mission =
            plan_waypoints_from_transfer(&transfer2, &input.base, &propagator).unwrap();
        let safety = compute_safety_analysis(
            &wp_mission,
            input.base.config.safety.as_ref(),
            input.base.cola.as_ref(),
            &propagator,
        );

        assert_eq!(output.safety.free_drift.is_some(), safety.free_drift.is_some());
        assert_eq!(output.safety.poca.is_some(), safety.poca.is_some());
        assert_eq!(
            output.safety.free_drift_poca.is_some(),
            safety.free_drift_poca.is_some(),
        );
        assert_eq!(output.safety.cola.is_some(), safety.cola.is_some());

        if let (Some(out_poca), Some(sa_poca)) = (&output.safety.poca, &safety.poca) {
            assert_eq!(out_poca.len(), sa_poca.len());
        }
    }

    #[test]
    fn accept_waypoint_enrichment_replans_with_safe_ei() {
        let input = proximity_input_with_enrichment();
        let baseline_output = execute_mission(&input).unwrap();

        let reqs = input.base.safety_requirements.unwrap();
        let leg = &baseline_output.mission.legs[0];
        let enriched = enrich_waypoint(
            &leg.to_position_ric_km,
            None,
            &leg.arrival_chief_mean,
            &reqs,
        )
        .unwrap();

        let mut updated_input = input.clone();
        let mut transfer = compute_transfer_from_pipeline(&updated_input).unwrap();
        let enriched_output = accept_waypoint_enrichment(
            &mut updated_input.base,
            &mut transfer,
            0,
            &enriched.roe,
            &leg.arrival_chief_mean,
        )
        .unwrap();

        let enriched_leg = &enriched_output.mission.legs[0];
        let ei_enriched =
            compute_ei_separation(&enriched_leg.post_arrival_roe, &enriched_leg.arrival_chief_mean);
        let ei_baseline = compute_ei_separation(&leg.post_arrival_roe, &leg.arrival_chief_mean);
        assert!(ei_enriched.min_separation_km > ei_baseline.min_separation_km);
        assert!(updated_input.base.waypoints[0].velocity_ric_km_s.is_some());

        let delta_da = enriched.roe.da - leg.post_arrival_roe.da;
        assert!(delta_da.abs() < ENRICHMENT_DA_DRIFT_TOL);
    }

    #[test]
    fn accept_waypoint_enrichment_rejects_out_of_bounds_index() {
        let input = proximity_input_with_enrichment();
        let baseline_output = execute_mission(&input).unwrap();
        let leg = &baseline_output.mission.legs[0];
        let reqs = input.base.safety_requirements.unwrap();
        let enriched = enrich_waypoint(
            &leg.to_position_ric_km,
            None,
            &leg.arrival_chief_mean,
            &reqs,
        )
        .unwrap();

        let mut updated_input = input.clone();
        let mut transfer = compute_transfer_from_pipeline(&updated_input).unwrap();
        let result = accept_waypoint_enrichment(
            &mut updated_input.base,
            &mut transfer,
            INVALID_REPLAN_INDEX,
            &enriched.roe,
            &leg.arrival_chief_mean,
        );
        assert!(matches!(
            result,
            Err(PipelineError::Mission(MissionError::InvalidReplanIndex {
                index,
                ..
            })) if index == INVALID_REPLAN_INDEX
        ));
    }

    #[test]
    fn formation_report_enrichment_uses_drift_compensation() {
        let input = proximity_input_with_enrichment();
        let output = execute_mission(&input).unwrap();
        let report = output.formation_design.unwrap();

        if let Some(ref drift) = report.drift_prediction {
            assert!(drift.predicted_min_ei_km > 0.0);
        }
        for enriched in report.waypoints.iter().flatten() {
            assert_eq!(enriched.mode, EnrichmentMode::PositionOnly);
        }
    }

    #[test]
    fn waypoint_enrichment_aligns_at_next_leg_midpoint() {
        let mut input = proximity_input_with_enrichment();
        input.base.waypoints.push(WaypointInput {
            position_ric_km: TEST_WP_EXTRA_POSITION_RIC_KM,
            velocity_ric_km_s: None,
            tof_s: Some(TEST_WAYPOINT_TOF_S),
            label: Some("WP2".into()),
        });

        let output = execute_mission(&input).unwrap();
        let report = output.formation_design.unwrap();
        let enriched_wp0 = report.waypoints[0].as_ref().unwrap();

        let next_leg = &output.mission.legs[1];
        let (mid_roe, mid_chief) = propagate_roe_stm(
            &enriched_wp0.roe,
            &output.mission.legs[0].arrival_chief_mean,
            next_leg.tof_s / 2.0,
        )
        .unwrap();
        let ei_mid = compute_ei_separation(&mid_roe, &mid_chief);

        assert!(ei_mid.phase_angle_rad.abs() < PIPELINE_ENRICHMENT_PHASE_TOL_RAD);
        assert!(ei_mid.min_separation_km >= enriched_wp0.baseline_ei.min_separation_km);
    }

    #[test]
    fn full_enrichment_cycle_suggest_accept_verify() {
        let mut input = proximity_input_with_enrichment();
        input.base.waypoints.push(WaypointInput {
            position_ric_km: TEST_WP_EXTRA_POSITION_RIC_KM,
            velocity_ric_km_s: None,
            tof_s: Some(TEST_WAYPOINT_TOF_S),
            label: Some("WP2".into()),
        });

        let baseline = execute_mission(&input).unwrap();
        let report = baseline.formation_design.as_ref().unwrap();
        let suggestion_0 = report.waypoints[0].as_ref().unwrap();
        assert!(
            suggestion_0.enriched_ei.min_separation_km
                > suggestion_0.baseline_ei.min_separation_km
        );

        let leg_0 = &baseline.mission.legs[0];
        let mut transfer = compute_transfer_from_pipeline(&input).unwrap();
        let enriched = accept_waypoint_enrichment(
            &mut input.base,
            &mut transfer,
            0,
            &suggestion_0.roe,
            &leg_0.arrival_chief_mean,
        )
        .unwrap();

        let enriched_leg_0 = &enriched.mission.legs[0];
        let ei_enriched = compute_ei_separation(
            &enriched_leg_0.post_arrival_roe,
            &enriched_leg_0.arrival_chief_mean,
        );
        let ei_baseline = compute_ei_separation(&leg_0.post_arrival_roe, &leg_0.arrival_chief_mean);
        assert!(ei_enriched.min_separation_km >= ei_baseline.min_separation_km);
        assert!(input.base.waypoints[0].velocity_ric_km_s.is_some());

        let baseline_leg_1 = &baseline.mission.legs[1];
        let enriched_leg_1 = &enriched.mission.legs[1];
        assert!(
            (enriched_leg_1.total_dv_km_s - baseline_leg_1.total_dv_km_s).abs()
                > CASCADE_DV_CHANGE_TOL_KM_S,
        );
    }
}

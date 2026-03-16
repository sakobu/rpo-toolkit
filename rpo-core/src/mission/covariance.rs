//! Mission-level covariance propagation orchestration.
//!
//! Threads covariance propagation kernels from `propagation::covariance`
//! across a multi-leg waypoint mission.

use hifitime::Duration;
use nalgebra::Vector3;

use crate::propagation::covariance::propagate::{
    compute_collision_metrics, propagate_sample, roe_covariance_to_ric_position,
    update_covariance_at_maneuver,
};
use crate::propagation::covariance::types::{
    CovarianceState, LegCovarianceReport, ManeuverUncertainty, MissionCovarianceReport,
    NavigationAccuracy,
};
use crate::propagation::covariance::CovarianceError;
use crate::propagation::j2_params::compute_j2_params;
use crate::propagation::propagator::PropagationModel;
use crate::propagation::stm::propagate_chief_mean;
use crate::types::{Matrix6, QuasiNonsingularROE};

use super::types::WaypointMission;

use crate::elements::roe_to_ric::roe_to_ric;

/// Propagate covariance across a complete waypoint mission.
///
/// For each leg in the mission:
/// 1. Update covariance at departure maneuver (add execution error via B matrix)
/// 2. Propagate covariance through the coast arc at `n_steps` sample points
/// 3. At each sample: project to RIC, compute 3σ bounds and Pc
/// 4. Update covariance at arrival maneuver
/// 5. Post-arrival covariance becomes next leg's initial covariance
///
/// The function takes a completed `WaypointMission` as input and produces
/// covariance overlays — it does not modify the mission plan.
///
/// # Invariants
/// - `mission` must be a valid completed waypoint mission (from `plan_waypoint_mission`)
/// - `initial_covariance` must be symmetric PSD (6×6 ROE-space covariance)
/// - Chief elements on each leg must be **mean** Keplerian, not osculating
/// - `n_steps` of 0 produces a single sample at departure per leg; typical values 50–200
/// - The `propagator` should match the one used to plan the mission
///
/// # Arguments
/// * `mission` - Planned waypoint mission (provides leg TOFs, Δv, chief elements)
/// * `initial_covariance` - Initial ROE covariance (from `ric_accuracy_to_roe_covariance`)
/// * `navigation_accuracy` - Navigation accuracy used to derive `initial_covariance` (recorded in report)
/// * `maneuver_uncertainty` - Optional maneuver execution uncertainty
/// * `propagator` - Propagation model (J2 or J2+Drag, determines which STM to use)
/// * `n_steps` - Number of covariance sample points per leg
///
/// # Errors
/// Returns `CovarianceError` if propagation or projection fails.
pub fn propagate_mission_covariance(
    mission: &WaypointMission,
    initial_covariance: &Matrix6,
    navigation_accuracy: &NavigationAccuracy,
    maneuver_uncertainty: Option<&ManeuverUncertainty>,
    propagator: &PropagationModel,
    n_steps: usize,
) -> Result<MissionCovarianceReport, CovarianceError> {
    let mut current_p = *initial_covariance;
    let mut leg_reports = Vec::with_capacity(mission.legs.len());
    let mut overall_max_sigma3 = 0.0_f64;
    let mut overall_max_pc = 0.0_f64;
    let mut terminal_ric_position = Vector3::zeros();
    let mut terminal_sigma3 = Vector3::zeros();

    for leg in &mission.legs {
        // 1. Departure maneuver update
        if let Some(unc) = maneuver_uncertainty {
            current_p = update_covariance_at_maneuver(
                &current_p,
                unc,
                &leg.departure_maneuver.dv_ric_km_s,
                &leg.departure_chief_mean,
            )?;
        }
        let p_post_dep = current_p;

        // 2. Coast arc setup
        let j2p = compute_j2_params(&leg.departure_chief_mean)?;
        let roe_vec_0 = leg.post_departure_roe.to_vector();
        // Convert to u32 for lossless f64 conversion (n_steps is typically 50–200).
        // Rejects absurd inputs rather than silently saturating.
        let n = u32::try_from(n_steps.max(1))
            .map_err(|_| CovarianceError::TooManySteps { n_steps })?;
        let n_div = f64::from(n);

        // 3. Sample loop — compute STM once per sample, reuse for both
        //    covariance propagation and nominal ROE evolution.
        let mut states = Vec::with_capacity(n as usize + 1);
        let mut leg_max_sigma3 = 0.0_f64;
        let mut leg_max_pc = 0.0_f64;
        let mut p_last = p_post_dep;

        for j in 0..=n {
            let tau_j = leg.tof_s * f64::from(j) / n_div;

            // Compute STM once, reuse for covariance and nominal ROE
            let (p_j, roe_vec_j) = propagate_sample(
                &p_post_dep, &roe_vec_0, &j2p, &leg.departure_chief_mean, propagator, tau_j,
            );
            let roe_j = QuasiNonsingularROE::from_vector(&roe_vec_j);

            // Advance chief mean elements
            let chief_j = propagate_chief_mean(&leg.departure_chief_mean, &j2p, tau_j);

            // Convert to RIC position
            let ric_j = roe_to_ric(&roe_j, &chief_j)?;
            let ric_position = ric_j.position_ric_km;

            // Project to RIC position covariance
            let p_ric_pos = roe_covariance_to_ric_position(&p_j, &chief_j)?;

            // 3-sigma bounds
            let sigma3 = Vector3::new(
                3.0 * p_ric_pos[(0, 0)].sqrt(),
                3.0 * p_ric_pos[(1, 1)].sqrt(),
                3.0 * p_ric_pos[(2, 2)].sqrt(),
            );

            // Collision metrics
            let (mahal, pc) = compute_collision_metrics(&ric_position, &p_ric_pos);

            // Track per-leg maximums
            let max_sigma3_component = sigma3.x.max(sigma3.y).max(sigma3.z);
            leg_max_sigma3 = leg_max_sigma3.max(max_sigma3_component);
            leg_max_pc = leg_max_pc.max(pc);

            // Overwrite each iteration; after the last leg's last sample,
            // these hold the mission terminal values.
            terminal_ric_position = ric_position;
            terminal_sigma3 = sigma3;

            p_last = p_j;

            states.push(CovarianceState {
                epoch: leg.departure_maneuver.epoch + Duration::from_seconds(tau_j),
                elapsed_s: tau_j,
                covariance_roe: p_j,
                covariance_ric_position_km2: p_ric_pos,
                sigma3_position_ric_km: sigma3,
                mahalanobis_distance: mahal,
                collision_probability: pc,
            });
        }

        // 4. Arrival maneuver update — reuse end-of-coast covariance from
        //    last sample (j=n → tau=tof), avoiding redundant STM computation.
        let p_end_coast = p_last;

        current_p = if let Some(unc) = maneuver_uncertainty {
            update_covariance_at_maneuver(
                &p_end_coast,
                unc,
                &leg.arrival_maneuver.dv_ric_km_s,
                &leg.arrival_chief_mean,
            )?
        } else {
            p_end_coast
        };

        // 5. Build leg report
        overall_max_sigma3 = overall_max_sigma3.max(leg_max_sigma3);
        overall_max_pc = overall_max_pc.max(leg_max_pc);

        leg_reports.push(LegCovarianceReport {
            states,
            max_sigma3_position_km: leg_max_sigma3,
            max_collision_probability: leg_max_pc,
        });
    }

    Ok(MissionCovarianceReport {
        legs: leg_reports,
        navigation_accuracy: *navigation_accuracy,
        maneuver_uncertainty: maneuver_uncertainty.copied(),
        max_sigma3_position_km: overall_max_sigma3,
        max_collision_probability: overall_max_pc,
        terminal_position_ric_km: terminal_ric_position,
        terminal_sigma3_position_ric_km: terminal_sigma3,
    })
}

// ── Mission-level covariance tests ──────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::covariance::ric_accuracy_to_roe_covariance;
    use crate::mission::waypoints::plan_waypoint_mission;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::mission::config::MissionConfig;
    use crate::mission::types::Waypoint;
    use crate::types::DepartureState;

    /// Monotonicity margin: trace differences below this are numerical
    /// noise from symmetrization. Set to ~machine-epsilon level.
    const MONOTONICITY_MARGIN: f64 = 1e-20;

    /// Leg boundary continuity tolerance: the end-of-coast covariance
    /// and start-of-next-leg covariance are computed via independent
    /// STM evaluations at the same tau, so they should match to
    /// machine precision. 1e-6 accounts for accumulated floating-point
    /// error across a full orbital period of propagation.
    const LEG_BOUNDARY_CONTINUITY_TOL: f64 = 1e-6;

    /// PSD eigenvalue floor: covariance eigenvalues should be non-negative,
    /// but numerical error from STM propagation and matrix operations can
    /// produce small negative values. -1e-10 km² is well below any
    /// physically meaningful variance.
    const PSD_EIGENVALUE_FLOOR: f64 = -1e-10;

    /// Tolerance for terminal sigma3 matching the last state's sigma3.
    /// Both values come from the same computation path (no independent
    /// STM evaluation), so agreement should be exact up to f64 rounding.
    const TERMINAL_SIGMA3_MATCH_TOL: f64 = 1e-15;

    /// Minimum expected terminal position norm (km) for a waypoint mission.
    /// The test targets a 5 km in-track waypoint; 0.1 km is a loose lower
    /// bound that catches only a zero/degenerate result.
    const TERMINAL_POSITION_MIN_NORM_KM: f64 = 0.1;

    fn default_nav() -> NavigationAccuracy {
        NavigationAccuracy::default()
    }

    fn mission_departure() -> DepartureState {
        DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: iss_like_elements(),
            epoch: test_epoch(),
        }
    }

    fn single_wp_mission() -> (WaypointMission, PropagationModel) {
        let departure = mission_departure();
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig::default();
        let tof = departure.chief.period().unwrap() * 0.75;

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(tof),
        }];

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("single waypoint mission should succeed");
        (mission, propagator)
    }

    fn two_wp_mission() -> (WaypointMission, PropagationModel) {
        let departure = mission_departure();
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig::default();
        let tof = departure.chief.period().unwrap() * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("two waypoint mission should succeed");
        (mission, propagator)
    }

    fn mission_initial_covariance() -> Matrix6 {
        let nav = default_nav();
        let chief = iss_like_elements();
        ric_accuracy_to_roe_covariance(&nav, &chief).unwrap()
    }

    #[test]
    fn single_leg_covariance_grows() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 50).unwrap();

        assert_eq!(report.legs.len(), 1);
        assert_eq!(report.legs[0].states.len(), 51); // 0..=50

        // Trace should increase monotonically over the coast arc
        for i in 1..report.legs[0].states.len() {
            let trace_prev = report.legs[0].states[i - 1].covariance_roe.trace();
            let trace_curr = report.legs[0].states[i].covariance_roe.trace();
            assert!(
                trace_curr >= trace_prev - MONOTONICITY_MARGIN,
                "Trace should grow monotonically: step {i}, prev={trace_prev}, curr={trace_curr}"
            );
        }

        assert!(
            report.max_sigma3_position_km > 0.0,
            "Max sigma3 should be positive"
        );
    }

    #[test]
    fn multi_leg_covariance_continuity() {
        let (mission, propagator) = two_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        // Without maneuver uncertainty: covariance should be continuous at leg boundary
        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 20).unwrap();

        assert_eq!(report.legs.len(), 2);

        // End of leg 0 coast (last sample at tau=tof) should match start of leg 1 (tau=0)
        let p_end_leg0 = report.legs[0].states.last().unwrap().covariance_roe;
        let p_start_leg1 = report.legs[1].states[0].covariance_roe;
        let diff = (p_end_leg0 - p_start_leg1).norm();

        assert!(
            diff < LEG_BOUNDARY_CONTINUITY_TOL,
            "Covariance should be continuous at leg boundary (no unc): diff={diff}"
        );
    }

    #[test]
    fn maneuver_uncertainty_jump() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();
        let unc = ManeuverUncertainty::default();
        let initial_trace = p0.trace();

        let report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            Some(&unc),
            &propagator,
            20,
        )
        .unwrap();

        // First sample includes departure maneuver update → trace should increase
        let first_trace = report.legs[0].states[0].covariance_roe.trace();
        assert!(
            first_trace > initial_trace,
            "Departure maneuver should increase trace: initial={initial_trace}, after={first_trace}"
        );
    }

    #[test]
    fn zero_maneuver_uncertainty_smooth() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();
        let initial_trace = p0.trace();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 20).unwrap();

        // Without maneuver uncertainty, first sample (tau=0) should match initial covariance
        let first_trace = report.legs[0].states[0].covariance_roe.trace();
        let diff = (first_trace - initial_trace).abs();
        assert!(
            diff < crate::constants::COVARIANCE_SYMMETRY_TOL,
            "Without unc, first trace should match initial: diff={diff}"
        );
    }

    #[test]
    fn j2drag_covariance_path_consistent() {
        let (mission, _) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let j2_report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            None,
            &PropagationModel::J2Stm,
            20,
        )
        .unwrap();

        let drag = crate::test_helpers::test_drag_config();
        let drag_report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            None,
            &PropagationModel::J2DragStm { drag },
            20,
        )
        .unwrap();

        assert_eq!(
            j2_report.legs[0].states.len(),
            drag_report.legs[0].states.len()
        );

        let j2_end = j2_report.legs[0].states.last().unwrap().covariance_roe;
        let drag_end = drag_report.legs[0].states.last().unwrap().covariance_roe;
        let diff = (j2_end - drag_end).norm();
        assert!(
            diff < crate::constants::COVARIANCE_SYMMETRY_TOL,
            "J2 and J2+drag should match with zero drag-rate uncertainty: diff={diff}"
        );

        let eig = drag_end.symmetric_eigenvalues();
        assert!(
            eig.min() > PSD_EIGENVALUE_FLOOR,
            "Drag path covariance should be PSD: min_eig={}",
            eig.min()
        );
    }

    /// The new `terminal_sigma3_position_ric_km` field must exactly match
    /// the 3-sigma values from the last state of the last leg.
    #[test]
    fn terminal_sigma3_matches_last_state() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 20).unwrap();

        let last_state = report.legs.last().unwrap().states.last().unwrap();
        let diff = (report.terminal_sigma3_position_ric_km - last_state.sigma3_position_ric_km).norm();
        assert!(
            diff < TERMINAL_SIGMA3_MATCH_TOL,
            "terminal_sigma3 should match last state sigma3: diff={diff}"
        );

        // Terminal position should be non-zero (waypoint at 5 km in-track)
        assert!(
            report.terminal_position_ric_km.norm() > TERMINAL_POSITION_MIN_NORM_KM,
            "terminal position should be near the waypoint, got {:?}",
            report.terminal_position_ric_km
        );
    }
}

//! Mission-level eclipse aggregation across waypoint legs.
//!
//! Computes chief celestial snapshots and deputy eclipse states for each leg,
//! then merges eclipse intervals across leg boundaries (the chief does not
//! maneuver, so shadow state is continuous).

use crate::elements::eclipse::{build_celestial_snapshot, is_deeper_shadow};
use crate::elements::{
    compute_eclipse_state, extract_eclipse_intervals, keplerian_to_state, ric_to_eci_position,
};
use crate::mission::errors::EclipseComputeError;
use crate::mission::types::ManeuverLeg;
use crate::types::{EclipseInterval, EclipseSummary, LegEclipseData, MissionEclipseData};

/// Compute eclipse data across all mission legs (chief and deputy).
///
/// Single-pass per leg: calls [`keplerian_to_state`] once per trajectory
/// point, then reuses the chief ECI state for both the chief
/// [`CelestialSnapshot`](crate::types::CelestialSnapshot) (via [`build_celestial_snapshot`]) and the deputy
/// eclipse state (via [`ric_to_eci_position`] + [`compute_eclipse_state`]).
///
/// # Errors
///
/// Returns [`EclipseComputeError::Conversion`] if a Keplerian-to-ECI
/// conversion fails, [`EclipseComputeError::Dcm`] if an RIC-to-ECI frame
/// transformation fails, or [`EclipseComputeError::EmptyTrajectory`] if
/// all legs have empty trajectories.
///
/// # Cross-leg interval merging
///
/// Eclipse intervals are extracted per-leg, then merged across boundaries.
/// The chief does not maneuver, so its eclipse state is continuous across
/// leg boundaries. If the last interval of leg N ends at the same epoch as
/// (or later than) the first interval of leg N+1, they are merged into one
/// contiguous interval.
///
/// # Deputy eclipse
///
/// Deputy ECI position at each point is reconstructed via:
/// 1. [`keplerian_to_state`]`(&chief_mean)` → chief [`StateVector`](crate::types::StateVector)
/// 2. [`ric_to_eci_position`]`(&chief_sv, &ric.position_ric_km)` → deputy ECI
/// 3. [`compute_eclipse_state`]`(&deputy_eci, &sun_eci)` → deputy shadow state
///
/// The Sun position for the deputy is obtained from the chief snapshot's
/// direction and distance fields (already computed by `build_celestial_snapshot`),
/// avoiding a redundant ephemeris call.
pub fn compute_mission_eclipse(
    legs: &[ManeuverLeg],
) -> Result<MissionEclipseData, EclipseComputeError> {
    let mut leg_data = Vec::with_capacity(legs.len());
    let mut merged_intervals: Vec<EclipseInterval> = Vec::new();

    for leg in legs {
        if leg.trajectory.is_empty() {
            leg_data.push(LegEclipseData {
                chief_celestial: Vec::new(),
                deputy_eclipse: Vec::new(),
            });
            continue;
        }

        // Single pass: keplerian_to_state once per point, reuse for both
        // chief snapshot and deputy RIC-to-ECI conversion.
        let mut chief_snapshots = Vec::with_capacity(leg.trajectory.len());
        let mut deputy_states = Vec::with_capacity(leg.trajectory.len());

        for state in &leg.trajectory {
            let chief_sv = keplerian_to_state(&state.chief_mean, state.epoch)?;

            // Chief celestial snapshot (Sun + Moon ephemeris + shadow)
            let snapshot = build_celestial_snapshot(state.epoch, &chief_sv.position_eci_km);

            // Deputy eclipse: reconstruct deputy ECI from chief + RIC offset
            let deputy_eci =
                ric_to_eci_position(&chief_sv, &state.ric.position_ric_km)?;
            // Reconstruct Sun ECI from chief snapshot (avoids redundant ephemeris call).
            // Approximation: chief-centered Sun direction used for deputy eclipse.
            // Deputy-chief separation (~km) << Sun distance (~AU) → ~1e-11 relative error.
            let sun_eci = chief_sv.position_eci_km
                + snapshot.sun_direction_eci * snapshot.sun_distance_km;
            deputy_states.push(compute_eclipse_state(&deputy_eci, &sun_eci));

            chief_snapshots.push(snapshot);
        }

        // Extract intervals for this leg, then merge across leg boundaries.
        let leg_summary = extract_eclipse_intervals(&chief_snapshots);
        merge_intervals(&mut merged_intervals, leg_summary.intervals);

        leg_data.push(LegEclipseData {
            chief_celestial: chief_snapshots,
            deputy_eclipse: deputy_states,
        });
    }

    if leg_data.iter().all(|d| d.chief_celestial.is_empty()) {
        return Err(EclipseComputeError::EmptyTrajectory);
    }

    // Recompute aggregate metrics from merged intervals.
    let total_shadow_duration_s: f64 = merged_intervals.iter().map(|i| i.duration_s).sum();
    let max_shadow_duration_s = merged_intervals
        .iter()
        .map(|i| i.duration_s)
        .fold(0.0_f64, f64::max);

    // Total mission duration from first non-empty leg start to last non-empty leg end.
    let (first_epoch, last_epoch) = leg_data.iter().fold(
        (None, None),
        |(first, prev_last), d| {
            let f = d.chief_celestial.first().map(|s| s.epoch);
            let l = d.chief_celestial.last().map(|s| s.epoch);
            (first.or(f), l.or(prev_last))
        },
    );

    let time_in_shadow_fraction = match (first_epoch, last_epoch) {
        (Some(first), Some(last)) => {
            let total_s = (last - first).to_seconds();
            if total_s > 0.0 { total_shadow_duration_s / total_s } else { 0.0 }
        }
        _ => 0.0,
    };

    Ok(MissionEclipseData {
        summary: EclipseSummary {
            intervals: merged_intervals,
            total_shadow_duration_s,
            time_in_shadow_fraction,
            max_shadow_duration_s,
        },
        legs: leg_data,
    })
}

/// Merge per-leg intervals into a running merged list.
///
/// If the last interval in `merged` ends at or after the start of the first
/// interval in `new`, they are joined into one contiguous interval (the chief
/// does not maneuver, so shadow state is continuous across leg boundaries).
fn merge_intervals(
    merged: &mut Vec<EclipseInterval>,
    new: Vec<EclipseInterval>,
) {
    for interval in new {
        if let Some(last) = merged.last_mut()
            && last.end >= interval.start
        {
            // Contiguous — extend the existing interval
            last.end = interval.end;
            last.duration_s = (last.end - last.start).to_seconds();
            // Keep worst-case state
            if is_deeper_shadow(&interval.state, &last.state) {
                last.state = interval.state;
            }
            continue;
        }
        merged.push(interval);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::EclipseState;
    use hifitime::Epoch;

    /// Tolerance for f64 duration roundtrip through `hifitime::Epoch` subtraction.
    /// Epoch arithmetic is nanosecond-precise; f64 representation introduces ~1e-14.
    const DURATION_ROUNDTRIP_TOL: f64 = 1e-12;

    fn make_interval(start_s: f64, end_s: f64, state: EclipseState) -> EclipseInterval {
        let epoch_base = Epoch::from_gregorian_utc_at_midnight(2024, 1, 1);
        let start = epoch_base + hifitime::Duration::from_seconds(start_s);
        let end = epoch_base + hifitime::Duration::from_seconds(end_s);
        EclipseInterval {
            start,
            end,
            duration_s: end_s - start_s,
            state,
        }
    }

    // ===================================================================
    // merge_intervals tests
    // ===================================================================

    /// Non-overlapping intervals from two legs remain separate.
    #[test]
    fn merge_disjoint_intervals() {
        let mut merged = vec![make_interval(0.0, 100.0, EclipseState::Umbra)];
        let new = vec![make_interval(200.0, 300.0, EclipseState::Umbra)];
        merge_intervals(&mut merged, new);

        assert_eq!(merged.len(), 2, "disjoint intervals should not merge");
        assert!((merged[0].duration_s - 100.0).abs() < DURATION_ROUNDTRIP_TOL);
        assert!((merged[1].duration_s - 100.0).abs() < DURATION_ROUNDTRIP_TOL);
    }

    /// Contiguous intervals (last.end == new.start) are merged into one.
    #[test]
    fn merge_contiguous_intervals() {
        let mut merged = vec![make_interval(0.0, 100.0, EclipseState::Umbra)];
        let new = vec![make_interval(100.0, 250.0, EclipseState::Umbra)];
        merge_intervals(&mut merged, new);

        assert_eq!(merged.len(), 1, "contiguous intervals should merge");
        assert!(
            (merged[0].duration_s - 250.0).abs() < DURATION_ROUNDTRIP_TOL,
            "merged duration should span both intervals"
        );
    }

    /// Overlapping intervals are merged, and the deeper shadow state is kept.
    #[test]
    fn merge_overlapping_promotes_deeper_state() {
        let mut merged = vec![make_interval(
            0.0,
            150.0,
            EclipseState::Penumbra { shadow_fraction: 0.5 },
        )];
        let new = vec![make_interval(100.0, 300.0, EclipseState::Umbra)];
        merge_intervals(&mut merged, new);

        assert_eq!(merged.len(), 1, "overlapping intervals should merge");
        assert!(
            (merged[0].duration_s - 300.0).abs() < DURATION_ROUNDTRIP_TOL,
            "merged duration should span full range"
        );
        assert!(
            matches!(merged[0].state, EclipseState::Umbra),
            "deeper shadow state (Umbra) should be kept"
        );
    }

    /// Merging an empty `new` list leaves `merged` unchanged.
    #[test]
    fn merge_empty_new_is_noop() {
        let mut merged = vec![make_interval(0.0, 100.0, EclipseState::Umbra)];
        merge_intervals(&mut merged, Vec::new());

        assert_eq!(merged.len(), 1);
        assert!((merged[0].duration_s - 100.0).abs() < DURATION_ROUNDTRIP_TOL);
    }

    /// Merging into an empty `merged` list populates it with all `new` intervals.
    #[test]
    fn merge_into_empty_populates() {
        let mut merged = Vec::new();
        let new = vec![
            make_interval(0.0, 100.0, EclipseState::Umbra),
            make_interval(200.0, 300.0, EclipseState::Umbra),
        ];
        merge_intervals(&mut merged, new);

        assert_eq!(merged.len(), 2);
    }

    /// Multiple new intervals: first merges with existing, second is disjoint.
    #[test]
    fn merge_mixed_contiguous_and_disjoint() {
        let mut merged = vec![make_interval(0.0, 100.0, EclipseState::Umbra)];
        let new = vec![
            make_interval(100.0, 200.0, EclipseState::Umbra),  // contiguous
            make_interval(500.0, 600.0, EclipseState::Umbra),  // disjoint
        ];
        merge_intervals(&mut merged, new);

        assert_eq!(merged.len(), 2, "contiguous should merge, disjoint should not");
        assert!((merged[0].duration_s - 200.0).abs() < DURATION_ROUNDTRIP_TOL);
        assert!((merged[1].duration_s - 100.0).abs() < DURATION_ROUNDTRIP_TOL);
    }

    // ===================================================================
    // compute_mission_eclipse tests
    // ===================================================================

    /// All legs with empty trajectories returns `EmptyTrajectory` error.
    #[test]
    fn all_empty_trajectories_returns_error() {
        let legs = vec![ManeuverLeg {
            trajectory: Vec::new(),
            ..test_leg_stub()
        }];

        let result = compute_mission_eclipse(&legs);
        assert!(
            matches!(result, Err(EclipseComputeError::EmptyTrajectory)),
            "all-empty trajectories should return EmptyTrajectory, got {result:?}"
        );
    }

    /// Eclipse data for a single non-empty leg has correct structure.
    #[test]
    fn single_leg_eclipse_structure() {
        let legs = vec![make_test_leg()];

        let result = compute_mission_eclipse(&legs).expect("should succeed");

        assert_eq!(result.legs.len(), 1, "should have 1 leg of eclipse data");
        assert_eq!(
            result.legs[0].chief_celestial.len(),
            legs[0].trajectory.len(),
            "celestial snapshot count should match trajectory length"
        );
        assert_eq!(
            result.legs[0].deputy_eclipse.len(),
            legs[0].trajectory.len(),
            "deputy eclipse count should match trajectory length"
        );
    }

    /// Aggregate metrics are self-consistent.
    #[test]
    fn aggregate_metrics_consistency() {
        let legs = vec![make_test_leg()];

        let result = compute_mission_eclipse(&legs).expect("should succeed");

        // total_shadow_duration_s should equal sum of interval durations
        let sum: f64 = result.summary.intervals.iter().map(|i| i.duration_s).sum();
        assert!(
            (result.summary.total_shadow_duration_s - sum).abs() < DURATION_ROUNDTRIP_TOL,
            "total_shadow_duration_s should equal sum of interval durations"
        );

        // max_shadow_duration_s should equal the longest interval
        let max = result.summary.intervals.iter()
            .map(|i| i.duration_s)
            .fold(0.0_f64, f64::max);
        assert!(
            (result.summary.max_shadow_duration_s - max).abs() < DURATION_ROUNDTRIP_TOL,
            "max_shadow_duration_s should equal longest interval"
        );

        // time_in_shadow_fraction should be in [0, 1]
        assert!(
            (0.0..=1.0).contains(&result.summary.time_in_shadow_fraction),
            "time_in_shadow_fraction should be in [0, 1], got {}",
            result.summary.time_in_shadow_fraction,
        );
    }

    // ===================================================================
    // Test helpers
    // ===================================================================

    use crate::mission::config::MissionConfig;
    use crate::mission::waypoints::plan_waypoint_mission;
    use crate::mission::types::Waypoint;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::{DepartureState, QuasiNonsingularROE};
    use nalgebra::Vector3;

    /// Stub `ManeuverLeg` with empty trajectory for error-path tests.
    fn test_leg_stub() -> ManeuverLeg {
        let epoch = test_epoch();
        ManeuverLeg {
            departure_maneuver: crate::mission::types::Maneuver {
                dv_ric_km_s: Vector3::zeros(),
                epoch,
            },
            arrival_maneuver: crate::mission::types::Maneuver {
                dv_ric_km_s: Vector3::zeros(),
                epoch,
            },
            tof_s: 0.0,
            total_dv_km_s: 0.0,
            pre_departure_roe: QuasiNonsingularROE::default(),
            post_departure_roe: QuasiNonsingularROE::default(),
            departure_chief_mean: iss_like_elements(),
            pre_arrival_roe: QuasiNonsingularROE::default(),
            post_arrival_roe: QuasiNonsingularROE::default(),
            arrival_chief_mean: iss_like_elements(),
            trajectory: Vec::new(),
            from_position_ric_km: Vector3::zeros(),
            to_position_ric_km: Vector3::zeros(),
            target_velocity_ric_km_s: Vector3::zeros(),
            iterations: 0,
            position_error_km: 0.0,
        }
    }

    /// Build a real single-leg mission via `plan_waypoint_mission` and return its leg.
    fn make_test_leg() -> ManeuverLeg {
        let departure = DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: iss_like_elements(),
            epoch: test_epoch(),
        };
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig::default();
        let period = std::f64::consts::TAU / departure.chief.mean_motion().unwrap();

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Some(Vector3::zeros()),
            tof_s: Some(period),
        }];

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("test mission should succeed");

        mission.legs.into_iter().next().unwrap()
    }
}

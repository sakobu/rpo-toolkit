//! Eclipse interval extraction and shadow comparison utilities.

use hifitime::Epoch;

use crate::types::{CelestialSnapshot, EclipseInterval, EclipseState, EclipseSummary};

/// Extract eclipse intervals from a sequence of celestial snapshots.
///
/// Walks the snapshot array, detects transitions between Sunlit and shadow
/// states (Penumbra or Umbra), and records each contiguous shadow interval
/// with its start/end epochs and duration.
///
/// Transition times are estimated at the sample point where the state
/// changes. For 1-second sampling, this gives sub-second accuracy on
/// entry/exit times.
///
/// # Arguments
///
/// * `snapshots` — Celestial snapshots (from [`super::compute_celestial_snapshots`]).
///   Each snapshot carries its own `epoch`, so no parallel epoch array is needed.
///
/// # Returns
///
/// [`EclipseSummary`] containing all shadow intervals, total shadow duration,
/// time-in-shadow fraction, and maximum single-interval duration.
/// Returns an empty summary (zero intervals, zero durations) for empty input
/// or single-point input (need >= 2 points for transition detection).
///
/// # Invariants
///
/// - Snapshots should be time-ordered (monotonically increasing epochs);
///   non-monotonic input produces intervals with negative or incorrect durations
/// - Snapshots are assumed well-formed (produced by [`super::compute_celestial_snapshots`])
///
/// # Validity
///
/// - Eclipse entry/exit timing accuracy is bounded by the sampling interval
///   (e.g., 1-second sampling gives sub-second transition accuracy)
/// - Shadow fraction within intervals uses worst-case (deepest) state, not
///   time-averaged; this is conservative for power budget analysis
/// - No singularities; the function is infallible on well-formed input
#[must_use]
pub fn extract_eclipse_intervals(snapshots: &[CelestialSnapshot]) -> EclipseSummary {
    if snapshots.len() < 2 {
        return EclipseSummary {
            intervals: Vec::new(),
            total_shadow_duration_s: 0.0,
            time_in_shadow_fraction: 0.0,
            max_shadow_duration_s: 0.0,
        };
    }

    let mut intervals: Vec<EclipseInterval> = Vec::with_capacity(16);
    let mut shadow_start: Option<Epoch> = None;
    let mut worst_state = EclipseState::Sunlit;

    for snapshot in snapshots {
        let in_shadow = snapshot.eclipse_state != EclipseState::Sunlit;

        match (in_shadow, shadow_start) {
            // Entering shadow
            (true, None) => {
                shadow_start = Some(snapshot.epoch);
                worst_state = snapshot.eclipse_state;
            }
            // Continuing in shadow — track worst state
            (true, Some(_)) => {
                if is_deeper_shadow(&snapshot.eclipse_state, &worst_state) {
                    worst_state = snapshot.eclipse_state;
                }
            }
            // Exiting shadow — close interval
            (false, Some(start)) => {
                let duration_s = (snapshot.epoch - start).to_seconds();
                intervals.push(EclipseInterval {
                    start,
                    end: snapshot.epoch,
                    duration_s,
                    state: worst_state,
                });
                shadow_start = None;
                worst_state = EclipseState::Sunlit;
            }
            // Remaining sunlit
            (false, None) => {}
        }
    }

    // Close any open interval at the last snapshot
    if let Some(start) = shadow_start
        && let Some(last) = snapshots.last()
    {
        let duration_s = (last.epoch - start).to_seconds();
        intervals.push(EclipseInterval {
            start,
            end: last.epoch,
            duration_s,
            state: worst_state,
        });
    }

    // Compute aggregate metrics in a single pass
    let (total_shadow_duration_s, max_shadow_duration_s) = intervals
        .iter()
        .fold((0.0_f64, 0.0_f64), |(sum, max_val), i| {
            (sum + i.duration_s, max_val.max(i.duration_s))
        });

    // Safety: we returned early above when snapshots.len() < 2
    let total_mission_duration_s =
        (snapshots[snapshots.len() - 1].epoch - snapshots[0].epoch).to_seconds();

    let time_in_shadow_fraction = if total_mission_duration_s > 0.0 {
        total_shadow_duration_s / total_mission_duration_s
    } else {
        0.0
    };

    EclipseSummary {
        intervals,
        total_shadow_duration_s,
        time_in_shadow_fraction,
        max_shadow_duration_s,
    }
}

/// Returns `true` if `new` represents a deeper shadow than `current`.
///
/// Depth ordering: Umbra > Penumbra (by increasing `shadow_fraction`) > Sunlit.
/// Returns `false` for equal or lesser shadow states, and for transitions
/// from any state to Sunlit.
pub(crate) fn is_deeper_shadow(new: &EclipseState, current: &EclipseState) -> bool {
    match (new, current) {
        (EclipseState::Umbra, EclipseState::Penumbra { .. }) => true,
        (
            EclipseState::Penumbra {
                shadow_fraction: f_new,
            },
            EclipseState::Penumbra {
                shadow_fraction: f_cur,
            },
        ) => f_new > f_cur,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::eclipse::{compute_celestial_snapshots, sun_position_eci_km};

    /// Expected ISS eclipse duration per orbit (~35 min = 2100 s).
    /// Published: ~36 min typical for low beta angle.
    const ISS_ECLIPSE_DURATION_EXPECTED_S: f64 = 2100.0;

    /// Tolerance on ISS eclipse duration (±5 min = 300 s).
    /// Accounts for Sun position uncertainty (~0.01 deg), beta angle variation,
    /// sampling discretization (1 deg M steps), and mean-vs-osculating offset.
    const ISS_ECLIPSE_DURATION_TOL_S: f64 = 300.0;

    /// Expected ~15 eclipses in 24 hours for ISS (15.5 orbits/day, each with one eclipse).
    const ISS_24H_ECLIPSE_COUNT_EXPECTED: usize = 15;

    /// Tolerance on 24h eclipse count (±2). Accounts for partial eclipses at
    /// start/end of 24h window and beta angle variation.
    const ISS_24H_ECLIPSE_COUNT_TOL: usize = 2;

    /// ~15 eclipses in a 24-hour ISS orbit propagation.
    ///
    /// Published: ISS completes ~15.5 orbits/day, each with one eclipse passage.
    /// We generate 16 orbits (~24.6 hours) and verify the eclipse count.
    #[test]
    fn iss_orbit_eclipse_count() {
        let points = super::super::test_fixtures::iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        let count = summary.intervals.len();
        let lo = ISS_24H_ECLIPSE_COUNT_EXPECTED - ISS_24H_ECLIPSE_COUNT_TOL;
        let hi = ISS_24H_ECLIPSE_COUNT_EXPECTED + ISS_24H_ECLIPSE_COUNT_TOL;
        assert!(
            (lo..=hi).contains(&count),
            "Expected {lo}–{hi} eclipses in ~24h ISS orbit, got {count}"
        );
    }

    /// Each ISS eclipse lasts ~35 minutes.
    ///
    /// Published: ~36 min typical for low beta angle. We propagate 3 orbits
    /// and check only the middle interval(s), which are guaranteed not to be
    /// truncated by the trajectory boundary.
    #[test]
    fn iss_eclipse_duration() {
        let points = super::super::test_fixtures::iss_trajectory_points(3);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        assert!(
            summary.intervals.len() >= 2,
            "Expected at least 2 eclipses in 3 ISS orbits, got {}",
            summary.intervals.len()
        );

        // Skip first and last intervals (may be truncated at trajectory boundary)
        let interior = &summary.intervals[1..summary.intervals.len() - 1];
        assert!(
            !interior.is_empty(),
            "Expected at least one interior eclipse interval in 3 orbits"
        );

        for interval in interior {
            assert!(
                (interval.duration_s - ISS_ECLIPSE_DURATION_EXPECTED_S).abs()
                    < ISS_ECLIPSE_DURATION_TOL_S,
                "Eclipse duration {:.0}s outside expected {:.0}±{:.0}s",
                interval.duration_s,
                ISS_ECLIPSE_DURATION_EXPECTED_S,
                ISS_ECLIPSE_DURATION_TOL_S
            );
        }
    }

    /// Dawn-dusk SSO at ~98° inclination has minimal or no eclipses.
    ///
    /// A dawn-dusk sun-synchronous orbit has its orbital plane aligned with
    /// the terminator, so the spacecraft stays in near-permanent sunlight.
    /// The RAAN is set 90° ahead of the Sun's right ascension at the test epoch.
    #[test]
    fn sun_sync_dawn_dusk_no_eclipse() {
        use crate::constants::TWO_PI;

        let epoch = crate::test_helpers::test_epoch();

        // Compute Sun RA at test epoch to set the dawn-dusk RAAN
        let sun_pos = sun_position_eci_km(epoch);
        let sun_ra = sun_pos.y.atan2(sun_pos.x);
        let dawn_dusk_raan = sun_ra + std::f64::consts::FRAC_PI_2;

        let base = crate::types::KeplerianElements {
            a_km: 7078.135,
            e: 0.001,
            i_rad: 98.19_f64.to_radians(),
            raan_rad: dawn_dusk_raan,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };

        // Generate 1-orbit trajectory (360 points)
        let n_steps: u32 = 360;
        let period_s = base.period().expect("period computation");
        let dt = period_s / f64::from(n_steps);
        let mut points = Vec::with_capacity(n_steps as usize);
        for k in 0..n_steps {
            let t = f64::from(k) * dt;
            let m = f64::from(k) * TWO_PI / f64::from(n_steps);
            let mut elements = base;
            elements.mean_anomaly_rad = m.rem_euclid(TWO_PI);
            points.push((
                epoch + hifitime::Duration::from_seconds(t),
                elements,
            ));
        }

        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        assert!(
            summary.time_in_shadow_fraction < 0.02,
            "Dawn-dusk SSO eclipse fraction {:.1}% exceeds 2% threshold",
            summary.time_in_shadow_fraction * 100.0
        );
    }

    /// Sum of all interval durations equals `total_shadow_duration_s`.
    #[test]
    fn interval_continuity() {
        let points = super::super::test_fixtures::iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        let manual_sum: f64 = summary.intervals.iter().map(|i| i.duration_s).sum();
        let n = u32::try_from(summary.intervals.len()).expect("interval count fits u32");
        let tol = f64::EPSILON * f64::from(n) * summary.total_shadow_duration_s.max(1.0);
        assert!(
            (manual_sum - summary.total_shadow_duration_s).abs() < tol,
            "Interval sum {manual_sum} != total_shadow_duration_s {}",
            summary.total_shadow_duration_s
        );
    }

    /// Eclipse intervals are sorted by start epoch and non-overlapping.
    #[test]
    fn monotonic_epochs() {
        let points = super::super::test_fixtures::iss_trajectory_points(16);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        let summary = extract_eclipse_intervals(&snapshots);

        for interval in &summary.intervals {
            assert!(
                interval.start < interval.end,
                "Interval start {:?} >= end {:?}",
                interval.start,
                interval.end
            );
            assert!(
                interval.duration_s > 0.0,
                "Interval duration must be positive, got {}",
                interval.duration_s
            );
        }

        for pair in summary.intervals.windows(2) {
            assert!(
                pair[0].end <= pair[1].start,
                "Intervals overlap: first ends {:?}, next starts {:?}",
                pair[0].end,
                pair[1].start
            );
        }
    }
}

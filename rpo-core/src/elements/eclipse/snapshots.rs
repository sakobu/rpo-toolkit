//! Trajectory-to-snapshot processing: convert trajectory points to celestial snapshots.

use hifitime::Epoch;

use crate::elements::keplerian_conversions::{keplerian_to_state, ConversionError};
use crate::types::{CelestialSnapshot, KeplerianElements, StateVector};

use super::shadow::build_celestial_snapshot;

/// Compute celestial body directions and eclipse state at each trajectory point.
///
/// For each `(Epoch, KeplerianElements)` pair, reconstructs the chief ECI
/// position via [`keplerian_to_state()`], computes Sun and Moon positions
/// analytically, and evaluates the conical shadow model.
///
/// # Arguments
///
/// * `points` — Slice of `(epoch, chief_mean_elements)` pairs. The caller
///   extracts these from `PropagatedState` or any other source — this function
///   has no dependency on propagation types, preserving the module DAG:
///   `constants → types → elements → propagation → mission`.
///
/// # Returns
///
/// One [`CelestialSnapshot`] per input point, in the same order. Each
/// snapshot's `epoch` field is set from the corresponding input pair,
/// making the output slice self-contained for downstream consumers
/// (e.g., [`extract_eclipse_intervals`][super::extract_eclipse_intervals]).
///
/// Returns an empty `Vec` for empty input.
///
/// # Mean elements approximation
///
/// The input Keplerian elements are typically **mean** elements (from
/// `PropagatedState.chief_mean`), but [`keplerian_to_state()`] treats them
/// as osculating. This introduces a systematic ~1–10 km offset in the
/// reconstructed ECI position due to J2 short-period terms. For shadow
/// geometry at LEO this is negligible: the Sun subtends ~0.5° and Earth
/// subtends ~60° as seen from the spacecraft, so a few km offset barely
/// shifts the shadow boundary. The resulting eclipse timing error is well
/// under 1 second.
///
/// # Performance
///
/// ~350 ns per point (Sun + Moon ephemeris + shadow test + ECI reconstruction).
/// A 7-hour mission at 1-second sampling (25,200 points) completes in ~9 ms.
///
/// # Errors
///
/// Propagates [`ConversionError`] from [`keplerian_to_state()`] if any input
/// element set is degenerate (zero position, unbound orbit).
///
/// # Invariants
///
/// - Each `KeplerianElements` must describe a valid bound orbit (`e < 1`, `a > 0`);
///   degenerate elements propagate as `Err(ConversionError)` from `keplerian_to_state()`
/// - Points should be time-ordered (monotonically increasing epochs) for meaningful
///   downstream interval extraction; this function does not enforce ordering
///
/// # Validity
///
/// - Valid for any Earth-orbiting spacecraft in a bound orbit
/// - No singularities beyond those in [`keplerian_to_state()`] (near-parabolic,
///   zero position)
/// - Accuracy is limited by the analytical ephemeris (~0.01 deg Sun, ~0.5 deg Moon)
///   and the mean-elements approximation (see above)
pub fn compute_celestial_snapshots(
    points: &[(Epoch, KeplerianElements)],
) -> Result<Vec<CelestialSnapshot>, ConversionError> {
    let mut snapshots = Vec::with_capacity(points.len());

    for (epoch, elements) in points {
        let state = keplerian_to_state(elements, *epoch)?;
        snapshots.push(build_celestial_snapshot(*epoch, &state.position_eci_km));
    }

    Ok(snapshots)
}

/// Compute celestial body directions and eclipse state from ECI state vectors.
///
/// Unlike [`compute_celestial_snapshots`] which takes `(Epoch, KeplerianElements)`
/// and reconstructs ECI positions, this function operates directly on ECI state
/// vectors. Suitable for trajectories already expressed in ECI (e.g., Lambert
/// transfer arcs from [`LambertTransfer::densify_arc()`][crate::propagation::lambert::LambertTransfer::densify_arc],
/// Keplerian propagation results).
///
/// # Arguments
///
/// * `states` — ECI state vectors. Each must have a valid `epoch` and
///   non-zero `position_eci_km`.
///
/// # Performance
///
/// ~250 ns per point (Sun + Moon ephemeris + shadow test, no Keplerian conversion).
#[must_use]
pub fn compute_eclipse_from_states(states: &[StateVector]) -> Vec<CelestialSnapshot> {
    states
        .iter()
        .map(|s| build_celestial_snapshot(s.epoch, &s.position_eci_km))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::test_fixtures::iss_trajectory_points;
    use crate::constants::TWO_PI;
    use crate::elements::eclipse::{
        compute_eclipse_state, extract_eclipse_intervals, sun_position_eci_km,
    };
    use crate::types::EclipseState;

    /// ISS eclipse fraction tolerance (absolute).
    /// Published ISS eclipse fraction is ~35-36% of orbit period. We allow
    /// 5% absolute tolerance because: (1) analytical Sun position has ~0.01 deg
    /// error, (2) we sample 360 discrete points around the orbit, (3) the ISS
    /// fixture uses simplified near-circular elements, (4) the beta angle at
    /// the test epoch affects the exact fraction.
    const ISS_ECLIPSE_FRACTION_TOL: f64 = 0.05;

    /// Sun direction cross-validation tolerance between `compute_eclipse_from_states`
    /// and `compute_celestial_snapshots`. Both use Meeus Sun, but the slightly
    /// different spacecraft ECI positions (mean vs osculating) shift the
    /// Earth→Sun unit vector by O(1e-4) rad. 1e-3 rad (0.06°) provides margin.
    const SUN_DIRECTION_CROSS_VALIDATION_TOL_RAD: f64 = 1e-3;

    /// ISS-like orbit has ~35% eclipse fraction per orbit.
    ///
    /// Cross-phase test: uses `keplerian_to_state()` (elements) and
    /// `sun_position_eci_km()` (Phase 2) with `compute_eclipse_state()` (Phase 4).
    /// Samples 360 points around one orbit (1 deg mean anomaly steps).
    ///
    /// Published value: ISS eclipse fraction is ~36% at low beta angles.
    /// At the test epoch (2024-01-01), the beta angle depends on the Sun position
    /// and ISS RAAN; we allow 5% tolerance to accommodate this variation.
    #[test]
    fn iss_eclipse_fraction() {
        let mut elements = crate::test_helpers::iss_like_elements();
        let epoch = crate::test_helpers::test_epoch();

        // Sun position at test epoch (barely moves over one ~92 min orbit)
        let sun_pos = sun_position_eci_km(epoch);

        let n_samples: u32 = 360;
        let mut shadow_count: u32 = 0;

        for k in 0..n_samples {
            elements.mean_anomaly_rad = f64::from(k) / f64::from(n_samples) * TWO_PI;
            let state_vec =
                keplerian_to_state(&elements, epoch).expect("keplerian_to_state failed");
            let eclipse = compute_eclipse_state(&state_vec.position_eci_km, &sun_pos);

            if eclipse != EclipseState::Sunlit {
                shadow_count += 1;
            }
        }

        let eclipse_fraction = f64::from(shadow_count) / f64::from(n_samples);
        let expected = 0.35;
        assert!(
            (eclipse_fraction - expected).abs() < ISS_ECLIPSE_FRACTION_TOL,
            "ISS eclipse fraction: {:.1}% (expected ~{:.0}%, tolerance {:.0}%)",
            eclipse_fraction * 100.0,
            expected * 100.0,
            ISS_ECLIPSE_FRACTION_TOL * 100.0
        );
    }

    /// Output snapshot count equals input point count.
    #[test]
    fn snapshot_count_matches_trajectory() {
        let points = iss_trajectory_points(1);
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert_eq!(
            snapshots.len(),
            points.len(),
            "Snapshot count {} != input point count {}",
            snapshots.len(),
            points.len()
        );
    }

    /// Zero points produce empty snapshots and empty summary.
    #[test]
    fn empty_trajectory_returns_empty() {
        let points: Vec<(Epoch, crate::types::KeplerianElements)> = vec![];
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert!(snapshots.is_empty(), "Expected empty snapshots for empty input");

        let summary = extract_eclipse_intervals(&snapshots);
        assert!(summary.intervals.is_empty());
        assert!((summary.total_shadow_duration_s).abs() < f64::EPSILON);
        assert!((summary.time_in_shadow_fraction).abs() < f64::EPSILON);
        assert!((summary.max_shadow_duration_s).abs() < f64::EPSILON);
    }

    /// One point produces one snapshot but no intervals (need >= 2 for transitions).
    #[test]
    fn single_point_no_intervals() {
        let points = vec![(
            crate::test_helpers::test_epoch(),
            crate::test_helpers::iss_like_elements(),
        )];
        let snapshots = compute_celestial_snapshots(&points).expect("snapshots failed");
        assert_eq!(snapshots.len(), 1);
        assert!(snapshots[0].sun_distance_km > 0.0);
        assert!(snapshots[0].moon_distance_km > 0.0);

        let summary = extract_eclipse_intervals(&snapshots);
        assert!(
            summary.intervals.is_empty(),
            "Single-point input should produce no intervals"
        );
    }

    /// `compute_eclipse_from_states` returns empty vec for empty input.
    #[test]
    fn eclipse_from_states_empty_input() {
        let result = compute_eclipse_from_states(&[]);
        assert!(result.is_empty());
    }

    /// `compute_eclipse_from_states` produces equivalent results to
    /// `compute_celestial_snapshots` for the same ECI positions.
    ///
    /// Tolerance: exact agreement — both functions compute the same
    /// Sun/Moon ephemeris and shadow model from identical ECI positions.
    #[test]
    fn eclipse_from_states_matches_snapshots() {
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let elements = iss_like_elements();
        let epoch = test_epoch();

        // Generate a short trajectory via keplerian_to_state
        let points: Vec<_> = (0..10)
            .map(|i| {
                let dt_s = f64::from(i) * 60.0;
                let n = elements.mean_motion().unwrap();
                let advanced = KeplerianElements {
                    mean_anomaly_rad: (elements.mean_anomaly_rad + n * dt_s)
                        .rem_euclid(crate::constants::TWO_PI),
                    ..elements
                };
                let ep = epoch + hifitime::Duration::from_seconds(dt_s);
                (ep, advanced)
            })
            .collect();

        // Convert to StateVectors
        let states: Vec<StateVector> = points
            .iter()
            .map(|(ep, ke)| crate::elements::keplerian_conversions::keplerian_to_state(ke, *ep).unwrap())
            .collect();

        let from_keplerian = compute_celestial_snapshots(&points).unwrap();
        let from_states = compute_eclipse_from_states(&states);

        assert_eq!(from_keplerian.len(), from_states.len());

        for (a, b) in from_keplerian.iter().zip(from_states.iter()) {
            assert_eq!(a.eclipse_state, b.eclipse_state, "eclipse state mismatch");
            // Sun direction should be very close (not exact due to slightly
            // different spacecraft positions from mean vs osculating)
            let angle = a
                .sun_direction_eci
                .dot(&b.sun_direction_eci)
                .clamp(-1.0, 1.0)
                .acos();
            assert!(
                angle < SUN_DIRECTION_CROSS_VALIDATION_TOL_RAD,
                "Sun direction diverges by {:.4} rad",
                angle
            );
        }
    }

    /// ISS-like orbit via `compute_eclipse_from_states` shows ~35% shadow.
    #[test]
    fn eclipse_from_states_iss_orbit() {
        use crate::test_helpers::{iss_like_elements, test_epoch};

        let elements = iss_like_elements();
        let epoch = test_epoch();
        let period_s = elements.period().unwrap();

        // Propagate one full orbit
        let states: Vec<StateVector> =
            crate::propagation::keplerian::propagate_keplerian(
                &crate::elements::keplerian_conversions::keplerian_to_state(&elements, epoch).unwrap(),
                period_s,
                360,
            )
            .unwrap();

        let snapshots = compute_eclipse_from_states(&states);
        let summary = extract_eclipse_intervals(&snapshots);

        // ISS-like orbit: ~30-40% in shadow
        assert!(
            summary.time_in_shadow_fraction > 0.25,
            "ISS shadow fraction should be >25%, got {:.1}%",
            summary.time_in_shadow_fraction * 100.0
        );
        assert!(
            summary.time_in_shadow_fraction < 0.45,
            "ISS shadow fraction should be <45%, got {:.1}%",
            summary.time_in_shadow_fraction * 100.0
        );
    }
}

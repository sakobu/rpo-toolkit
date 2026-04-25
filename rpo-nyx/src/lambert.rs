//! Lambert transfer solver for far-field orbital transfers.
//!
//! # Solver selection
//!
//! - **Single-rev (`revolutions == 0`)**: nyx-space's Gooding solver. Gooding
//!   honors `ShortWay`/`LongWay` unambiguously via a direction-of-motion
//!   multiplier (`+1.0` / `-1.0`), so the Direction toggle maps to two
//!   distinct Lambert solutions for *every* geometry.
//! - **Multi-rev (`revolutions > 0`)**: Izzo's method. Gooding does not
//!   support multi-rev; nyx returns `MultiRevNotSupported`.
//!
//! Izzo's short/long-way selection is coupled to the geometry check
//! `i_h.z < 0.0 || retrograde` in nyx-space 2.3.1, so for any arc with
//! naturally-retrograde geometry (`r_init × r_final` has negative z)
//! `TransferKind::ShortWay` and `TransferKind::LongWay` collapse to the same
//! solution. Gooding is unaffected by this and is the right choice for
//! single-rev user-facing direction control. Multi-rev users get the
//! documented Izzo behavior.
//!
//! # Crate boundary
//!
//! Lambert types ([`LambertConfig`], [`LambertError`], [`LambertTransfer`],
//! [`TransferDirection`]) live in `rpo_core::propagation::lambert` so they are
//! available to WASM consumers. This module provides the nyx-backed solver
//! functions that depend on nyx-space.
//!
//! # References
//!
//! - Gooding, R. H. "A procedure for the solution of Lambert's orbital
//!   boundary-value problem." *Celestial Mechanics and Dynamical Astronomy*
//!   48.2 (1990): 145–165. Used for single-rev (`revolutions == 0`);
//!   nyx 2.3.1 `nyx_space::tools::lambert::gooding`.
//! - Izzo, D. "Revisiting Lambert's Problem." *Celestial Mechanics and
//!   Dynamical Astronomy* 121.1 (2015): 1–15. Used for multi-rev
//!   (`revolutions > 0`); nyx 2.3.1 `nyx_space::tools::lambert::izzo`.
//! - nyx-space 2.3.1 Izzo bug (`izzo.rs`): `TransferKind::Auto` uses
//!   `r_init[1].atan2(r_final[1])` (typo — the second argument should
//!   reference `r_final[0]`); `ShortWay`/`LongWay` dispatch is coupled to
//!   `i_h.z < 0.0 || retrograde`, collapsing both branches for any arc
//!   whose angular-momentum unit vector points to −z. See
//!   `docs/nyx-lambert-bug-report.md` for the full diagnosis.

use nalgebra::Vector3;
use nyx_space::tools::lambert::{self, LambertInput, LambertSolution, TransferKind};

use rpo_core::constants::LAMBERT_MIN_SEPARATION_KM;
use rpo_core::propagation::lambert::{
    LambertConfig, LambertError, LambertTransfer, TransferDirection, TransferFeasibility,
};
use rpo_core::types::StateVector;

use crate::nyx_bridge::state_to_orbit;

/// Validate Lambert inputs.
///
/// Checks that TOF > 0 and position separation > `LAMBERT_MIN_SEPARATION_KM`.
fn validate_inputs(
    departure: &StateVector,
    arrival: &StateVector,
) -> Result<f64, LambertError> {
    let tof = (arrival.epoch - departure.epoch).to_seconds();
    if tof <= 0.0 {
        return Err(LambertError::NonPositiveTimeOfFlight { tof_s: tof });
    }

    let sep = (arrival.position_eci_km - departure.position_eci_km).norm();
    if sep < LAMBERT_MIN_SEPARATION_KM {
        return Err(LambertError::IdenticalPositions { separation_km: sep });
    }

    Ok(tof)
}

/// Build a [`LambertTransfer`] from computed transfer velocities.
///
/// Pure math kernel — internal to `rpo-nyx` so it can keep the raw scalar
/// argument list that mirrors the solver solution directly. Public solver
/// entry points wrap this behind the struct-based API (see
/// [`solve_lambert_with_config`]).
#[must_use]
pub(crate) fn build_transfer(
    departure: &StateVector,
    arrival: &StateVector,
    v1_vec: Vector3<f64>,
    v2_vec: Vector3<f64>,
    tof_s: f64,
    c3_km2_s2: f64,
    direction: TransferDirection,
) -> LambertTransfer {
    let departure_dv = v1_vec - departure.velocity_eci_km_s;
    let arrival_dv = arrival.velocity_eci_km_s - v2_vec;
    let total_dv = departure_dv.norm() + arrival_dv.norm();
    let feasibility = TransferFeasibility::from_state(departure.position_eci_km, v1_vec);

    LambertTransfer {
        departure_state: StateVector {
            epoch: departure.epoch,
            position_eci_km: departure.position_eci_km,
            velocity_eci_km_s: v1_vec,
        },
        arrival_state: StateVector {
            epoch: arrival.epoch,
            position_eci_km: arrival.position_eci_km,
            velocity_eci_km_s: v2_vec,
        },
        departure_dv_eci_km_s: departure_dv,
        arrival_dv_eci_km_s: arrival_dv,
        total_dv_km_s: total_dv,
        tof_s,
        c3_km2_s2,
        direction,
        feasibility,
    }
}

/// Map a [`TransferDirection`] onto the single-rev nyx [`TransferKind`]
/// variants.
///
/// Caller must ensure single-rev context — multi-rev routes through
/// `TransferKind::NRevs(u8)` directly in [`solve_lambert_with_config`] and
/// bypasses this helper.
fn direction_to_kind(direction: TransferDirection) -> TransferKind {
    match direction {
        TransferDirection::Auto => TransferKind::Auto,
        TransferDirection::ShortWay => TransferKind::ShortWay,
        TransferDirection::LongWay => TransferKind::LongWay,
    }
}

/// Build a [`LambertTransfer`] from a nyx [`LambertSolution`].
fn transfer_from_solution(
    departure: &StateVector,
    arrival: &StateVector,
    solution: &LambertSolution,
    tof: f64,
    direction: TransferDirection,
) -> LambertTransfer {
    let v1_vec = solution.v_init_km_s;
    let v2_vec = solution.v_final_km_s;

    build_transfer(
        departure,
        arrival,
        v1_vec,
        v2_vec,
        tof,
        solution.c3_km2_s2(),
        direction,
    )
}

/// Solve Lambert's problem between two ECI states.
///
/// Uses the default [`LambertConfig`] (short-way direction, single-rev) and
/// routes through Gooding. The default was flipped away from `Auto` until
/// nyx-space ships the upstream fix for `TransferKind::Auto` — see the
/// module-level references.
///
/// # Invariants
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - Departure and arrival positions must be non-degenerate
///   (separation > [`LAMBERT_MIN_SEPARATION_KM`])
/// - Transfer angle must not be exactly 0 or π (degenerate geometry)
///
/// # Near-degenerate behavior
/// For transfer angles within a few degrees of 0 or π, the underlying
/// solver may produce large Δv or return
/// [`LambertError::SolverConvergenceFailure`]. Gooding's convergence
/// envelope is controlled by the upstream `LAMBERT_EPSILON_TIME` tolerance
/// (nyx 2.3.1: `1e-4 s`), which resolves to sub-meter agreement at LEO
/// orbital speeds. The `near_180_degree_transfer` test documents this
/// graceful-degradation envelope.
///
/// # Errors
/// Returns [`LambertError`] if the solver fails or inputs are invalid.
pub fn solve_lambert(
    departure: &StateVector,
    arrival: &StateVector,
) -> Result<LambertTransfer, LambertError> {
    solve_lambert_with_config(departure, arrival, &LambertConfig::default())
}

/// Solve Lambert's problem with explicit configuration.
///
/// Routes single-rev through Gooding (honors direction for every geometry)
/// and multi-rev through Izzo (the only nyx solver that supports `NRevs`).
/// See the module-level documentation for the full solver-selection and
/// upstream-bug rationale.
///
/// # Invariants
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - Departure and arrival positions must be non-degenerate
///   (separation > [`LAMBERT_MIN_SEPARATION_KM`])
/// - For multi-rev (`config.revolutions > 0`), TOF must be long enough to
///   accommodate the requested number of revolutions. `config.revolutions`
///   is `u8`-bounded (≤ 255) — no separate physical cap is enforced; the
///   solver will fail to converge for unreasonably high counts.
/// - Multi-rev ignores `config.direction` — nyx's `TransferKind` has no
///   combined `NRevs { long_way }` variant, and Izzo's geometry-coupled
///   direction handling makes any user override unreliable. Callers that
///   need direction control must stay single-rev.
///
/// # Errors
/// Returns [`LambertError`] if the solver fails or inputs are invalid.
pub fn solve_lambert_with_config(
    departure: &StateVector,
    arrival: &StateVector,
    config: &LambertConfig,
) -> Result<LambertTransfer, LambertError> {
    let tof = validate_inputs(departure, arrival)?;

    let dep_orbit = state_to_orbit(departure);
    let arr_orbit = state_to_orbit(arrival);

    let input = LambertInput::from_planetary_states(dep_orbit, arr_orbit)
        .map_err(|e| LambertError::InvalidInput { details: format!("{e}") })?;

    let solution = if config.revolutions > 0 {
        lambert::izzo(input, TransferKind::NRevs(config.revolutions))
    } else {
        lambert::gooding(input, direction_to_kind(config.direction))
    }
    .map_err(|e| LambertError::SolverConvergenceFailure { details: format!("{e}") })?;

    Ok(transfer_from_solution(
        departure,
        arrival,
        &solution,
        tof,
        config.direction,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::constants::R_EARTH;
    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use rpo_core::test_helpers::{
        ellipse_state_at_apogee, leo_400km_elements, leo_800km_target_elements, test_epoch,
    };
    use rpo_core::types::KeplerianElements;
    use hifitime::Duration;

    /// Keplerian propagation endpoint position agreement with Lambert transfer
    /// endpoints. 1e-3 km (1 m) covers Gooding's velocity convergence envelope
    /// (nyx `LAMBERT_EPSILON_TIME = 1e-4 s` → sub-meter at orbital speeds) plus
    /// accumulated Kepler equation error over the full arc.
    const ARC_ENDPOINT_TOL_KM: f64 = 1e-3;

    /// Lower bound (km/s) for coplanar LEO transfer Δv.
    /// A 400→800 km Hohmann transfer is ~0.2 km/s; 10 m/s is conservative
    /// lower bound well above numerical noise.
    const LEO_COPLANAR_DV_LOWER_KM_S: f64 = 0.01;

    /// Upper bound (km/s) for coplanar LEO transfer Δv.
    /// Maximum LEO Hohmann (200→2000 km) is ~3.9 km/s; 5 km/s provides
    /// margin for non-optimal transfer geometry.
    const LEO_COPLANAR_DV_UPPER_KM_S: f64 = 5.0;

    /// Lower bound (km/s) for non-coplanar transfer Δv.
    /// Plane change at ISS inclination offset (~10 deg RAAN) requires
    /// >100 m/s; bound is well above numerical noise.
    const NON_COPLANAR_DV_LOWER_KM_S: f64 = 0.1;

    /// Upper LEO altitude (km above Earth surface) for orbit radius
    /// reasonableness checks. 2000 km is the canonical LEO upper bound per
    /// ITU-R S.1003 ("Environmental protection of the geostationary
    /// satellite orbit"), adopted by most mission-analysis references as
    /// the LEO/MEO boundary. The 400–800 km transfers exercised here stay
    /// well below this bound; any densified arc point above it indicates
    /// the Lambert arc has escaped LEO, which is a bug.
    const LEO_MAX_ALTITUDE_KM: f64 = 2000.0;

    #[test]
    fn coplanar_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let config = LambertConfig::default();
        let transfer = solve_lambert_with_config(&dep, &arr, &config).expect("Lambert should succeed");

        assert!(
            transfer.total_dv_km_s > LEO_COPLANAR_DV_LOWER_KM_S && transfer.total_dv_km_s < LEO_COPLANAR_DV_UPPER_KM_S,
            "Lambert Δv = {} km/s seems unreasonable",
            transfer.total_dv_km_s
        );
    }

    #[test]
    fn non_coplanar_transfer() {
        let epoch = test_epoch();

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();

        let arr_ke = KeplerianElements {
            a_km: R_EARTH + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 10.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(3600.0)).unwrap();

        let config = LambertConfig::default();
        let transfer = solve_lambert_with_config(&dep, &arr, &config).expect("non-coplanar Lambert failed");

        assert!(
            transfer.total_dv_km_s > NON_COPLANAR_DV_LOWER_KM_S,
            "Non-coplanar Δv should be significant, got {}",
            transfer.total_dv_km_s
        );
    }

    #[test]
    fn multi_rev_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();

        let arr_ke = KeplerianElements {
            a_km: R_EARTH + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 90.0_f64.to_radians(),
        };
        // Long TOF to allow 1-rev solution (~2 orbital periods)
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(12000.0)).unwrap();

        let config = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 1,
        };

        let transfer = solve_lambert_with_config(&dep, &arr, &config).expect("Multi-rev should succeed");
        assert!(
            transfer.total_dv_km_s > 0.0,
            "Multi-rev Δv should be positive"
        );
    }

    #[test]
    fn long_way_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();

        let arr_ke = KeplerianElements {
            a_km: R_EARTH + 600.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 90.0_f64.to_radians(),
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(3600.0)).unwrap();

        let config = LambertConfig {
            direction: TransferDirection::LongWay,
            revolutions: 0,
        };

        let transfer =
            solve_lambert_with_config(&dep, &arr, &config).expect("Long-way transfer should succeed");
        assert!(transfer.total_dv_km_s > 0.0, "Long-way Δv should be positive");
        assert_eq!(transfer.direction, TransferDirection::LongWay);
    }

    #[test]
    fn densify_arc_endpoints_match() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let densified = transfer.densify_arc(100).unwrap();

        let dep_err = (densified.first().unwrap().position_eci_km - transfer.departure_state.position_eci_km).norm();
        let arr_err = (densified.last().unwrap().position_eci_km - transfer.arrival_state.position_eci_km).norm();

        assert!(
            dep_err < ARC_ENDPOINT_TOL_KM,
            "Departure position mismatch: {dep_err} km"
        );
        assert!(arr_err < ARC_ENDPOINT_TOL_KM, "Arrival position mismatch: {arr_err} km");
    }

    #[test]
    fn densify_arc_correct_count() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let densified = transfer.densify_arc(100).unwrap();

        assert_eq!(densified.len(), 101, "Expected 101 points (100 steps + 1)");
    }

    #[test]
    fn densify_arc_monotonic_epochs() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let densified = transfer.densify_arc(50).unwrap();

        for window in densified.windows(2) {
            let t0 = window[0].epoch;
            let t1 = window[1].epoch;
            assert!(
                t1 > t0,
                "Epochs not monotonically increasing: {t0} >= {t1}"
            );
        }
    }

    #[test]
    fn densify_arc_orbit_radius_reasonable() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let densified = transfer.densify_arc(100).unwrap();

        for (k, state) in densified.iter().enumerate() {
            let r = state.position_eci_km.norm();
            assert!(
                r > R_EARTH,
                "Point {k} below Earth surface: r = {r} km"
            );
            assert!(
                r < R_EARTH + LEO_MAX_ALTITUDE_KM,
                "Point {k} radius too large for LEO: r = {r} km"
            );
        }
    }

    #[test]
    fn c3_is_populated() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let config = LambertConfig::default();
        let transfer = solve_lambert_with_config(&dep, &arr, &config).expect("Lambert failed");
        assert!(
            transfer.c3_km2_s2 >= 0.0,
            "C3 should be non-negative, got {}",
            transfer.c3_km2_s2
        );
    }

    /// Verify `NonPositiveTimeOfFlight` is returned when arrival precedes departure.
    #[test]
    fn error_non_positive_tof() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        // Arrival epoch is in the past relative to departure
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch - Duration::from_seconds(60.0),
        ).unwrap();

        let result = solve_lambert(&dep, &arr);
        match result {
            Err(LambertError::NonPositiveTimeOfFlight { tof_s }) => {
                assert!(tof_s < 0.0, "Expected negative TOF, got {tof_s}");
            }
            other => panic!("Expected NonPositiveTimeOfFlight, got {other:?}"),
        }
    }

    /// Verify `IdenticalPositions` is returned when departure and arrival share the same position.
    #[test]
    fn error_identical_positions() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        // Arrival state has the same ECI position as departure but a later epoch
        let mut arr = dep;
        arr.epoch = epoch + Duration::from_seconds(60.0);

        let result = solve_lambert(&dep, &arr);
        match result {
            Err(LambertError::IdenticalPositions { separation_km }) => {
                assert!(
                    separation_km < LAMBERT_MIN_SEPARATION_KM,
                    "Expected near-zero separation, got {separation_km} km"
                );
            }
            other => panic!("Expected IdenticalPositions, got {other:?}"),
        }
    }

    /// Near-180° transfer (degenerate geometry): transfer angle close to π.
    ///
    /// Constructs a departure and arrival where the transfer angle is ~179°.
    /// The Lambert solver may either succeed or return an error for near-degenerate cases.
    /// This test verifies that the solver does not panic.
    #[test]
    fn near_180_degree_transfer() {
        let epoch = test_epoch();
        let dep_ke = leo_400km_elements();
        let dep = keplerian_to_state(&dep_ke, epoch).unwrap();

        // Arrival orbit offset by ~179° in mean anomaly on a slightly different altitude,
        // producing a near-180° transfer angle
        let arr_ke = KeplerianElements {
            a_km: R_EARTH + 450.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 179.0_f64.to_radians(),
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(2700.0)).unwrap();

        let config = LambertConfig::default();
        // Accept either a valid solution or a solver error — must not panic.
        match solve_lambert_with_config(&dep, &arr, &config) {
            Ok(transfer) => {
                // Near-180° geometry may produce large but finite Δv
                assert!(
                    transfer.total_dv_km_s.is_finite() && transfer.total_dv_km_s >= 0.0,
                    "Δv must be finite and non-negative for near-180° transfer, got {}",
                    transfer.total_dv_km_s
                );
            }
            Err(LambertError::InvalidInput { .. } | LambertError::SolverConvergenceFailure { .. }) => {
                // Degenerate geometry is an acceptable failure mode for near-180° transfers.
            }
            Err(e) => panic!("Unexpected error type for near-180° transfer: {e}"),
        }
    }

    /// Regression guard for nyx-space 2.3.1 Izzo bug (`izzo.rs:72`):
    /// `if i_h.z < 0.0 || retrograde` collapses ShortWay/LongWay/Auto to the
    /// same solution whenever the geometry's natural angular-momentum unit
    /// vector points into -z. Routing single-rev through Gooding avoids this;
    /// this test pins that behavior with an explicit `i_h.z < 0` fixture.
    ///
    /// Geometry: `r1 = (7000, 0, 0)`, `r2 = (0, -7000, -500)` km.
    /// `r1 × r2 = (0, 3.5e6, -4.9e7)`, so `i_h.z < 0`. If this test ever
    /// starts passing through a solver that uses the buggy Izzo direction
    /// logic, `ShortWay` and `LongWay` would produce bit-identical
    /// `v_init_km_s`.
    #[test]
    fn short_and_long_way_differ_for_negative_ih_z() {
        let epoch = test_epoch();
        let dep = StateVector {
            epoch,
            position_eci_km: Vector3::new(7000.0, 0.0, 0.0),
            velocity_eci_km_s: Vector3::zeros(),
        };
        let arr = StateVector {
            epoch: epoch + Duration::from_seconds(3600.0),
            position_eci_km: Vector3::new(0.0, -7000.0, -500.0),
            velocity_eci_km_s: Vector3::zeros(),
        };

        // Confirm the fixture actually reproduces the `i_h.z < 0` geometry
        // that the nyx Izzo bug hinges on. If this precondition ever flips
        // sign, the test is no longer guarding the right thing.
        let i_h = dep.position_eci_km.cross(&arr.position_eci_km);
        assert!(
            i_h.z < 0.0,
            "test precondition: r_init × r_final must have negative z, got i_h = {i_h:?}",
        );

        let short = solve_lambert_with_config(
            &dep,
            &arr,
            &LambertConfig { direction: TransferDirection::ShortWay, revolutions: 0 },
        )
        .expect("short-way Lambert should succeed for this geometry");
        let long = solve_lambert_with_config(
            &dep,
            &arr,
            &LambertConfig { direction: TransferDirection::LongWay, revolutions: 0 },
        )
        .expect("long-way Lambert should succeed for this geometry");

        // Under the Izzo bug, both calls would produce identical `v_init`.
        // Gooding returns two distinct solutions; require a difference well
        // above floating-point noise (10 m/s is conservative — the actual
        // delta for this geometry is several km/s).
        let delta_km_s = (short.departure_state.velocity_eci_km_s
            - long.departure_state.velocity_eci_km_s)
            .norm();
        assert!(
            delta_km_s > 0.01,
            "ShortWay and LongWay v_init must differ for i_h.z < 0 geometry; got Δ = {delta_km_s} km/s",
        );
    }

    /// Tolerance on derived periapsis altitude (km) for the wiring tests
    /// below. Looser than the rpo-core kernel test (1 µm) because these
    /// tests construct an ellipse from `(r_p, r_apo)` then re-extract `r_p`
    /// via `from_state`, adding one round-trip of f64 arithmetic on top of
    /// the kernel computation. 1 m is conservative.
    const PERIAPSIS_ALT_TEST_TOL_KM: f64 = 1.0e-3;

    /// `build_transfer` must mark a circular LEO state as `Feasible` —
    /// circular orbit at 700 km altitude has periapsis = apoapsis = 700 km,
    /// well above the 200 km feasibility threshold
    /// (`MIN_PERIAPSIS_ALTITUDE_KM`). Mirrors the sub-surface test: same
    /// kernel, same constructor, opposite expected outcome.
    ///
    /// (The previous version of this test exercised `solve_lambert` over the
    /// existing 400→800 km fixture, but that fixture's TOF/geometry pairing
    /// puts the conic's perigee deep inside the Earth — see
    /// `coplanar_transfer` which only checks Δv magnitude, not feasibility.
    /// Fixing the upstream fixture is out of scope for the feasibility-
    /// surfacing change; we use a synthetic feasible state here so the
    /// wiring test doesn't depend on subtle TOF tuning.)
    #[test]
    fn build_transfer_flags_circular_leo_as_feasible() {
        let epoch = test_epoch();
        let r_km = R_EARTH + 700.0;
        let v_circ_km_s = (rpo_core::constants::MU_EARTH / r_km).sqrt();

        let dep = StateVector {
            epoch,
            position_eci_km: Vector3::new(r_km, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, v_circ_km_s * 0.5, 0.0),
        };
        let arr = StateVector {
            epoch: epoch + Duration::from_seconds(600.0),
            position_eci_km: Vector3::new(0.0, r_km, 0.0),
            velocity_eci_km_s: Vector3::zeros(),
        };

        let v1 = Vector3::new(0.0, v_circ_km_s, 0.0);
        let v2 = Vector3::new(-v_circ_km_s, 0.0, 0.0);
        let transfer = build_transfer(
            &dep,
            &arr,
            v1,
            v2,
            600.0,
            0.0,
            TransferDirection::Auto,
        );

        assert!(
            !transfer.feasibility.is_sub_surface(),
            "circular 700 km LEO must classify as feasible; got {:?}",
            transfer.feasibility,
        );
        let alt = transfer.feasibility.periapsis_altitude_km();
        assert!(
            (alt - 700.0).abs() < PERIAPSIS_ALT_TEST_TOL_KM,
            "circular orbit periapsis altitude should be ~700 km, got {alt}",
        );
    }

    /// `build_transfer` must mark a synthesized v1 with sub-surface periapsis
    /// as `SubSurface`, independent of solver behavior. This is the wiring
    /// check between the (well-tested) `TransferFeasibility::from_state`
    /// kernel in rpo-core and the public `LambertTransfer` constructor here.
    ///
    /// Geometry: place departure at 1500 km altitude (apogee of a target
    /// ellipse) with the apogee speed of an ellipse whose perigee is at
    /// 50 km altitude. The resulting conic has periapsis below the 200 km
    /// feasibility threshold (`MIN_PERIAPSIS_ALTITUDE_KM`).
    #[test]
    fn build_transfer_flags_sub_surface_geometry() {
        let epoch = test_epoch();
        let (r_apo_eci_km, v_apo_eci_km_s) =
            ellipse_state_at_apogee(R_EARTH + 50.0, R_EARTH + 1500.0);
        let v_apogee_km_s = v_apo_eci_km_s.y;

        let dep = StateVector {
            epoch,
            position_eci_km: r_apo_eci_km,
            velocity_eci_km_s: Vector3::new(0.0, v_apogee_km_s * 0.5, 0.0),
        };
        // Arrival doesn't matter for the feasibility check — feasibility is
        // derived from the post-burn departure velocity (v1).
        let arr = StateVector {
            epoch: epoch + Duration::from_seconds(600.0),
            position_eci_km: Vector3::new(0.0, r_apo_eci_km.x, 0.0),
            velocity_eci_km_s: Vector3::zeros(),
        };

        let transfer = build_transfer(
            &dep,
            &arr,
            v_apo_eci_km_s,
            Vector3::new(-v_apogee_km_s, 0.0, 0.0),
            600.0,
            0.0,
            TransferDirection::Auto,
        );

        assert!(
            transfer.feasibility.is_sub_surface(),
            "synthesized 50 km perigee conic must classify as sub-surface; got {:?}",
            transfer.feasibility,
        );
        let alt = transfer.feasibility.periapsis_altitude_km();
        assert!(
            (alt - 50.0).abs() < PERIAPSIS_ALT_TEST_TOL_KM,
            "periapsis altitude should be ~50 km, got {alt}",
        );
    }

    /// `Auto` must not silently collapse to the same result as `ShortWay`
    /// (or `LongWay`) in all cases. It's allowed to equal either one for a
    /// specific geometry — that's the whole point of auto-dispatch. This
    /// test just exercises the Auto path with an `i_h.z < 0` geometry to
    /// make sure it doesn't panic and returns a finite, sensible ΔV.
    #[test]
    fn auto_direction_succeeds_for_negative_ih_z() {
        let epoch = test_epoch();
        let dep = StateVector {
            epoch,
            position_eci_km: Vector3::new(7000.0, 0.0, 0.0),
            velocity_eci_km_s: Vector3::zeros(),
        };
        let arr = StateVector {
            epoch: epoch + Duration::from_seconds(3600.0),
            position_eci_km: Vector3::new(0.0, -7000.0, -500.0),
            velocity_eci_km_s: Vector3::zeros(),
        };

        let transfer = solve_lambert_with_config(
            &dep,
            &arr,
            &LambertConfig { direction: TransferDirection::Auto, revolutions: 0 },
        )
        .expect("auto-direction Lambert should succeed for this geometry");

        assert!(
            transfer.total_dv_km_s.is_finite() && transfer.total_dv_km_s > 0.0,
            "Auto ΔV must be finite and positive, got {}",
            transfer.total_dv_km_s,
        );
    }
}

//! Lambert transfer solver for far-field orbital transfers.
//!
//! Uses nyx-space's implementation of Izzo's method (Izzo, D. "Revisiting
//! Lambert's Problem." *Celestial Mechanics and Dynamical Astronomy*, 2015)
//! with multi-revolution transfers, C3 computation, and short/long-way selection.
//!
//! The simple [`solve_lambert`] API calls Izzo by default.
//! Use [`solve_lambert_with_config`] for direction and multi-rev control,
//! or call [`solve_lambert_izzo`] directly.
//!
//! # Crate boundary
//!
//! Lambert types ([`LambertConfig`], [`LambertError`], [`LambertTransfer`],
//! [`TransferDirection`]) live in `rpo_core::propagation::lambert` so they are
//! available to WASM consumers. This module provides the nyx-backed solver
//! functions that depend on nyx-space.

use nalgebra::Vector3;
use nyx_space::tools::lambert::{self, LambertInput, LambertSolution, TransferKind};

use rpo_core::propagation::lambert::{LambertConfig, LambertError, LambertTransfer, TransferDirection};
use rpo_core::types::StateVector;

use crate::nyx_bridge::state_to_orbit;

/// Minimum position separation (km) between departure and arrival for a valid Lambert problem.
const LAMBERT_MIN_SEPARATION_KM: f64 = 1e-6;

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
#[must_use]
pub fn build_transfer(
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
    }
}

/// Convert a [`TransferDirection`] to a nyx [`TransferKind`].
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
/// Uses Izzo's method (nyx-space) with default configuration.
///
/// # Invariants
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - Departure and arrival positions must be non-degenerate (separation > `LAMBERT_MIN_SEPARATION_KM`)
/// - Transfer angle must not be exactly 0 or π (degenerate geometry)
///
/// # Near-degenerate behavior
/// For transfer angles within a few degrees of 0 or π, the Izzo solver
/// may produce large Δv or return `IzzoConvergenceFailure`. The
/// `near_180_degree_transfer` test documents this graceful degradation.
///
/// # Errors
/// Returns `LambertError` if the solver fails or inputs are invalid.
pub fn solve_lambert(
    departure: &StateVector,
    arrival: &StateVector,
) -> Result<LambertTransfer, LambertError> {
    solve_lambert_with_config(departure, arrival, &LambertConfig::default())
}

/// Solve Lambert's problem with explicit configuration.
///
/// Uses Izzo's method (nyx-space), which supports all config options
/// including multi-revolution transfers and transfer direction selection.
///
/// # Invariants
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - Departure and arrival positions must be non-degenerate (separation > `LAMBERT_MIN_SEPARATION_KM`)
/// - For multi-rev (`config.revolutions > 0`), TOF must be long enough to
///   accommodate the requested number of revolutions
///
/// # Errors
/// Returns `LambertError` if the solver fails or inputs are invalid.
pub fn solve_lambert_with_config(
    departure: &StateVector,
    arrival: &StateVector,
    config: &LambertConfig,
) -> Result<LambertTransfer, LambertError> {
    solve_lambert_izzo(departure, arrival, config)
}

/// Solve Lambert's problem using Izzo's method (nyx-space).
///
/// Supports multi-revolution transfers (`config.revolutions > 0`) and
/// all transfer directions. Populates C3 from the solution.
///
/// # Invariants
/// - `arrival.epoch > departure.epoch` (positive time of flight)
/// - Departure and arrival positions must be non-degenerate (separation > `LAMBERT_MIN_SEPARATION_KM`)
/// - `mu > 0` (uses `EARTH_J2000` gravitational parameter internally)
///
/// # Errors
/// Returns `LambertError` if the solver fails or inputs are invalid.
pub fn solve_lambert_izzo(
    departure: &StateVector,
    arrival: &StateVector,
    config: &LambertConfig,
) -> Result<LambertTransfer, LambertError> {
    let tof = validate_inputs(departure, arrival)?;

    let dep_orbit = state_to_orbit(departure);
    let arr_orbit = state_to_orbit(arrival);

    let input = LambertInput::from_planetary_states(dep_orbit, arr_orbit)
        .map_err(|e| LambertError::InvalidInput { details: format!("{e}") })?;

    let kind = if config.revolutions > 0 {
        TransferKind::NRevs(config.revolutions)
    } else {
        direction_to_kind(config.direction)
    };

    let solution = lambert::izzo(input, kind)
        .map_err(|e| LambertError::IzzoConvergenceFailure { details: format!("{e}") })?;

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
    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use rpo_core::test_helpers::{leo_400km_elements, leo_800km_target_elements, test_epoch};
    use rpo_core::types::KeplerianElements;
    use hifitime::Duration;

    /// Keplerian propagation endpoint position agreement with Lambert transfer
    /// endpoints. 1e-6 km accounts for accumulated Kepler equation convergence
    /// error over the full arc.
    const ARC_ENDPOINT_TOL_KM: f64 = 1e-6;

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
    /// reasonableness checks. Transfer between 400-800 km stays well below.
    const LEO_MAX_ALTITUDE_KM: f64 = 2000.0;

    #[test]
    fn izzo_coplanar_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let config = LambertConfig::default();
        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo should succeed");

        assert!(
            transfer.total_dv_km_s > LEO_COPLANAR_DV_LOWER_KM_S && transfer.total_dv_km_s < LEO_COPLANAR_DV_UPPER_KM_S,
            "Izzo Δv = {} km/s seems unreasonable",
            transfer.total_dv_km_s
        );
    }

    #[test]
    fn izzo_non_coplanar_transfer() {
        let epoch = test_epoch();

        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();

        let arr_ke = KeplerianElements {
            a_km: 6378.137 + 500.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 10.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(3600.0)).unwrap();

        let config = LambertConfig::default();
        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo non-coplanar failed");

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
            a_km: 6378.137 + 500.0,
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

        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Multi-rev should succeed");
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
            a_km: 6378.137 + 600.0,
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
            solve_lambert_izzo(&dep, &arr, &config).expect("Long-way transfer should succeed");
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
        let arc = transfer.densify_arc(100).unwrap();

        let dep_err = (arc.first().unwrap().position_eci_km - transfer.departure_state.position_eci_km).norm();
        let arr_err = (arc.last().unwrap().position_eci_km - transfer.arrival_state.position_eci_km).norm();

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
        let arc = transfer.densify_arc(100).unwrap();

        assert_eq!(arc.len(), 101, "Expected 101 points (100 steps + 1)");
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
        let arc = transfer.densify_arc(50).unwrap();

        for window in arc.windows(2) {
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
        use rpo_core::constants::R_EARTH;

        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch).unwrap();
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        ).unwrap();

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let arc = transfer.densify_arc(100).unwrap();

        for (k, state) in arc.iter().enumerate() {
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
        let izzo = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo failed");
        assert!(
            izzo.c3_km2_s2 >= 0.0,
            "C3 should be non-negative, got {}",
            izzo.c3_km2_s2
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
        let mut arr = dep.clone();
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
    /// The Izzo solver may either succeed or return an error for near-degenerate cases.
    /// This test verifies that the solver does not panic.
    #[test]
    fn near_180_degree_transfer() {
        let epoch = test_epoch();
        let dep_ke = leo_400km_elements();
        let dep = keplerian_to_state(&dep_ke, epoch).unwrap();

        // Arrival orbit offset by ~179° in mean anomaly on a slightly different altitude,
        // producing a near-180° transfer angle
        let arr_ke = KeplerianElements {
            a_km: 6378.137 + 450.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 179.0_f64.to_radians(),
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(2700.0)).unwrap();

        let config = LambertConfig::default();
        // Accept either a valid solution or a solver error — must not panic.
        match solve_lambert_izzo(&dep, &arr, &config) {
            Ok(transfer) => {
                // Near-180° geometry may produce large but finite Δv
                assert!(
                    transfer.total_dv_km_s.is_finite() && transfer.total_dv_km_s >= 0.0,
                    "Δv must be finite and non-negative for near-180° transfer, got {}",
                    transfer.total_dv_km_s
                );
            }
            Err(LambertError::InvalidInput { .. } | LambertError::IzzoConvergenceFailure { .. }) => {
                // Degenerate geometry is an acceptable failure mode for near-180° transfers.
            }
            Err(e) => panic!("Unexpected error type for near-180° transfer: {e}"),
        }
    }
}

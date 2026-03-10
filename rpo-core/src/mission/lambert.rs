//! Lambert transfer solver for far-field orbital transfers.
//!
//! Uses nyx-space's Izzo solver with multi-revolution transfers,
//! C3 computation, and short/long-way selection.
//!
//! The simple [`solve_lambert`] API calls Izzo by default.
//! Use [`solve_lambert_with_config`] for direction and multi-rev control,
//! or call [`solve_lambert_izzo`] directly.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use anise::prelude::Orbit;
use nyx_space::tools::lambert::{self, LambertInput, LambertSolution, TransferKind};

use crate::constants::EARTH_J2000;
use crate::types::StateVector;

/// Transfer direction for Lambert solutions.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TransferDirection {
    /// Automatically determine short or long way based on geometry.
    #[default]
    Auto,
    /// Short-way (prograde) transfer: transfer angle < 180°.
    ShortWay,
    /// Long-way (retrograde) transfer: transfer angle > 180°.
    LongWay,
}

/// Configuration for Lambert solver behavior.
#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct LambertConfig {
    /// Transfer direction selection.
    pub direction: TransferDirection,
    /// Number of complete revolutions (0 = direct transfer).
    pub revolutions: u8,
}

/// A solved Lambert transfer between two ECI states.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LambertTransfer {
    /// Departure state (original deputy position, computed departure velocity).
    pub departure_state: StateVector,
    /// Arrival state (target position, computed arrival velocity).
    pub arrival_state: StateVector,
    /// Required Δv at departure (km/s, ECI).
    pub departure_dv: Vector3<f64>,
    /// Required Δv at arrival (km/s, ECI).
    pub arrival_dv: Vector3<f64>,
    /// Total Δv magnitude (km/s).
    pub total_dv: f64,
    /// Time of flight (seconds).
    pub tof: f64,
    /// Characteristic energy C3 = v∞² (km²/s²).
    pub c3_km2_s2: Option<f64>,
    /// Transfer direction used.
    pub direction: TransferDirection,
}

impl LambertTransfer {
    /// Generate a dense ECI trajectory along the Lambert transfer arc.
    ///
    /// The arc is a two-body Keplerian orbit defined by `departure_state`.
    /// Produces `n_steps + 1` states from departure to arrival.
    #[must_use]
    pub fn densify_arc(&self, n_steps: usize) -> Vec<StateVector> {
        crate::propagation::keplerian::propagate_keplerian(&self.departure_state, self.tof, n_steps)
    }
}

/// Errors from the Lambert solver.
#[derive(Debug, Clone)]
pub enum LambertError {
    /// The solver failed to converge.
    ConvergenceFailure(String),
    /// Invalid input (e.g., identical positions, non-positive TOF).
    InvalidInput(String),
}

impl std::fmt::Display for LambertError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ConvergenceFailure(msg) => {
                write!(f, "LambertError: convergence failure — {msg}")
            }
            Self::InvalidInput(msg) => {
                write!(f, "LambertError: invalid input — {msg}")
            }
        }
    }
}

impl std::error::Error for LambertError {}

/// Validate Lambert inputs.
///
/// Checks that TOF > 0 and position separation > 1e-6 km.
fn validate_inputs(
    departure: &StateVector,
    arrival: &StateVector,
) -> Result<f64, LambertError> {
    let tof = (arrival.epoch - departure.epoch).to_seconds();
    if tof <= 0.0 {
        return Err(LambertError::InvalidInput(
            "arrival epoch must be after departure epoch".into(),
        ));
    }

    let sep = (arrival.position - departure.position).norm();
    if sep < 1e-6 {
        return Err(LambertError::InvalidInput(
            "departure and arrival positions are identical".into(),
        ));
    }

    Ok(tof)
}

/// Build a [`LambertTransfer`] from computed transfer velocities.
pub(crate) fn build_transfer(
    departure: &StateVector,
    arrival: &StateVector,
    v1_vec: Vector3<f64>,
    v2_vec: Vector3<f64>,
    tof: f64,
    c3: Option<f64>,
    direction: TransferDirection,
) -> LambertTransfer {
    let departure_dv = v1_vec - departure.velocity;
    let arrival_dv = arrival.velocity - v2_vec;
    let total_dv = departure_dv.norm() + arrival_dv.norm();

    LambertTransfer {
        departure_state: StateVector {
            epoch: departure.epoch,
            position: departure.position,
            velocity: v1_vec,
        },
        arrival_state: StateVector {
            epoch: arrival.epoch,
            position: arrival.position,
            velocity: v2_vec,
        },
        departure_dv,
        arrival_dv,
        total_dv,
        tof,
        c3_km2_s2: c3,
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

/// Convert a [`StateVector`] to an anise [`Orbit`].
fn state_to_orbit(sv: &StateVector) -> Orbit {
    Orbit::new(
        sv.position.x,
        sv.position.y,
        sv.position.z,
        sv.velocity.x,
        sv.velocity.y,
        sv.velocity.z,
        sv.epoch,
        EARTH_J2000,
    )
}

/// Build a [`LambertTransfer`] from a nyx [`LambertSolution`].
fn transfer_from_solution(
    departure: &StateVector,
    arrival: &StateVector,
    solution: &LambertSolution,
    tof: f64,
    direction: TransferDirection,
) -> LambertTransfer {
    let v1_vec = Vector3::new(
        solution.v_init_km_s[0],
        solution.v_init_km_s[1],
        solution.v_init_km_s[2],
    );
    let v2_vec = Vector3::new(
        solution.v_final_km_s[0],
        solution.v_final_km_s[1],
        solution.v_final_km_s[2],
    );

    build_transfer(
        departure,
        arrival,
        v1_vec,
        v2_vec,
        tof,
        Some(solution.c3_km2_s2()),
        direction,
    )
}

/// Solve Lambert's problem between two ECI states.
///
/// Uses Izzo's method (nyx-space) with default configuration.
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
        .map_err(|e| LambertError::InvalidInput(format!("{e}")))?;

    let kind = if config.revolutions > 0 {
        TransferKind::NRevs(config.revolutions)
    } else {
        direction_to_kind(config.direction)
    };

    let solution = lambert::izzo(input, kind)
        .map_err(|e| LambertError::ConvergenceFailure(format!("{e}")))?;

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
    use crate::elements::conversions::keplerian_to_state;
    use crate::test_helpers::{leo_400km_elements, leo_800km_target_elements, test_epoch};
    use crate::types::KeplerianElements;
    use hifitime::Duration;

    #[test]
    fn izzo_coplanar_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let config = LambertConfig::default();
        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo should succeed");

        assert!(
            transfer.total_dv > 0.01 && transfer.total_dv < 5.0,
            "Izzo Δv = {} km/s seems unreasonable",
            transfer.total_dv
        );
    }

    #[test]
    fn izzo_non_coplanar_transfer() {
        let epoch = test_epoch();

        let dep = keplerian_to_state(&leo_400km_elements(), epoch);

        let arr_ke = KeplerianElements {
            a: 6378.137 + 500.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 10.0_f64.to_radians(),
            aop: 0.0,
            mean_anomaly: 2.0,
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(3600.0));

        let config = LambertConfig::default();
        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo non-coplanar failed");

        assert!(
            transfer.total_dv > 0.1,
            "Non-coplanar Δv should be significant, got {}",
            transfer.total_dv
        );
    }

    #[test]
    fn multi_rev_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);

        let arr_ke = KeplerianElements {
            a: 6378.137 + 500.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 0.0,
            aop: 0.0,
            mean_anomaly: 90.0_f64.to_radians(),
        };
        // Long TOF to allow 1-rev solution (~2 orbital periods)
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(12000.0));

        let config = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 1,
        };

        let transfer = solve_lambert_izzo(&dep, &arr, &config).expect("Multi-rev should succeed");
        assert!(
            transfer.total_dv > 0.0,
            "Multi-rev Δv should be positive"
        );
    }

    #[test]
    fn long_way_transfer() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);

        let arr_ke = KeplerianElements {
            a: 6378.137 + 600.0,
            e: 0.001,
            i: 51.6_f64.to_radians(),
            raan: 0.0,
            aop: 0.0,
            mean_anomaly: 90.0_f64.to_radians(),
        };
        let arr = keplerian_to_state(&arr_ke, epoch + Duration::from_seconds(3600.0));

        let config = LambertConfig {
            direction: TransferDirection::LongWay,
            revolutions: 0,
        };

        let transfer =
            solve_lambert_izzo(&dep, &arr, &config).expect("Long-way transfer should succeed");
        assert!(transfer.total_dv > 0.0, "Long-way Δv should be positive");
        assert_eq!(transfer.direction, TransferDirection::LongWay);
    }

    #[test]
    fn densify_arc_endpoints_match() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let arc = transfer.densify_arc(100);

        let dep_err = (arc.first().unwrap().position - transfer.departure_state.position).norm();
        let arr_err = (arc.last().unwrap().position - transfer.arrival_state.position).norm();

        assert!(
            dep_err < 1e-6,
            "Departure position mismatch: {dep_err} km"
        );
        assert!(arr_err < 1e-6, "Arrival position mismatch: {arr_err} km");
    }

    #[test]
    fn densify_arc_correct_count() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let arc = transfer.densify_arc(100);

        assert_eq!(arc.len(), 101, "Expected 101 points (100 steps + 1)");
    }

    #[test]
    fn densify_arc_monotonic_epochs() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let arc = transfer.densify_arc(50);

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
        use crate::constants::R_EARTH;

        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let transfer = solve_lambert(&dep, &arr).expect("Lambert should succeed");
        let arc = transfer.densify_arc(100);

        for (k, state) in arc.iter().enumerate() {
            let r = state.position.norm();
            assert!(
                r > R_EARTH,
                "Point {k} below Earth surface: r = {r} km"
            );
            assert!(
                r < R_EARTH + 2000.0,
                "Point {k} radius too large for LEO: r = {r} km"
            );
        }
    }

    #[test]
    fn c3_is_populated() {
        let epoch = test_epoch();
        let dep = keplerian_to_state(&leo_400km_elements(), epoch);
        let arr = keplerian_to_state(
            &leo_800km_target_elements(),
            epoch + Duration::from_seconds(2400.0),
        );

        let config = LambertConfig::default();
        let izzo = solve_lambert_izzo(&dep, &arr, &config).expect("Izzo failed");
        assert!(izzo.c3_km2_s2.is_some(), "Izzo should populate C3");
        assert!(
            izzo.c3_km2_s2.unwrap() >= 0.0,
            "C3 should be non-negative"
        );
    }
}

//! Mission planning with Lambert transfer support.
//!
//! Contains the full mission planner that combines classification, perch ROE
//! geometry, and Lambert transfers (via nyx-space) into a complete
//! [`MissionPlan`].
//!
//! The analytical helpers (`classify_separation`, `perch_to_roe`,
//! `compute_transfer_eclipse`, etc.) remain in `rpo_core::mission::planning`.
//! This module provides the nyx-dependent `plan_mission()` entry point.

use rpo_core::constants::{INC_TOL, TWO_PI};
use rpo_core::elements::keplerian_conversions::keplerian_to_state;
use rpo_core::mission::config::ProximityConfig;
use rpo_core::mission::errors::MissionError;
use rpo_core::mission::planning::{classify_separation, perch_to_roe};
use rpo_core::mission::types::{MissionPhase, MissionPlan, PerchGeometry};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::types::{KeplerianElements, QuasiNonsingularROE, StateVector};

use crate::lambert::solve_lambert_with_config;

/// Plan a complete mission with Lambert transfer support.
///
/// If the spacecraft are in proximity, computes the perch ROE directly.
/// If far-field, computes a Lambert transfer to the perch orbit.
///
/// `lambert_tof_s` is the time of flight for the Lambert transfer (seconds).
/// It is only used when the spacecraft are in the far-field regime.
///
/// `lambert_config` controls transfer direction and multi-revolution
/// selection. Only used in the far-field regime.
///
/// # Invariants
/// - Both states must represent bound orbits (`e < 1`, `a > 0`)
/// - `lambert_tof_s > 0` (when used in far-field regime)
/// - For multi-rev (`lambert_config.revolutions > 0`), `lambert_tof_s` must
///   be long enough to accommodate the requested number of revolutions
/// - Position vectors must be non-zero
///
/// # Validity regime
/// - ROE linearization assumes |δa/a| << 1; the `ProximityConfig.roe_threshold`
///   parameter (default 0.005) enforces this at the classification boundary.
/// - Near-equatorial chief orbits (`i ~ 0` or `i ~ π`) degrade RAAN-based
///   ROE recovery in `perch_roe_to_keplerian` (see `INC_TOL` guard).
/// - Near-circular chief orbits (`e ~ 0`) are handled correctly by the
///   eccentricity vector formulation (no AOP singularity).
///
/// # Arguments
/// * `chief` — Chief spacecraft ECI state at departure
/// * `deputy` — Deputy spacecraft ECI state at departure
/// * `perch` — Target perch geometry
/// * `config` — Proximity threshold configuration
/// * `lambert_tof_s` — Time of flight for Lambert transfer (seconds, far-field only)
/// * `lambert_config` — Lambert solver configuration (direction, revolutions)
///
/// # Errors
/// Returns `MissionError` if the Lambert solver fails or the perch geometry is invalid.
pub fn plan_mission(
    chief: &StateVector,
    deputy: &StateVector,
    perch: &PerchGeometry,
    config: &ProximityConfig,
    lambert_tof_s: f64,
    lambert_config: &LambertConfig,
) -> Result<MissionPlan, MissionError> {
    let phase = classify_separation(chief, deputy, config)?;

    match phase {
        MissionPhase::Proximity {
            ref chief_elements,
            ..
        } => {
            let chief_at_arrival = *chief_elements;
            let perch_roe = perch_to_roe(perch, chief_elements)?;

            Ok(MissionPlan {
                phase,
                transfer: None,
                perch_roe,
                chief_at_arrival,
            })
        }
        MissionPhase::FarField {
            ref chief_elements, ..
        } => {
            let chief_ke = *chief_elements;

            // Advance chief mean anomaly to arrival epoch (two-body).
            // The Lambert target must be at the chief's position at arrival,
            // not at the departure epoch.
            let n = chief_ke.mean_motion()?;
            let chief_ke_arrival = KeplerianElements {
                mean_anomaly_rad: (chief_ke.mean_anomaly_rad + n * lambert_tof_s)
                    .rem_euclid(TWO_PI),
                ..chief_ke
            };

            let perch_roe = perch_to_roe(perch, &chief_ke_arrival)?;

            // Convert perch ROE to a target Keplerian orbit for Lambert
            let target_ke = perch_roe_to_keplerian(&perch_roe, &chief_ke_arrival);
            let arrival_epoch =
                deputy.epoch + hifitime::Duration::from_seconds(lambert_tof_s);
            let target_state = keplerian_to_state(&target_ke, arrival_epoch)?;

            // Solve Lambert: deputy → perch
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
/// Inverts the QNS ROE definition (Koenig Eq. 2 / D'Amico Eq. 2.2) to
/// recover deputy Keplerian elements from chief elements and relative state.
///
/// # Reference
/// - ROE definition: Koenig et al. Eq. 2, D'Amico Eq. 2.2
/// - Eccentricity vector recovery: `ex = e cos(ω)`, `ey = e sin(ω)`
/// - RAAN recovery from `δiy = (Ω_d - Ω_c) sin(i_c)`
///
/// # Singularities
/// - Near-equatorial orbits (`i_c ~ 0`): RAAN recovery degenerates; falls
///   back to `chief.raan_rad` when `|sin(i_c)| < INC_TOL`.
#[allow(clippy::similar_names)]
fn perch_roe_to_keplerian(
    roe: &QuasiNonsingularROE,
    chief: &KeplerianElements,
) -> KeplerianElements {
    let a = chief.a_km * (1.0 + roe.da);
    let i = chief.i_rad + roe.dix;

    // Recover eccentricity vector components: ex = e*cos(ω), ey = e*sin(ω)
    let (sin_aop, cos_aop) = chief.aop_rad.sin_cos();
    let ecc_x_chief = chief.e * cos_aop;
    let ecc_y_chief = chief.e * sin_aop;
    let ecc_x_dep = ecc_x_chief + roe.dex;
    let ecc_y_dep = ecc_y_chief + roe.dey;
    let e = (ecc_x_dep * ecc_x_dep + ecc_y_dep * ecc_y_dep).sqrt();
    let aop = ecc_y_dep.atan2(ecc_x_dep).rem_euclid(TWO_PI);

    // Recover RAAN from δiy = (Ω_d - Ω_c) * sin(i_c)
    let raan = if chief.i_rad.sin().abs() > INC_TOL {
        chief.raan_rad + roe.diy / chief.i_rad.sin()
    } else {
        chief.raan_rad
    };

    // Recover mean anomaly from δλ definition
    // δλ = (M_d + ω_d) - (M_c + ω_c) + (Ω_d - Ω_c) * cos(i_c)
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

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use rpo_core::elements::roe::compute_roe;
    use rpo_core::mission::planning::compute_transfer_eclipse;
    use rpo_core::test_helpers::{iss_like_elements, test_epoch};

    /// Tolerance for the perch ROE→Keplerian→ROE roundtrip.
    /// The roundtrip involves atan2, sin/cos, and rem_euclid operations that
    /// accumulate ~1e-10 error on dimensionless ROE components.
    const PERCH_ROE_ROUNDTRIP_TOL: f64 = 1e-10;

    /// Tolerance for structural zeros: ROE components that are set to exactly 0.0
    /// in the perch geometry constructor and never undergo arithmetic.
    /// 1e-15 is near the f64 machine epsilon floor.
    const STRUCTURAL_ZERO_TOL: f64 = 1e-15;

    /// Tolerance for perch ROE expected-value comparison.
    /// A single division (offset_km / a_km) introduces one ULP of rounding;
    /// 1e-12 is conservative for this single-operation error.
    const PERCH_ROE_EXPECTED_TOL: f64 = 1e-12;

    /// Tolerance for serde JSON roundtrip of ROE values.
    /// JSON serialization preserves f64 to full precision; 1e-12 matches
    /// the single-division error in the original perch ROE computation.
    const SERDE_ROUNDTRIP_TOL: f64 = 1e-12;

    /// Minimum Δv difference (km/s) between 0-rev and 1-rev Lambert solutions.
    /// This is a structural check — different solution branches must produce
    /// detectably different Δv. 1e-6 km/s = 1 mm/s is well above numerical noise.
    const MULTI_REV_DV_DIFFERENCE_MIN: f64 = 1e-6;

    /// Time-of-flight for single-revolution Lambert test cases (seconds).
    /// 1 hour — nominal transfer time for ISS-like orbit altitude changes.
    const TEST_LAMBERT_TOF_S: f64 = 3600.0;

    /// Time-of-flight for multi-revolution Lambert test cases (seconds).
    /// ~2 ISS orbital periods (~5570 s each), long enough to accommodate
    /// a 1-revolution transfer.
    const MULTI_REV_LAMBERT_TOF_S: f64 = 12000.0;

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

        // Convert ROE → Keplerian → back to ROE
        let deputy_ke = perch_roe_to_keplerian(&original_roe, &chief);
        let recovered_roe = compute_roe(&chief, &deputy_ke).unwrap();

        assert!(
            (original_roe.da - recovered_roe.da).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "da roundtrip: {} vs {}",
            original_roe.da,
            recovered_roe.da
        );
        assert!(
            (original_roe.dlambda - recovered_roe.dlambda).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "dlambda roundtrip: {} vs {}",
            original_roe.dlambda,
            recovered_roe.dlambda
        );
        assert!(
            (original_roe.dex - recovered_roe.dex).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "dex roundtrip: {} vs {}",
            original_roe.dex,
            recovered_roe.dex
        );
        assert!(
            (original_roe.dey - recovered_roe.dey).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "dey roundtrip: {} vs {}",
            original_roe.dey,
            recovered_roe.dey
        );
        assert!(
            (original_roe.dix - recovered_roe.dix).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "dix roundtrip: {} vs {}",
            original_roe.dix,
            recovered_roe.dix
        );
        assert!(
            (original_roe.diy - recovered_roe.diy).abs() < PERCH_ROE_ROUNDTRIP_TOL,
            "diy roundtrip: {} vs {}",
            original_roe.diy,
            recovered_roe.diy
        );
    }

    #[test]
    fn farfield_mission_with_lambert() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 200.0,
            e: 0.005,
            i_rad: chief_ke.i_rad + 0.05,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();
        let perch = PerchGeometry::VBar {
            along_track_km: 5.0,
        };

        let plan = plan_mission(&chief, &deputy, &perch, &config, TEST_LAMBERT_TOF_S, &LambertConfig::default())
            .expect("far-field mission should succeed");

        assert!(matches!(plan.phase, MissionPhase::FarField { .. }));
        assert!(plan.transfer.is_some(), "should have Lambert transfer");
        assert!(
            plan.transfer.as_ref().unwrap().total_dv_km_s > 0.0,
            "Lambert Δv should be positive"
        );
    }

    /// Run `plan_mission` in proximity regime with the given perch geometry.
    fn run_proximity_plan_mission(perch: PerchGeometry) -> (MissionPlan, KeplerianElements) {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher → Proximity regime
        deputy_ke.mean_anomaly_rad += 0.01;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        let plan = plan_mission(&chief, &deputy, &perch, &config, TEST_LAMBERT_TOF_S, &LambertConfig::default())
            .expect("proximity mission should succeed");
        (plan, chief_ke)
    }

    #[test]
    fn proximity_mission_uses_perch_roe() {
        let perch = PerchGeometry::VBar { along_track_km: 5.0 };
        let (plan, chief_ke) = run_proximity_plan_mission(perch);

        assert!(matches!(plan.phase, MissionPhase::Proximity { .. }));
        assert!(plan.transfer.is_none(), "proximity should have no Lambert transfer");

        let expected_dlambda = 5.0 / chief_ke.a_km;
        assert!(plan.perch_roe.da.abs() < STRUCTURAL_ZERO_TOL,
            "V-bar perch should have δa = 0, got {}", plan.perch_roe.da);
        assert!((plan.perch_roe.dlambda - expected_dlambda).abs() < PERCH_ROE_EXPECTED_TOL,
            "V-bar perch should have δλ = {expected_dlambda}, got {}", plan.perch_roe.dlambda);
    }

    #[test]
    fn proximity_mission_uses_rbar_perch() {
        let perch = PerchGeometry::RBar { radial_km: 2.0 };
        let (plan, chief_ke) = run_proximity_plan_mission(perch);

        assert!(matches!(plan.phase, MissionPhase::Proximity { .. }));
        assert!(plan.transfer.is_none(), "proximity should have no Lambert transfer");

        let expected_da = 2.0 / chief_ke.a_km;
        assert!((plan.perch_roe.da - expected_da).abs() < PERCH_ROE_EXPECTED_TOL,
            "R-bar perch should have δa = {expected_da}, got {}", plan.perch_roe.da);
        assert!(plan.perch_roe.dlambda.abs() < STRUCTURAL_ZERO_TOL,
            "R-bar perch should have δλ = 0, got {}", plan.perch_roe.dlambda);
    }

    #[test]
    fn mission_plan_serde_roundtrip() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();
        let perch = PerchGeometry::VBar { along_track_km: 5.0 };

        let plan = plan_mission(&chief, &deputy, &perch, &config, TEST_LAMBERT_TOF_S, &LambertConfig::default())
            .expect("mission should succeed");

        let json = serde_json::to_string(&plan).expect("serialize should work");
        let deserialized: MissionPlan =
            serde_json::from_str(&json).expect("deserialize should work");

        let expected_dlambda = 5.0 / chief_ke.a_km;
        assert!(
            (deserialized.perch_roe.dlambda - expected_dlambda).abs() < SERDE_ROUNDTRIP_TOL,
            "perch_roe should survive serde roundtrip"
        );
    }

    /// Far-field mission with a 1-revolution Lambert transfer.
    ///
    /// Uses a longer TOF (~12000 s, ~2 orbital periods) to accommodate 1 revolution.
    /// Verifies that the solver succeeds and produces a different Δv than 0-rev.
    ///
    /// Tolerance: Δv difference is structural (different solution branch), not numerical.
    /// Any nonzero difference confirms multi-rev dispatch.
    #[test]
    fn farfield_mission_multi_rev_lambert() {
        use rpo_core::propagation::lambert::TransferDirection;

        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 200.0,
            e: 0.005,
            i_rad: chief_ke.i_rad + 0.05,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();
        let perch = PerchGeometry::VBar {
            along_track_km: 5.0,
        };

        // Long TOF to accommodate 1 revolution (~2 orbital periods)
        let tof_s = MULTI_REV_LAMBERT_TOF_S;

        let config_0rev = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 0,
        };
        let config_1rev = LambertConfig {
            direction: TransferDirection::Auto,
            revolutions: 1,
        };

        let plan_0 = plan_mission(&chief, &deputy, &perch, &config, tof_s, &config_0rev)
            .expect("0-rev far-field mission should succeed");
        let plan_1 = plan_mission(&chief, &deputy, &perch, &config, tof_s, &config_1rev)
            .expect("1-rev far-field mission should succeed");

        assert!(matches!(plan_1.phase, MissionPhase::FarField { .. }));
        assert!(plan_1.transfer.is_some(), "should have Lambert transfer");

        let dv_0 = plan_0.transfer.as_ref().unwrap().total_dv_km_s;
        let dv_1 = plan_1.transfer.as_ref().unwrap().total_dv_km_s;
        assert!(
            (dv_0 - dv_1).abs() > MULTI_REV_DV_DIFFERENCE_MIN,
            "Multi-rev should produce different Δv: 0-rev={dv_0:.6}, 1-rev={dv_1:.6}"
        );
    }

    // =======================================================================
    // Transfer eclipse tests
    // =======================================================================

    /// `compute_transfer_eclipse` returns eclipse data for a far-field Lambert transfer.
    #[test]
    fn transfer_eclipse_far_field() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 200.0,
            e: 0.005,
            i_rad: chief_ke.i_rad + 0.05,
            raan_rad: chief_ke.raan_rad,
            aop_rad: 0.0,
            mean_anomaly_rad: 2.0,
        };

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();
        let perch = PerchGeometry::VBar { along_track_km: 5.0 };

        let plan = plan_mission(
            &chief, &deputy, &perch, &config, TEST_LAMBERT_TOF_S, &LambertConfig::default(),
        )
        .expect("far-field mission should succeed");

        let transfer = plan.transfer.as_ref().expect("should have Lambert transfer");
        let eclipse = compute_transfer_eclipse(transfer, &chief, 200)
            .expect("transfer eclipse should succeed");

        // 201 points (200 steps + 1)
        assert_eq!(eclipse.deputy_celestial.len(), 201);
        assert_eq!(eclipse.chief_eclipse.len(), 201);

        // ISS-like orbit over 1 hour — expect at least some shadow
        // (may or may not have eclipse depending on epoch, but structure is valid)
        assert!(
            eclipse.summary.time_in_shadow_fraction >= 0.0
                && eclipse.summary.time_in_shadow_fraction <= 1.0,
            "shadow fraction should be in [0, 1]"
        );
    }

    /// Transfer eclipse returns `None` for proximity missions (no Lambert transfer).
    #[test]
    fn transfer_eclipse_proximity_returns_none() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();
        let perch = PerchGeometry::VBar { along_track_km: 5.0 };

        let plan = plan_mission(
            &chief, &deputy, &perch, &config, TEST_LAMBERT_TOF_S, &LambertConfig::default(),
        )
        .expect("proximity mission should succeed");

        assert!(plan.transfer.is_none(), "proximity missions have no Lambert transfer");
    }
}

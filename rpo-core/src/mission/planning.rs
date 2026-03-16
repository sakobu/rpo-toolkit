//! Mission phase classification, separation analysis, and mission planning.
//!
//! Determines whether two spacecraft are within ROE-valid proximity
//! or require a far-field transfer, using a physically-motivated
//! dimensionless threshold based on D'Amico Sec. 2.3.4 and Koenig Sec. V.
//!
//! Provides mission planning functions that combine Lambert transfers
//! (when available) with ROE-based proximity operations.

use crate::elements::keplerian_conversions::{state_to_keplerian, ConversionError};
use crate::elements::roe::compute_roe;
use crate::propagation::lambert::{solve_lambert_with_config, LambertConfig};
use crate::types::{KeplerianElements, QuasiNonsingularROE, StateVector};

use super::config::ProximityConfig;
use super::errors::MissionError;
use super::types::{MissionPhase, MissionPlan, PerchGeometry};

/// Minimum perch offset (km) — guards against degenerate zero-offset perch geometry.
const PERCH_OFFSET_MIN_KM: f64 = 1e-10;

/// Analyze two ECI states and determine the mission phase.
///
/// Converts both states to Keplerian elements, computes the dimensionless
/// separation metric, and classifies as `Proximity` or `FarField` based
/// on the configured threshold.
///
/// # Invariants
/// - Both states must represent bound orbits (`e < 1`, `a > 0`)
/// - Position vectors must be non-zero
/// - `config.roe_threshold > 0`
///
/// # Errors
/// Returns `MissionError::Conversion` if either state cannot be converted
/// to Keplerian elements (e.g., escape trajectory or zero position).
pub fn classify_separation(
    chief: &StateVector,
    deputy: &StateVector,
    config: &ProximityConfig,
) -> Result<MissionPhase, MissionError> {
    let chief_elements = state_to_keplerian(chief)?;
    let deputy_elements = state_to_keplerian(deputy)?;
    let separation_km = eci_separation_km(chief, deputy);
    let roe = compute_roe(&chief_elements, &deputy_elements)?;
    let delta_r_over_r = roe.dimensionless_norm();

    if delta_r_over_r < config.roe_threshold {
        Ok(MissionPhase::Proximity {
            roe,
            chief_elements,
            deputy_elements,
            separation_km,
            delta_r_over_r,
        })
    } else {
        Ok(MissionPhase::FarField {
            chief_elements,
            deputy_elements,
            separation_km,
            delta_r_over_r,
        })
    }
}

/// Compute ECI separation distance (km).
///
/// # Invariants
/// - Both states should be at the same epoch for meaningful results
#[must_use]
pub fn eci_separation_km(chief: &StateVector, deputy: &StateVector) -> f64 {
    (deputy.position_eci_km - chief.position_eci_km).norm()
}

/// Compute the dimensionless δr/r separation metric from Keplerian elements.
///
/// Uses `max(|δa|, |δex|, |δey|, |δix|)` — excludes δλ and δiy per
/// Koenig Sec. V, which notes these components can be arbitrarily large
/// without violating linearization assumptions.
///
/// Here δa = (`a_d` - `a_c`) / `a_c`, and δex, δey, δix are the ROE components
/// (already dimensionless, normalized by chief SMA).
///
/// # Invariants
/// - `chief.a_km > 0` (used as normalizing denominator in ROE computation)
/// - Both elements must be at the same epoch
///
/// # Errors
/// Returns `ConversionError` if `chief` has invalid SMA or eccentricity.
pub fn dimensionless_separation(
    chief: &KeplerianElements,
    deputy: &KeplerianElements,
) -> Result<f64, ConversionError> {
    let roe = compute_roe(chief, deputy)?;
    Ok(roe.dimensionless_norm())
}

/// Convert a [`PerchGeometry`] to a [`QuasiNonsingularROE`] state relative to the chief.
///
/// # Invariants
/// - `chief.a_km > 0` (used as normalizing denominator)
///
/// # Errors
/// Returns `MissionError::InvalidVBarOffset` or `MissionError::InvalidRBarOffset`
/// if the perch geometry has a zero offset.
pub fn perch_to_roe(
    perch: &PerchGeometry,
    chief: &KeplerianElements,
) -> Result<QuasiNonsingularROE, MissionError> {
    match perch {
        PerchGeometry::VBar { along_track_km } => {
            if along_track_km.abs() < PERCH_OFFSET_MIN_KM {
                return Err(MissionError::InvalidVBarOffset {
                    along_track_km: *along_track_km,
                });
            }
            // Along-track offset maps primarily to δλ
            // From D'Amico Eq. 2.17: along-track ≈ a * δλ (at u where sin/cos terms vanish)
            let dlambda = along_track_km / chief.a_km;
            Ok(QuasiNonsingularROE {
                da: 0.0,
                dlambda,
                dex: 0.0,
                dey: 0.0,
                dix: 0.0,
                diy: 0.0,
            })
        }
        PerchGeometry::RBar { radial_km } => {
            if radial_km.abs() < PERCH_OFFSET_MIN_KM {
                return Err(MissionError::InvalidRBarOffset {
                    radial_km: *radial_km,
                });
            }
            // Radial offset maps primarily to δa
            // From D'Amico Eq. 2.17: radial ≈ a * δa (at u where cos/sin terms vanish)
            let da = radial_km / chief.a_km;
            Ok(QuasiNonsingularROE {
                da,
                dlambda: 0.0,
                dex: 0.0,
                dey: 0.0,
                dix: 0.0,
                diy: 0.0,
            })
        }
        PerchGeometry::Custom(roe) => Ok(*roe),
    }
}

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
                    .rem_euclid(crate::constants::TWO_PI),
                ..chief_ke
            };

            let perch_roe = perch_to_roe(perch, &chief_ke_arrival)?;

            // Convert perch ROE to a target Keplerian orbit for Lambert
            let target_ke = perch_roe_to_keplerian(&perch_roe, &chief_ke_arrival);
            let arrival_epoch =
                deputy.epoch + hifitime::Duration::from_seconds(lambert_tof_s);
            let target_state =
                crate::elements::keplerian_conversions::keplerian_to_state(&target_ke, arrival_epoch)?;

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
/// Inverts the ROE definition (Koenig Eq. 2) to recover deputy Keplerian
/// elements from chief elements and relative state.
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
    let aop = ecc_y_dep.atan2(ecc_x_dep).rem_euclid(crate::constants::TWO_PI);

    // Recover RAAN from δiy = (Ω_d - Ω_c) * sin(i_c)
    let raan = if chief.i_rad.sin().abs() > crate::constants::INC_TOL {
        chief.raan_rad + roe.diy / chief.i_rad.sin()
    } else {
        chief.raan_rad
    };

    // Recover mean anomaly from δλ definition
    // δλ = (M_d + ω_d) - (M_c + ω_c) + (Ω_d - Ω_c) * cos(i_c)
    let d_raan = raan - chief.raan_rad;
    let lambda_c = chief.mean_anomaly_rad + chief.aop_rad;
    let lambda_d = roe.dlambda + lambda_c - d_raan * chief.i_rad.cos();
    let mean_anomaly = (lambda_d - aop).rem_euclid(crate::constants::TWO_PI);

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
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::elements::roe_to_ric::roe_to_ric;
    use crate::elements::roe::compute_roe;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::KeplerianElements;

    #[test]
    fn classify_proximity_for_close_spacecraft() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        let phase = classify_separation(&chief, &deputy, &config).unwrap();
        assert!(
            matches!(phase, MissionPhase::Proximity { .. }),
            "1 km offset should be Proximity, got {phase:?}"
        );
    }

    #[test]
    fn classify_farfield_for_distant_spacecraft() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 500.0, // 500 km higher
            e: 0.01,
            i_rad: 45.0_f64.to_radians(), // different inclination
            raan_rad: 90.0_f64.to_radians(),
            aop_rad: 10.0_f64.to_radians(),
            mean_anomaly_rad: 180.0_f64.to_radians(),
        };

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        let phase = classify_separation(&chief, &deputy, &config).unwrap();
        assert!(
            matches!(phase, MissionPhase::FarField { .. }),
            "500 km + inclination offset should be FarField, got {phase:?}"
        );
    }

    #[test]
    fn dimensionless_separation_excludes_dlambda_diy() {
        let chief = iss_like_elements();
        let mut deputy = chief;
        // Only change mean anomaly (affects δλ) and RAAN (affects δiy)
        deputy.mean_anomaly_rad += 0.5; // large δλ
        deputy.raan_rad += 0.1; // large δiy

        // These should NOT increase dimensionless separation
        // because δλ and δiy are excluded
        let sep = dimensionless_separation(&chief, &deputy).unwrap();

        // δa, δex, δey, δix should all be zero (same a, e, i, aop)
        assert!(
            sep < 1e-10,
            "dimensionless_separation should exclude δλ and δiy, got {sep}"
        );
    }

    #[test]
    fn threshold_edge_exactly_at_boundary() {
        let epoch = test_epoch();
        let chief_ke = KeplerianElements {
            a_km: 7000.0,
            e: 0.001,
            i_rad: 0.9,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };

        // Set δa/a exactly at threshold
        let threshold = 0.005;
        let mut deputy_below = chief_ke;
        deputy_below.a_km = chief_ke.a_km * (1.0 + threshold * 0.99); // just below

        let mut deputy_above = chief_ke;
        deputy_above.a_km = chief_ke.a_km * (1.0 + threshold * 1.01); // just above

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let dep_below = keplerian_to_state(&deputy_below, epoch).unwrap();
        let dep_above = keplerian_to_state(&deputy_above, epoch).unwrap();
        let config = ProximityConfig { roe_threshold: threshold };

        assert!(matches!(
            classify_separation(&chief, &dep_below, &config).unwrap(),
            MissionPhase::Proximity { .. }
        ));
        assert!(matches!(
            classify_separation(&chief, &dep_above, &config).unwrap(),
            MissionPhase::FarField { .. }
        ));
    }

    #[test]
    fn custom_proximity_config() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 50.0; // 50 km higher → δa/a ≈ 0.0074

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();

        // Default threshold (0.005) should classify as FarField
        let strict = ProximityConfig::default();
        assert!(matches!(
            classify_separation(&chief, &deputy, &strict).unwrap(),
            MissionPhase::FarField { .. }
        ));

        // Relaxed threshold (0.01) should classify as Proximity
        let relaxed = ProximityConfig { roe_threshold: 0.01 };
        assert!(matches!(
            classify_separation(&chief, &deputy, &relaxed).unwrap(),
            MissionPhase::Proximity { .. }
        ));
    }

    #[test]
    fn proximity_roe_matches_compute_roe() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0;
        deputy_ke.mean_anomaly_rad += 0.01;

        let chief = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let config = ProximityConfig::default();

        let phase = classify_separation(&chief, &deputy, &config).unwrap();

        if let MissionPhase::Proximity {
            roe,
            chief_elements,
            deputy_elements,
            ..
        } = phase
        {
            // ROE from classify should match direct compute_roe
            let direct_roe = compute_roe(&chief_elements, &deputy_elements).unwrap();
            assert!(
                (roe.da - direct_roe.da).abs() < 1e-14,
                "da mismatch"
            );
            assert!(
                (roe.dlambda - direct_roe.dlambda).abs() < 1e-14,
                "dlambda mismatch"
            );
            assert!(
                (roe.dex - direct_roe.dex).abs() < 1e-14,
                "dex mismatch"
            );
            assert!(
                (roe.dey - direct_roe.dey).abs() < 1e-14,
                "dey mismatch"
            );
            assert!(
                (roe.dix - direct_roe.dix).abs() < 1e-14,
                "dix mismatch"
            );
            assert!(
                (roe.diy - direct_roe.diy).abs() < 1e-14,
                "diy mismatch"
            );
        } else {
            panic!("Expected Proximity phase");
        }
    }

    #[test]
    fn vbar_perch_gives_along_track_roe() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::VBar {
            along_track_km: 5.0,
        };
        let roe = perch_to_roe(&perch, &chief).expect("V-bar perch should work");

        // V-bar maps to δλ only
        let expected_dlambda = 5.0 / chief.a_km;
        assert!(
            (roe.dlambda - expected_dlambda).abs() < 1e-12,
            "dlambda should be {expected_dlambda}, got {}",
            roe.dlambda
        );
        assert!(roe.da.abs() < 1e-15, "da should be zero for V-bar");
        assert!(roe.dex.abs() < 1e-15, "dex should be zero");
        assert!(roe.dey.abs() < 1e-15, "dey should be zero");

        // Verify the RIC position has along-track component
        let ric = roe_to_ric(&roe, &chief).unwrap();
        // Along-track ≈ a * δλ + oscillatory terms
        assert!(
            ric.position_ric_km.y.abs() > 1.0,
            "V-bar perch should have along-track offset, got {}",
            ric.position_ric_km.y
        );
    }

    #[test]
    fn rbar_perch_gives_radial_roe() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::RBar { radial_km: 2.0 };
        let roe = perch_to_roe(&perch, &chief).expect("R-bar perch should work");

        // R-bar maps to δa only
        let expected_da = 2.0 / chief.a_km;
        assert!(
            (roe.da - expected_da).abs() < 1e-12,
            "da should be {expected_da}, got {}",
            roe.da
        );
        assert!(roe.dlambda.abs() < 1e-15, "dlambda should be zero for R-bar");
    }

    #[test]
    fn vbar_zero_offset_returns_error() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::VBar { along_track_km: 0.0 };
        let result = perch_to_roe(&perch, &chief);
        assert!(
            matches!(result, Err(MissionError::InvalidVBarOffset { .. })),
            "Zero V-bar offset should return InvalidVBarOffset, got {result:?}"
        );
    }

    #[test]
    fn rbar_zero_offset_returns_error() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::RBar { radial_km: 0.0 };
        let result = perch_to_roe(&perch, &chief);
        assert!(
            matches!(result, Err(MissionError::InvalidRBarOffset { .. })),
            "Zero R-bar offset should return InvalidRBarOffset, got {result:?}"
        );
    }

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
            (original_roe.da - recovered_roe.da).abs() < 1e-10,
            "da roundtrip: {} vs {}",
            original_roe.da,
            recovered_roe.da
        );
        assert!(
            (original_roe.dlambda - recovered_roe.dlambda).abs() < 1e-10,
            "dlambda roundtrip: {} vs {}",
            original_roe.dlambda,
            recovered_roe.dlambda
        );
        assert!(
            (original_roe.dex - recovered_roe.dex).abs() < 1e-10,
            "dex roundtrip: {} vs {}",
            original_roe.dex,
            recovered_roe.dex
        );
        assert!(
            (original_roe.dey - recovered_roe.dey).abs() < 1e-10,
            "dey roundtrip: {} vs {}",
            original_roe.dey,
            recovered_roe.dey
        );
        assert!(
            (original_roe.dix - recovered_roe.dix).abs() < 1e-10,
            "dix roundtrip: {} vs {}",
            original_roe.dix,
            recovered_roe.dix
        );
        assert!(
            (original_roe.diy - recovered_roe.diy).abs() < 1e-10,
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

        let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0, &LambertConfig::default())
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

        let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0, &LambertConfig::default())
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
        assert!(plan.perch_roe.da.abs() < 1e-15,
            "V-bar perch should have δa = 0, got {}", plan.perch_roe.da);
        assert!((plan.perch_roe.dlambda - expected_dlambda).abs() < 1e-12,
            "V-bar perch should have δλ = {expected_dlambda}, got {}", plan.perch_roe.dlambda);
    }

    #[test]
    fn proximity_mission_uses_rbar_perch() {
        let perch = PerchGeometry::RBar { radial_km: 2.0 };
        let (plan, chief_ke) = run_proximity_plan_mission(perch);

        assert!(matches!(plan.phase, MissionPhase::Proximity { .. }));
        assert!(plan.transfer.is_none(), "proximity should have no Lambert transfer");

        let expected_da = 2.0 / chief_ke.a_km;
        assert!((plan.perch_roe.da - expected_da).abs() < 1e-12,
            "R-bar perch should have δa = {expected_da}, got {}", plan.perch_roe.da);
        assert!(plan.perch_roe.dlambda.abs() < 1e-15,
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

        let plan = plan_mission(&chief, &deputy, &perch, &config, 3600.0, &LambertConfig::default())
            .expect("mission should succeed");

        let json = serde_json::to_string(&plan).expect("serialize should work");
        let deserialized: MissionPlan =
            serde_json::from_str(&json).expect("deserialize should work");

        let expected_dlambda = 5.0 / chief_ke.a_km;
        assert!(
            (deserialized.perch_roe.dlambda - expected_dlambda).abs() < 1e-12,
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
        use crate::propagation::lambert::TransferDirection;

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
        let tof_s = 12000.0;

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
            (dv_0 - dv_1).abs() > 1e-6,
            "Multi-rev should produce different Δv: 0-rev={dv_0:.6}, 1-rev={dv_1:.6}"
        );
    }
}

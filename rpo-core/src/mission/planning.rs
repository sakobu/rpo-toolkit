//! Mission phase classification, separation analysis, and perch geometry.
//!
//! Determines whether two spacecraft are within ROE-valid proximity
//! or require a far-field transfer, using a physically-motivated
//! dimensionless threshold based on D'Amico Sec. 2.3.4 and Koenig Sec. V.
//!
//! The `plan_mission` function (classify + Lambert) that requires nyx-space
//! lives in `rpo-nyx`. This module retains the analytical functions:
//! classification, perch ROE construction, and transfer eclipse computation.

use crate::elements::eclipse::{compute_eclipse_from_states, extract_eclipse_intervals};
use crate::elements::keplerian_conversions::{state_to_keplerian, ConversionError};
use crate::elements::roe::compute_roe;
use crate::propagation::lambert::LambertTransfer;
use crate::types::{
    EclipseState, KeplerianElements, QuasiNonsingularROE, StateVector, TransferEclipseData,
};

use super::config::ProximityConfig;
use super::errors::{EclipseComputeError, MissionError};
use super::types::{MissionPhase, PerchGeometry};

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
/// # Arguments
/// * `chief` — Chief spacecraft ECI state
/// * `deputy` — Deputy spacecraft ECI state
/// * `config` — Proximity threshold configuration
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
///
/// # Arguments
/// * `chief` — Chief spacecraft ECI state
/// * `deputy` — Deputy spacecraft ECI state
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
/// # Arguments
/// * `chief` — Chief Keplerian elements (provides normalizing SMA)
/// * `deputy` — Deputy Keplerian elements
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
/// # Arguments
/// * `perch` — Perch geometry specification (V-bar, R-bar, or custom ROE)
/// * `chief` — Chief Keplerian elements (provides normalizing SMA)
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

/// Compute eclipse data for the Lambert transfer phase.
///
/// Densifies the Lambert arc (deputy trajectory) and propagates the chief
/// orbit over the same time of flight, then computes eclipse snapshots for
/// both. The deputy and chief can be hundreds to thousands of km apart
/// during the transfer — genuinely different eclipse states.
///
/// # Arguments
///
/// * `transfer` — Solved Lambert transfer (departure/arrival states, TOF)
/// * `chief` — Chief ECI state at the transfer departure epoch
/// * `arc_steps` — Number of arc steps for densification (e.g., 200).
///   Produces `arc_steps + 1` sample points along each trajectory.
///
/// # Errors
///
/// Returns [`EclipseComputeError::Conversion`] if arc densification or
/// chief propagation fails (degenerate orbit geometry).
pub fn compute_transfer_eclipse(
    transfer: &LambertTransfer,
    chief: &StateVector,
    arc_steps: u32,
) -> Result<TransferEclipseData, EclipseComputeError> {
    let deputy_trajectory = transfer.densify_arc(arc_steps)?;
    let chief_trajectory =
        crate::propagation::keplerian::propagate_keplerian(chief, transfer.tof_s, arc_steps)?;

    let deputy_celestial = compute_eclipse_from_states(&deputy_trajectory);
    let chief_celestial = compute_eclipse_from_states(&chief_trajectory);
    let chief_eclipse: Vec<EclipseState> =
        chief_celestial.iter().map(|s| s.eclipse_state).collect();
    let summary = extract_eclipse_intervals(&deputy_celestial);

    Ok(TransferEclipseData {
        summary,
        deputy_celestial,
        chief_eclipse,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::elements::roe::compute_roe;
    use crate::elements::roe_to_ric::roe_to_ric;
    use crate::test_helpers::{iss_like_elements, test_epoch};
    use crate::types::KeplerianElements;

    /// Tolerance for ROE identity comparison: classify_separation computes
    /// the same ROE as a direct compute_roe call on the same elements.
    /// Both paths invoke the identical function — error is f64 arithmetic only.
    const ROE_IDENTITY_TOL: f64 = 1e-14;

    /// Tolerance for dimensionless separation near-zero check.
    /// When δa, δex, δey, δix are all structurally zero (only δλ/δiy differ),
    /// the norm should be negligible; 1e-10 accommodates Keplerian→ROE roundtrip noise.
    const DIMENSIONLESS_NEAR_ZERO_TOL: f64 = 1e-10;

    /// Tolerance for structural zeros: ROE components that are set to exactly 0.0
    /// in the perch geometry constructor and never undergo arithmetic.
    /// 1e-15 is near the f64 machine epsilon floor.
    const STRUCTURAL_ZERO_TOL: f64 = 1e-15;

    /// Tolerance for perch ROE expected-value comparison.
    /// A single division (offset_km / a_km) introduces one ULP of rounding;
    /// 1e-12 is conservative for this single-operation error.
    const PERCH_ROE_EXPECTED_TOL: f64 = 1e-12;

    #[test]
    fn classify_proximity_for_close_spacecraft() {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 1.0; // 1 km higher → Proximity regime

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
            i_rad: 45.0_f64.to_radians(), // different inclination plane
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
        // Only change mean anomaly (affects δλ) and RAAN (affects δiy).
        // These should NOT increase dimensionless separation because δλ and δiy are excluded.
        deputy.mean_anomaly_rad += 0.5; // large δλ
        deputy.raan_rad += 0.1; // large δiy

        // δa, δex, δey, δix should all be zero (same a, e, i, aop)
        let sep = dimensionless_separation(&chief, &deputy).unwrap();
        assert!(
            sep < DIMENSIONLESS_NEAR_ZERO_TOL,
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
            assert!((roe.da - direct_roe.da).abs() < ROE_IDENTITY_TOL, "da mismatch");
            assert!((roe.dlambda - direct_roe.dlambda).abs() < ROE_IDENTITY_TOL, "dlambda mismatch");
            assert!((roe.dex - direct_roe.dex).abs() < ROE_IDENTITY_TOL, "dex mismatch");
            assert!((roe.dey - direct_roe.dey).abs() < ROE_IDENTITY_TOL, "dey mismatch");
            assert!((roe.dix - direct_roe.dix).abs() < ROE_IDENTITY_TOL, "dix mismatch");
            assert!((roe.diy - direct_roe.diy).abs() < ROE_IDENTITY_TOL, "diy mismatch");
        } else {
            panic!("Expected Proximity phase");
        }
    }

    #[test]
    fn vbar_perch_gives_along_track_roe() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::VBar { along_track_km: 5.0 };
        let roe = perch_to_roe(&perch, &chief).expect("V-bar perch should work");

        let expected_dlambda = 5.0 / chief.a_km;
        assert!(
            (roe.dlambda - expected_dlambda).abs() < PERCH_ROE_EXPECTED_TOL,
            "dlambda should be {expected_dlambda}, got {}", roe.dlambda
        );
        assert!(roe.da.abs() < STRUCTURAL_ZERO_TOL, "da should be zero for V-bar");
        assert!(roe.dex.abs() < STRUCTURAL_ZERO_TOL, "dex should be zero");
        assert!(roe.dey.abs() < STRUCTURAL_ZERO_TOL, "dey should be zero");

        let ric = roe_to_ric(&roe, &chief).unwrap();
        assert!(
            ric.position_ric_km.y.abs() > 1.0,
            "V-bar perch should have along-track offset, got {}", ric.position_ric_km.y
        );
    }

    #[test]
    fn rbar_perch_gives_radial_roe() {
        let chief = iss_like_elements();
        let perch = PerchGeometry::RBar { radial_km: 2.0 };
        let roe = perch_to_roe(&perch, &chief).expect("R-bar perch should work");

        let expected_da = 2.0 / chief.a_km;
        assert!(
            (roe.da - expected_da).abs() < PERCH_ROE_EXPECTED_TOL,
            "da should be {expected_da}, got {}", roe.da
        );
        assert!(roe.dlambda.abs() < STRUCTURAL_ZERO_TOL, "dlambda should be zero for R-bar");
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
}

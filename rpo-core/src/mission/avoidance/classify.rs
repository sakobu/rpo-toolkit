//! Correction type classification for collision avoidance.
//!
//! Determines whether a COLA maneuver should target in-plane eccentricity,
//! cross-track inclination, or a combined correction, based on the geometry
//! of the predicted closest approach and the ROE state.

use crate::mission::closest_approach::ClosestApproach;
use crate::types::QuasiNonsingularROE;

use super::types::{CorrectionType, COLA_DOMINANCE_RATIO, COLA_MIN_ROE_MAGNITUDE};

/// Classify the correction type based on POCA RIC position and ROE magnitudes.
///
/// Uses the radial/cross-track dominance ratio to determine whether the
/// correction should be in-plane (eccentricity), cross-track (inclination),
/// or combined. Degeneracy guards force a particular type when one vector
/// component is too small.
///
/// # Arguments
///
/// * `poca` — Predicted closest approach whose RIC position determines the
///   dominant correction axis (radial vs cross-track).
/// * `roe` — Deputy ROE state; eccentricity (`dex`, `dey`) and inclination
///   (`dix`, `diy`) vector magnitudes determine degeneracy.
///
/// # Invariants
///
/// - If both `|δe|` and `|δi|` are below [`COLA_MIN_ROE_MAGNITUDE`], returns
///   `InPlane` as a default; the caller is expected to reject degenerate
///   geometry separately.
/// - Dominance threshold: a component must exceed the other by a factor of
///   [`COLA_DOMINANCE_RATIO`] (2×) to be classified as single-axis.
#[allow(clippy::similar_names)]
pub(super) fn classify_correction(
    poca: &ClosestApproach,
    roe: &QuasiNonsingularROE,
) -> CorrectionType {
    let de_mag = (roe.dex * roe.dex + roe.dey * roe.dey).sqrt();
    let di_mag = (roe.dix * roe.dix + roe.diy * roe.diy).sqrt();

    // Degeneracy guards
    if de_mag < COLA_MIN_ROE_MAGNITUDE && di_mag < COLA_MIN_ROE_MAGNITUDE {
        // Both degenerate — will hit DegenerateGeometry later; pick InPlane as default
        return CorrectionType::InPlane;
    }
    if de_mag < COLA_MIN_ROE_MAGNITUDE {
        return CorrectionType::CrossTrack;
    }
    if di_mag < COLA_MIN_ROE_MAGNITUDE {
        return CorrectionType::InPlane;
    }

    // Compare radial (R) and cross-track (C) components of POCA position
    let r_component = poca.position_ric_km.x.abs();
    let c_component = poca.position_ric_km.z.abs();

    if r_component > COLA_DOMINANCE_RATIO * c_component {
        CorrectionType::InPlane
    } else if c_component > COLA_DOMINANCE_RATIO * r_component {
        CorrectionType::CrossTrack
    } else {
        CorrectionType::Combined
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{damico_table21_chief, test_epoch};
    use nalgebra::Vector3;

    #[test]
    fn combined_correction_type() {
        let chief = damico_table21_chief();
        let a = chief.a_km;

        // ROE with both significant e and i components
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.200 / a,
            dey: 0.200 / a,
            dix: 0.200 / a,
            diy: 0.200 / a,
        };

        // POCA with balanced R and C components (neither dominates by 2x)
        let poca = ClosestApproach {
            epoch: test_epoch(),
            elapsed_s: 100.0,
            distance_km: 0.1,
            position_ric_km: Vector3::new(0.05, 0.02, 0.05),
            velocity_ric_km_s: Vector3::zeros(),
            leg_index: 0,
            is_global_minimum: true,
        };

        let ct = classify_correction(&poca, &roe);
        assert_eq!(
            ct,
            CorrectionType::Combined,
            "balanced R/C should give Combined, got {ct:?}"
        );
    }
}

//! Safe perch enrichment for V-bar/R-bar/Custom geometries.
//!
//! Enriches geometric perch ROE with passively safe e/i vectors.
//! V-bar and R-bar perches have degenerate (zero) e/i vectors from
//! `perch_to_roe()` — this module adds sized, aligned e/i components
//! to ensure passive safety (D'Amico Eq. 2.23).
//!
//! Custom perches delegate to `enrich_waypoint()` from the null-space
//! safety projection.
//!
//! # References
//!
//! - D'Amico Eq. 2.23: `d_min = a · min(|δe|, |δi|)` for parallel e/i
//! - D'Amico Eq. 2.32: nominal safe formation configuration

use crate::elements::roe_to_ric::roe_to_ric;
use crate::mission::errors::MissionError;
use crate::mission::planning::perch_to_roe;
use crate::mission::types::PerchGeometry;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::safety_envelope::enrich_waypoint;
use super::{EiAlignment, FormationDesignError, SafePerch, SafetyRequirements};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Enrich a perch geometry with safe e/i vectors.
///
/// # Algorithm
///
/// **V-bar / R-bar** (zero e/i baseline):
/// 1. Compute geometric ROE via `perch_to_roe()`
/// 2. Size e/i: `de_nom = min_separation_km / a`
/// 3. Assign `dey = ±de_nom`, `diy = ±de_nom` (sign from alignment)
/// 4. `min_rc_separation_km = a · de_nom` (D'Amico Eq. 2.23, equal magnitudes)
///
/// **Custom** (arbitrary ROE baseline):
/// 1. Convert ROE to RIC position via `roe_to_ric()`
/// 2. Delegate to `enrich_waypoint()` (null-space projection, 3 DOF free)
/// 3. Map `EnrichedWaypoint` to `SafePerch`
///
/// # Invariants
///
/// - `chief_mean.a_km > 0`, `chief_mean.e < 1`
/// - `requirements.min_separation_km > 0`
///
/// # Errors
///
/// - `SingularGeometry` — V-bar/R-bar perch with zero offset
/// - `InvalidChiefElements` — invalid chief orbital elements
/// - `SeparationUnachievable` — requested separation exceeds linearization bound (Custom only)
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). V-bar/R-bar enrichment is a direct
/// assignment (no linearization involved). Custom perch enrichment has
/// the same linearization regime as `enrich_waypoint()`.
///
/// # References
///
/// - D'Amico Eq. 2.23 (`d_min = a · min(|δe|, |δi|)` for parallel e/i)
/// - D'Amico Eq. 2.32 (nominal safe configuration)
#[must_use = "enrichment result should be inspected"]
pub fn enrich_perch(
    perch: &PerchGeometry,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<SafePerch, FormationDesignError> {
    match perch {
        PerchGeometry::VBar { .. } | PerchGeometry::RBar { .. } => {
            enrich_simple_perch(perch, chief_mean, requirements)
        }
        PerchGeometry::Custom(custom_roe) => {
            enrich_custom_perch(custom_roe, chief_mean, requirements)
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Enrich V-bar or R-bar perch with direct e/i assignment.
///
/// V-bar/R-bar baselines have zero e/i vectors, so enrichment directly
/// assigns `dey` and `diy` to reach the target separation. The geometric
/// offset (`dlambda` for V-bar, `da` for R-bar) is preserved.
fn enrich_simple_perch(
    perch: &PerchGeometry,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<SafePerch, FormationDesignError> {
    // 1. Geometric baseline ROE
    let mut roe =
        perch_to_roe(perch, chief_mean).map_err(|e| map_mission_error(e, chief_mean))?;

    // 2. Dimensionless target magnitude
    let de_nom = requirements.min_separation_km / chief_mean.a_km;

    // 3. Resolve alignment: Auto → Parallel for zero-baseline e/i
    let resolved = match requirements.alignment {
        EiAlignment::Auto | EiAlignment::Parallel => EiAlignment::Parallel,
        EiAlignment::AntiParallel => EiAlignment::AntiParallel,
    };

    // 4. Set dey, diy based on alignment (D'Amico Eq. 2.32)
    match resolved {
        EiAlignment::Parallel => {
            roe.dey = de_nom;
            roe.diy = de_nom;
        }
        EiAlignment::AntiParallel => {
            roe.dey = de_nom;
            roe.diy = -de_nom;
        }
        EiAlignment::Auto => unreachable!("Auto resolved before this point"),
    }

    // 5. Compute separation (D'Amico Eq. 2.23 with equal magnitudes)
    let magnitude_km = chief_mean.a_km * de_nom;

    Ok(SafePerch {
        roe,
        de_magnitude_km: magnitude_km,
        di_magnitude_km: magnitude_km,
        min_rc_separation_km: magnitude_km,
        alignment: resolved,
    })
}

/// Enrich Custom perch via null-space projection.
///
/// Converts the custom ROE to RIC position, then delegates to
/// `enrich_waypoint()` with `velocity = None` (position-only, 3 DOF free).
fn enrich_custom_perch(
    custom_roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<SafePerch, FormationDesignError> {
    // 1. Convert custom ROE to RIC position
    let ric = roe_to_ric(custom_roe, chief_mean)?;

    // 2. Enrich via null-space projection (position-only, 3 DOF free)
    let enriched = enrich_waypoint(&ric.position_ric_km, None, chief_mean, requirements)?;

    // 3. Map EnrichedWaypoint → SafePerch (enriched_ei already computed by enrich_waypoint)
    Ok(SafePerch {
        roe: enriched.roe,
        de_magnitude_km: chief_mean.a_km * enriched.roe.de_magnitude(),
        di_magnitude_km: chief_mean.a_km * enriched.roe.di_magnitude(),
        min_rc_separation_km: enriched.enriched_ei.min_separation_km,
        alignment: enriched.resolved_alignment,
    })
}

/// Map `MissionError` from `perch_to_roe()` to `FormationDesignError`.
///
/// V-bar/R-bar zero-offset errors map to `SingularGeometry` — a degenerate
/// perch with zero offset has undefined formation geometry. Conversion errors
/// are forwarded directly.
fn map_mission_error(
    err: MissionError,
    chief_mean: &KeplerianElements,
) -> FormationDesignError {
    match err {
        MissionError::Conversion(ce) => FormationDesignError::InvalidChiefElements(ce),
        // perch_to_roe() returns InvalidVBarOffset, InvalidRBarOffset, or
        // Conversion. The first two indicate degenerate perch geometry.
        _ => FormationDesignError::SingularGeometry {
            mean_arg_lat_rad: chief_mean.mean_arg_of_lat(),
        },
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::planning::perch_to_roe;
    use crate::test_helpers::damico_table21_chief;

    /// Tolerance for enriched separation threshold tests (km).
    /// The V-bar/R-bar enrichment formula is exact: `min_rc = a * (min_sep / a) = min_sep`.
    /// Only f64 rounding contributes error. 1e-10 km = 0.1 nanometer.
    const ENRICHMENT_SEPARATION_TOL: f64 = 1e-10;

    /// Tolerance for ROE component preservation tests (dimensionless).
    /// V-bar/R-bar enrichment only modifies dey/diy; other components should be
    /// unchanged at the bit level. 1e-15 covers any f64 arithmetic noise from
    /// the `perch_to_roe` normalization (`offset / a_km`).
    const ROE_PRESERVATION_TOL: f64 = 1e-15;

    /// V-bar perch enriched with min_separation_km = 0.15 km (TanDEM-X baseline)
    /// on D'Amico Table 2.1 chief orbit (a = 7078.135 km).
    ///
    /// Verify: `min_rc_separation_km ≥ 0.15 km` (D'Amico Eq. 2.23).
    /// Verify: `dey` and `diy` are both nonzero and have the same sign (parallel).
    ///
    /// Tolerance: `ENRICHMENT_SEPARATION_TOL` = 1e-10 km.
    /// Regression target: `min_rc = a · (0.15 / a) = 0.15 km` (exact by construction).
    ///
    /// # References
    /// - D'Amico Eq. 2.23 (`d_min = a · min(|δe|, |δi|)` for parallel e/i)
    /// - D'Amico Eq. 2.32 (nominal safe configuration)
    #[test]
    fn vbar_perch_meets_separation_threshold() {
        let chief = damico_table21_chief();
        let perch = PerchGeometry::VBar {
            along_track_km: 1.0,
        };
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_perch(&perch, &chief, &reqs).expect("enrichment should succeed");

        assert!(
            result.min_rc_separation_km >= 0.15 - ENRICHMENT_SEPARATION_TOL,
            "min_rc_separation_km = {}, expected >= 0.15",
            result.min_rc_separation_km
        );
        assert!(
            result.roe.dey.abs() > 0.0,
            "dey should be nonzero after enrichment"
        );
        assert!(
            result.roe.diy.abs() > 0.0,
            "diy should be nonzero after enrichment"
        );
        assert_eq!(
            result.roe.dey.signum(),
            result.roe.diy.signum(),
            "dey and diy should have same sign for Parallel alignment"
        );
    }

    /// R-bar perch enriched with min_separation_km = 0.15 km on D'Amico Table 2.1
    /// chief orbit.
    ///
    /// Verify: `min_rc_separation_km ≥ 0.15 km`.
    /// Verify: `da` preserved from geometric perch.
    /// Verify: `dey` and `diy` nonzero with same sign (Parallel).
    ///
    /// Tolerance: `ENRICHMENT_SEPARATION_TOL` for separation,
    /// `ROE_PRESERVATION_TOL` for `da` preservation.
    #[test]
    fn rbar_perch_meets_separation_threshold() {
        let chief = damico_table21_chief();
        let perch = PerchGeometry::RBar { radial_km: 0.5 };
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let geometric_roe = perch_to_roe(&perch, &chief).expect("geometric perch valid");
        let result = enrich_perch(&perch, &chief, &reqs).expect("enrichment should succeed");

        // Separation met
        assert!(
            result.min_rc_separation_km >= 0.15 - ENRICHMENT_SEPARATION_TOL,
            "min_rc_separation_km = {}, expected >= 0.15",
            result.min_rc_separation_km
        );

        // da preserved from geometric perch
        assert!(
            (result.roe.da - geometric_roe.da).abs() < ROE_PRESERVATION_TOL,
            "da should be preserved: enriched = {}, geometric = {}",
            result.roe.da,
            geometric_roe.da
        );

        // e/i vectors nonzero and parallel
        assert!(result.roe.dey.abs() > 0.0, "dey should be nonzero");
        assert!(result.roe.diy.abs() > 0.0, "diy should be nonzero");
        assert_eq!(
            result.roe.dey.signum(),
            result.roe.diy.signum(),
            "dey and diy should have same sign for Parallel"
        );
    }

    /// V-bar perch enrichment preserves geometric offset.
    ///
    /// The enriched perch ROE must preserve the original geometric offset:
    /// `dlambda` unchanged, `da = dex = dix = 0`. Only `dey` and `diy`
    /// are added.
    ///
    /// Tolerance: `ROE_PRESERVATION_TOL` = 1e-15 (f64 arithmetic noise only).
    #[test]
    fn vbar_perch_preserves_dlambda() {
        let chief = damico_table21_chief();
        let perch = PerchGeometry::VBar {
            along_track_km: 2.0,
        };
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let geometric_roe = perch_to_roe(&perch, &chief).expect("geometric perch valid");
        let result = enrich_perch(&perch, &chief, &reqs).expect("enrichment should succeed");

        // dlambda preserved
        assert!(
            (result.roe.dlambda - geometric_roe.dlambda).abs() < ROE_PRESERVATION_TOL,
            "dlambda should be preserved: enriched = {}, geometric = {}",
            result.roe.dlambda,
            geometric_roe.dlambda
        );

        // Only dey, diy modified — da, dex, dix remain zero
        assert!(
            result.roe.da.abs() < ROE_PRESERVATION_TOL,
            "da should remain zero: {}",
            result.roe.da
        );
        assert!(
            result.roe.dex.abs() < ROE_PRESERVATION_TOL,
            "dex should remain zero: {}",
            result.roe.dex
        );
        assert!(
            result.roe.dix.abs() < ROE_PRESERVATION_TOL,
            "dix should remain zero: {}",
            result.roe.dix
        );

        // dey and diy are nonzero
        assert!(result.roe.dey.abs() > ROE_PRESERVATION_TOL, "dey should be nonzero");
        assert!(result.roe.diy.abs() > ROE_PRESERVATION_TOL, "diy should be nonzero");
    }
}

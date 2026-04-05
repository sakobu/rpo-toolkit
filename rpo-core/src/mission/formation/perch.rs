//! Safe perch enrichment for V-bar/R-bar/Custom geometries.
//!
//! Enriches geometric perch ROE with passively safe e/i vectors.
//! V-bar and R-bar perches have degenerate (zero) e/i vectors from
//! `perch_to_roe()` — this module adds sized, aligned e/i components
//! to ensure passive safety (D'Amico Eq. 2.23).
//!
//! Custom perches apply `compute_safety_projection()` directly to the
//! original ROE, preserving e/i structure when already above threshold.
//!
//! # References
//!
//! - D'Amico Eq. 2.23: `d_min = a · min(|δe|, |δi|)` for parallel e/i
//! - D'Amico Eq. 2.32: nominal safe formation configuration

use crate::mission::errors::MissionError;
use crate::mission::planning::perch_to_roe;
use crate::mission::safety::compute_ei_separation;
use crate::mission::types::PerchGeometry;
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::safety_envelope::compute_safety_projection;
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
/// 1. Apply `compute_safety_projection()` directly to the original custom ROE
/// 2. Preserves existing e/i structure when already above threshold
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
    let baseline_roe =
        perch_to_roe(perch, chief_mean).map_err(|e| map_mission_error(e, chief_mean))?;
    let mut roe = baseline_roe;

    // 2. Dimensionless target magnitude
    let de_nom = requirements.min_separation_km / chief_mean.a_km;

    // 3. Resolve alignment and set dey, diy (D'Amico Eq. 2.32)
    //    Auto → Parallel for zero-baseline e/i (simple perch has no prior e/i to compare).
    let resolved = match requirements.alignment {
        EiAlignment::Auto | EiAlignment::Parallel => {
            roe.dey = de_nom;
            roe.diy = de_nom;
            EiAlignment::Parallel
        }
        EiAlignment::AntiParallel => {
            roe.dey = de_nom;
            roe.diy = -de_nom;
            EiAlignment::AntiParallel
        }
    };

    // 4. Compute separation (D'Amico Eq. 2.23 with equal magnitudes)
    let magnitude_km = chief_mean.a_km * de_nom;

    Ok(SafePerch {
        baseline_roe,
        roe,
        de_magnitude_km: magnitude_km,
        di_magnitude_km: magnitude_km,
        min_rc_separation_km: magnitude_km,
        alignment: resolved,
    })
}

/// Enrich Custom perch via direct safety projection on original ROE.
///
/// Applies `compute_safety_projection` directly to the user's custom ROE,
/// preserving the original e/i structure when it already meets the safety
/// threshold. This avoids the lossy ROE→RIC→minimum-norm roundtrip that
/// would discard the original ROE's e/i information.
fn enrich_custom_perch(
    custom_roe: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<SafePerch, FormationDesignError> {
    let (enriched_roe, resolved) =
        compute_safety_projection(custom_roe, chief_mean, requirements)?;
    let enriched_ei = compute_ei_separation(&enriched_roe, chief_mean);

    Ok(SafePerch {
        baseline_roe: *custom_roe,
        roe: enriched_roe,
        de_magnitude_km: chief_mean.a_km * enriched_roe.de_magnitude(),
        di_magnitude_km: chief_mean.a_km * enriched_roe.di_magnitude(),
        min_rc_separation_km: enriched_ei.min_separation_km,
        alignment: resolved,
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

    /// Custom perch with e/i already above threshold → enrichment is near-no-op.
    ///
    /// Regression test for the bug where `enrich_custom_perch` converted custom ROE
    /// to RIC position, then recomputed minimum-norm ROE via pseudo-inverse, discarding
    /// the original ROE's e/i structure. With the fix, `compute_safety_projection` sees
    /// the original ROE's |δe| = 209m and |δi| = 313m (both above 100m threshold) and
    /// preserves them with minimal adjustment.
    ///
    /// Tolerance: `CUSTOM_PERCH_NOOP_TOL` = 1e-6 dimensionless ROE. The safety
    /// projection adjusts e/i phase alignment (rotating e-vector to match i-vector),
    /// which can shift individual components while preserving magnitudes. The norm
    /// change should be small but not exactly zero.
    #[test]
    fn custom_perch_above_threshold_is_near_noop() {
        let chief = damico_table21_chief();
        // Custom ROE from bug report reproduction: |δe| ≈ 209m, |δi| ≈ 313m
        let custom_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 4.42e-4,
            dex: 0.0,
            dey: 2.95e-5,
            dix: 0.0,
            diy: 4.42e-5,
        };
        let reqs = SafetyRequirements {
            min_separation_km: 0.1, // 100m threshold
            alignment: EiAlignment::Auto,
        };

        let result = enrich_perch(
            &PerchGeometry::Custom(custom_roe),
            &chief,
            &reqs,
        )
        .expect("enrichment should succeed");

        // Enriched magnitudes should preserve originals (both above threshold)
        let original_de_km = chief.a_km * custom_roe.de_magnitude();
        let original_di_km = chief.a_km * custom_roe.di_magnitude();
        assert!(
            (result.de_magnitude_km - original_de_km).abs() < 0.001,
            "de should be preserved: enriched={:.4} km, original={:.4} km",
            result.de_magnitude_km,
            original_de_km,
        );
        assert!(
            (result.di_magnitude_km - original_di_km).abs() < 0.001,
            "di should be preserved: enriched={:.4} km, original={:.4} km",
            result.di_magnitude_km,
            original_di_km,
        );

        // ROE vector change should be small (near-no-op)
        let norm_change = ((result.roe.da - custom_roe.da).powi(2)
            + (result.roe.dlambda - custom_roe.dlambda).powi(2)
            + (result.roe.dex - custom_roe.dex).powi(2)
            + (result.roe.dey - custom_roe.dey).powi(2)
            + (result.roe.dix - custom_roe.dix).powi(2)
            + (result.roe.diy - custom_roe.diy).powi(2))
        .sqrt();
        assert!(
            norm_change < 1e-6,
            "enrichment should be near-no-op for ROE already above threshold, norm_change={}",
            norm_change,
        );

        // Baseline ROE should be the original custom ROE
        assert!(
            (result.baseline_roe.dlambda - custom_roe.dlambda).abs() < 1e-15,
            "baseline should preserve original dlambda",
        );
    }

    /// Custom perch with e/i below threshold → enrichment inflates to meet threshold.
    ///
    /// The custom ROE has |δe| ≈ 35m and |δi| ≈ 35m, both below the 100m threshold.
    /// Enrichment should increase both to at least 100m. Note: `compute_safety_projection`
    /// applies null-space adjustments to `da`/`dlambda` to preserve the RIC position
    /// when inflating e/i, so those components may shift.
    #[test]
    fn custom_perch_below_threshold_inflates_to_meet() {
        let chief = damico_table21_chief();
        // Small e/i: a * 5e-6 ≈ 35m (below 100m threshold)
        let custom_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 4.42e-4,
            dex: 0.0,
            dey: 5e-6,
            dix: 0.0,
            diy: 5e-6,
        };
        let reqs = SafetyRequirements {
            min_separation_km: 0.1, // 100m threshold
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_perch(
            &PerchGeometry::Custom(custom_roe),
            &chief,
            &reqs,
        )
        .expect("enrichment should succeed");

        // Both magnitudes should meet threshold
        assert!(
            result.de_magnitude_km >= 0.1 - ENRICHMENT_SEPARATION_TOL,
            "de should meet threshold: {:.4} km",
            result.de_magnitude_km,
        );
        assert!(
            result.di_magnitude_km >= 0.1 - ENRICHMENT_SEPARATION_TOL,
            "di should meet threshold: {:.4} km",
            result.di_magnitude_km,
        );

        // min_rc_separation should meet threshold
        assert!(
            result.min_rc_separation_km >= 0.1 - ENRICHMENT_SEPARATION_TOL,
            "min_rc should meet threshold: {:.4} km",
            result.min_rc_separation_km,
        );

        // Baseline should be the original custom ROE
        assert!(
            (result.baseline_roe.dey - custom_roe.dey).abs() < ROE_PRESERVATION_TOL,
            "baseline should preserve original dey",
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

        // Baseline ROE should have zero e/i components (geometric V-bar)
        assert!(
            result.baseline_roe.dey.abs() < ROE_PRESERVATION_TOL,
            "baseline dey should be zero (geometric V-bar): {}",
            result.baseline_roe.dey
        );
        assert!(
            result.baseline_roe.diy.abs() < ROE_PRESERVATION_TOL,
            "baseline diy should be zero (geometric V-bar): {}",
            result.baseline_roe.diy
        );
    }

}

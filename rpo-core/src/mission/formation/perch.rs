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
use super::{
    EiAlignment, FormationDesignError, LINEARIZATION_PERTURBATION_BOUND, SafePerch,
    SafetyRequirements,
};

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
/// - `SeparationUnachievable` — requested separation would produce a
///   perturbation norm exceeding the shared `LINEARIZATION_PERTURBATION_BOUND`
///   (formation module constant, 0.01 dimensionless); reachable from both
///   the V-bar/R-bar branch and the Custom branch (the latter via
///   `compute_safety_projection`).
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). Both branches enforce the same
/// linearization envelope (`LINEARIZATION_PERTURBATION_BOUND`, D'Amico §2.3.4):
/// V-bar/R-bar checks `sqrt(2) · de_nom` directly (since only `dey`/`diy`
/// are modified); Custom delegates to `compute_safety_projection` which
/// checks the full 6-vector perturbation norm.
///
/// # References
///
/// - D'Amico Eq. 2.23 (`d_min = a · min(|δe|, |δi|)` for parallel e/i)
/// - D'Amico Eq. 2.32 (nominal safe configuration)
/// - D'Amico §2.3.4 (ROE linearization validity)
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
///
/// # Errors
///
/// - [`FormationDesignError::SeparationUnachievable`] when `sqrt(2) · de_nom`
///   would exceed [`LINEARIZATION_PERTURBATION_BOUND`]. Since only `dey` and
///   `diy` are modified, the 6-vector perturbation norm reduces to
///   `sqrt(dey² + diy²) = sqrt(2) · de_nom`; this gate matches the envelope
///   `compute_safety_projection` enforces for the Custom branch, keeping the
///   two paths behaviorally indistinguishable at the boundary. See D'Amico
///   §2.3.4.
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

    // Validity gate — identical envelope to `compute_safety_projection`.
    // The enrichment only modifies `dey` and `diy`, so the 6-vector perturbation
    // norm is `sqrt(dey² + diy²) = sqrt(2) · de_nom`. Reject when that norm would
    // exceed the shared linearization bound. D'Amico §2.3.4.
    let perturbation_norm = std::f64::consts::SQRT_2 * de_nom;
    if perturbation_norm > LINEARIZATION_PERTURBATION_BOUND {
        let achievable_km =
            chief_mean.a_km * LINEARIZATION_PERTURBATION_BOUND / std::f64::consts::SQRT_2;
        return Err(FormationDesignError::SeparationUnachievable {
            requested_km: requirements.min_separation_km,
            achievable_km,
        });
    }

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
    let (enriched_roe, resolved) = compute_safety_projection(
        custom_roe,
        chief_mean,
        requirements,
        0.0, // no J2 pre-rotation: static perch geometry
    )?;
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
    // Tests intentionally mirror D'Amico paper symbols (δe/δi magnitudes)
    // for equation traceability; renaming to satisfy `similar_names` would
    // break the paper-to-code mapping required by Codebase Published-method
    // fidelity and Traceability rules.
    #![allow(clippy::similar_names)]

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

    /// V-bar perch enriched with `min_separation_km` = 0.15 km (TanDEM-X baseline)
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
            result.roe.dey.is_sign_positive(),
            result.roe.diy.is_sign_positive(),
            "dey and diy should have same sign for Parallel alignment"
        );
    }

    /// R-bar perch enriched with `min_separation_km` = 0.15 km on D'Amico Table 2.1
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
            result.roe.dey.is_sign_positive(),
            result.roe.diy.is_sign_positive(),
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
            "enrichment should be near-no-op for ROE already above threshold, norm_change={norm_change}",
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

    // -----------------------------------------------------------------------
    // Validity-gate tests (Phase 2c)
    //
    // `enrich_simple_perch` now rejects requests whose perturbation norm
    // would exceed `LINEARIZATION_PERTURBATION_BOUND`, matching the envelope
    // `compute_safety_projection` has always enforced for the Custom branch.
    // The four tests below pin the inequality direction, confirm the two
    // branches agree at the boundary, and prevent the V-bar/R-bar path from
    // silently drifting past validity in the future.
    // -----------------------------------------------------------------------

    /// V-bar perch at D'Amico Table 2.1 orbit (a ≈ 7078.135 km) with
    /// `min_separation_km = 100.0` has `sqrt(2) · de_nom ≈ 2.0e-2`, well past
    /// `LINEARIZATION_PERTURBATION_BOUND = 0.01`. The engine must reject with
    /// `SeparationUnachievable` reporting the exact symbolic boundary
    /// `a · bound / sqrt(2)`.
    ///
    /// Regression target: achievable ≈ 7078.135 · 0.01 / sqrt(2) ≈ 50.05 km.
    #[test]
    fn vbar_perch_rejects_separation_past_validity() {
        let chief = damico_table21_chief();
        let perch = PerchGeometry::VBar { along_track_km: 1.0 };
        let reqs = SafetyRequirements {
            min_separation_km: 100.0,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_perch(&perch, &chief, &reqs);
        let expected_achievable =
            chief.a_km * LINEARIZATION_PERTURBATION_BOUND / std::f64::consts::SQRT_2;

        match result {
            Err(FormationDesignError::SeparationUnachievable {
                requested_km,
                achievable_km,
            }) => {
                assert!(
                    (requested_km - 100.0).abs() < ENRICHMENT_SEPARATION_TOL,
                    "requested_km = {requested_km}, expected 100.0"
                );
                assert!(
                    (achievable_km - expected_achievable).abs() < ENRICHMENT_SEPARATION_TOL,
                    "achievable_km = {achievable_km}, expected {expected_achievable}"
                );
            }
            other => panic!("expected SeparationUnachievable, got {other:?}"),
        }
    }

    /// R-bar variant of `vbar_perch_rejects_separation_past_validity`.
    /// Same algebra (only `dey`/`diy` are modified for R-bar too), so the
    /// same symbolic achievable bound applies.
    #[test]
    fn rbar_perch_rejects_separation_past_validity() {
        let chief = damico_table21_chief();
        let perch = PerchGeometry::RBar { radial_km: 0.5 };
        let reqs = SafetyRequirements {
            min_separation_km: 100.0,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_perch(&perch, &chief, &reqs);
        let expected_achievable =
            chief.a_km * LINEARIZATION_PERTURBATION_BOUND / std::f64::consts::SQRT_2;

        match result {
            Err(FormationDesignError::SeparationUnachievable {
                achievable_km, ..
            }) => {
                assert!(
                    (achievable_km - expected_achievable).abs() < ENRICHMENT_SEPARATION_TOL,
                    "achievable_km = {achievable_km}, expected {expected_achievable}"
                );
            }
            other => panic!("expected SeparationUnachievable, got {other:?}"),
        }
    }

    /// At `min_separation_km = a · bound / sqrt(2)` (the strict symbolic
    /// boundary), the gate must accept — pins the inequality as strict `>`
    /// and guards against future regressions flipping to `>=`.
    ///
    /// Tolerance: `ENRICHMENT_SEPARATION_TOL` = 1e-10 km on the enriched
    /// magnitude, matching existing V-bar/R-bar tests.
    #[test]
    fn vbar_perch_accepts_separation_at_validity_edge() {
        let chief = damico_table21_chief();
        let edge_km = chief.a_km * LINEARIZATION_PERTURBATION_BOUND / std::f64::consts::SQRT_2;
        let perch = PerchGeometry::VBar { along_track_km: 1.0 };
        let reqs = SafetyRequirements {
            min_separation_km: edge_km,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_perch(&perch, &chief, &reqs).expect(
            "request at the symbolic validity edge must be accepted (strict `>` boundary)",
        );

        // Regression target: min_rc_separation_km = a · (edge_km / a) = edge_km.
        assert!(
            (result.min_rc_separation_km - edge_km).abs() < ENRICHMENT_SEPARATION_TOL,
            "min_rc_separation_km = {}, expected {edge_km}",
            result.min_rc_separation_km
        );
    }

    /// Architectural invariant: both enrichment branches gate on the same
    /// `LINEARIZATION_PERTURBATION_BOUND` constant. V-bar uses a tight
    /// shape-aware bound (`a · BOUND / sqrt(2)`) because its perturbation has
    /// a known two-component shape; Custom uses the loose norm bound
    /// (`a · BOUND`) because `compute_safety_projection` reports against the
    /// 6-norm directly without assuming a fixed shape. Both are derived from
    /// the same constant — if future work harmonizes the reporting, update
    /// this test to match and note the decision in the PR.
    ///
    /// This test is the sentinel that catches **drift in the gating
    /// constant itself**: if someone introduces a second threshold for the
    /// V-bar path (e.g. `0.005`), the V-bar assertion fails against the
    /// symbolic `BOUND / sqrt(2)` expected value. If someone relaxes
    /// `compute_safety_projection`'s clamp, the Custom assertion fails.
    #[test]
    fn simple_and_custom_reject_at_same_threshold() {
        let chief = damico_table21_chief();
        let reqs = SafetyRequirements {
            min_separation_km: 100.0,
            alignment: EiAlignment::Parallel,
        };

        // V-bar baseline: geometric perch has zero e/i (same shape as the
        // zero-e/i Custom baseline built below).
        let vbar = PerchGeometry::VBar { along_track_km: 1.0 };
        let vbar_err = enrich_perch(&vbar, &chief, &reqs).expect_err(
            "V-bar at 100 km must reject at the validity boundary",
        );

        // Custom baseline: matches `perch_to_roe(vbar)` — zero e/i, nonzero dlambda.
        let custom_baseline = perch_to_roe(&vbar, &chief).expect("geometric V-bar is valid");
        let custom = PerchGeometry::Custom(custom_baseline);
        let custom_err = enrich_perch(&custom, &chief, &reqs).expect_err(
            "Custom zero-e/i at 100 km must reject at the validity boundary",
        );

        let vbar_expected = chief.a_km * LINEARIZATION_PERTURBATION_BOUND / std::f64::consts::SQRT_2;
        let custom_expected = chief.a_km * LINEARIZATION_PERTURBATION_BOUND;

        match (vbar_err, custom_err) {
            (
                FormationDesignError::SeparationUnachievable {
                    achievable_km: vbar_cap,
                    ..
                },
                FormationDesignError::SeparationUnachievable {
                    achievable_km: custom_cap,
                    ..
                },
            ) => {
                // V-bar reports tight shape-aware bound.
                assert!(
                    (vbar_cap - vbar_expected).abs() < ENRICHMENT_SEPARATION_TOL,
                    "V-bar achievable = {vbar_cap} km; expected a · BOUND / sqrt(2) = {vbar_expected}"
                );
                // Custom reports loose 6-norm bound (compute_safety_projection convention).
                assert!(
                    (custom_cap - custom_expected).abs() < ENRICHMENT_SEPARATION_TOL,
                    "Custom achievable = {custom_cap} km; expected a · BOUND = {custom_expected}"
                );
                // And the loose bound must always exceed the tight one —
                // i.e. nothing the V-bar path rejects would have passed the
                // Custom path's clamp on the same baseline.
                assert!(
                    custom_cap > vbar_cap,
                    "loose Custom bound ({custom_cap}) should exceed tight V-bar bound ({vbar_cap})"
                );
            }
            (vbar_other, custom_other) => panic!(
                "expected SeparationUnachievable from both; got vbar={vbar_other:?}, custom={custom_other:?}"
            ),
        }
    }
}

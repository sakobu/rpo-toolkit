//! Null-space safety projection for ROE enrichment.
//!
//! Computes the analytical null-space of `T_pos` (D'Amico Eq. 2.17) and uses it
//! to inject passively safe e/i geometry into ROE states without changing the
//! operator's requested RIC position.
//!
//! # Algorithm overview
//!
//! The `T_pos` matrix maps 6 ROE to 3 RIC position components. Its null space
//! has dimension 3, spanned by analytically derived basis vectors. The safety
//! projection adds a null-space perturbation that:
//!
//! 1. Sizes the inclination vector to meet the minimum separation requirement
//! 2. Aligns the eccentricity vector parallel (or anti-parallel) to the
//!    inclination vector, maximizing passive safety (D'Amico Eq. 2.22)
//! 3. Computes the coupled δa/δλ side effects from the null-space structure
//!
//! No SVD, no iteration, no optimization — the entire algorithm is a quadratic
//! formula + atan2 + direct assignment.
//!
//! # References
//!
//! - D'Amico Eq. 2.17: `T_pos` matrix (ROE → RIC position mapping)
//! - D'Amico Eq. 2.22: e/i vector separation metric
//! - D'Amico Eq. 2.23: parallel e/i simplification (`d_min = a·min(|δe|, |δi|)`)

use nalgebra::{SVector, Vector3};

use crate::elements::roe_to_ric::{compute_t_matrix, ric_position_to_roe, RicError};
use crate::elements::wrap_angle;
use crate::mission::safety::{compute_ei_separation, ROE_MAG_EPSILON};
use crate::types::{KeplerianElements, QuasiNonsingularROE};

use super::{
    EiAlignment, EnrichedWaypoint, EnrichmentMode, FormationDesignError, SafetyRequirements,
};

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// Maximum perturbation norm (dimensionless) before linearization is unreliable.
/// At norm = 0.01, the ROE separation is ~1% of SMA (~70 km for a 7000 km orbit).
/// Beyond this, second-order terms in the `T_pos` linearization produce position
/// errors exceeding the sub-km accuracy target. Consistent with the existing
/// `dimensionless_norm()` threshold used for proximity classification.
const LINEARIZATION_PERTURBATION_BOUND: f64 = 0.01;

// E/I magnitude threshold imported from safety.rs as ROE_MAG_EPSILON.
// Below this, atan2 is numerically meaningless and the phase angle is undefined.

// ---------------------------------------------------------------------------
// Alignment resolution (classification before execution)
// ---------------------------------------------------------------------------

/// Resolve `Auto` alignment to `Parallel` or `AntiParallel` based on which
/// requires less perturbation from the current e/i phase.
///
/// For degenerate baselines (both |δe| and |δi| below [`ROE_MAG_EPSILON`]),
/// defaults to `Parallel`.
fn resolve_alignment(
    baseline: &QuasiNonsingularROE,
    alignment: EiAlignment,
) -> EiAlignment {
    match alignment {
        EiAlignment::Parallel | EiAlignment::AntiParallel => alignment,
        EiAlignment::Auto => {
            let ecc_mag = baseline.de_magnitude();
            let inc_mag = baseline.di_magnitude();

            // Degenerate: no phase to compare — default to Parallel
            if ecc_mag < ROE_MAG_EPSILON || inc_mag < ROE_MAG_EPSILON {
                return EiAlignment::Parallel;
            }

            let phase_e = baseline.dey.atan2(baseline.dex);
            let phase_i = baseline.diy.atan2(baseline.dix);

            // Normalize phase differences to [-π, π]
            let parallel_err = wrap_angle(phase_e - phase_i).abs();
            let anti_err = wrap_angle(phase_e - phase_i - std::f64::consts::PI).abs();

            if parallel_err <= anti_err {
                EiAlignment::Parallel
            } else {
                EiAlignment::AntiParallel
            }
        }
    }
}


// ---------------------------------------------------------------------------
// Safety projection
// ---------------------------------------------------------------------------

/// Compute the null-space perturbation that maximizes e/i parallelism.
///
/// # Algorithm
///
/// 1. Resolve alignment: for `Auto`, evaluate both `Parallel` and `AntiParallel`
/// 2. Compute i-vector shift (α₃) via 1D quadratic to reach target magnitude
/// 3. Compute target e-vector direction (parallel to shifted i-vector)
/// 4. Set α₁ = target δex − baseline δex, α₂ = target δey − baseline δey
/// 5. Compute δa/δλ side effects from null-space structure
/// 6. Verify perturbation norm < [`LINEARIZATION_PERTURBATION_BOUND`]
///
/// # Invariants
///
/// - `requirements.min_separation_km > 0`
/// - `chief_mean.a_km > 0`, `chief_mean.e < 1`
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). The enriched ROE must remain within the
/// linearization regime: perturbation norm < 0.01.
///
/// # Near-singular behavior
///
/// When baseline e/i are both near zero (magnitude < [`ROE_MAG_EPSILON`]),
/// the phase angle atan2 is undefined. The algorithm handles this by choosing
/// the i-vector direction from the null-space shift direction \[cos u, sin u\]
/// and aligning the e-vector to match. No singularity.
///
/// # References
///
/// - D'Amico Eqs. 2.3–2.4 (e/i polar form)
/// - D'Amico Eq. 2.22 (minimum R/C distance)
/// - D'Amico Eq. 2.23 (parallel e/i simplification)
#[allow(clippy::similar_names)] // x/y component pairs (dex/dey, dix/diy) are standard orbital mechanics
pub(crate) fn compute_safety_projection(
    baseline: &QuasiNonsingularROE,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<(QuasiNonsingularROE, EiAlignment), FormationDesignError> {
    let sma = chief_mean.a_km;
    let u = chief_mean.mean_arg_of_lat();
    let (sin_u, cos_u) = u.sin_cos();

    // Dimensionless target magnitude
    let d_min = requirements.min_separation_km / sma;

    // 1. Resolve alignment
    let resolved = resolve_alignment(baseline, requirements.alignment);

    // 2. Size the i-vector via 1D quadratic in α₃
    //    |δi + α₃·[cos u, sin u]|² = d_target²
    let inc_mag = baseline.di_magnitude();
    let d_target_i = d_min.max(inc_mag);

    let proj = baseline.dix * cos_u + baseline.diy * sin_u;
    let inc_sq = inc_mag * inc_mag;
    let disc = proj * proj - (inc_sq - d_target_i * d_target_i);

    if disc < 0.0 {
        // Should not happen when d_target >= |di_baseline|, but guard numerics
        let achievable = sma * inc_mag;
        return Err(FormationDesignError::SeparationUnachievable {
            requested_km: requirements.min_separation_km,
            achievable_km: achievable,
        });
    }

    let sqrt_disc = disc.sqrt();
    let root_pos = -proj + sqrt_disc;
    let root_neg = -proj - sqrt_disc;
    let alpha_3 = if root_pos.abs() <= root_neg.abs() {
        root_pos
    } else {
        root_neg
    };

    // New i-vector after shift
    let new_dix = baseline.dix + alpha_3 * cos_u;
    let new_diy = baseline.diy + alpha_3 * sin_u;

    // 3. Align the e-vector to the (shifted) i-vector direction
    let theta_i = new_diy.atan2(new_dix);
    let ecc_mag = baseline.de_magnitude();
    let d_target_e = d_min.max(ecc_mag);

    let (tgt_ex, tgt_ey) = match resolved {
        EiAlignment::Parallel => (d_target_e * theta_i.cos(), d_target_e * theta_i.sin()),
        EiAlignment::AntiParallel => {
            let theta_anti = theta_i + std::f64::consts::PI;
            (d_target_e * theta_anti.cos(), d_target_e * theta_anti.sin())
        }
        EiAlignment::Auto => unreachable!("Auto resolved before this point"),
    };

    // 4. Null-space coefficients
    let coeff_1 = tgt_ex - baseline.dex;
    let coeff_2 = tgt_ey - baseline.dey;

    // 5. δa/δλ side effects from null-space structure
    let new_da = baseline.da + coeff_1 * cos_u + coeff_2 * sin_u;
    let new_dlambda = baseline.dlambda - 2.0 * coeff_1 * sin_u + 2.0 * coeff_2 * cos_u;

    let enriched = QuasiNonsingularROE {
        da: new_da,
        dlambda: new_dlambda,
        dex: tgt_ex,
        dey: tgt_ey,
        dix: new_dix,
        diy: new_diy,
    };

    // 6. Clamp check
    let perturbation = (enriched.to_vector() - baseline.to_vector()).norm();
    if perturbation > LINEARIZATION_PERTURBATION_BOUND {
        let achievable = sma * LINEARIZATION_PERTURBATION_BOUND;
        return Err(FormationDesignError::SeparationUnachievable {
            requested_km: requirements.min_separation_km,
            achievable_km: achievable,
        });
    }

    Ok((enriched, resolved))
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Enrich a waypoint's ROE with safe e/i geometry.
///
/// When only position is specified (3 DOF free), computes the minimum-norm ROE
/// via pseudo-inverse, then adds a null-space perturbation to maximize passive
/// safety. The RIC position is preserved exactly (null-space guarantee).
///
/// When velocity is also specified (0 DOF free), the ROE is uniquely determined
/// by the full 6×6 T matrix inverse. The module cannot inject safety without
/// changing the operator's intent. It reports the nearest safe ROE as an
/// advisory with `mode = EnrichmentMode::VelocityConstrained`.
///
/// # Invariants
///
/// - `chief_mean.a_km > 0`, `chief_mean.e < 1`
/// - `requirements.min_separation_km > 0`
///
/// # Errors
///
/// - [`FormationDesignError::SingularGeometry`] if the `T_pos` pseudo-inverse
///   or 6×6 T-matrix inverse is singular.
/// - [`FormationDesignError::InvalidChiefElements`] if chief elements are invalid.
/// - [`FormationDesignError::SeparationUnachievable`] if the requested separation
///   exceeds the linearization bound.
///
/// # Validity regime
///
/// Near-circular chief (e < ~0.1). Position preservation guaranteed within
/// sub-millimeter tolerance for ROE within the linearization regime.
///
/// # References
///
/// - D'Amico Eq. 2.17 (T matrix, pseudo-inverse)
/// - D'Amico Eq. 2.22 (e/i separation)
#[must_use = "enrichment result should be inspected"]
pub fn enrich_waypoint(
    position_ric_km: &Vector3<f64>,
    velocity_ric_km_s: Option<&Vector3<f64>>,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<EnrichedWaypoint, FormationDesignError> {
    match velocity_ric_km_s {
        None => enrich_position_only(position_ric_km, chief_mean, requirements),
        Some(vel) => {
            // TODO(wasm): activates when frontend sends velocity_ric_km_s: None
            // for position-only waypoints. Currently all waypoints carry velocity
            // via to_waypoints() filling None → [0,0,0].
            enrich_velocity_constrained(position_ric_km, vel, chief_mean, requirements)
        }
    }
}

/// Apply safety projection to a baseline ROE and build the enrichment result.
///
/// Shared by both position-only and velocity-constrained paths.
fn build_enriched_result(
    baseline: &QuasiNonsingularROE,
    position_ric_km: &Vector3<f64>,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
    mode: EnrichmentMode,
) -> Result<EnrichedWaypoint, FormationDesignError> {
    let baseline_ei = compute_ei_separation(baseline, chief_mean);
    let (enriched, resolved) = compute_safety_projection(baseline, chief_mean, requirements)?;
    let enriched_ei = compute_ei_separation(&enriched, chief_mean);
    let perturbation_norm = (enriched.to_vector() - baseline.to_vector()).norm();

    Ok(EnrichedWaypoint {
        roe: enriched,
        baseline_roe: *baseline,
        position_ric_km: *position_ric_km,
        enriched_ei,
        baseline_ei,
        perturbation_norm,
        mode,
        resolved_alignment: resolved,
    })
}

/// Position-only enrichment path (3 DOF free, null-space projection applied).
fn enrich_position_only(
    position_ric_km: &Vector3<f64>,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<EnrichedWaypoint, FormationDesignError> {
    // Minimum-norm ROE from position pseudo-inverse
    let baseline = ric_position_to_roe(position_ric_km, chief_mean).map_err(|e| match e {
        RicError::SingularPositionMatrix => FormationDesignError::SingularGeometry {
            mean_arg_lat_rad: chief_mean.mean_arg_of_lat(),
        },
        RicError::InvalidChiefElements(ce) => FormationDesignError::InvalidChiefElements(ce),
    })?;

    build_enriched_result(&baseline, position_ric_km, chief_mean, requirements, EnrichmentMode::PositionOnly)
}

/// Velocity-constrained enrichment path (0 DOF free, advisory only).
fn enrich_velocity_constrained(
    position_ric_km: &Vector3<f64>,
    velocity_ric_km_s: &Vector3<f64>,
    chief_mean: &KeplerianElements,
    requirements: &SafetyRequirements,
) -> Result<EnrichedWaypoint, FormationDesignError> {
    // Full 6×6 T-matrix inverse → unique ROE
    let t_full = compute_t_matrix(chief_mean)?;
    let t_inv = t_full.try_inverse().ok_or(FormationDesignError::SingularGeometry {
        mean_arg_lat_rad: chief_mean.mean_arg_of_lat(),
    })?;

    let ric_full = SVector::<f64, 6>::new(
        position_ric_km.x,
        position_ric_km.y,
        position_ric_km.z,
        velocity_ric_km_s.x,
        velocity_ric_km_s.y,
        velocity_ric_km_s.z,
    );
    let baseline = QuasiNonsingularROE::from_vector(&(t_inv * ric_full));

    build_enriched_result(&baseline, position_ric_km, chief_mean, requirements, EnrichmentMode::VelocityConstrained)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::roe_to_ric::{compute_t_position, ric_position_to_roe};
    use crate::test_helpers::{damico_table21_chief, iss_like_elements};
    use nalgebra::SMatrix;

    /// Position preservation tolerance (km). Sub-millimeter: the null-space
    /// guarantee means `T_pos` · perturbation = 0 exactly in exact arithmetic.
    /// 1e-10 km covers f64 matrix multiplication roundoff for 3×6 · 6×1.
    const POSITION_PRESERVATION_TOL: f64 = 1e-10;

    /// Tolerance for `T_pos` · n_j being zero. Covers 3×6 · 6×1 multiplication
    /// roundoff at O(1e-14).
    const NULL_SPACE_ORTHOGONALITY_TOL: f64 = 1e-12;

    /// Phase angle tolerance for e/i parallelism tests (rad).
    /// The algorithm directly aligns via atan2 — error is f64 trig precision only.
    const EI_PHASE_TOL: f64 = 1e-6;

    /// Tolerance for enrichment separation threshold tests (km).
    /// Formula is exact; only f64 rounding contributes error.
    const ENRICHMENT_SEPARATION_TOL: f64 = 1e-6;

    /// Tolerance for dimensionless ROE component checks (dex, dey, dix, diy).
    /// At u=0 the null-space shift direction is [1,0], so off-axis components
    /// should be exactly zero — 1e-12 covers f64 arithmetic noise from
    /// sin/cos and quadratic solver roundoff.
    const ROE_COMPONENT_TOL: f64 = 1e-12;

    /// Tolerance for Auto alignment perturbation-norm comparison.
    /// Auto selects whichever alignment produces smaller perturbation — the
    /// comparison is between two f64 norms differing by O(1e-4), so 1e-14
    /// covers any rounding in the subtraction.
    const AUTO_SELECTION_TOL: f64 = 1e-14;

    // -----------------------------------------------------------------------
    // Null-space basis
    // -----------------------------------------------------------------------

    /// Compute the analytical null-space basis of `T_pos` (D'Amico Eq. 2.17).
    ///
    /// Returns a 6×3 matrix whose columns span `ker(T_pos)`. The basis is derived
    /// analytically from the `T_pos` structure, not via SVD.
    ///
    /// Column semantics:
    /// - n₁ = \[cos u, −2 sin u, 1, 0, 0, 0\] — shifts δex
    /// - n₂ = \[sin u,  2 cos u, 0, 1, 0, 0\] — shifts δey
    /// - n₃ = \[0, 0, 0, 0, cos u, sin u\]     — shifts δi along \[cos u, sin u\]
    ///
    /// where `u = aop_rad + mean_anomaly_rad` (mean argument of latitude).
    ///
    /// # References
    ///
    /// - D'Amico Eq. 2.17 (`T_pos` matrix structure — derivation source)
    fn compute_null_space_basis(chief_mean: &KeplerianElements) -> SMatrix<f64, 6, 3> {
        let u = chief_mean.mean_arg_of_lat();
        let (sin_u, cos_u) = u.sin_cos();

        let n1 = SVector::<f64, 6>::new(cos_u, -2.0 * sin_u, 1.0, 0.0, 0.0, 0.0);
        let n2 = SVector::<f64, 6>::new(sin_u, 2.0 * cos_u, 0.0, 1.0, 0.0, 0.0);
        let n3 = SVector::<f64, 6>::new(0.0, 0.0, 0.0, 0.0, cos_u, sin_u);

        SMatrix::<f64, 6, 3>::from_columns(&[n1, n2, n3])
    }

    // -----------------------------------------------------------------------
    // Null-space basis tests
    // -----------------------------------------------------------------------

    /// Null-space basis vectors must satisfy `T_pos` · nⱼ ≈ 0.
    ///
    /// Tests at three chief elements: ISS-like, D'Amico Table 2.1 (u=0),
    /// and D'Amico Table 2.1 at u=π/2.
    ///
    /// # References
    /// - D'Amico Eq. 2.17
    #[test]
    fn null_space_orthogonal_to_t_pos() {
        let chiefs = [
            iss_like_elements(),
            damico_table21_chief(),
            {
                let mut c = damico_table21_chief();
                c.mean_anomaly_rad = std::f64::consts::FRAC_PI_2;
                c
            },
        ];

        for (idx, chief) in chiefs.iter().enumerate() {
            let t_pos = compute_t_position(chief).unwrap();
            let basis = compute_null_space_basis(chief);

            for j in 0..3 {
                let col = basis.column(j);
                let product = t_pos * col;
                let norm = product.norm();
                assert!(
                    norm < NULL_SPACE_ORTHOGONALITY_TOL,
                    "chief[{idx}], basis col {j}: T_pos · n_{j} norm = {norm}, \
                     exceeds {NULL_SPACE_ORTHOGONALITY_TOL}"
                );
            }
        }
    }

    /// Any null-space perturbation added to baseline ROE must not change
    /// the RIC position: `T_pos` · (baseline + α·nⱼ) = `T_pos` · baseline.
    ///
    /// Uses `iss_like_elements()`, computes baseline via `ric_position_to_roe()`
    /// for position \[0.1, 5.0, 0.05\] km, adds each basis vector scaled by 1e-4.
    ///
    /// # References
    /// - D'Amico Eq. 2.17 (`T_pos` matrix)
    #[test]
    fn null_space_preserves_ric_position() {
        let target_pos = Vector3::new(0.1, 5.0, 0.05);

        let chiefs = [
            iss_like_elements(),
            damico_table21_chief(),
            {
                let mut c = damico_table21_chief();
                c.mean_anomaly_rad = std::f64::consts::PI;
                c
            },
        ];

        for (idx, chief) in chiefs.iter().enumerate() {
            let baseline = ric_position_to_roe(&target_pos, chief).unwrap();
            let t_pos = compute_t_position(chief).unwrap();
            let baseline_pos = t_pos * baseline.to_vector();
            let basis = compute_null_space_basis(chief);

            for j in 0..3 {
                let col = basis.column(j);
                let perturbed_roe = baseline.to_vector() + col * 1e-4;
                let perturbed_pos = t_pos * perturbed_roe;
                let diff = (perturbed_pos - baseline_pos).norm();
                assert!(
                    diff < POSITION_PRESERVATION_TOL,
                    "chief[{idx}], basis col {j}: position shift = {diff} km, \
                     exceeds {POSITION_PRESERVATION_TOL}"
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // Safety projection tests
    // -----------------------------------------------------------------------

    /// After enrichment with `Parallel` alignment, the e/i phase angle
    /// |φ − θ| must be near zero.
    ///
    /// Uses `damico_table21_chief()`, enriches a V-bar-like position \[0.0, 2.0, 0.0\] km
    /// with `min_separation_km = 0.15`.
    ///
    /// # References
    /// - D'Amico Eqs. 2.3–2.4 (e/i polar form)
    /// - D'Amico Eq. 2.22 (safety metric)
    #[test]
    fn enrichment_produces_parallel_ei_vectors() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.0, 2.0, 0.0);
        let baseline = ric_position_to_roe(&pos, &chief).unwrap();
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let (enriched, resolved) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();
        assert!(matches!(resolved, EiAlignment::Parallel));

        let ei = compute_ei_separation(&enriched, &chief);
        assert!(
            ei.phase_angle_rad.abs() < EI_PHASE_TOL,
            "Phase angle = {} rad, expected near 0 (parallel)",
            ei.phase_angle_rad
        );
    }

    /// After enrichment, `a·min(|δe|, |δi|)` must be ≥ `min_separation_km`.
    ///
    /// Uses `damico_table21_chief()`, enriches position \[0.1, 5.0, 0.05\] km
    /// with `min_separation_km = 0.15` (150m, TanDEM-X baseline).
    ///
    /// # References
    /// - D'Amico Eq. 2.23 (parallel e/i simplification)
    #[test]
    fn enrichment_meets_separation_threshold() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.1, 5.0, 0.05);
        let baseline = ric_position_to_roe(&pos, &chief).unwrap();
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let (enriched, _) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();
        let ei = compute_ei_separation(&enriched, &chief);

        assert!(
            ei.min_separation_km >= reqs.min_separation_km - ENRICHMENT_SEPARATION_TOL,
            "Enriched separation = {} km, required ≥ {} km",
            ei.min_separation_km,
            reqs.min_separation_km
        );
    }

    /// When baseline has zero e/i (typical for V-bar/R-bar perch), the enrichment
    /// must produce nonzero e/i vectors.
    ///
    /// Uses `damico_table21_chief()` (u = 0 → shift direction = \[1, 0\]).
    /// Verifies: δe ∥ δi, both magnitudes ≈ `min_separation_km / a`.
    #[test]
    fn enrichment_handles_zero_baseline_ei() {
        let chief = damico_table21_chief(); // u = 0
        let d_min = 0.15;
        let d_min_dimless = d_min / chief.a_km;

        // Create baseline with zero e/i (V-bar perch: only dlambda)
        let baseline = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 1.0 / chief.a_km,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let reqs = SafetyRequirements {
            min_separation_km: d_min,
            alignment: EiAlignment::Parallel,
        };

        let (enriched, _) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();

        // Both vectors should have magnitude ≈ d_min / a
        let de = enriched.de_magnitude();
        let di = enriched.di_magnitude();
        assert!(
            (de - d_min_dimless).abs() < ROE_COMPONENT_TOL,
            "δe magnitude = {de}, expected ≈ {d_min_dimless}"
        );
        assert!(
            (di - d_min_dimless).abs() < ROE_COMPONENT_TOL,
            "δi magnitude = {di}, expected ≈ {d_min_dimless}"
        );

        // At u=0, shift direction is [1, 0], so dix ≈ d_min/a, diy ≈ 0
        assert!(
            (enriched.dix - d_min_dimless).abs() < ROE_COMPONENT_TOL,
            "dix = {}, expected ≈ {d_min_dimless}",
            enriched.dix
        );
        assert!(
            enriched.diy.abs() < ROE_COMPONENT_TOL,
            "diy = {}, expected ≈ 0",
            enriched.diy
        );

        // Phase angle should be near zero (parallel)
        let ei = compute_ei_separation(&enriched, &chief);
        assert!(
            ei.phase_angle_rad.abs() < EI_PHASE_TOL,
            "Phase angle = {} rad, expected near 0",
            ei.phase_angle_rad
        );
    }

    // -----------------------------------------------------------------------
    // enrich_waypoint() API tests
    // -----------------------------------------------------------------------

    /// When velocity is explicitly specified (0 free DOF), enrichment cannot
    /// inject safety without changing the operator's intent. Sets mode =
    /// `VelocityConstrained` and reports what the safe ROE would be.
    #[test]
    fn velocity_constrained_sets_mode() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.1, 2.0, 0.05);
        let vel = Vector3::new(0.0, 0.0, 0.0);
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_waypoint(&pos, Some(&vel), &chief, &reqs).unwrap();
        assert!(
            matches!(result.mode, EnrichmentMode::VelocityConstrained),
            "Expected VelocityConstrained mode, got {:?}",
            result.mode
        );
    }

    /// Requesting `min_separation_km = 100` km on a 7078 km orbit exceeds
    /// the linearization bound. Returns `SeparationUnachievable`.
    #[test]
    fn enrichment_returns_error_for_unreachable_separation() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.1, 2.0, 0.05);
        let reqs = SafetyRequirements {
            min_separation_km: 100.0, // ~1.4% of SMA → exceeds 0.01 bound
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_waypoint(&pos, None, &chief, &reqs);
        assert!(
            matches!(
                result,
                Err(FormationDesignError::SeparationUnachievable { .. })
            ),
            "Expected SeparationUnachievable, got {result:?}"
        );
    }

    /// Position-only enrichment preserves RIC position and reports `PositionOnly` mode.
    #[test]
    fn position_only_enrichment_preserves_position() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.1, 3.0, 0.1);
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_waypoint(&pos, None, &chief, &reqs).unwrap();
        assert!(matches!(result.mode, EnrichmentMode::PositionOnly));

        // Verify position preservation via T_pos
        let t_pos = compute_t_position(&chief).unwrap();
        let enriched_pos = t_pos * result.roe.to_vector();
        let diff = (Vector3::new(enriched_pos[0], enriched_pos[1], enriched_pos[2]) - pos).norm();
        assert!(
            diff < POSITION_PRESERVATION_TOL,
            "Position shift = {diff} km, exceeds {POSITION_PRESERVATION_TOL}"
        );
    }

    // -----------------------------------------------------------------------
    // Edge-case and coverage tests
    // -----------------------------------------------------------------------

    /// After enrichment with `AntiParallel` alignment, the e/i phase angle
    /// |φ − θ| must be near π (anti-parallel).
    #[test]
    fn enrichment_antiparallel_produces_pi_phase() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.0, 2.0, 0.0);
        let baseline = ric_position_to_roe(&pos, &chief).unwrap();
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::AntiParallel,
        };

        let (enriched, resolved) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();
        assert!(matches!(resolved, EiAlignment::AntiParallel));

        let ei = compute_ei_separation(&enriched, &chief);
        // Phase angle should be near ±π
        assert!(
            (ei.phase_angle_rad.abs() - std::f64::consts::PI).abs() < EI_PHASE_TOL,
            "Phase angle = {} rad, expected near ±π",
            ei.phase_angle_rad
        );
    }

    /// For `Auto` alignment, the selected alignment must produce a smaller
    /// (or equal) `perturbation_norm` than the other option.
    #[test]
    fn enrichment_auto_selects_minimum_perturbation() {
        let chief = damico_table21_chief();
        let pos = Vector3::new(0.1, 3.0, 0.1);

        let reqs_auto = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Auto,
        };
        let reqs_parallel = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };
        let reqs_anti = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::AntiParallel,
        };

        let result_auto = enrich_waypoint(&pos, None, &chief, &reqs_auto).unwrap();
        let result_par = enrich_waypoint(&pos, None, &chief, &reqs_parallel).unwrap();
        let result_anti = enrich_waypoint(&pos, None, &chief, &reqs_anti).unwrap();

        let min_explicit = result_par.perturbation_norm.min(result_anti.perturbation_norm);
        assert!(
            result_auto.perturbation_norm <= min_explicit + AUTO_SELECTION_TOL,
            "Auto perturbation = {}, min explicit = {}",
            result_auto.perturbation_norm,
            min_explicit
        );
    }

    /// Enrichment at u = 0, u = π/2, u = π, u = 3π/2 must all preserve
    /// position and produce parallel e/i.
    #[test]
    fn enrichment_at_different_mean_arg_lat() {
        let u_values = [
            0.0,
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
            3.0 * std::f64::consts::FRAC_PI_2,
        ];
        let pos = Vector3::new(0.1, 3.0, 0.1);
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        for &u in &u_values {
            let mut chief = damico_table21_chief();
            chief.mean_anomaly_rad = u; // aop=0, so u = mean_anomaly_rad

            let result = enrich_waypoint(&pos, None, &chief, &reqs).unwrap();

            // Position preservation
            let t_pos = compute_t_position(&chief).unwrap();
            let enriched_pos = t_pos * result.roe.to_vector();
            let diff =
                (Vector3::new(enriched_pos[0], enriched_pos[1], enriched_pos[2]) - pos).norm();
            assert!(
                diff < POSITION_PRESERVATION_TOL,
                "u = {u:.2}: position shift = {diff} km"
            );

            // Parallel e/i
            assert!(
                result.enriched_ei.phase_angle_rad.abs() < EI_PHASE_TOL,
                "u = {u:.2}: phase angle = {} rad",
                result.enriched_ei.phase_angle_rad
            );

            // Separation met
            assert!(
                result.enriched_ei.min_separation_km
                    >= reqs.min_separation_km - ENRICHMENT_SEPARATION_TOL,
                "u = {u:.2}: separation = {} km",
                result.enriched_ei.min_separation_km
            );
        }
    }

    /// Enriching an already-safe state (parallel e/i with magnitude ≥ threshold)
    /// must produce `perturbation_norm` near zero — the algorithm recognizes
    /// the baseline is already safe and makes minimal changes.
    #[test]
    fn enrichment_idempotent_for_safe_baseline() {
        let chief = damico_table21_chief();
        // Baseline already has parallel e/i well above threshold
        let baseline = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.3 / chief.a_km, // 300m — well above 100m threshold
            dix: 0.0,
            diy: 0.3 / chief.a_km,
        };
        let reqs = SafetyRequirements {
            min_separation_km: 0.1, // 100m — below existing 300m
            alignment: EiAlignment::Parallel,
        };

        let (enriched, _) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();
        let perturbation = (enriched.to_vector() - baseline.to_vector()).norm();
        assert!(
            perturbation < LINEARIZATION_PERTURBATION_BOUND / 100.0,
            "Perturbation = {perturbation}, expected near-zero for already-safe state"
        );
    }

    /// Enrichment with a moderately eccentric chief (e = 0.05, within validity regime)
    /// must still preserve position and produce parallel e/i.
    #[test]
    fn enrichment_moderately_eccentric_chief() {
        let mut chief = damico_table21_chief();
        chief.e = 0.05; // At validity regime boundary
        let pos = Vector3::new(0.1, 3.0, 0.1);
        let reqs = SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        };

        let result = enrich_waypoint(&pos, None, &chief, &reqs).unwrap();

        // Position preservation
        let t_pos = compute_t_position(&chief).unwrap();
        let enriched_pos = t_pos * result.roe.to_vector();
        let diff = (Vector3::new(enriched_pos[0], enriched_pos[1], enriched_pos[2]) - pos).norm();
        assert!(
            diff < POSITION_PRESERVATION_TOL,
            "Position shift = {diff} km (e=0.05 chief)"
        );

        // Parallel e/i
        assert!(
            result.enriched_ei.phase_angle_rad.abs() < EI_PHASE_TOL,
            "Phase angle = {} rad (e=0.05 chief)",
            result.enriched_ei.phase_angle_rad
        );
    }

    /// R1: Cross-track geometry test at u + π/2.
    ///
    /// After enrichment at u₀, the cross-track motion `C(u) = a·(dix·sin(u) - diy·cos(u))`
    /// reaches maximum `a·d_min` at `u₀ + π/2`. The null-space guarantees position
    /// preservation at the enrichment epoch, but the purpose of enrichment is
    /// free-drift safety at *other* epochs.
    ///
    /// # References
    /// - D'Amico Eq. 2.17 (cross-track row: `C = a·(δix·sin u − δiy·cos u)`)
    #[test]
    fn cross_track_geometry_at_u_plus_pi_half() {
        let chief = damico_table21_chief(); // u₀ = 0
        let sma = chief.a_km;
        let d_min = 0.15;

        // Create a zero-e/i baseline (V-bar perch)
        let baseline = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 1.0 / sma,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let reqs = SafetyRequirements {
            min_separation_km: d_min,
            alignment: EiAlignment::Parallel,
        };

        let (enriched, _) = compute_safety_projection(&baseline, &chief, &reqs).unwrap();

        // At u₀ = 0: C = a·(dix·sin(0) - diy·cos(0)) = -a·diy ≈ 0
        // (since at u=0 the i-vector is along [1,0] → diy ≈ 0)
        let c_at_u0 = sma * (enriched.dix * 0.0_f64.sin() - enriched.diy * 0.0_f64.cos());
        assert!(
            c_at_u0.abs() < ENRICHMENT_SEPARATION_TOL,
            "C at u₀ = {c_at_u0} km, expected ≈ 0"
        );

        // At u₀ + π/2: C = a·(dix·sin(π/2) - diy·cos(π/2)) = a·dix ≈ a·(d_min/a) = d_min
        let u_half = std::f64::consts::FRAC_PI_2;
        let c_at_u_plus_90 =
            sma * (enriched.dix * u_half.sin() - enriched.diy * u_half.cos());
        assert!(
            (c_at_u_plus_90.abs() - d_min).abs() < ENRICHMENT_SEPARATION_TOL,
            "C at u₀+π/2 = {} km, expected ≈ {d_min} km",
            c_at_u_plus_90.abs()
        );
    }
}

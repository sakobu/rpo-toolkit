//! Covariance propagation using STM-based linear mapping.
//!
//! Implements P₁ = Φ P₀ Φᵀ using J2/drag STMs, RIC↔ROE covariance
//! conversion via the T matrix, maneuver covariance updates via the
//! B matrix, and collision probability from Mahalanobis distance.
//!
//! Mission-level orchestration (`propagate_mission_covariance`) threads
//! these kernels across a multi-leg waypoint mission.

use std::fmt;

use hifitime::Duration;
use nalgebra::{Matrix3, SMatrix, Vector3};

use crate::constants::{COLLISION_PROBABILITY_FLOOR, DV_NORM_ZERO_THRESHOLD_KM_S};
use crate::elements::conversions::ConversionError;
use crate::elements::gve::compute_b_matrix;
use crate::elements::ric::{compute_t_matrix, compute_t_position, roe_to_ric};
use crate::propagation::drag_stm::{compute_j2_drag_stm, compute_j2_drag_stm_with_params};
use crate::propagation::j2_params::{compute_j2_params, J2Params};
use crate::propagation::propagator::{PropagationError, PropagationModel};
use crate::propagation::stm::{compute_stm, compute_stm_with_params, propagate_chief_mean};
use crate::types::{
    CovarianceState, DragConfig, KeplerianElements, LegCovarianceReport, ManeuverUncertainty,
    Matrix6, Matrix9, MissionCovarianceReport, NavigationAccuracy, QuasiNonsingularROE,
    WaypointMission,
};

// ── Error type ──────────────────────────────────────────────────────────────

/// Errors from covariance propagation.
#[derive(Debug, Clone)]
pub enum CovarianceError {
    /// T matrix is singular (cannot invert for ROE↔RIC conversion).
    SingularTMatrix,
    /// Underlying STM computation failed.
    StmFailure(PropagationError),
    /// Chief element validation failed (delegates to `ConversionError`).
    InvalidChiefElements(ConversionError),
    /// `n_steps` exceeds `u32::MAX` (would cause enormous loop / OOM).
    TooManySteps {
        /// The requested step count that exceeded `u32::MAX`.
        n_steps: usize,
    },
}

impl fmt::Display for CovarianceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SingularTMatrix => write!(f, "CovarianceError: T matrix is singular"),
            Self::StmFailure(e) => write!(f, "CovarianceError: STM computation failed — {e}"),
            Self::InvalidChiefElements(e) => {
                write!(f, "CovarianceError: invalid chief elements — {e}")
            }
            Self::TooManySteps { n_steps } => {
                write!(f, "CovarianceError: n_steps = {n_steps} exceeds u32::MAX")
            }
        }
    }
}

impl std::error::Error for CovarianceError {}

impl From<PropagationError> for CovarianceError {
    fn from(e: PropagationError) -> Self {
        Self::StmFailure(e)
    }
}

impl From<ConversionError> for CovarianceError {
    fn from(e: ConversionError) -> Self {
        Self::InvalidChiefElements(e)
    }
}

// ── Core functions ──────────────────────────────────────────────────────────

/// Convert diagonal RIC navigation accuracy to ROE-space covariance.
///
/// Constructs a diagonal 6×6 RIC covariance from per-axis sigmas,
/// then maps to ROE space: `P_roe` = T⁻¹ × `P_ric` × (T⁻¹)ᵀ.
///
/// The T matrix is the 6×6 ROE-to-RIC transformation from D'Amico Eq. 2.17
/// (computed by `compute_t_matrix()` in `elements/ric.rs`).
///
/// # Invariants
/// - `chief.a_km > 0` and `0 <= chief.e < 1`
/// - `chief` must be **mean** Keplerian elements, not osculating
/// - Navigation sigma values must be positive (zero sigma produces singular covariance)
///
/// # Arguments
/// * `nav` - Navigation accuracy in RIC frame (1-sigma per axis)
/// * `chief` - Chief mean Keplerian elements (for T matrix computation)
///
/// # Errors
/// Returns `CovarianceError::SingularTMatrix` if T is not invertible.
/// Returns `CovarianceError::InvalidChiefElements` if chief elements are invalid.
pub fn ric_accuracy_to_roe_covariance(
    nav: &NavigationAccuracy,
    chief: &KeplerianElements,
) -> Result<Matrix6, CovarianceError> {
    // Build diagonal 6×6 P_ric from sigma² values
    let sigmas = [
        nav.position_sigma_ric_km[0],
        nav.position_sigma_ric_km[1],
        nav.position_sigma_ric_km[2],
        nav.velocity_sigma_ric_km_s[0],
        nav.velocity_sigma_ric_km_s[1],
        nav.velocity_sigma_ric_km_s[2],
    ];
    let p_ric = Matrix6::from_diagonal(&nalgebra::SVector::<f64, 6>::from_fn(|i, _| {
        sigmas[i] * sigmas[i]
    }));

    // T matrix: ROE → RIC
    let t = compute_t_matrix(chief)?;
    let t_inv = t.try_inverse().ok_or(CovarianceError::SingularTMatrix)?;

    // P_roe = T⁻¹ × P_ric × (T⁻¹)ᵀ
    let p_roe = t_inv * p_ric * t_inv.transpose();
    Ok(symmetrize6(&p_roe))
}

/// Propagate ROE covariance through the J2 STM.
///
/// Implements P(t) = Φ(t) × P(0) × Φ(t)ᵀ using the 6×6 J2 STM
/// from `compute_stm()` in `propagation/stm.rs`.
///
/// # Invariants
/// - `chief_mean.a_km > 0` and `0 <= chief_mean.e < 1`
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `covariance_roe` should be symmetric positive semi-definite (PSD)
/// - `tau` must be finite
///
/// # Arguments
/// * `covariance_roe` - Initial 6×6 ROE covariance (symmetric PSD)
/// * `chief_mean` - Chief mean Keplerian elements at initial epoch
/// * `tau` - Propagation time (seconds)
///
/// # Errors
/// Returns `CovarianceError::StmFailure` if STM computation fails.
pub fn propagate_covariance(
    covariance_roe: &Matrix6,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Result<Matrix6, CovarianceError> {
    let phi = compute_stm(chief_mean, tau)?;
    let p1 = phi * covariance_roe * phi.transpose();
    Ok(symmetrize6(&p1))
}

/// Propagate covariance with precomputed J2 parameters.
///
/// Uses `compute_stm_with_params()` which skips J2 parameter
/// validation — caller is responsible for providing valid `J2Params`.
/// Returns `Matrix6` directly (infallible once `J2Params` are validated).
///
/// # Invariants
/// - `j2p` must correspond to `chief_mean` (caller responsibility)
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `covariance_roe` should be symmetric PSD
/// - `tau` must be finite
#[must_use]
pub fn propagate_covariance_with_params(
    covariance_roe: &Matrix6,
    j2p: &J2Params,
    chief_mean: &KeplerianElements,
    tau: f64,
) -> Matrix6 {
    let phi = compute_stm_with_params(j2p, chief_mean, tau);
    let p1 = phi * covariance_roe * phi.transpose();
    symmetrize6(&p1)
}

/// Propagate ROE covariance through the J2+drag STM.
///
/// Uses the 9×9 augmented STM from `compute_j2_drag_stm()`.
/// The 6×6 input covariance is embedded in the upper-left block of a
/// 9×9 matrix (drag rate rows/cols are zero — drag rates are treated
/// as deterministic). Returns the 6×6 upper-left block of the result.
///
/// # Invariants
/// - `chief_mean.a_km > 0` and `0 <= chief_mean.e < 1`
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `covariance_roe` should be symmetric PSD
/// - `tau` must be finite
/// - Drag rates in `drag` are treated as deterministic (no uncertainty on drag state)
///
/// # Arguments
/// * `covariance_roe` - Initial 6×6 ROE covariance
/// * `chief_mean` - Chief mean Keplerian elements
/// * `drag` - Drag configuration (for STM computation)
/// * `tau` - Propagation time (seconds)
///
/// # Errors
/// Returns `CovarianceError::StmFailure` if STM computation fails.
pub fn propagate_covariance_with_drag(
    covariance_roe: &Matrix6,
    chief_mean: &KeplerianElements,
    drag: &DragConfig,
    tau: f64,
) -> Result<Matrix6, CovarianceError> {
    let phi9 = compute_j2_drag_stm(chief_mean, drag, tau)?;

    // Embed 6×6 covariance into upper-left block of 9×9
    let mut p9 = Matrix9::zeros();
    for r in 0..6 {
        for c in 0..6 {
            p9[(r, c)] = covariance_roe[(r, c)];
        }
    }

    // P₉_out = Φ₉ × P₉ × Φ₉ᵀ
    let p9_out = phi9 * p9 * phi9.transpose();

    // Extract upper-left 6×6
    let p6_out: Matrix6 = p9_out.fixed_view::<6, 6>(0, 0).into();
    Ok(symmetrize6(&p6_out))
}

/// Project ROE covariance to 3×3 RIC position covariance.
///
/// `P_ric_pos` = `T_pos` × `P_roe` × `T_pos`ᵀ
/// where `T_pos` is the 3×6 position submatrix of the T matrix
/// (computed by `compute_t_position()` in `elements/ric.rs`).
///
/// # Invariants
/// - `chief.a_km > 0` and `0 <= chief.e < 1`
/// - `chief` must be **mean** Keplerian elements, not osculating
/// - `covariance_roe` should be symmetric PSD
///
/// # Arguments
/// * `covariance_roe` - 6×6 ROE covariance
/// * `chief` - Chief mean Keplerian elements (for T matrix)
///
/// # Errors
/// Returns `CovarianceError::InvalidChiefElements` if T matrix computation fails.
pub fn roe_covariance_to_ric_position(
    covariance_roe: &Matrix6,
    chief: &KeplerianElements,
) -> Result<SMatrix<f64, 3, 3>, CovarianceError> {
    let t_pos = compute_t_position(chief)?;
    let p_ric_pos = t_pos * covariance_roe * t_pos.transpose();
    Ok(symmetrize3(p_ric_pos))
}

/// Update covariance at a maneuver: add execution error covariance.
///
/// At each burn, the covariance receives an additive contribution from
/// maneuver execution uncertainty:
///
///   `P_post` = `P_pre` + B × `Q_dv` × Bᵀ
///
/// where B is the 6×3 GVE B matrix (maps RIC Δv to ROE change, from
/// `compute_b_matrix()` in `elements/gve.rs`) and `Q_dv` is the 3×3
/// Δv execution covariance in RIC frame.
///
/// `Q_dv` is constructed from the maneuver uncertainty model:
/// - Magnitude error: proportional to |Δv|, applied along the Δv direction
/// - Pointing error: cross-axis uncertainty perpendicular to Δv direction
///
/// # Invariants
/// - `chief.a_km > 0` and `0 <= chief.e < 1`
/// - `chief` must be **mean** Keplerian elements, not osculating
/// - `uncertainty.magnitude_sigma >= 0` and `uncertainty.pointing_sigma_rad >= 0`
/// - `covariance_roe` should be symmetric PSD
///
/// # Arguments
/// * `covariance_roe` - Pre-maneuver 6×6 ROE covariance
/// * `uncertainty` - Maneuver execution uncertainty model
/// * `nominal_dv_ric_km_s` - Nominal Δv vector in RIC frame
/// * `chief` - Chief mean Keplerian elements (for B matrix)
///
/// # Errors
/// Returns `CovarianceError::InvalidChiefElements` if B matrix computation fails.
pub fn update_covariance_at_maneuver(
    covariance_roe: &Matrix6,
    uncertainty: &ManeuverUncertainty,
    nominal_dv_ric_km_s: &Vector3<f64>,
    chief: &KeplerianElements,
) -> Result<Matrix6, CovarianceError> {
    let dv_norm = nominal_dv_ric_km_s.norm();
    if dv_norm < DV_NORM_ZERO_THRESHOLD_KM_S {
        return Ok(*covariance_roe);
    }

    // Unit direction of nominal Δv
    let d = nominal_dv_ric_km_s / dv_norm;

    // Direction-dependent 3×3 Δv execution covariance
    let sigma_mag = uncertainty.magnitude_sigma * dv_norm;
    let sigma_point = uncertainty.pointing_sigma_rad * dv_norm;
    let d_dt = d * d.transpose(); // outer product: d × dᵀ
    let q_dv =
        sigma_mag * sigma_mag * d_dt + sigma_point * sigma_point * (Matrix3::identity() - d_dt);

    // B matrix: Δv → ΔROE
    let b = compute_b_matrix(chief)?;

    // P_post = P_pre + B × Q_dv × Bᵀ
    let p_post = covariance_roe + b * q_dv * b.transpose();
    Ok(symmetrize6(&p_post))
}

/// Compute Mahalanobis distance and approximate collision probability.
///
/// Mahalanobis: d = sqrt(xᵀ × P⁻¹ × x) where x is the nominal RIC
/// position and P is the 3×3 RIC position covariance.
///
/// Collision probability approximation: Pc ≈ erfc(d / √2) using the
/// complementary error function. This is conservative (assumes the
/// most pessimistic axis dominates).
///
/// If the covariance is singular (degenerate axis), returns
/// `(f64::INFINITY, 0.0)`.
///
/// # Invariants
/// - `covariance_ric_pos` should be symmetric PSD (singular case handled gracefully)
/// - Collision probability approximation assumes Gaussian distribution
///
/// # Arguments
/// * `ric_position_km` - Nominal deputy position in RIC frame (km)
/// * `covariance_ric_pos` - 3×3 RIC position covariance (km²)
///
/// # Returns
/// Tuple of (Mahalanobis distance, collision probability)
#[must_use]
pub fn compute_collision_metrics(
    ric_position_km: &Vector3<f64>,
    covariance_ric_pos: &SMatrix<f64, 3, 3>,
) -> (f64, f64) {
    let Some(p_inv) = covariance_ric_pos.try_inverse() else {
        return (f64::INFINITY, 0.0);
    };

    // Mahalanobis distance: d = sqrt(xᵀ P⁻¹ x)
    let x = ric_position_km;
    let d_sq = (x.transpose() * p_inv * x)[(0, 0)];
    let d = d_sq.sqrt();

    // Pc ≈ erfc(d / √2)
    let pc = libm::erfc(d / std::f64::consts::SQRT_2);
    let pc = pc.clamp(COLLISION_PROBABILITY_FLOOR, 1.0);

    (d, pc)
}

// ── Mission-level orchestration ─────────────────────────────────────────────

/// Propagate covariance across a complete waypoint mission.
///
/// For each leg in the mission:
/// 1. Update covariance at departure maneuver (add execution error via B matrix)
/// 2. Propagate covariance through the coast arc at `n_steps` sample points
/// 3. At each sample: project to RIC, compute 3σ bounds and Pc
/// 4. Update covariance at arrival maneuver
/// 5. Post-arrival covariance becomes next leg's initial covariance
///
/// The function takes a completed `WaypointMission` as input and produces
/// covariance overlays — it does not modify the mission plan.
///
/// # Invariants
/// - `mission` must be a valid completed waypoint mission (from `plan_waypoint_mission`)
/// - `initial_covariance` must be symmetric PSD (6×6 ROE-space covariance)
/// - Chief elements on each leg must be **mean** Keplerian, not osculating
/// - `n_steps` of 0 produces a single sample at departure per leg; typical values 50–200
/// - The `propagator` should match the one used to plan the mission
///
/// # Arguments
/// * `mission` - Planned waypoint mission (provides leg TOFs, Δv, chief elements)
/// * `initial_covariance` - Initial ROE covariance (from `ric_accuracy_to_roe_covariance`)
/// * `navigation_accuracy` - Navigation accuracy used to derive `initial_covariance` (recorded in report)
/// * `maneuver_uncertainty` - Optional maneuver execution uncertainty
/// * `propagator` - Propagation model (J2 or J2+Drag, determines which STM to use)
/// * `n_steps` - Number of covariance sample points per leg
///
/// # Errors
/// Returns `CovarianceError` if propagation or projection fails.
pub fn propagate_mission_covariance(
    mission: &WaypointMission,
    initial_covariance: &Matrix6,
    navigation_accuracy: &NavigationAccuracy,
    maneuver_uncertainty: Option<&ManeuverUncertainty>,
    propagator: &PropagationModel,
    n_steps: usize,
) -> Result<MissionCovarianceReport, CovarianceError> {
    let mut current_p = *initial_covariance;
    let mut leg_reports = Vec::with_capacity(mission.legs.len());
    let mut overall_max_sigma3 = 0.0_f64;
    let mut overall_max_pc = 0.0_f64;

    for leg in &mission.legs {
        // 1. Departure maneuver update
        if let Some(unc) = maneuver_uncertainty {
            current_p = update_covariance_at_maneuver(
                &current_p,
                unc,
                &leg.departure_maneuver.dv_ric_km_s,
                &leg.departure_chief_mean,
            )?;
        }
        let p_post_dep = current_p;

        // 2. Coast arc setup
        let j2p = compute_j2_params(&leg.departure_chief_mean)?;
        let roe_vec_0 = leg.post_departure_roe.to_vector();
        // Convert to u32 for lossless f64 conversion (n_steps is typically 50–200).
        // Rejects absurd inputs rather than silently saturating.
        let n = u32::try_from(n_steps.max(1))
            .map_err(|_| CovarianceError::TooManySteps { n_steps })?;
        let n_div = f64::from(n);

        // 3. Sample loop — compute STM once per sample, reuse for both
        //    covariance propagation and nominal ROE evolution.
        let mut states = Vec::with_capacity(n as usize + 1);
        let mut leg_max_sigma3 = 0.0_f64;
        let mut leg_max_pc = 0.0_f64;
        let mut p_last = p_post_dep;

        for j in 0..=n {
            let tau_j = leg.tof_s * f64::from(j) / n_div;

            // Compute STM once, reuse for covariance and nominal ROE
            let (p_j, roe_vec_j) = propagate_sample(
                &p_post_dep, &roe_vec_0, &j2p, &leg.departure_chief_mean, propagator, tau_j,
            );
            let roe_j = QuasiNonsingularROE::from_vector(&roe_vec_j);

            // Advance chief mean elements
            let chief_j = propagate_chief_mean(&leg.departure_chief_mean, &j2p, tau_j);

            // Convert to RIC position
            let ric_j = roe_to_ric(&roe_j, &chief_j)?;
            let ric_position = ric_j.position_ric_km;

            // Project to RIC position covariance
            let p_ric_pos = roe_covariance_to_ric_position(&p_j, &chief_j)?;

            // 3-sigma bounds
            let sigma3 = Vector3::new(
                3.0 * p_ric_pos[(0, 0)].sqrt(),
                3.0 * p_ric_pos[(1, 1)].sqrt(),
                3.0 * p_ric_pos[(2, 2)].sqrt(),
            );

            // Collision metrics
            let (mahal, pc) = compute_collision_metrics(&ric_position, &p_ric_pos);

            // Track per-leg maximums
            let max_sigma3_component = sigma3.x.max(sigma3.y).max(sigma3.z);
            leg_max_sigma3 = leg_max_sigma3.max(max_sigma3_component);
            leg_max_pc = leg_max_pc.max(pc);

            p_last = p_j;

            states.push(CovarianceState {
                epoch: leg.departure_maneuver.epoch + Duration::from_seconds(tau_j),
                elapsed_s: tau_j,
                covariance_roe: p_j,
                covariance_ric_position_km2: p_ric_pos,
                sigma3_position_ric_km: sigma3,
                mahalanobis_distance: mahal,
                collision_probability: pc,
            });
        }

        // 4. Arrival maneuver update — reuse end-of-coast covariance from
        //    last sample (j=n → tau=tof), avoiding redundant STM computation.
        let p_end_coast = p_last;

        current_p = if let Some(unc) = maneuver_uncertainty {
            update_covariance_at_maneuver(
                &p_end_coast,
                unc,
                &leg.arrival_maneuver.dv_ric_km_s,
                &leg.arrival_chief_mean,
            )?
        } else {
            p_end_coast
        };

        // 5. Build leg report
        overall_max_sigma3 = overall_max_sigma3.max(leg_max_sigma3);
        overall_max_pc = overall_max_pc.max(leg_max_pc);

        leg_reports.push(LegCovarianceReport {
            states,
            max_sigma3_position_km: leg_max_sigma3,
            max_collision_probability: leg_max_pc,
        });
    }

    Ok(MissionCovarianceReport {
        legs: leg_reports,
        navigation_accuracy: *navigation_accuracy,
        maneuver_uncertainty: maneuver_uncertainty.copied(),
        max_sigma3_position_km: overall_max_sigma3,
        max_collision_probability: overall_max_pc,
    })
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Propagate covariance and nominal ROE at a single sample point.
///
/// Evaluates the STM once (J2 6×6 or J2+drag 9×9) and reuses it for both
/// covariance propagation P(τ) = Φ P₀ Φᵀ and nominal ROE evolution
/// δα(τ) = Φ δα₀. This avoids computing the STM twice per sample.
///
/// For the J2+drag path, the 6×6 ROE covariance is embedded in the
/// upper-left block of a 9×9 zero matrix (drag-rate rows/cols are zero,
/// treating drag rates as deterministic). The nominal ROE is propagated
/// using the upper-left 6×6 block of the 9×9 STM, which is identical
/// to the J2-only STM (drag coupling is in the upper-right 6×3 block).
///
/// # Invariants
/// - `j2p` must correspond to `chief_mean` (caller responsibility)
/// - `chief_mean` must be **mean** Keplerian elements, not osculating
/// - `p_post_dep` should be symmetric PSD
/// - `tau` must be finite
///
/// # Arguments
/// * `p_post_dep` - Post-departure 6×6 ROE covariance (symmetric PSD)
/// * `roe_vec_0` - Initial ROE state vector (6×1)
/// * `j2p` - Pre-computed J2 perturbation parameters
/// * `chief_mean` - Chief mean Keplerian elements at departure epoch
/// * `propagator` - Propagation model (determines J2 vs J2+drag STM)
/// * `tau` - Propagation time from departure (seconds)
///
/// # Returns
/// Tuple of (propagated covariance, propagated ROE state vector).
fn propagate_sample(
    p_post_dep: &Matrix6,
    roe_vec_0: &nalgebra::SVector<f64, 6>,
    j2p: &J2Params,
    chief_mean: &KeplerianElements,
    propagator: &PropagationModel,
    tau: f64,
) -> (Matrix6, nalgebra::SVector<f64, 6>) {
    match propagator {
        PropagationModel::J2Stm => {
            let phi = compute_stm_with_params(j2p, chief_mean, tau);
            let p = symmetrize6(&(phi * p_post_dep * phi.transpose()));
            let roe = phi * roe_vec_0;
            (p, roe)
        }
        PropagationModel::J2DragStm { drag: _ } => {
            let phi9 = compute_j2_drag_stm_with_params(j2p, chief_mean, tau);
            // Embed 6×6 covariance in upper-left block of 9×9
            let mut p9 = Matrix9::zeros();
            for r in 0..6 {
                for c in 0..6 {
                    p9[(r, c)] = p_post_dep[(r, c)];
                }
            }
            let p9_out = phi9 * p9 * phi9.transpose();
            let p: Matrix6 = p9_out.fixed_view::<6, 6>(0, 0).into();
            // Nominal ROE via upper-left 6×6 block of the 9×9 STM
            let phi6: Matrix6 = phi9.fixed_view::<6, 6>(0, 0).into();
            let roe = phi6 * roe_vec_0;
            (symmetrize6(&p), roe)
        }
    }
}

/// Force symmetry on a 6×6 matrix: P = (P + Pᵀ) / 2.
fn symmetrize6(m: &Matrix6) -> Matrix6 {
    (m + m.transpose()) * 0.5
}

/// Force symmetry on a 3×3 matrix: P = (P + Pᵀ) / 2.
fn symmetrize3(m: SMatrix<f64, 3, 3>) -> SMatrix<f64, 3, 3> {
    (m + m.transpose()) * 0.5
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::COVARIANCE_SYMMETRY_TOL;
    use crate::test_helpers::iss_like_elements;

    // ── Test tolerances (documented per CLAUDE.md policy) ──────────────────

    /// PSD eigenvalue floor: eigenvalues below this are numerical noise,
    /// not a true PSD violation. 1e-10 is ~100× above machine epsilon
    /// for the O(1e-8) covariance diagonal values in these tests.
    const PSD_EIGENVALUE_FLOOR: f64 = -1e-10;

    /// Relative error tolerance for roundtrip and scaling tests.
    /// T and T⁻¹ are ~O(1e3) for ISS-like elements, so two T-matrix
    /// multiplications introduce ~O(1e-12) relative error. 1e-10 is
    /// conservative with margin.
    const ROUNDTRIP_RELATIVE_TOL: f64 = 1e-10;

    /// Monotonicity margin: trace differences below this are numerical
    /// noise from symmetrization. Set to ~machine-epsilon level.
    const MONOTONICITY_MARGIN: f64 = 1e-20;

    /// Leg boundary continuity tolerance: the end-of-coast covariance
    /// and start-of-next-leg covariance are computed via independent
    /// STM evaluations at the same tau, so they should match to
    /// machine precision. 1e-6 accounts for accumulated floating-point
    /// error across a full orbital period of propagation.
    const LEG_BOUNDARY_CONTINUITY_TOL: f64 = 1e-6;

    /// Serde roundtrip tolerance: f64 → JSON → f64 should be lossless
    /// within printing precision. 1e-14 is conservative.
    const SERDE_ROUNDTRIP_TOL: f64 = 1e-14;

    /// Build a simple diagonal ROE covariance for testing.
    fn test_covariance() -> Matrix6 {
        Matrix6::from_diagonal(&nalgebra::SVector::<f64, 6>::new(
            1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8,
        ))
    }

    /// ISS orbital period in seconds (≈ 92.5 min).
    fn iss_period_s() -> f64 {
        let chief = iss_like_elements();
        let n = (crate::constants::MU_EARTH / chief.a_km.powi(3)).sqrt();
        crate::constants::TWO_PI / n
    }

    #[test]
    fn zero_covariance_stays_zero() {
        // Phi * 0 * Phi^T = 0 exactly in floating-point arithmetic
        let chief = iss_like_elements();
        let p0 = Matrix6::zeros();
        let p1 = propagate_covariance(&p0, &chief, 3600.0).unwrap();
        assert!(
            p1.norm() < COVARIANCE_SYMMETRY_TOL,
            "P(zero) should remain zero after propagation: norm={}",
            p1.norm()
        );
    }

    #[test]
    fn identity_covariance_at_zero_tau() {
        let chief = iss_like_elements();
        let p0 = test_covariance();
        let p1 = propagate_covariance(&p0, &chief, 0.0).unwrap();
        let diff = (p1 - p0).norm();
        assert!(
            diff < COVARIANCE_SYMMETRY_TOL,
            "P(τ=0) differs from P(0) by {diff}"
        );
    }

    #[test]
    fn covariance_stays_symmetric() {
        let chief = iss_like_elements();
        let p0 = test_covariance();
        let p1 = propagate_covariance(&p0, &chief, iss_period_s()).unwrap();
        let asym = (p1 - p1.transpose()).norm();
        assert!(
            asym < COVARIANCE_SYMMETRY_TOL,
            "Asymmetry after propagation: {asym}"
        );
    }

    #[test]
    fn covariance_stays_positive_semidefinite() {
        let chief = iss_like_elements();
        let p0 = test_covariance();
        let p1 = propagate_covariance(&p0, &chief, iss_period_s()).unwrap();
        let eig = p1.symmetric_eigenvalues();
        let min_eig = eig.min();
        assert!(
            min_eig > PSD_EIGENVALUE_FLOOR,
            "Negative eigenvalue after propagation: {min_eig}"
        );
    }

    #[test]
    fn covariance_grows_with_time() {
        let chief = iss_like_elements();
        let p0 = test_covariance();
        let p1 = propagate_covariance(&p0, &chief, iss_period_s()).unwrap();
        assert!(
            p1.trace() > p0.trace(),
            "Covariance should grow: trace(P1)={} ≤ trace(P0)={}",
            p1.trace(),
            p0.trace()
        );
    }

    #[test]
    fn ric_projection_diagonal_dominance() {
        let chief = iss_like_elements();
        let nav = NavigationAccuracy::default();
        let p_roe = ric_accuracy_to_roe_covariance(&nav, &chief).unwrap();
        let p_ric_pos = roe_covariance_to_ric_position(&p_roe, &chief).unwrap();

        // Diagonal elements should dominate off-diagonal elements
        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        p_ric_pos[(i, i)].abs() > p_ric_pos[(i, j)].abs(),
                        "Diagonal not dominant: P[{i},{i}]={} vs P[{i},{j}]={}",
                        p_ric_pos[(i, i)],
                        p_ric_pos[(i, j)]
                    );
                }
            }
        }
    }

    #[test]
    fn roundtrip_ric_roe_ric() {
        let chief = iss_like_elements();
        let nav = NavigationAccuracy::default();

        // RIC → ROE
        let p_roe = ric_accuracy_to_roe_covariance(&nav, &chief).unwrap();

        // ROE → propagate(τ=0) → ROE (should be identity operation)
        let p_roe_prop = propagate_covariance(&p_roe, &chief, 0.0).unwrap();

        // ROE → RIC position
        let p_ric_pos = roe_covariance_to_ric_position(&p_roe_prop, &chief).unwrap();

        // Original RIC position variances (sigma²)
        let expected_var = nav.position_sigma_ric_km[0] * nav.position_sigma_ric_km[0];

        // Diagonal should recover original variances within tolerance
        for i in 0..3 {
            let rel_err = (p_ric_pos[(i, i)] - expected_var).abs() / expected_var;
            assert!(
                rel_err < ROUNDTRIP_RELATIVE_TOL,
                "RIC roundtrip axis {i}: got {}, expected {expected_var}, rel_err={rel_err}",
                p_ric_pos[(i, i)]
            );
        }
    }

    #[test]
    fn maneuver_update_increases_trace() {
        let chief = iss_like_elements();
        let p_pre = test_covariance();
        let uncertainty = ManeuverUncertainty::default();
        let dv = Vector3::new(1e-4, 2e-4, 0.0); // small maneuver

        let p_post =
            update_covariance_at_maneuver(&p_pre, &uncertainty, &dv, &chief).unwrap();

        assert!(
            p_post.trace() > p_pre.trace(),
            "Maneuver should increase trace: post={} ≤ pre={}",
            p_post.trace(),
            p_pre.trace()
        );
    }

    #[test]
    fn mahalanobis_zero_at_origin() {
        // sqrt(0^T * P^-1 * 0) = 0 exactly in floating-point
        let p = SMatrix::<f64, 3, 3>::identity();
        let x = Vector3::zeros();
        let (d, _pc) = compute_collision_metrics(&x, &p);
        assert!(
            d.abs() < COVARIANCE_SYMMETRY_TOL,
            "Mahalanobis at origin should be ~0: got {d}"
        );
    }

    #[test]
    fn mahalanobis_scales_with_sigma() {
        let pos = Vector3::new(1.0, 0.0, 0.0);

        // σ = 1
        let p1 = SMatrix::<f64, 3, 3>::identity();
        let (d1, _) = compute_collision_metrics(&pos, &p1);

        // σ = 2 → variance = 4
        let p2 = SMatrix::<f64, 3, 3>::identity() * 4.0;
        let (d2, _) = compute_collision_metrics(&pos, &p2);

        // 2× sigma → d/2
        let rel_err = (d2 - d1 / 2.0).abs() / d1;
        assert!(
            rel_err < ROUNDTRIP_RELATIVE_TOL,
            "Mahalanobis scaling: d1={d1}, d2={d2}, expected d2={}, rel_err={rel_err}",
            d1 / 2.0
        );
    }

    // ── Mission-level covariance tests (Phase 6C) ──────────────────────────

    use crate::mission::waypoints::plan_waypoint_mission;
    use crate::test_helpers::test_epoch;
    use crate::types::{DepartureState, MissionConfig, Waypoint};

    fn default_nav() -> NavigationAccuracy {
        NavigationAccuracy::default()
    }

    fn mission_departure() -> DepartureState {
        DepartureState {
            roe: QuasiNonsingularROE::default(),
            chief: iss_like_elements(),
            epoch: test_epoch(),
        }
    }

    fn single_wp_mission() -> (WaypointMission, PropagationModel) {
        let departure = mission_departure();
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig::default();
        let tof = departure.chief.period() * 0.75;

        let waypoints = vec![Waypoint {
            position_ric_km: Vector3::new(0.0, 5.0, 0.0),
            velocity_ric_km_s: Vector3::zeros(),
            tof_s: Some(tof),
        }];

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("single waypoint mission should succeed");
        (mission, propagator)
    }

    fn two_wp_mission() -> (WaypointMission, PropagationModel) {
        let departure = mission_departure();
        let propagator = PropagationModel::J2Stm;
        let config = MissionConfig::default();
        let tof = departure.chief.period() * 0.75;

        let waypoints = vec![
            Waypoint {
                position_ric_km: Vector3::new(0.0, 5.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
            Waypoint {
                position_ric_km: Vector3::new(2.0, 3.0, 0.0),
                velocity_ric_km_s: Vector3::zeros(),
                tof_s: Some(tof),
            },
        ];

        let mission = plan_waypoint_mission(&departure, &waypoints, &config, &propagator)
            .expect("two waypoint mission should succeed");
        (mission, propagator)
    }

    fn mission_initial_covariance() -> Matrix6 {
        let nav = default_nav();
        let chief = iss_like_elements();
        ric_accuracy_to_roe_covariance(&nav, &chief).unwrap()
    }

    #[test]
    fn single_leg_covariance_grows() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 50).unwrap();

        assert_eq!(report.legs.len(), 1);
        assert_eq!(report.legs[0].states.len(), 51); // 0..=50

        // Trace should increase monotonically over the coast arc
        for i in 1..report.legs[0].states.len() {
            let trace_prev = report.legs[0].states[i - 1].covariance_roe.trace();
            let trace_curr = report.legs[0].states[i].covariance_roe.trace();
            assert!(
                trace_curr >= trace_prev - MONOTONICITY_MARGIN,
                "Trace should grow monotonically: step {i}, prev={trace_prev}, curr={trace_curr}"
            );
        }

        assert!(
            report.max_sigma3_position_km > 0.0,
            "Max sigma3 should be positive"
        );
    }

    #[test]
    fn multi_leg_covariance_continuity() {
        let (mission, propagator) = two_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        // Without maneuver uncertainty: covariance should be continuous at leg boundary
        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 20).unwrap();

        assert_eq!(report.legs.len(), 2);

        // End of leg 0 coast (last sample at tau=tof) should match start of leg 1 (tau=0)
        let p_end_leg0 = report.legs[0].states.last().unwrap().covariance_roe;
        let p_start_leg1 = report.legs[1].states[0].covariance_roe;
        let diff = (p_end_leg0 - p_start_leg1).norm();

        // Without maneuver uncertainty, arrival/departure updates are identity,
        // but leg 1 has a different chief → STM at tau=0 is identity, so
        // the initial P for leg 1 = p_end_coast_leg0, and propagating at tau=0
        // returns that same P. However, the p_end_coast is independently
        // computed, so check both are consistent.
        assert!(
            diff < LEG_BOUNDARY_CONTINUITY_TOL,
            "Covariance should be continuous at leg boundary (no unc): diff={diff}"
        );
    }

    #[test]
    fn maneuver_uncertainty_jump() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();
        let unc = ManeuverUncertainty::default();
        let initial_trace = p0.trace();

        let report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            Some(&unc),
            &propagator,
            20,
        )
        .unwrap();

        // First sample includes departure maneuver update → trace should increase
        let first_trace = report.legs[0].states[0].covariance_roe.trace();
        assert!(
            first_trace > initial_trace,
            "Departure maneuver should increase trace: initial={initial_trace}, after={first_trace}"
        );
    }

    #[test]
    fn zero_maneuver_uncertainty_smooth() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();
        let initial_trace = p0.trace();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 20).unwrap();

        // Without maneuver uncertainty, first sample (tau=0) should match initial covariance
        let first_trace = report.legs[0].states[0].covariance_roe.trace();
        let diff = (first_trace - initial_trace).abs();
        assert!(
            diff < COVARIANCE_SYMMETRY_TOL,
            "Without unc, first trace should match initial: diff={diff}"
        );
    }

    #[test]
    fn j2drag_covariance_path_consistent() {
        // The upper-left 6×6 of the 9×9 drag STM IS the J2 STM (drag_stm.rs).
        // Drag only couples through the upper-right 6×3 block, so with zero
        // drag-rate uncertainty the 6×6 covariance must match J2-only exactly.
        let (mission, _) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let j2_report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            None,
            &PropagationModel::J2Stm,
            20,
        )
        .unwrap();

        let drag = crate::test_helpers::test_drag_config();
        let drag_report = propagate_mission_covariance(
            &mission,
            &p0,
            &nav,
            None,
            &PropagationModel::J2DragStm { drag },
            20,
        )
        .unwrap();

        // Same number of samples
        assert_eq!(
            j2_report.legs[0].states.len(),
            drag_report.legs[0].states.len()
        );

        // End-of-coast covariance traces should match (zero drag-rate uncertainty)
        let j2_end = j2_report.legs[0].states.last().unwrap().covariance_roe;
        let drag_end = drag_report.legs[0].states.last().unwrap().covariance_roe;
        let diff = (j2_end - drag_end).norm();
        assert!(
            diff < COVARIANCE_SYMMETRY_TOL,
            "J2 and J2+drag should match with zero drag-rate uncertainty: diff={diff}"
        );

        // Drag path should produce valid PSD result
        let eig = drag_end.symmetric_eigenvalues();
        assert!(
            eig.min() > PSD_EIGENVALUE_FLOOR,
            "Drag path covariance should be PSD: min_eig={}",
            eig.min()
        );
    }

    #[test]
    fn covariance_report_serde_roundtrip() {
        let (mission, propagator) = single_wp_mission();
        let p0 = mission_initial_covariance();
        let nav = default_nav();

        let report =
            propagate_mission_covariance(&mission, &p0, &nav, None, &propagator, 10).unwrap();

        let json = serde_json::to_string(&report).expect("serialize should work");
        let deserialized: MissionCovarianceReport =
            serde_json::from_str(&json).expect("deserialize should work");

        assert_eq!(report.legs.len(), deserialized.legs.len());
        assert!(
            (report.max_sigma3_position_km - deserialized.max_sigma3_position_km).abs()
                < SERDE_ROUNDTRIP_TOL,
            "max_sigma3 should survive roundtrip"
        );
        assert_eq!(
            report.navigation_accuracy, deserialized.navigation_accuracy,
            "NavigationAccuracy should survive roundtrip"
        );
    }
}

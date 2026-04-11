//! ROE ↔ RIC frame transformations (D'Amico Eq. 2.17).
//!
//! Provides the 6×6 matrix T that maps the ROE state vector to the
//! RIC relative state vector: `[pos; vel] = T · δα`, plus the inverse
//! (pseudo-inverse) mapping from RIC position back to ROE.

use nalgebra::{SMatrix, Vector3};

use crate::elements::keplerian_conversions::ConversionError;
use crate::types::{KeplerianElements, QuasiNonsingularROE, RICState};

/// Errors from RIC ↔ ROE operations.
#[derive(Debug, Clone)]
pub enum RicError {
    /// `T_pos` · `T_pos^T` is singular; cannot compute pseudo-inverse.
    SingularPositionMatrix,
    /// Chief Keplerian elements are invalid (e.g., `a <= 0` or `e >= 1`).
    InvalidChiefElements(ConversionError),
}

impl std::fmt::Display for RicError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SingularPositionMatrix => {
                write!(f, "T_pos · T_pos^T is singular; cannot compute pseudo-inverse")
            }
            Self::InvalidChiefElements(e) => {
                write!(f, "RicError: invalid chief elements — {e}")
            }
        }
    }
}

impl std::error::Error for RicError {}

impl From<ConversionError> for RicError {
    fn from(e: ConversionError) -> Self {
        Self::InvalidChiefElements(e)
    }
}

/// Compute the full 6×6 ROE→RIC transformation matrix (D'Amico Eq. 2.17).
///
/// Maps `[δa, δλ, δex, δey, δix, δiy]` → `[r_R, r_I, r_C, v_R, v_I, v_C]`.
///
/// # Arguments
/// * `chief` - Chief Keplerian elements
///
/// # Invariants
/// - `chief.a_km > 0`
/// - Near-circular assumption: accurate for `e < ~0.1`
///
/// # Errors
/// Returns `ConversionError::KeplerFailure` if `chief.a_km <= 0` or `chief.e` is outside [0, 1).
pub fn compute_t_matrix(chief: &KeplerianElements) -> Result<SMatrix<f64, 6, 6>, ConversionError> {
    chief.validate()?;
    let a = chief.a_km;
    let n = chief.mean_motion()?;
    let u = chief.mean_arg_of_lat();
    let (sin_u, cos_u) = u.sin_cos();

    let mut t = SMatrix::<f64, 6, 6>::zeros();

    // Position rows (D'Amico Eq. 2.17 position)
    // r_R = a * (δa - δex·cos(u) - δey·sin(u))
    t[(0, 0)] = a;
    t[(0, 2)] = -a * cos_u;
    t[(0, 3)] = -a * sin_u;

    // r_I = a * (δλ + 2·δex·sin(u) - 2·δey·cos(u))
    t[(1, 1)] = a;
    t[(1, 2)] = 2.0 * a * sin_u;
    t[(1, 3)] = -2.0 * a * cos_u;

    // r_C = a * (δix·sin(u) - δiy·cos(u))
    t[(2, 4)] = a * sin_u;
    t[(2, 5)] = -a * cos_u;

    // Velocity rows (D'Amico Eq. 2.17 velocity)
    // v_R = a·n * (δex·sin(u) - δey·cos(u))
    t[(3, 2)] = a * n * sin_u;
    t[(3, 3)] = -a * n * cos_u;

    // v_I = a·n * (-1.5·δa + 2·δex·cos(u) + 2·δey·sin(u))
    t[(4, 0)] = -1.5 * a * n;
    t[(4, 2)] = 2.0 * a * n * cos_u;
    t[(4, 3)] = 2.0 * a * n * sin_u;

    // v_C = a·n * (δix·cos(u) + δiy·sin(u))
    t[(5, 4)] = a * n * cos_u;
    t[(5, 5)] = a * n * sin_u;

    Ok(t)
}

/// Compute the 3×6 position submatrix of the T matrix (top 3 rows).
///
/// Delegates to [`compute_t_matrix`] and extracts rows 0–2.
///
/// # Arguments
/// * `chief` — chief Keplerian elements
///
/// # Invariants
/// - `chief.a_km > 0`
/// - Near-circular assumption: accurate for `e < ~0.1` (see [`compute_t_matrix`])
///
/// # Errors
/// Returns `ConversionError` if `chief` has invalid SMA or eccentricity.
pub fn compute_t_position(chief: &KeplerianElements) -> Result<SMatrix<f64, 3, 6>, ConversionError> {
    let t = compute_t_matrix(chief)?;
    Ok(t.fixed_rows::<3>(0).into())
}

/// Compute the 3×6 velocity submatrix of the T matrix (bottom 3 rows).
///
/// Delegates to [`compute_t_matrix`] and extracts rows 3–5.
///
/// # Arguments
/// * `chief` — chief Keplerian elements
///
/// # Invariants
/// - `chief.a_km > 0`
/// - Near-circular assumption: accurate for `e < ~0.1` (see [`compute_t_matrix`])
///
/// # Errors
/// Returns `ConversionError` if `chief` has invalid SMA or eccentricity.
pub fn compute_t_velocity(chief: &KeplerianElements) -> Result<SMatrix<f64, 3, 6>, ConversionError> {
    let t = compute_t_matrix(chief)?;
    Ok(t.fixed_rows::<3>(3).into())
}

/// Recover ROE from a target RIC position using the pseudo-inverse of `T_pos`.
///
/// Since `T_pos` is 3x6 (under-determined), this finds the minimum-norm ROE
/// that produces the given RIC position: `δα = T_pos^† · r_RIC`.
///
/// # Arguments
/// * `ric_pos` - Target position in RIC frame (km)
/// * `chief` - Chief Keplerian elements
///
/// # Invariants
/// - `chief.a_km > 0`
/// - Result is the minimum-norm ROE; other valid ROE solutions exist
///
/// # Singularities
/// `T_pos · T_pos^T` can become ill-conditioned at specific argument-of-latitude
/// values where the 3×6 position mapping loses effective rank (e.g., when
/// radial and in-track rows become nearly linearly dependent). In practice
/// this is rare for typical ROE magnitudes, but callers should handle the
/// `SingularPositionMatrix` error.
///
/// # Errors
/// Returns `RicError::SingularPositionMatrix` if `T_pos · T_pos^T` is singular.
pub fn ric_position_to_roe(
    ric_pos: &Vector3<f64>,
    chief: &KeplerianElements,
) -> Result<QuasiNonsingularROE, RicError> {
    let t_pos = compute_t_position(chief)?;
    // Pseudo-inverse: T^† = Tᵀ(T·Tᵀ)⁻¹
    let t_tt = t_pos * t_pos.transpose();
    let t_tt_inv = t_tt.try_inverse().ok_or(RicError::SingularPositionMatrix)?;
    let pseudo_inv = t_pos.transpose() * t_tt_inv;
    let roe_vec = pseudo_inv * ric_pos;
    Ok(QuasiNonsingularROE::from_vector(&roe_vec.fixed_rows::<6>(0).into()))
}

/// Convert quasi-nonsingular ROEs to RIC-frame relative state (D'Amico Eq. 2.17).
///
/// Implemented via the 6×6 T matrix from [`compute_t_matrix`].
///
/// # Arguments
/// * `roe` - Quasi-nonsingular relative orbital elements
/// * `chief` - Chief Keplerian elements (used for a and mean argument of latitude u)
///
/// # Invariants
/// - `chief.a_km > 0`
/// - ROE must satisfy linearization validity (`dimensionless_norm() < ~0.01`)
///
/// # Errors
/// Returns `ConversionError` if `chief` has invalid SMA or eccentricity.
pub fn roe_to_ric(roe: &QuasiNonsingularROE, chief: &KeplerianElements) -> Result<RICState, ConversionError> {
    let t = compute_t_matrix(chief)?;
    let ric_vec = t * roe.to_vector();
    Ok(RICState {
        position_ric_km: Vector3::new(ric_vec[0], ric_vec[1], ric_vec[2]),
        velocity_ric_km_s: Vector3::new(ric_vec[3], ric_vec[4], ric_vec[5]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief, iss_like_elements};

    // Named tolerance constants for ROE↔RIC mapping tests
    //
    // Categories:
    //   ZERO_ROE     — zero ROE → zero RIC (algebraic identity)
    //   T_MATRIX     — T-matrix element and product agreement
    //   RIC_POSITION — RIC position coordinate agreement
    //   ROUNDTRIP    — RIC→ROE→RIC roundtrip precision

    /// Zero ROE produces zero RIC state. Exact algebraic identity;
    /// 1e-12 covers matrix multiplication roundoff for 6×6 × 6×1.
    const ZERO_ROE_TOL: f64 = 1e-12;

    /// T-matrix structural entries vs analytical D'Amico Eq. 2.17 values,
    /// and T·roe vs `roe_to_ric` cross-validation. Agreement to ~O(1e-14);
    /// 1e-10 is conservative.
    const T_MATRIX_TOL: f64 = 1e-10;

    /// Pseudo-inverse roundtrip (ROE→RIC→ROE). Dominated by pseudo-inverse
    /// condition number; 1e-12 covers well-conditioned cases.
    const ROUNDTRIP_TOL: f64 = 1e-12;

    /// Pure δa radial offset: cross-track component is algebraically zero.
    /// 1e-12 allows for trigonometric evaluation at non-zero mean anomaly.
    const RADIAL_CROSS_TRACK_ZERO_TOL: f64 = 1e-12;

    /// Pure δix cross-track offset: radial component tolerance.
    /// D'Amico Eq. 2.17 gives δx ∝ cos(u)·δix; at u≠0 we need 1e-8
    /// to cover the full T-matrix evaluation error chain.
    const CROSS_TRACK_RADIAL_TOL: f64 = 1e-8;

    #[test]
    fn t_matrix_negative_sma_returns_error() {
        let chief = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = compute_t_matrix(&chief);
        assert!(
            matches!(result, Err(crate::elements::keplerian_conversions::ConversionError::KeplerFailure(
                crate::types::KeplerError::InvalidSemiMajorAxis { .. }
            ))),
            "Negative SMA should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    #[test]
    fn roe_to_ric_negative_sma_returns_error() {
        let chief = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.5,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let roe = QuasiNonsingularROE::default();
        let result = roe_to_ric(&roe, &chief);
        assert!(
            matches!(result, Err(crate::elements::keplerian_conversions::ConversionError::KeplerFailure(
                crate::types::KeplerError::InvalidSemiMajorAxis { .. }
            ))),
            "Negative SMA should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    // --- Tests from frames.rs ---

    #[test]
    fn zero_roe_gives_zero_ric() {
        let chief = KeplerianElements {
            a_km: 6786.0,
            e: 0.001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 30.0_f64.to_radians(),
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let roe = QuasiNonsingularROE::default();
        let ric = roe_to_ric(&roe, &chief).unwrap();
        assert!(ric.position_ric_km.norm() < ZERO_ROE_TOL);
        assert!(ric.velocity_ric_km_s.norm() < ZERO_ROE_TOL);
    }

    #[test]
    fn pure_da_gives_radial_offset() {
        // A pure δa offset should produce a radial offset at u=0 and along-track drift
        let chief = KeplerianElements {
            a_km: 6786.0,
            e: 0.0,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0, // u = 0
        };
        let da = 1.0 / 6786.0; // ~1 km offset in SMA
        let roe = QuasiNonsingularROE {
            da,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let ric = roe_to_ric(&roe, &chief).unwrap();

        // Radial position should be a * da = 1 km
        assert!(
            (ric.position_ric_km.x - 1.0).abs() < CROSS_TRACK_RADIAL_TOL,
            "Radial offset should be ~1 km, got {}",
            ric.position_ric_km.x
        );
        // Cross-track should be zero
        assert!(ric.position_ric_km.z.abs() < RADIAL_CROSS_TRACK_ZERO_TOL);
    }

    #[test]
    fn pure_dix_gives_cross_track() {
        let chief = KeplerianElements {
            a_km: 6786.0,
            e: 0.0,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 90.0_f64.to_radians(), // u = π/2
        };
        let dix = 0.001_f64; // small inclination offset
        let roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix,
            diy: 0.0,
        };
        let ric = roe_to_ric(&roe, &chief).unwrap();

        // At u=π/2, cross-track = a * dix * sin(u) = a * dix
        let expected_cross = chief.a_km * dix;
        assert!(
            (ric.position_ric_km.z - expected_cross).abs() < CROSS_TRACK_RADIAL_TOL,
            "Cross-track should be ~{expected_cross} km, got {}",
            ric.position_ric_km.z
        );
    }

    // --- Tests from t_matrix.rs ---

    /// T · roe must match `roe_to_ric()` at the default u value.
    #[test]
    fn t_matrix_matches_roe_to_ric() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0002,
            dey: -0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let t = compute_t_matrix(&chief).unwrap();
        let roe_vec = roe.to_vector();
        let ric_vec = t * roe_vec;

        let ric_ref = roe_to_ric(&roe, &chief).unwrap();

        assert!((ric_vec[0] - ric_ref.position_ric_km.x).abs() < T_MATRIX_TOL, "R mismatch");
        assert!((ric_vec[1] - ric_ref.position_ric_km.y).abs() < T_MATRIX_TOL, "I mismatch");
        assert!((ric_vec[2] - ric_ref.position_ric_km.z).abs() < T_MATRIX_TOL, "C mismatch");
        assert!((ric_vec[3] - ric_ref.velocity_ric_km_s.x).abs() < T_MATRIX_TOL, "vR mismatch");
        assert!((ric_vec[4] - ric_ref.velocity_ric_km_s.y).abs() < T_MATRIX_TOL, "vI mismatch");
        assert!((ric_vec[5] - ric_ref.velocity_ric_km_s.z).abs() < T_MATRIX_TOL, "vC mismatch");
    }

    /// T · roe matches `roe_to_ric` at multiple u values.
    #[test]
    fn t_matrix_matches_at_multiple_u() {
        for u_deg in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0, 350.0] {
            let chief = KeplerianElements {
                a_km: 6786.0,
                e: 0.0001,
                i_rad: 51.6_f64.to_radians(),
                raan_rad: 30.0_f64.to_radians(),
                aop_rad: 0.0,
                mean_anomaly_rad: u_deg.to_radians(),
            };
            let roe = QuasiNonsingularROE {
                da: 0.5 / chief.a_km,
                dlambda: 0.002,
                dex: 0.0003,
                dey: -0.0002,
                dix: 0.0004,
                diy: 0.0001,
            };

            let t = compute_t_matrix(&chief).unwrap();
            let ric_vec = t * roe.to_vector();
            let ric_ref = roe_to_ric(&roe, &chief).unwrap();

            let pos_err = (Vector3::new(ric_vec[0], ric_vec[1], ric_vec[2])
                - ric_ref.position_ric_km)
                .norm();
            assert!(
                pos_err < T_MATRIX_TOL,
                "Position mismatch at u={u_deg}°: err={pos_err}"
            );
        }
    }

    /// Zero ROE gives zero RIC via T matrix.
    #[test]
    fn zero_roe_gives_zero_ric_via_t() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let t = compute_t_matrix(&chief).unwrap();
        let ric_vec = t * roe.to_vector();
        assert!(ric_vec.norm() < ROUNDTRIP_TOL);
    }

    /// Position submatrix gives correct position.
    #[test]
    fn position_submatrix() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };
        let t_pos = compute_t_position(&chief).unwrap();
        let pos = t_pos * roe.to_vector();
        let ric_ref = roe_to_ric(&roe, &chief).unwrap();

        assert!((pos[0] - ric_ref.position_ric_km.x).abs() < T_MATRIX_TOL);
        assert!((pos[1] - ric_ref.position_ric_km.y).abs() < T_MATRIX_TOL);
        assert!((pos[2] - ric_ref.position_ric_km.z).abs() < T_MATRIX_TOL);
    }

    /// Pseudo-inverse roundtrip: `ric_position_to_roe` → `T_pos·roe` recovers position.
    #[test]
    fn pseudo_inverse_roundtrip() {
        let chief = iss_like_elements();
        let target_pos = nalgebra::Vector3::new(1.0, 5.0, 0.5);

        let roe = ric_position_to_roe(&target_pos, &chief).unwrap();
        let t_pos = compute_t_position(&chief).unwrap();
        let recovered_pos = t_pos * roe.to_vector();

        assert!(
            (recovered_pos[0] - target_pos.x).abs() < T_MATRIX_TOL,
            "R roundtrip: {} vs {}",
            recovered_pos[0],
            target_pos.x
        );
        assert!(
            (recovered_pos[1] - target_pos.y).abs() < T_MATRIX_TOL,
            "I roundtrip: {} vs {}",
            recovered_pos[1],
            target_pos.y
        );
        assert!(
            (recovered_pos[2] - target_pos.z).abs() < T_MATRIX_TOL,
            "C roundtrip: {} vs {}",
            recovered_pos[2],
            target_pos.z
        );
    }

    // --- Paper-traced regression tests (D'Amico Eq. 2.17) ---

    /// D'Amico Eq. 2.17: T-matrix entries at u=0 (sin=0, cos=1).
    /// All entries are exact closed-form values at this argument of latitude.
    #[test]
    fn damico_eq217_t_matrix_at_u0() {
        let chief = damico_table21_chief(); // aop=0, M=0 → u=0
        let a = chief.a_km;
        let t = compute_t_matrix(&chief).unwrap();

        // At u=0: sin(u)=0, cos(u)=1
        // Row 0 (R): a·δa - a·cos(u)·δex - a·sin(u)·δey
        assert!((t[(0, 0)] - a).abs() < T_MATRIX_TOL, "T[0,0]={}, expected {a}", t[(0, 0)]);
        assert!((t[(0, 2)] - (-a)).abs() < T_MATRIX_TOL, "T[0,2]={}, expected {}", t[(0, 2)], -a);
        assert!(t[(0, 3)].abs() < T_MATRIX_TOL, "T[0,3]={}, expected 0", t[(0, 3)]);

        // Row 1 (I): a·δλ + 2a·sin(u)·δex - 2a·cos(u)·δey
        assert!((t[(1, 1)] - a).abs() < T_MATRIX_TOL, "T[1,1]={}, expected {a}", t[(1, 1)]);
        assert!(t[(1, 2)].abs() < T_MATRIX_TOL, "T[1,2]={}, expected 0 (2a·sin(0))", t[(1, 2)]);
        assert!((t[(1, 3)] - (-2.0 * a)).abs() < T_MATRIX_TOL, "T[1,3]={}, expected {}", t[(1, 3)], -2.0 * a);

        // Row 2 (C): a·sin(u)·δix - a·cos(u)·δiy
        assert!(t[(2, 4)].abs() < T_MATRIX_TOL, "T[2,4]={}, expected 0 (a·sin(0))", t[(2, 4)]);
        assert!((t[(2, 5)] - (-a)).abs() < T_MATRIX_TOL, "T[2,5]={}, expected {}", t[(2, 5)], -a);
    }

    /// D'Amico Eq. 2.17: T-matrix entries at u=90° (sin=1, cos=0).
    /// Complementary entries are nonzero compared to u=0.
    #[test]
    fn damico_eq217_t_matrix_at_u90() {
        let mut chief = damico_table21_chief();
        chief.mean_anomaly_rad = std::f64::consts::FRAC_PI_2; // u = π/2
        let a = chief.a_km;
        let t = compute_t_matrix(&chief).unwrap();

        // At u=π/2: sin(u)=1, cos(u)=0
        // Row 0: T[0,2]=0 (-a·cos(π/2)), T[0,3]=-a (-a·sin(π/2))
        assert!(t[(0, 2)].abs() < T_MATRIX_TOL, "T[0,2]={}, expected ~0", t[(0, 2)]);
        assert!((t[(0, 3)] - (-a)).abs() < T_MATRIX_TOL, "T[0,3]={}, expected {}", t[(0, 3)], -a);

        // Row 1: T[1,2]=2a (2a·sin(π/2)), T[1,3]=0 (-2a·cos(π/2))
        assert!((t[(1, 2)] - 2.0 * a).abs() < T_MATRIX_TOL, "T[1,2]={}, expected {}", t[(1, 2)], 2.0 * a);
        assert!(t[(1, 3)].abs() < T_MATRIX_TOL, "T[1,3]={}, expected ~0", t[(1, 3)]);

        // Row 2: T[2,4]=a (a·sin(π/2)), T[2,5]=0 (-a·cos(π/2))
        assert!((t[(2, 4)] - a).abs() < T_MATRIX_TOL, "T[2,4]={}, expected {a}", t[(2, 4)]);
        assert!(t[(2, 5)].abs() < T_MATRIX_TOL, "T[2,5]={}, expected ~0", t[(2, 5)]);
    }

    /// D'Amico Table 2.1 Case 1: ROE → RIC at u=0.
    /// Expected: R=0, I=-0.8 km, C=-0.2 km (closed-form from T at u=0).
    /// I = a·(-2·δey·cos(0)) = -2·0.400 = -0.8 km; C = a·(-δiy·cos(0)) = -0.200 = -0.2 km.
    #[test]
    fn damico_table21_case1_ric_at_u0() {
        let chief = damico_table21_chief(); // u=0
        let roe = damico_table21_case1_roe(); // δey=0.4/a, δiy=0.2/a
        let ric = roe_to_ric(&roe, &chief).unwrap();

        // R = a·(δa - δex·cos(0) - δey·sin(0)) = 0
        assert!(
            ric.position_ric_km.x.abs() < T_MATRIX_TOL,
            "R={}, expected 0", ric.position_ric_km.x
        );
        // I = a·(δλ + 2·δex·sin(0) - 2·δey·cos(0)) = -2·a·δey = -0.8 km
        assert!(
            (ric.position_ric_km.y - (-0.8)).abs() < T_MATRIX_TOL,
            "I={}, expected -0.8", ric.position_ric_km.y
        );
        // C = a·(δix·sin(0) - δiy·cos(0)) = -a·δiy = -0.2 km
        assert!(
            (ric.position_ric_km.z - (-0.2)).abs() < T_MATRIX_TOL,
            "C={}, expected -0.2", ric.position_ric_km.z
        );
    }

    /// D'Amico Table 2.1 Case 1: ROE → RIC at u=90°.
    /// Expected: R=-0.4 km, I≈0, C≈0 (closed-form from T at u=π/2).
    #[test]
    fn damico_table21_case1_ric_at_u90() {
        let mut chief = damico_table21_chief();
        chief.mean_anomaly_rad = std::f64::consts::FRAC_PI_2; // u = π/2
        let roe = damico_table21_case1_roe();
        let ric = roe_to_ric(&roe, &chief).unwrap();

        // R = a·(-δey·sin(π/2)) = -a·δey = -0.4 km
        assert!(
            (ric.position_ric_km.x - (-0.4)).abs() < CROSS_TRACK_RADIAL_TOL,
            "R={}, expected -0.4", ric.position_ric_km.x
        );
        // I = a·(-2·δey·cos(π/2)) ≈ 0 (cos(π/2) ≈ 6e-17)
        assert!(
            ric.position_ric_km.y.abs() < CROSS_TRACK_RADIAL_TOL,
            "I={}, expected ~0", ric.position_ric_km.y
        );
        // C = a·(-δiy·cos(π/2)) ≈ 0
        assert!(
            ric.position_ric_km.z.abs() < CROSS_TRACK_RADIAL_TOL,
            "C={}, expected ~0", ric.position_ric_km.z
        );
    }
}

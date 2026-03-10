//! ROE ↔ RIC frame transformations (D'Amico Eq. 2.17).
//!
//! Provides the 6×6 matrix T that maps the ROE state vector to the
//! RIC relative state vector: `[pos; vel] = T · δα`, plus the inverse
//! (pseudo-inverse) mapping from RIC position back to ROE.

use nalgebra::{SMatrix, Vector3};

use crate::types::{KeplerianElements, QuasiNonsingularROE, RICState};

/// Errors from RIC ↔ ROE operations.
#[derive(Debug, Clone)]
pub enum RicError {
    /// `T_pos` · `T_pos^T` is singular; cannot compute pseudo-inverse.
    SingularPositionMatrix,
}

impl std::fmt::Display for RicError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SingularPositionMatrix => {
                write!(f, "T_pos · T_pos^T is singular; cannot compute pseudo-inverse")
            }
        }
    }
}

impl std::error::Error for RicError {}

/// Compute the full 6×6 ROE→RIC transformation matrix (D'Amico Eq. 2.17).
///
/// Maps `[δa, δλ, δex, δey, δix, δiy]` → `[r_R, r_I, r_C, v_R, v_I, v_C]`.
///
/// # Arguments
/// * `chief` - Chief Keplerian elements
#[must_use]
pub fn compute_t_matrix(chief: &KeplerianElements) -> SMatrix<f64, 6, 6> {
    let a = chief.a_km;
    let n = chief.mean_motion();
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

    t
}

/// Compute the 3×6 position submatrix of the T matrix (top 3 rows).
#[must_use]
pub fn compute_t_position(chief: &KeplerianElements) -> SMatrix<f64, 3, 6> {
    let t = compute_t_matrix(chief);
    t.fixed_rows::<3>(0).into()
}

/// Compute the 3×6 velocity submatrix of the T matrix (bottom 3 rows).
#[must_use]
pub fn compute_t_velocity(chief: &KeplerianElements) -> SMatrix<f64, 3, 6> {
    let t = compute_t_matrix(chief);
    t.fixed_rows::<3>(3).into()
}

/// Recover ROE from a target RIC position using the pseudo-inverse of `T_pos`.
///
/// Since `T_pos` is 3x6 (under-determined), this finds the minimum-norm ROE
/// that produces the given RIC position: `δα = T_pos^† · r_RIC`.
///
/// # Errors
/// Returns `RicError::SingularPositionMatrix` if `T_pos · T_pos^T` is singular.
///
/// # Arguments
/// * `ric_pos` - Target position in RIC frame (km)
/// * `chief` - Chief Keplerian elements
pub fn ric_position_to_roe(
    ric_pos: &Vector3<f64>,
    chief: &KeplerianElements,
) -> Result<QuasiNonsingularROE, RicError> {
    let t_pos = compute_t_position(chief);
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
#[must_use]
pub fn roe_to_ric(roe: &QuasiNonsingularROE, chief: &KeplerianElements) -> RICState {
    debug_assert!(chief.a_km > 0.0, "chief semi-major axis must be positive");
    let t = compute_t_matrix(chief);
    let ric_vec = t * roe.to_vector();
    RICState {
        position_ric_km: Vector3::new(ric_vec[0], ric_vec[1], ric_vec[2]),
        velocity_ric_km_s: Vector3::new(ric_vec[3], ric_vec[4], ric_vec[5]),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;

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
        let ric = roe_to_ric(&roe, &chief);
        assert!(ric.position_ric_km.norm() < 1e-12);
        assert!(ric.velocity_ric_km_s.norm() < 1e-12);
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
        let ric = roe_to_ric(&roe, &chief);

        // Radial position should be a * da = 1 km
        assert!(
            (ric.position_ric_km.x - 1.0).abs() < 1e-8,
            "Radial offset should be ~1 km, got {}",
            ric.position_ric_km.x
        );
        // Cross-track should be zero
        assert!(ric.position_ric_km.z.abs() < 1e-12);
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
        let ric = roe_to_ric(&roe, &chief);

        // At u=π/2, cross-track = a * dix * sin(u) = a * dix
        let expected_cross = chief.a_km * dix;
        assert!(
            (ric.position_ric_km.z - expected_cross).abs() < 1e-8,
            "Cross-track should be ~{expected_cross} km, got {}",
            ric.position_ric_km.z
        );
    }

    // --- Tests from t_matrix.rs ---

    /// T · roe must match roe_to_ric() at the default u value.
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

        let t = compute_t_matrix(&chief);
        let roe_vec = roe.to_vector();
        let ric_vec = t * roe_vec;

        let ric_ref = roe_to_ric(&roe, &chief);

        assert!((ric_vec[0] - ric_ref.position_ric_km.x).abs() < 1e-10, "R mismatch");
        assert!((ric_vec[1] - ric_ref.position_ric_km.y).abs() < 1e-10, "I mismatch");
        assert!((ric_vec[2] - ric_ref.position_ric_km.z).abs() < 1e-10, "C mismatch");
        assert!((ric_vec[3] - ric_ref.velocity_ric_km_s.x).abs() < 1e-10, "vR mismatch");
        assert!((ric_vec[4] - ric_ref.velocity_ric_km_s.y).abs() < 1e-10, "vI mismatch");
        assert!((ric_vec[5] - ric_ref.velocity_ric_km_s.z).abs() < 1e-10, "vC mismatch");
    }

    /// T · roe matches roe_to_ric at multiple u values.
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

            let t = compute_t_matrix(&chief);
            let ric_vec = t * roe.to_vector();
            let ric_ref = roe_to_ric(&roe, &chief);

            let pos_err = ((ric_vec[0] - ric_ref.position_ric_km.x).powi(2)
                + (ric_vec[1] - ric_ref.position_ric_km.y).powi(2)
                + (ric_vec[2] - ric_ref.position_ric_km.z).powi(2))
            .sqrt();
            assert!(
                pos_err < 1e-10,
                "Position mismatch at u={u_deg}°: err={pos_err}"
            );
        }
    }

    /// Zero ROE gives zero RIC via T matrix.
    #[test]
    fn zero_roe_gives_zero_ric_via_t() {
        let chief = iss_like_elements();
        let roe = QuasiNonsingularROE::default();
        let t = compute_t_matrix(&chief);
        let ric_vec = t * roe.to_vector();
        assert!(ric_vec.norm() < 1e-12);
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
        let t_pos = compute_t_position(&chief);
        let pos = t_pos * roe.to_vector();
        let ric_ref = roe_to_ric(&roe, &chief);

        assert!((pos[0] - ric_ref.position_ric_km.x).abs() < 1e-10);
        assert!((pos[1] - ric_ref.position_ric_km.y).abs() < 1e-10);
        assert!((pos[2] - ric_ref.position_ric_km.z).abs() < 1e-10);
    }

    /// Pseudo-inverse roundtrip: ric_position_to_roe → T_pos·roe recovers position.
    #[test]
    fn pseudo_inverse_roundtrip() {
        let chief = iss_like_elements();
        let target_pos = nalgebra::Vector3::new(1.0, 5.0, 0.5);

        let roe = ric_position_to_roe(&target_pos, &chief).unwrap();
        let t_pos = compute_t_position(&chief);
        let recovered_pos = t_pos * roe.to_vector();

        assert!(
            (recovered_pos[0] - target_pos.x).abs() < 1e-10,
            "R roundtrip: {} vs {}",
            recovered_pos[0],
            target_pos.x
        );
        assert!(
            (recovered_pos[1] - target_pos.y).abs() < 1e-10,
            "I roundtrip: {} vs {}",
            recovered_pos[1],
            target_pos.y
        );
        assert!(
            (recovered_pos[2] - target_pos.z).abs() < 1e-10,
            "C roundtrip: {} vs {}",
            recovered_pos[2],
            target_pos.z
        );
    }
}

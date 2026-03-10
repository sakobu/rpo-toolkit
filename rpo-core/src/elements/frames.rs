//! ECI ↔ RIC frame transformations via Direction Cosine Matrix.
//!
//! Provides geometric conversions between the Earth-Centered Inertial (ECI)
//! frame and the Radial-In-track-Cross-track (RIC) rotating frame, operating
//! directly on state vectors without going through orbital elements.

use nalgebra::{SMatrix, Vector3};

use crate::types::{RICState, StateVector};

/// 3×3 matrix type alias (Direction Cosine Matrix).
type Matrix3 = SMatrix<f64, 3, 3>;

/// Compute the ECI→RIC Direction Cosine Matrix from the chief state vector.
///
/// The DCM rows are the RIC unit vectors expressed in ECI coordinates:
/// - `R_hat = r / |r|` (radial)
/// - `C_hat = (r × v) / |r × v|` (cross-track, along angular momentum)
/// - `I_hat = C_hat × R_hat` (in-track, completes right-hand triad)
///
/// Multiply: `v_ric = DCM * v_eci`. Inverse: `v_eci = DCM^T * v_ric`.
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero (degenerate at origin)
/// - `chief` angular momentum `r × v` must be non-zero (rectilinear orbits unsupported)
#[must_use]
pub fn eci_to_ric_dcm(chief: &StateVector) -> Matrix3 {
    let r = chief.position_eci_km;
    let v = chief.velocity_eci_km_s;
    let r_norm = r.norm();
    let h = r.cross(&v);
    let h_norm = h.norm();

    debug_assert!(r_norm > 0.0, "chief position must be non-zero");
    debug_assert!(h_norm > 0.0, "chief angular momentum must be non-zero");

    let r_hat = r / r_norm;
    let c_hat = h / h_norm;
    let i_hat = c_hat.cross(&r_hat);

    // DCM rows: R_hat, I_hat, C_hat
    Matrix3::new(
        r_hat.x, r_hat.y, r_hat.z,
        i_hat.x, i_hat.y, i_hat.z,
        c_hat.x, c_hat.y, c_hat.z,
    )
}

/// Transform a Δv from RIC frame to ECI frame.
///
/// `dv_eci = DCM^T * dv_ric`
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero
/// - `chief` angular momentum must be non-zero
#[must_use]
pub fn ric_to_eci_dv(dv_ric: &Vector3<f64>, chief: &StateVector) -> Vector3<f64> {
    let dcm = eci_to_ric_dcm(chief);
    dcm.transpose() * dv_ric
}

/// Transform a Δv from ECI frame to RIC frame.
///
/// `dv_ric = DCM * dv_eci`
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero
/// - `chief` angular momentum must be non-zero
#[must_use]
pub fn eci_to_ric_dv(dv_eci: &Vector3<f64>, chief: &StateVector) -> Vector3<f64> {
    let dcm = eci_to_ric_dcm(chief);
    dcm * dv_eci
}

/// Convert chief and deputy ECI states to a relative RIC state.
///
/// Position is the deputy–chief separation expressed in the chief-centered
/// RIC frame:
///     ρ = C · (`r_dep` − `r_chief`)
///
/// Velocity is the time derivative of ρ in the rotating RIC frame (ρ̇):
///     ρ̇ = C · (`v_dep` − `v_chief`) − ω × ρ
///
/// where the RIC frame angular velocity is
///     `ω_ric` = \[0, 0, |h| / r²\]
/// with h = r × v of the chief.
///
/// This is the exact instantaneous kinematic relation for the RIC frame
/// defined by the chief position and velocity.
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero
/// - `chief` angular momentum must be non-zero
/// - Both states must be at the same epoch
#[must_use]
pub fn eci_to_ric_relative(chief: &StateVector, deputy: &StateVector) -> RICState {
    let dcm = eci_to_ric_dcm(chief);

    let delta_r = deputy.position_eci_km - chief.position_eci_km;
    let delta_v = deputy.velocity_eci_km_s - chief.velocity_eci_km_s;

    let rho = dcm * delta_r;
    let dv_ric_inertial = dcm * delta_v;

    // Rotating frame correction: ω = |h| / r²
    let r = chief.position_eci_km.norm();
    let h = chief.position_eci_km.cross(&chief.velocity_eci_km_s);
    let omega = h.norm() / (r * r);

    // ω × ρ in RIC = [0, 0, ω] × [R, I, C] = [-ω·I, ω·R, 0]
    let omega_cross_rho = Vector3::new(-omega * rho.y, omega * rho.x, 0.0);

    RICState {
        position_ric_km: rho,
        velocity_ric_km_s: dv_ric_inertial - omega_cross_rho,
    }
}

/// Convert a RIC position offset to an ECI position.
///
/// `r_deputy_eci = r_chief_eci + DCM^T * ric_offset`
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero
/// - `chief` angular momentum must be non-zero
#[must_use]
pub fn ric_to_eci_position(chief: &StateVector, ric_offset: &Vector3<f64>) -> Vector3<f64> {
    let dcm = eci_to_ric_dcm(chief);
    chief.position_eci_km + dcm.transpose() * ric_offset
}

/// Convert a chief ECI state and a relative RIC state to the deputy ECI state.
///
/// Inverse of [`eci_to_ric_relative`]: recovers the inertial relative
/// velocity from the rotating-frame derivative ρ̇ before rotating to ECI.
///
/// Position: `r_chief + C^T · ρ`
/// Velocity: `v_chief + C^T · (ρ̇ + ω × ρ)`
///
/// # Invariants
/// - `chief.position_eci_km` must be non-zero
/// - `chief` angular momentum must be non-zero
#[must_use]
pub fn ric_to_eci_state(chief: &StateVector, ric_state: &RICState) -> StateVector {
    let dcm = eci_to_ric_dcm(chief);

    let r = chief.position_eci_km.norm();
    let h = chief.position_eci_km.cross(&chief.velocity_eci_km_s);
    let omega = h.norm() / (r * r);

    // ω × ρ in RIC = [-ω·I, ω·R, 0]
    let rho = &ric_state.position_ric_km;
    let omega_cross_rho = Vector3::new(-omega * rho.y, omega * rho.x, 0.0);

    let position = chief.position_eci_km + dcm.transpose() * ric_state.position_ric_km;
    let velocity = chief.velocity_eci_km_s + dcm.transpose() * (ric_state.velocity_ric_km_s + omega_cross_rho);

    StateVector {
        epoch: chief.epoch,
        position_eci_km: position,
        velocity_eci_km_s: velocity,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::conversions::keplerian_to_state;
    use crate::elements::ric::roe_to_ric;
    use crate::elements::roe::compute_roe;
    use crate::test_helpers::{eccentric_elements, iss_like_elements, test_epoch};
    use crate::types::KeplerianElements;

    /// Chief at (r,0,0) with v=(0,v,0) in equatorial plane → DCM ≈ Identity
    #[test]
    fn dcm_equatorial_orbit() {
        let ke = KeplerianElements {
            a_km: 7000.0,
            e: 0.0,
            i_rad: 0.0,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0, // at periapsis → r along x, v along y
        };
        let sv = keplerian_to_state(&ke, test_epoch()).unwrap();
        let dcm = eci_to_ric_dcm(&sv);

        // R_hat should be along +x, I_hat along +y, C_hat along +z
        let identity = Matrix3::identity();
        let err = (dcm - identity).norm();
        assert!(
            err < 1e-10,
            "DCM should be near identity for equatorial orbit at M=0, error={err}"
        );
    }

    /// DCM must be orthogonal: DCM·DCM^T = I, det(DCM) = +1
    #[test]
    fn dcm_orthogonality() {
        let sv = keplerian_to_state(&iss_like_elements(), test_epoch()).unwrap();
        let dcm = eci_to_ric_dcm(&sv);

        let product = dcm * dcm.transpose();
        let identity = Matrix3::identity();
        let orth_err = (product - identity).norm();
        assert!(
            orth_err < 1e-14,
            "DCM·DCM^T should be identity, error={orth_err}"
        );

        let det = dcm.determinant();
        assert!(
            (det - 1.0).abs() < 1e-14,
            "DCM determinant should be +1, got {det}"
        );
    }

    /// RIC→ECI→RIC roundtrip for Δv vectors
    #[test]
    fn dv_roundtrip() {
        let dv_ric = Vector3::new(0.001, -0.005, 0.002);

        for ke in &[iss_like_elements(), eccentric_elements()] {
            let sv = keplerian_to_state(ke, test_epoch()).unwrap();
            let dv_eci = ric_to_eci_dv(&dv_ric, &sv);
            let dv_back = eci_to_ric_dv(&dv_eci, &sv);

            let err = (dv_back - dv_ric).norm();
            assert!(
                err < 1e-14,
                "Δv roundtrip error = {err} km/s for orbit a={}, e={}",
                ke.a_km,
                ke.e
            );
        }
    }

    /// Cross-validate geometric ECI→RIC against analytical ROE→RIC path.
    /// Agreement within ~10m for a 1 km separation in an ISS-like orbit.
    #[test]
    fn eci_ric_relative_vs_roe_to_ric() {
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 1.0, // +1 km SMA offset
            e: chief_ke.e + 0.0001,
            i_rad: chief_ke.i_rad + 0.0001,
            ..chief_ke
        };

        let epoch = test_epoch();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        // Geometric path: direct ECI→RIC
        let ric_geometric = eci_to_ric_relative(&chief_sv, &deputy_sv);

        // Analytical path: Keplerian→ROE→RIC
        let roe = compute_roe(&chief_ke, &deputy_ke).unwrap();
        let ric_analytical = roe_to_ric(&roe, &chief_ke).unwrap();

        let pos_err = (ric_geometric.position_ric_km - ric_analytical.position_ric_km).norm();
        assert!(
            pos_err < 0.010, // 10 m agreement
            "Geometric vs analytical RIC position disagree by {:.4} km (should be < 0.010 km)",
            pos_err
        );

        let vel_err = (ric_geometric.velocity_ric_km_s - ric_analytical.velocity_ric_km_s).norm();
        assert!(
            vel_err < 1e-4, // 0.1 m/s agreement
            "Geometric vs analytical RIC velocity disagree by {vel_err:.6} km/s",
        );
    }

    /// Roundtrip: chief + RIC → deputy ECI → back to RIC
    #[test]
    fn deputy_eci_reconstruction_roundtrip() {
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 0.5,
            i_rad: chief_ke.i_rad + 0.0002,
            ..chief_ke
        };

        let epoch = test_epoch();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        // Forward: ECI → RIC
        let ric = eci_to_ric_relative(&chief_sv, &deputy_sv);

        // Inverse: RIC → ECI
        let deputy_reconstructed = ric_to_eci_state(&chief_sv, &ric);

        let pos_err = (deputy_reconstructed.position_eci_km - deputy_sv.position_eci_km).norm();
        let vel_err = (deputy_reconstructed.velocity_eci_km_s - deputy_sv.velocity_eci_km_s).norm();

        assert!(
            pos_err < 1e-12,
            "Deputy position reconstruction error = {pos_err} km"
        );
        assert!(
            vel_err < 1e-12,
            "Deputy velocity reconstruction error = {vel_err} km/s"
        );
    }

    /// R_hat ∥ position, C_hat ⊥ orbital plane at multiple mean anomaly values
    #[test]
    fn dcm_inclined_orbit() {
        let base = iss_like_elements();

        for u_deg in [0.0_f64, 45.0, 90.0, 180.0, 270.0, 315.0] {
            let ke = KeplerianElements {
                mean_anomaly_rad: u_deg.to_radians(),
                ..base
            };
            let sv = keplerian_to_state(&ke, test_epoch()).unwrap();
            let dcm = eci_to_ric_dcm(&sv);

            // Row 0 (R_hat) should be parallel to position
            let r_hat_from_dcm = Vector3::new(dcm[(0, 0)], dcm[(0, 1)], dcm[(0, 2)]);
            let r_hat_expected = sv.position_eci_km.normalize();
            let r_err = (r_hat_from_dcm - r_hat_expected).norm();
            assert!(
                r_err < 1e-14,
                "R_hat not parallel to position at u={u_deg}°, error={r_err}"
            );

            // Row 2 (C_hat) should be perpendicular to both r and v
            let c_hat = Vector3::new(dcm[(2, 0)], dcm[(2, 1)], dcm[(2, 2)]);
            let dot_r = c_hat.dot(&sv.position_eci_km);
            let dot_v = c_hat.dot(&sv.velocity_eci_km_s);
            assert!(
                dot_r.abs() < 1e-10,
                "C_hat not ⊥ to r at u={u_deg}°, dot={dot_r}"
            );
            assert!(
                dot_v.abs() < 1e-10,
                "C_hat not ⊥ to v at u={u_deg}°, dot={dot_v}"
            );
        }
    }

    /// Rotating-frame velocity correction ω×ρ should be physically reasonable:
    /// ~1 m/s at 1 km separation in LEO.
    ///
    /// We verify by comparing inertial-only vs corrected velocity: the difference
    /// (i.e., the ω×ρ term) should be on the order of n × 1 km ≈ 1 m/s.
    #[test]
    fn omega_correction_magnitude() {
        let chief_ke = iss_like_elements();
        let deputy_ke = KeplerianElements {
            a_km: chief_ke.a_km + 1.0, // +1 km SMA offset → ~1 km radial + drift
            ..chief_ke
        };
        let epoch = test_epoch();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();

        // Compute the ω×ρ correction magnitude directly
        let dcm = eci_to_ric_dcm(&chief_sv);
        let rho = dcm * (deputy_sv.position_eci_km - chief_sv.position_eci_km);
        let r = chief_sv.position_eci_km.norm();
        let h = chief_sv.position_eci_km.cross(&chief_sv.velocity_eci_km_s).norm();
        let omega = h / (r * r);
        let correction = Vector3::new(-omega * rho.y, omega * rho.x, 0.0);
        let correction_mag = correction.norm();

        // ω ≈ n ≈ 0.00114 rad/s for ISS orbit, ρ ~ 1 km → correction ~ 0.001 km/s
        // Should be between 0.0001 and 0.01 km/s (0.1 to 10 m/s)
        assert!(
            correction_mag < 0.01,
            "ω×ρ correction {correction_mag} km/s is too large for ~1 km separation"
        );
        assert!(
            correction_mag > 0.0001,
            "ω×ρ correction {correction_mag} km/s is suspiciously small for ~1 km separation"
        );
    }
}

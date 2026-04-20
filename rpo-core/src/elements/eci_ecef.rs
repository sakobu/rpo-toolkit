//! ECI ↔ ECEF frame transformations via the Earth Rotation Angle.
//!
//! Provides pure rotations about the (shared) Z axis between the Earth-
//! Centered Inertial frame (J2000) and the Earth-Centered Earth-Fixed
//! frame (body-fixed, co-rotating with Earth). Polar motion, nutation,
//! and precession since J2000 are **not** applied — see
//! [`crate::elements::earth_rotation`] for the precision regime.
//!
//! Velocity transforms include the Earth-rotation correction:
//! `v_ecef = R(-θ) · (v_eci − ω × r_eci)`.
//!
//! # References
//! - Vallado, D. A., *Fundamentals of Astrodynamics and Applications*,
//!   4e, Ch. 3 (rotating-frame velocity transforms).
//! - IERS TN 36 §5.5 Eq. 5.15 (ERA, used inside [`eci_to_ecef_dcm`]).

use nalgebra::Vector3;

use crate::constants::EARTH_ROTATION_RATE_RAD_S;
use crate::elements::earth_rotation::earth_rotation_angle_rad;
use crate::types::{EcefState, Matrix3, StateVector};

use hifitime::Epoch;

/// Rotation about the Z axis by angle `theta`, active on column vectors.
///
/// `Rz(θ) · v = [cos θ · v.x − sin θ · v.y, sin θ · v.x + cos θ · v.y, v.z]`.
fn rotation_z(theta: f64) -> Matrix3 {
    let (s, c) = theta.sin_cos();
    Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

/// ECI → ECEF direction cosine matrix at the given epoch.
///
/// Computed as `Rz(−ERA(epoch))`.
///
/// # Invariants
/// - Result is orthonormal with `det = +1` (proper rotation about z).
/// - `eci_to_ecef_dcm(epoch).transpose() == ecef_to_eci_dcm(epoch)` within
///   round-off.
///
/// # References
/// - Vallado Ch. 3.
#[must_use]
pub fn eci_to_ecef_dcm(epoch: Epoch) -> Matrix3 {
    rotation_z(-earth_rotation_angle_rad(epoch))
}

/// ECEF → ECI direction cosine matrix at the given epoch.
///
/// Computed as `Rz(+ERA(epoch))`; transpose of [`eci_to_ecef_dcm`].
///
/// # Invariants
/// - Result is orthonormal with `det = +1` (proper rotation about z).
///
/// # References
/// - Vallado Ch. 3.
#[must_use]
pub fn ecef_to_eci_dcm(epoch: Epoch) -> Matrix3 {
    rotation_z(earth_rotation_angle_rad(epoch))
}

/// Transform a position from ECI to ECEF at the given epoch.
///
/// # Invariants
/// - Magnitude is preserved (pure rotation): `|r_ecef| = |r_eci|`.
#[must_use]
pub fn eci_to_ecef_position_km(r_eci_km: &Vector3<f64>, epoch: Epoch) -> Vector3<f64> {
    EciEcefTransform::at(epoch).forward_position(r_eci_km)
}

/// Transform a position from ECEF to ECI at the given epoch.
///
/// # Invariants
/// - Magnitude is preserved (pure rotation): `|r_eci| = |r_ecef|`.
#[must_use]
pub fn ecef_to_eci_position_km(r_ecef_km: &Vector3<f64>, epoch: Epoch) -> Vector3<f64> {
    EciEcefTransform::at(epoch).inverse_position(r_ecef_km)
}

/// Transform a full ECI state (position + velocity) to ECEF.
///
/// Applies the Earth-rotation correction to the velocity:
///
/// ```text
/// v_ecef = R(-ERA) · (v_eci − ω × r_eci)
/// ```
///
/// where `ω = [0, 0, EARTH_ROTATION_RATE_RAD_S]` is along the shared z-axis of
/// both frames. This is the velocity a stationary ECEF observer measures.
///
/// # Invariants
/// - Position magnitude is preserved.
/// - The output `epoch` matches the input.
/// - Because `ω` is along the rotation axis, `R(θ)·(ω × r) = ω × R(θ)·r`,
///   so the equivalent forms `dcm·v_eci − ω × r_ecef` and
///   `dcm·(v_eci − ω × r_eci)` are mathematically identical.
///
/// # References
/// - Vallado Ch. 3 (rotating-frame velocity transforms).
#[must_use]
pub fn eci_to_ecef_state_km(state: &StateVector) -> EcefState {
    EciEcefTransform::at(state.epoch).forward_state(state)
}

/// Transform a full ECEF state to ECI. Inverse of [`eci_to_ecef_state_km`].
///
/// ```text
/// r_eci = R(+ERA) · r_ecef
/// v_eci = R(+ERA) · v_ecef + ω × r_eci
/// ```
///
/// # Invariants
/// - Position magnitude is preserved.
/// - The output `epoch` matches the input.
/// - Roundtrip with [`eci_to_ecef_state_km`] recovers the original state
///   within named tolerances.
///
/// # References
/// - Vallado Ch. 3.
#[must_use]
pub fn ecef_to_eci_state_km(state: &EcefState) -> StateVector {
    EciEcefTransform::at(state.epoch).inverse_state(state)
}

/// Cached ECI ↔ ECEF transform at a fixed epoch.
///
/// Amortizes the ERA polynomial + sin/cos across many state transforms at
/// the same epoch (e.g., a propagated trajectory rendered in the viewport).
/// Build once via [`EciEcefTransform::at`], then call forward/inverse
/// position/state methods O(1).
///
/// # Invariants
/// - `dcm_eci_to_ecef` is orthonormal with `det = +1`.
/// - State transforms apply the `ω × r` velocity correction with `ω` along
///   the shared z-axis with magnitude [`EARTH_ROTATION_RATE_RAD_S`].
///
/// # References
/// - Vallado, *Fundamentals of Astrodynamics*, 4e, Ch. 3.
/// - IERS TN 36 §5.5 Eq. 5.15 (ERA, used inside [`Self::at`]).
#[derive(Debug, Clone, Copy)]
pub struct EciEcefTransform {
    epoch: Epoch,
    dcm_eci_to_ecef: Matrix3,
    dcm_ecef_to_eci: Matrix3,
}

impl EciEcefTransform {
    /// Build the cached transform at `epoch` (one ERA + sin/cos).
    #[must_use]
    pub fn at(epoch: Epoch) -> Self {
        let theta = earth_rotation_angle_rad(epoch);
        let dcm_eci_to_ecef = rotation_z(-theta);
        // Transpose of a proper rotation is its inverse — explicit for clarity
        // and to avoid recomputing trig.
        let dcm_ecef_to_eci = dcm_eci_to_ecef.transpose();
        Self {
            epoch,
            dcm_eci_to_ecef,
            dcm_ecef_to_eci,
        }
    }

    /// Epoch this transform was built at.
    #[must_use]
    pub fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// ECI → ECEF direction cosine matrix.
    #[must_use]
    pub fn dcm_eci_to_ecef(&self) -> &Matrix3 {
        &self.dcm_eci_to_ecef
    }

    /// ECEF → ECI direction cosine matrix.
    #[must_use]
    pub fn dcm_ecef_to_eci(&self) -> &Matrix3 {
        &self.dcm_ecef_to_eci
    }

    /// Transform a position ECI → ECEF.
    #[must_use]
    pub fn forward_position(&self, r_eci_km: &Vector3<f64>) -> Vector3<f64> {
        self.dcm_eci_to_ecef * r_eci_km
    }

    /// Transform a position ECEF → ECI.
    #[must_use]
    pub fn inverse_position(&self, r_ecef_km: &Vector3<f64>) -> Vector3<f64> {
        self.dcm_ecef_to_eci * r_ecef_km
    }

    /// Transform a full ECI state to ECEF, applying the `ω × r` velocity
    /// correction.
    #[must_use]
    pub fn forward_state(&self, state: &StateVector) -> EcefState {
        let omega = Vector3::new(0.0, 0.0, EARTH_ROTATION_RATE_RAD_S);
        let r_eci = state.position_eci_km;
        let v_eci = state.velocity_eci_km_s;
        EcefState {
            epoch: state.epoch,
            position_ecef_km: self.dcm_eci_to_ecef * r_eci,
            velocity_ecef_km_s: self.dcm_eci_to_ecef * (v_eci - omega.cross(&r_eci)),
        }
    }

    /// Transform a full ECEF state to ECI; inverse of [`Self::forward_state`].
    #[must_use]
    pub fn inverse_state(&self, state: &EcefState) -> StateVector {
        let omega = Vector3::new(0.0, 0.0, EARTH_ROTATION_RATE_RAD_S);
        let r_ecef = state.position_ecef_km;
        let v_ecef = state.velocity_ecef_km_s;
        let r_eci = self.dcm_ecef_to_eci * r_ecef;
        StateVector {
            epoch: state.epoch,
            position_eci_km: r_eci,
            velocity_eci_km_s: self.dcm_ecef_to_eci * v_ecef + omega.cross(&r_eci),
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{
        ecef_to_eci_dcm, ecef_to_eci_position_km, ecef_to_eci_state_km, eci_to_ecef_dcm,
        eci_to_ecef_position_km, eci_to_ecef_state_km, EciEcefTransform,
    };
    use crate::constants::{EARTH_ROTATION_RATE_RAD_S, MU_EARTH, R_EARTH};
    use crate::test_helpers::{j2000_epoch, test_epoch};
    use crate::types::{EcefState, Matrix3, StateVector};

    // ---------------------------------------------------------------------
    // Named tolerance constants
    // ---------------------------------------------------------------------

    /// Geometry tolerance: ECI↔ECEF DCM orthogonality (`|R·Rᵀ − I|_∞`) and
    /// determinant residual. Two trig evals + one 3×3 multiply; ~f64 round-off.
    const DCM_ORTHOGONALITY_TOL: f64 = 1e-14;

    /// Test tolerance: position roundtrip (ECI → ECEF → ECI) at LEO km scale.
    /// Two matrix-vector multiplies; ~1e-12 km is achievable.
    const POSITION_ROUNDTRIP_TOL_KM: f64 = 1e-11;

    /// Test tolerance: velocity roundtrip at LEO km/s scale. Two matrix-
    /// vector multiplies + two cross products; ~1e-12 km/s.
    const VELOCITY_ROUNDTRIP_TOL_KM_S: f64 = 1e-12;

    /// Test tolerance: composition of `eci_to_ecef_dcm` with `ecef_to_eci_dcm`
    /// must equal identity to f64 round-off (single 3×3 multiply).
    const DCM_INVERSE_COMPOSITION_TOL: f64 = 1e-14;

    /// Test tolerance: `ω · R_EARTH` magnitude (~0.465 km/s) — single product
    /// of two well-known constants. 1e-9 km/s is conservative.
    const OMEGA_R_MAGNITUDE_TOL_KM_S: f64 = 1e-9;

    /// Test tolerance: bit-identical agreement between `EciEcefTransform`
    /// methods and the per-call free functions over a fixed test set.
    /// Both paths perform the same arithmetic; agreement should be exact.
    const BATCH_AGREEMENT_TOL: f64 = 1e-15;

    /// Test fixture: ECI position used for position-roundtrip tests.
    /// Magnitude ≈ 6900 km (LEO scale); not on any axis to exercise all
    /// three rotation matrix entries.
    fn sample_position_eci_km() -> Vector3<f64> {
        Vector3::new(5876.261, 3392.661, 1234.567)
    }

    /// Test state: 7000 km circular equatorial orbit at the standard 2024
    /// test epoch. Velocity derived from vis-viva: v = sqrt(μ / a).
    fn sample_leo_state() -> StateVector {
        let a_km = 7000.0;
        let v_circular_km_s = (MU_EARTH / a_km).sqrt();
        StateVector {
            epoch: test_epoch(),
            position_eci_km: Vector3::new(a_km, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, v_circular_km_s, 0.0),
        }
    }

    /// DCM must be orthonormal with determinant +1.
    #[test]
    fn dcm_is_orthonormal() {
        for epoch in [j2000_epoch(), test_epoch()] {
            let dcm = eci_to_ecef_dcm(epoch);
            let identity = Matrix3::identity();
            let err = (dcm * dcm.transpose() - identity).amax();
            assert!(
                err < DCM_ORTHOGONALITY_TOL,
                "DCM not orthonormal at {epoch}: |R·Rᵀ − I| = {err}"
            );
            let det_err = (dcm.determinant() - 1.0).abs();
            assert!(
                det_err < DCM_ORTHOGONALITY_TOL,
                "DCM det ≠ 1 at {epoch}: det = {}",
                dcm.determinant()
            );
        }
    }

    /// `ecef_to_eci_dcm` must be the transpose / inverse of `eci_to_ecef_dcm`.
    #[test]
    fn ecef_to_eci_dcm_is_inverse() {
        for epoch in [j2000_epoch(), test_epoch()] {
            let a = eci_to_ecef_dcm(epoch);
            let b = ecef_to_eci_dcm(epoch);
            let err = (a * b - Matrix3::identity()).amax();
            assert!(
                err < DCM_INVERSE_COMPOSITION_TOL,
                "ECI↔ECEF DCMs do not invert at {epoch}: err = {err}"
            );
        }
    }

    /// Position ECI → ECEF → ECI must round-trip to machine precision.
    #[test]
    fn position_roundtrip() {
        let epoch = test_epoch();
        let r_eci = sample_position_eci_km();
        let r_back = ecef_to_eci_position_km(&eci_to_ecef_position_km(&r_eci, epoch), epoch);
        let err = (r_back - r_eci).norm();
        assert!(
            err < POSITION_ROUNDTRIP_TOL_KM,
            "Position roundtrip err = {err} km for r_eci = {r_eci:?}"
        );
    }

    /// Full-state ECI → ECEF → ECI must round-trip in both position and
    /// velocity, confirming the ω × r correction is self-consistent.
    #[test]
    fn state_roundtrip_preserves_velocity() {
        let state = sample_leo_state();
        let ecef = eci_to_ecef_state_km(&state);
        let back = ecef_to_eci_state_km(&ecef);

        let pos_err = (back.position_eci_km - state.position_eci_km).norm();
        let vel_err = (back.velocity_eci_km_s - state.velocity_eci_km_s).norm();

        assert!(
            pos_err < POSITION_ROUNDTRIP_TOL_KM,
            "State pos roundtrip err = {pos_err} km"
        );
        assert!(
            vel_err < VELOCITY_ROUNDTRIP_TOL_KM_S,
            "State vel roundtrip err = {vel_err} km/s"
        );
    }

    /// A stationary ECEF observer (e.g., a ground station) at some position,
    /// when converted to ECI and back, must produce zero ECEF velocity.
    /// Confirms that `ω × r` is applied in the correct sense.
    #[test]
    fn stationary_ecef_has_zero_ecef_velocity_after_roundtrip() {
        let station_ecef = EcefState {
            epoch: test_epoch(),
            position_ecef_km: Vector3::new(R_EARTH, 0.0, 0.0),
            velocity_ecef_km_s: Vector3::zeros(),
        };
        let eci = ecef_to_eci_state_km(&station_ecef);
        let back = eci_to_ecef_state_km(&eci);
        let vel_err = back.velocity_ecef_km_s.norm();
        assert!(
            vel_err < VELOCITY_ROUNDTRIP_TOL_KM_S,
            "Stationary ECEF observer has non-zero velocity after ECEF→ECI→ECEF: {vel_err} km/s"
        );
    }

    /// Sanity check on `ω × r` magnitude: Earth's surface point at equator
    /// has ECI velocity magnitude ≈ ω · R ≈ 0.465 km/s.
    #[test]
    fn equator_surface_eci_velocity_magnitude() {
        let station_ecef = EcefState {
            epoch: test_epoch(),
            position_ecef_km: Vector3::new(R_EARTH, 0.0, 0.0),
            velocity_ecef_km_s: Vector3::zeros(),
        };
        let eci = ecef_to_eci_state_km(&station_ecef);
        let expected = EARTH_ROTATION_RATE_RAD_S * R_EARTH;
        let got = eci.velocity_eci_km_s.norm();
        let diff = (got - expected).abs();
        assert!(
            diff < OMEGA_R_MAGNITUDE_TOL_KM_S,
            "equator surface ECI |v| = {got} km/s, expected ~{expected} km/s (ω·R), diff = {diff}"
        );
    }

    /// `EciEcefTransform` produces bit-identical output to the per-call free
    /// functions for both position and full-state transforms. This guards
    /// against future drift where one path is updated and the other isn't.
    #[test]
    fn batch_transform_matches_per_call_free_functions() {
        let epoch = test_epoch();
        let xform = EciEcefTransform::at(epoch);

        // Position parity
        let r_eci = sample_position_eci_km();
        let p_batch = xform.forward_position(&r_eci);
        let p_free = eci_to_ecef_position_km(&r_eci, epoch);
        assert!(
            (p_batch - p_free).amax() < BATCH_AGREEMENT_TOL,
            "forward_position parity err = {}",
            (p_batch - p_free).amax()
        );

        let r_back_batch = xform.inverse_position(&p_batch);
        let r_back_free = ecef_to_eci_position_km(&p_free, epoch);
        assert!(
            (r_back_batch - r_back_free).amax() < BATCH_AGREEMENT_TOL,
            "inverse_position parity err = {}",
            (r_back_batch - r_back_free).amax()
        );

        // State parity
        let state = sample_leo_state();
        let s_batch = xform.forward_state(&state);
        let s_free = eci_to_ecef_state_km(&state);
        assert!(
            (s_batch.position_ecef_km - s_free.position_ecef_km).amax() < BATCH_AGREEMENT_TOL
        );
        assert!(
            (s_batch.velocity_ecef_km_s - s_free.velocity_ecef_km_s).amax() < BATCH_AGREEMENT_TOL
        );

        let back_batch = xform.inverse_state(&s_batch);
        let back_free = ecef_to_eci_state_km(&s_free);
        assert!(
            (back_batch.position_eci_km - back_free.position_eci_km).amax() < BATCH_AGREEMENT_TOL
        );
        assert!(
            (back_batch.velocity_eci_km_s - back_free.velocity_eci_km_s).amax()
                < BATCH_AGREEMENT_TOL
        );
    }

    /// `EciEcefTransform::dcm_eci_to_ecef` matches `eci_to_ecef_dcm` directly.
    #[test]
    fn batch_dcm_matches_free_function() {
        let epoch = test_epoch();
        let xform = EciEcefTransform::at(epoch);
        let direct = eci_to_ecef_dcm(epoch);
        let err = (xform.dcm_eci_to_ecef() - direct).amax();
        assert!(
            err < BATCH_AGREEMENT_TOL,
            "batch DCM disagrees with direct: err = {err}"
        );
    }

    /// `EciEcefTransform` round-trips an arbitrary state through forward
    /// then inverse, recovering the original within named tolerances.
    #[test]
    fn batch_state_roundtrip() {
        let xform = EciEcefTransform::at(test_epoch());
        let state = sample_leo_state();
        let ecef = xform.forward_state(&state);
        let back = xform.inverse_state(&ecef);
        let pos_err = (back.position_eci_km - state.position_eci_km).norm();
        let vel_err = (back.velocity_eci_km_s - state.velocity_eci_km_s).norm();
        assert!(pos_err < POSITION_ROUNDTRIP_TOL_KM, "pos err = {pos_err}");
        assert!(vel_err < VELOCITY_ROUNDTRIP_TOL_KM_S, "vel err = {vel_err}");
    }
}

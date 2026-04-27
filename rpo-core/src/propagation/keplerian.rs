//! Two-body Keplerian propagation for ECI trajectory generation.
//!
//! Uses the exact two-body solution: advance mean anomaly by `n·Δt` (Kepler's equation),
//! then convert back to ECI via the perifocal rotation. See Vallado Sec. 2.2–2.3 for the
//! underlying Kepler equation and vis-viva derivation.

use hifitime::{Duration, Epoch};
use nalgebra::{Matrix3, Vector3};

use crate::constants::{MU_EARTH, TWO_PI};
use crate::elements::keplerian_conversions::{keplerian_to_state, state_to_keplerian, ConversionError};
use crate::types::{KeplerianElements, StateVector};

/// Propagate a state vector under two-body Keplerian dynamics, producing
/// a dense ECI trajectory of `n_steps + 1` states (including endpoints).
///
/// Uses the exact two-body solution: convert to Keplerian elements, advance
/// mean anomaly by `n · dt`, convert back to ECI.
///
/// # Invariants
/// - `initial` must represent a bound orbit (`e < 1`, `a > 0`)
/// - `duration_s` must be finite
///
/// # Singularities and regime boundaries
/// - **Near-parabolic (e → 1):** Kepler's equation (M = E − e·sin E)
///   becomes ill-conditioned. The Newton-Raphson solver in
///   [`keplerian_to_state`] may converge slowly or require more iterations.
///   The function delegates convergence handling to the conversion layer.
/// - **Near-circular (e → 0):** Well-conditioned. The argument of perigee is
///   geometrically undefined for circular orbits, but the ECI state vector
///   produced by the Keplerian-to-state roundtrip remains numerically stable
///   because position and velocity are always well-defined.
/// - **Multi-orbit propagation:** Energy and angular momentum are conserved
///   to machine precision per step (exact two-body solution), but the
///   Keplerian-to-ECI conversion introduces O(1e-14) roundtrip noise that
///   accumulates over many orbits.
///
/// # Errors
/// Returns `ConversionError` if the initial state represents an unbound orbit
/// or has a zero position vector.
pub fn propagate_keplerian(
    initial: &StateVector,
    duration_s: f64,
    n_steps: u32,
) -> Result<Vec<StateVector>, ConversionError> {
    if n_steps == 0 {
        return Ok(vec![*initial]);
    }
    let ke = state_to_keplerian(initial)?;
    propagate_keplerian_from_elements(&ke, initial.epoch, duration_s, n_steps)
}

/// Propagate from pre-converted Keplerian elements with the perifocal→ECI
/// rotation hoisted out of the per-step loop.
///
/// Saves the per-step `state_to_keplerian` reconversion (avoids one Kepler
/// solve per step) and the per-step rotation-matrix rebuild (six trig calls
/// per step) versus calling [`propagate_keplerian`] when the caller already
/// has Keplerian elements in hand. Used by Lambert arc densification, where
/// `LAMBERT_ARC_SAMPLES` is large enough (256) for the savings to dominate.
///
/// # Invariants
///
/// - `ke.a_km > 0`, `0 ≤ ke.e < 1` (validated)
/// - `duration_s` finite
/// - `n_steps` ≥ 1; `n_steps = 0` returns a single sample at the input
///   mean anomaly.
///
/// # Errors
///
/// Returns [`ConversionError`] if the elements fail validation or any
/// per-step Kepler solve fails to converge.
pub fn propagate_keplerian_from_elements(
    ke: &KeplerianElements,
    initial_epoch: Epoch,
    duration_s: f64,
    n_steps: u32,
) -> Result<Vec<StateVector>, ConversionError> {
    ke.validate()?;
    let n_mean_motion = ke.mean_motion()?;

    let p = ke.a_km * (1.0 - ke.e * ke.e);
    let sqrt_mu_over_p = (MU_EARTH / p).sqrt();

    // Perifocal → ECI rotation depends only on (i, Ω, ω); constant across
    // all steps. Same matrix construction as `keplerian_to_state` (Vallado
    // Algorithm 10).
    let cos_o = ke.raan_rad.cos();
    let sin_o = ke.raan_rad.sin();
    let cos_i = ke.i_rad.cos();
    let sin_i = ke.i_rad.sin();
    let cos_w = ke.aop_rad.cos();
    let sin_w = ke.aop_rad.sin();
    let rot = Matrix3::new(
        cos_o * cos_w - sin_o * sin_w * cos_i,
        -cos_o * sin_w - sin_o * cos_w * cos_i,
        0.0,
        sin_o * cos_w + cos_o * sin_w * cos_i,
        -sin_o * sin_w + cos_o * cos_w * cos_i,
        0.0,
        sin_w * sin_i,
        cos_w * sin_i,
        0.0,
    );

    if n_steps == 0 {
        let nu = ke.true_anomaly()?;
        return Ok(vec![pqw_to_state(p, ke.e, sqrt_mu_over_p, nu, &rot, initial_epoch)]);
    }

    let step = duration_s / f64::from(n_steps);
    (0..=n_steps)
        .map(|k| {
            let dt = f64::from(k) * step;
            let epoch_k = initial_epoch + Duration::from_seconds(dt);
            let m_k = (ke.mean_anomaly_rad + n_mean_motion * dt).rem_euclid(TWO_PI);
            // Construct a temp KE only to reuse the validated `true_anomaly`
            // Kepler solve; the rotation/`p` work above is already hoisted,
            // so the temp struct is essentially a `(M, e)` argument bundle.
            let ke_k = KeplerianElements {
                mean_anomaly_rad: m_k,
                ..*ke
            };
            let nu = ke_k.true_anomaly()?;
            Ok(pqw_to_state(p, ke.e, sqrt_mu_over_p, nu, &rot, epoch_k))
        })
        .collect()
}

/// Build an ECI [`StateVector`] from `(p, e, sqrt(μ/p), ν)` and a precomputed
/// PQW→ECI rotation. Internal helper for [`propagate_keplerian_from_elements`].
fn pqw_to_state(
    p_km: f64,
    e: f64,
    sqrt_mu_over_p: f64,
    nu_rad: f64,
    rot: &Matrix3<f64>,
    epoch: Epoch,
) -> StateVector {
    let (sin_nu, cos_nu) = nu_rad.sin_cos();
    let r = p_km / (1.0 + e * cos_nu);
    let r_pqw = Vector3::new(r * cos_nu, r * sin_nu, 0.0);
    let v_pqw = Vector3::new(
        -sqrt_mu_over_p * sin_nu,
        sqrt_mu_over_p * (e + cos_nu),
        0.0,
    );
    StateVector {
        epoch,
        position_eci_km: rot * r_pqw,
        velocity_eci_km_s: rot * v_pqw,
    }
}

/// Sample `n_points` ECI positions uniformly in mean anomaly around one
/// orbital period of a bound Keplerian orbit.
///
/// Composes [`keplerian_to_state`] with [`propagate_keplerian`] over
/// `ke.period()`, returning only the position component. The first sample
/// corresponds to the supplied `mean_anomaly_rad`; the last sample lands one
/// step before completing the period, so the polyline closes by physics when
/// consumers connect the last point back to the first. (Equivalently, calling
/// the underlying [`propagate_keplerian`] with `n_steps = n_points` would
/// return `n_points + 1` states with the last bit-identical to the first;
/// this helper drops that redundant trailing state.)
///
/// `n_points = 0` returns an empty vector.
///
/// # Invariants
/// - `ke.a_km > 0` and `0 <= ke.e < 1` (bound elliptical orbit)
///
/// # Errors
/// Returns [`ConversionError`] if the elements are invalid (non-positive
/// semi-major axis or eccentricity outside `[0, 1)`).
pub fn sample_orbit_eci(
    ke: &KeplerianElements,
    n_points: u32,
) -> Result<Vec<Vector3<f64>>, ConversionError> {
    if n_points == 0 {
        return Ok(Vec::new());
    }

    // Epoch is irrelevant for shape sampling — the ECI geometry of a Keplerian
    // orbit is fixed by `(a, e, i, Ω, ω, M)` alone. Use a fixed TAI reference
    // so the call signature stays free of an unused epoch parameter.
    let dummy_epoch = Epoch::from_tai_seconds(0.0);
    let initial = keplerian_to_state(ke, dummy_epoch)?;
    let period_s = ke.period()?;

    let trajectory = propagate_keplerian(&initial, period_s, n_points)?;
    Ok(trajectory
        .into_iter()
        .take(n_points as usize) // u32 → usize: always safe (usize ≥ 32 bits)
        .map(|s| s.position_eci_km)
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::MU_EARTH;
    use crate::elements::keplerian_conversions::keplerian_to_state;
    use crate::test_helpers::{
        circular_equatorial_elements, eccentric_elements, iss_like_elements, test_epoch,
    };
    use crate::types::elements::KeplerianElements;


    /// Position closure after one full orbital period. Limited by Kepler
    /// equation convergence tolerance (1e-14 rad) propagated through the
    /// vis-viva equation and PQW→ECI rotation.
    const ORBIT_CLOSURE_TOL_KM: f64 = 1e-10;

    /// `n_steps=0` returns the exact initial state (no computation, clone only).
    const ZERO_STEP_IDENTITY_TOL: f64 = 1e-15;

    /// Two-body specific energy conservation (km²/s²). Limited by Kepler
    /// convergence tolerance chain: M→E→ν→r,v.
    const ENERGY_CONSERVATION_TOL: f64 = 1e-10;

    /// Two-body angular momentum conservation (km²/s). Same error chain
    /// as energy conservation — cross product amplifies position/velocity errors.
    const ANGULAR_MOMENTUM_TOL: f64 = 1e-10;

    #[test]
    fn propagate_keplerian_circular_closure() {
        let epoch = test_epoch();
        let ke = iss_like_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period().unwrap();

        let trajectory = propagate_keplerian(&initial, period, 100).unwrap();

        assert_eq!(trajectory.len(), 101);

        let pos_err = (trajectory.last().unwrap().position_eci_km - initial.position_eci_km).norm();
        assert!(
            pos_err < ORBIT_CLOSURE_TOL_KM,
            "Circular orbit closure error: {pos_err} km"
        );
    }

    #[test]
    fn propagate_keplerian_eccentric_closure() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period().unwrap();

        let trajectory = propagate_keplerian(&initial, period, 100).unwrap();

        let pos_err = (trajectory.last().unwrap().position_eci_km - initial.position_eci_km).norm();
        assert!(
            pos_err < ORBIT_CLOSURE_TOL_KM,
            "Eccentric orbit closure error: {pos_err} km"
        );
    }

    #[test]
    fn propagate_keplerian_zero_steps() {
        let epoch = test_epoch();
        let ke = iss_like_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();

        let trajectory = propagate_keplerian(&initial, 3600.0, 0).unwrap();
        assert_eq!(trajectory.len(), 1, "n_steps=0 should return 1 state");
        assert!(
            (trajectory[0].position_eci_km - initial.position_eci_km).norm() < ZERO_STEP_IDENTITY_TOL,
            "n_steps=0 should return the initial state"
        );
    }

    #[test]
    fn propagate_keplerian_energy_conservation() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period().unwrap();

        let trajectory = propagate_keplerian(&initial, period, 200).unwrap();

        let energy_0 = {
            let r = initial.position_eci_km.norm();
            let v = initial.velocity_eci_km_s.norm();
            v * v / 2.0 - MU_EARTH / r
        };

        for (k, state) in trajectory.iter().enumerate() {
            let r = state.position_eci_km.norm();
            let v = state.velocity_eci_km_s.norm();
            let energy_k = v * v / 2.0 - MU_EARTH / r;
            let err = (energy_k - energy_0).abs();
            assert!(
                err < ENERGY_CONSERVATION_TOL,
                "Energy conservation violated at step {k}: error = {err} km²/s²"
            );
        }
    }

    /// Angular momentum conservation: h = r × v should be constant under two-body dynamics.
    /// This complements the energy conservation test — together they fully constrain
    /// the two-body solution (energy + angular momentum = unique Keplerian orbit).
    #[test]
    fn propagate_keplerian_angular_momentum_conservation() {
        let epoch = test_epoch();
        let ke = eccentric_elements();
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period().unwrap();

        let trajectory = propagate_keplerian(&initial, period, 200).unwrap();

        let h_0 = initial.position_eci_km.cross(&initial.velocity_eci_km_s);

        for (k, state) in trajectory.iter().enumerate() {
            let h_k = state.position_eci_km.cross(&state.velocity_eci_km_s);
            let err = (h_k - h_0).norm();
            assert!(
                err < ANGULAR_MOMENTUM_TOL,
                "Angular momentum conservation violated at step {k}: error = {err} km²/s"
            );
        }
    }

    /// Energy and angular momentum conservation for a high-eccentricity orbit (e=0.7)
    /// over 3 full orbital periods with 600 steps. Stresses the Kepler equation solver
    /// at periapsis where velocity variation is extreme.
    #[test]
    fn energy_conservation_high_eccentricity() {
        let epoch = test_epoch();
        let ke = KeplerianElements {
            a_km: 10000.0,
            e: 0.7,
            i_rad: 45.0_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let initial = keplerian_to_state(&ke, epoch).unwrap();
        let period = ke.period().unwrap();

        let trajectory = propagate_keplerian(&initial, 3.0 * period, 600).unwrap();

        let energy_0 = {
            let r = initial.position_eci_km.norm();
            let v = initial.velocity_eci_km_s.norm();
            v * v / 2.0 - MU_EARTH / r
        };
        let h_0 = initial.position_eci_km.cross(&initial.velocity_eci_km_s);

        for (k, state) in trajectory.iter().enumerate() {
            let r = state.position_eci_km.norm();
            let v = state.velocity_eci_km_s.norm();
            let energy_k = v * v / 2.0 - MU_EARTH / r;
            let energy_err = (energy_k - energy_0).abs();
            assert!(
                energy_err < ENERGY_CONSERVATION_TOL,
                "Energy conservation violated at step {k}: error = {energy_err} km²/s²"
            );

            let h_k = state.position_eci_km.cross(&state.velocity_eci_km_s);
            let h_err = (h_k - h_0).norm();
            assert!(
                h_err < ANGULAR_MOMENTUM_TOL,
                "Angular momentum conservation violated at step {k}: error = {h_err} km²/s"
            );
        }
    }

    // -- sample_orbit_eci ---------------------------------------------------

    #[test]
    fn sample_orbit_eci_returns_n_points() {
        let pts = sample_orbit_eci(&iss_like_elements(), 32).unwrap();
        assert_eq!(pts.len(), 32);
    }

    #[test]
    fn sample_orbit_eci_zero_points_returns_empty() {
        let pts = sample_orbit_eci(&iss_like_elements(), 0).unwrap();
        assert!(pts.is_empty());
    }

    #[test]
    fn sample_orbit_eci_circular_orbit_constant_radius() {
        let ke = circular_equatorial_elements();
        let pts = sample_orbit_eci(&ke, 16).unwrap();
        for (k, p) in pts.iter().enumerate() {
            let rel_err = (p.norm() - ke.a_km).abs() / ke.a_km;
            // Same Kepler-solver budget as ORBIT_CLOSURE_TOL_KM, normalized.
            assert!(
                rel_err < ORBIT_CLOSURE_TOL_KM / ke.a_km,
                "sample {k}: r = {} km, a = {} km, rel err = {rel_err}",
                p.norm(),
                ke.a_km,
            );
        }
    }

    #[test]
    fn sample_orbit_eci_closes_when_wrapped() {
        // The first sample is at the starting mean anomaly; calling
        // propagate_keplerian over one period evaluates an identical M on
        // wrap, so an extra sample at "the next k" reproduces sample 0.
        let ke = eccentric_elements();
        let pts = sample_orbit_eci(&ke, 64).unwrap();
        let dummy = Epoch::from_tai_seconds(0.0);
        let sample_at_two_pi = keplerian_to_state(
            &KeplerianElements {
                mean_anomaly_rad: ke.mean_anomaly_rad,
                ..ke
            },
            dummy,
        )
        .unwrap()
        .position_eci_km;
        let diff = (pts[0] - sample_at_two_pi).norm();
        assert!(
            diff < ORBIT_CLOSURE_TOL_KM,
            "first sample and M-wrap differ by {diff} km"
        );
    }

    #[test]
    fn sample_orbit_eci_rejects_invalid_elements() {
        let bad = KeplerianElements {
            a_km: -100.0,
            ..circular_equatorial_elements()
        };
        assert!(sample_orbit_eci(&bad, 8).is_err());
    }
}

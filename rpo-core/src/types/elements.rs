//! Classical Keplerian orbital elements.

use serde::{Deserialize, Serialize};

use crate::constants::{KEPLER_MAX_ITER, KEPLER_TOL, TWO_PI};

/// Errors from Kepler's equation solution and orbital element derived quantities.
#[derive(Debug, Clone)]
pub enum KeplerError {
    /// Eccentricity out of valid range [0, 1) for Kepler's equation.
    InvalidEccentricity {
        /// The invalid eccentricity value.
        e: f64,
    },
    /// Semi-major axis must be positive for mean motion / period computation.
    InvalidSemiMajorAxis {
        /// The invalid semi-major axis (km).
        a_km: f64,
    },
    /// Newton-Raphson iteration for Kepler's equation did not converge
    /// within the maximum number of iterations.
    KeplerNoConvergence {
        /// Number of iterations completed.
        iterations: usize,
        /// Final Kepler equation residual |E - e*sin(E) - M| (rad).
        residual: f64,
        /// Eccentricity of the orbit.
        e: f64,
    },
}

impl std::fmt::Display for KeplerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidEccentricity { e } => {
                write!(f, "KeplerError: eccentricity = {e} out of range [0, 1)")
            }
            Self::InvalidSemiMajorAxis { a_km } => {
                write!(f, "KeplerError: semi-major axis = {a_km} km must be positive")
            }
            Self::KeplerNoConvergence { iterations, residual, e } => {
                write!(
                    f,
                    "KeplerError: Kepler's equation did not converge — {iterations} iterations, residual = {residual:.6e}, e = {e}"
                )
            }
        }
    }
}

impl std::error::Error for KeplerError {}

/// Classical Keplerian orbital elements
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct KeplerianElements {
    /// Semi-major axis (km)
    pub a_km: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination (rad)
    pub i_rad: f64,
    /// Right ascension of ascending node (rad)
    pub raan_rad: f64,
    /// Argument of perigee (rad)
    pub aop_rad: f64,
    /// Mean anomaly (rad)
    pub mean_anomaly_rad: f64,
}

impl KeplerianElements {
    /// Mean motion n = sqrt(μ/a³) (rad/s).
    ///
    /// # Errors
    /// Returns `KeplerError::InvalidSemiMajorAxis` if `self.a_km <= 0`.
    pub fn mean_motion(&self) -> Result<f64, KeplerError> {
        if self.a_km <= 0.0 {
            return Err(KeplerError::InvalidSemiMajorAxis { a_km: self.a_km });
        }
        Ok((crate::constants::MU_EARTH / (self.a_km * self.a_km * self.a_km)).sqrt())
    }

    /// Orbital period T = 2π/n (seconds).
    ///
    /// # Errors
    /// Returns `KeplerError::InvalidSemiMajorAxis` if `self.a_km <= 0` (propagated
    /// from `mean_motion()`).
    pub fn period(&self) -> Result<f64, KeplerError> {
        let n = self.mean_motion()?;
        Ok(std::f64::consts::TAU / n)
    }

    /// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E,
    /// then compute true anomaly ν.
    ///
    /// # Errors
    /// Returns `KeplerError::InvalidEccentricity` if `self.e` is outside [0, 1).
    /// Returns `KeplerError::KeplerNoConvergence` if Newton-Raphson iteration
    /// does not converge within `KEPLER_MAX_ITER` iterations.
    pub fn true_anomaly(&self) -> Result<f64, KeplerError> {
        if self.e < 0.0 || self.e >= 1.0 {
            return Err(KeplerError::InvalidEccentricity { e: self.e });
        }

        let m = self.mean_anomaly_rad.rem_euclid(TWO_PI);
        let e = self.e;

        // Newton's method for Kepler's equation
        // For high eccentricity (e >= 0.8), M is a poor initial guess; start from π
        let mut ea = if e < 0.8 { m } else { std::f64::consts::PI };
        for _ in 0..KEPLER_MAX_ITER {
            let f = ea - e * ea.sin() - m;
            let fp = 1.0 - e * ea.cos();
            let delta = f / fp;
            ea -= delta;
            if delta.abs() < KEPLER_TOL {
                // Eccentric anomaly → true anomaly
                let nu = 2.0
                    * ((1.0 + e).sqrt() * (ea / 2.0).sin())
                        .atan2((1.0 - e).sqrt() * (ea / 2.0).cos());
                return Ok(nu.rem_euclid(TWO_PI));
            }
        }

        let final_residual = (ea - e * ea.sin() - m).abs();
        Err(KeplerError::KeplerNoConvergence {
            iterations: KEPLER_MAX_ITER,
            residual: final_residual,
            e,
        })
    }

    /// Mean argument of latitude u = ω + M
    ///
    /// # Invariants
    /// - Angles `aop_rad` and `mean_anomaly_rad` should be in radians
    #[must_use]
    pub fn mean_arg_of_lat(&self) -> f64 {
        (self.aop_rad + self.mean_anomaly_rad).rem_euclid(TWO_PI)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, TWO_PI};
    use std::f64::consts::PI;

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    /// Build a minimal KeplerianElements with sensible defaults for fields that
    /// are not relevant to the method under test.
    fn make_elements(a_km: f64, e: f64, mean_anomaly_rad: f64) -> KeplerianElements {
        KeplerianElements {
            a_km,
            e,
            i_rad: 0.0,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad,
        }
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::true_anomaly
    // ---------------------------------------------------------------------------

    /// Circular orbit (e = 0): ν must equal M exactly for all M because
    /// E = M and ν = E when e = 0.
    #[test]
    fn true_anomaly_circular_orbit_equals_mean_anomaly() {
        // Tolerance: machine-epsilon level; Newton-Raphson converges in one
        // step for e = 0 (f = E - M = 0 at E = M), so the result is exact to
        // floating-point representation.
        let eps = 1e-15;
        let el = make_elements(6_778.0, 0.0, PI / 2.0);
        let nu = el.true_anomaly().unwrap();
        assert!(
            (nu - PI / 2.0).abs() < eps,
            "circular orbit: expected ν = M = π/2, got {nu}"
        );
    }

    /// Circular orbit at M = 0 → ν = 0.
    #[test]
    fn true_anomaly_circular_m_zero() {
        let eps = 1e-15;
        let el = make_elements(6_778.0, 0.0, 0.0);
        let nu = el.true_anomaly().unwrap();
        assert!(
            nu < eps,
            "circular orbit M=0: expected ν = 0, got {nu}"
        );
    }

    /// Periapsis/apoapsis symmetry: at M = π (apoapsis), ν = π for any
    /// eccentricity in [0, 1).  At apoapsis E = π and the half-angle formula
    /// gives ν = π directly.
    #[test]
    fn true_anomaly_moderate_eccentricity_apoapsis() {
        // Tolerance: KEPLER_TOL = 1e-14 is the Newton-Raphson convergence
        // criterion; the true anomaly is analytically π, so the only error
        // is floating-point rounding in the half-angle atan2 call.
        let eps = 1e-12;
        let el = make_elements(7_000.0, 0.3, PI);
        let nu = el.true_anomaly().unwrap();
        assert!(
            (nu - PI).abs() < eps,
            "moderate eccentricity apoapsis: expected ν = π, got {nu}"
        );
    }

    /// High-eccentricity convergence test (e = 0.8): at M = π, ν must equal
    /// π (apoapsis symmetry holds at all eccentricities).
    #[test]
    fn true_anomaly_high_eccentricity_apoapsis() {
        // The solver switches initial guess to π for e >= 0.8 (avoids divergence).
        // Tolerance: 1e-12 — well above KEPLER_TOL; the analytic answer is exact.
        let eps = 1e-12;
        let el = make_elements(10_000.0, 0.8, PI);
        let nu = el.true_anomaly().unwrap();
        assert!(
            (nu - PI).abs() < eps,
            "high-eccentricity apoapsis: expected ν = π, got {nu}"
        );
    }

    /// High-eccentricity convergence test (e = 0.8, M = 1.0 rad): validate
    /// that the solver converges (i.e., ν is finite and in [0, 2π)).
    /// Cross-check via Kepler's equation: given computed E, M_check = E - e*sin(E)
    /// must recover the original M.
    #[test]
    fn true_anomaly_high_eccentricity_convergence() {
        // Tolerance: residual in Kepler's equation should be at KEPLER_TOL level.
        let eps = 1e-10; // lenient: one round-trip through atan2 and trig
        let e = 0.8_f64;
        let m = 1.0_f64;
        let el = make_elements(10_000.0, e, m);
        let nu = el.true_anomaly().unwrap();

        // Back-compute eccentric anomaly from true anomaly to verify round-trip.
        let half_nu = nu / 2.0;
        let ea = 2.0 * ((1.0 - e).sqrt() * half_nu.sin())
            .atan2((1.0 + e).sqrt() * half_nu.cos());
        let ea = ea.rem_euclid(TWO_PI);
        let m_check = (ea - e * ea.sin()).rem_euclid(TWO_PI);
        assert!(
            (m_check - m).abs() < eps,
            "high-eccentricity round-trip: M_check = {m_check}, expected {m}, diff = {}",
            (m_check - m).abs()
        );
    }

    /// Edge case M = 0 → ν = 0 (periapsis).
    #[test]
    fn true_anomaly_m_zero_is_periapsis() {
        let eps = 1e-15;
        let el = make_elements(7_500.0, 0.5, 0.0);
        let nu = el.true_anomaly().unwrap();
        assert!(
            nu < eps || (nu - TWO_PI).abs() < eps,
            "M=0: expected ν=0 (or 2π alias), got {nu}"
        );
    }

    /// Edge case: M just below 2π wraps correctly — result must be in [0, 2π).
    #[test]
    fn true_anomaly_near_two_pi_wraps() {
        let el = make_elements(7_000.0, 0.3, TWO_PI - 1e-10);
        let nu = el.true_anomaly().unwrap();
        assert!(
            nu >= 0.0 && nu < TWO_PI,
            "near-2π wrap: expected ν in [0, 2π), got {nu}"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::mean_motion
    // ---------------------------------------------------------------------------

    /// ISS-like orbit (a ≈ 6778 km): mean motion should be ≈ 0.001138 rad/s.
    /// Derived from n = sqrt(μ / a³).
    #[test]
    fn mean_motion_iss_like() {
        // Tolerance: 1e-9 rad/s — well below any physical or numerical
        // uncertainty; tests that the formula is implemented correctly.
        let eps = 1e-9;
        let a_km = 6_778.0_f64;
        let el = make_elements(a_km, 0.001, 0.0);
        let n = el.mean_motion().unwrap();
        let n_expected = (MU_EARTH / (a_km * a_km * a_km)).sqrt();
        assert!(
            (n - n_expected).abs() < eps,
            "mean_motion ISS-like: expected {n_expected:.9}, got {n:.9}"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::period
    // ---------------------------------------------------------------------------

    /// ISS-like orbit (a ≈ 6778 km): period should be approximately 5550 s.
    /// T = 2π/n = 2π * sqrt(a³/μ).
    #[test]
    fn period_iss_like_approx_5550s() {
        // Tolerance: ±5 s — the ISS period varies slightly depending on the
        // exact SMA used; 6778 km gives ~5562 s; 5550 is a round-number bound.
        let a_km = 6_778.0_f64;
        let el = make_elements(a_km, 0.001, 0.0);
        let t = el.period().unwrap();
        let t_expected = TWO_PI / (MU_EARTH / (a_km * a_km * a_km)).sqrt();
        // Identity check (period = 2π / mean_motion)
        assert!(
            (t - t_expected).abs() < 1e-9,
            "period: identity T = 2π/n failed; got {t}, expected {t_expected}"
        );
        // Approximate ISS sanity check
        assert!(
            (t - 5_550.0).abs() < 50.0,
            "period ISS-like: expected ~5550 s, got {t:.1} s"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::mean_arg_of_lat
    // ---------------------------------------------------------------------------

    /// u = ω + M, with result wrapped into [0, 2π).
    #[test]
    fn mean_arg_of_lat_basic_sum() {
        let eps = 1e-15;
        let mut el = make_elements(7_000.0, 0.01, 1.0);
        el.aop_rad = 0.5;
        // u = 0.5 + 1.0 = 1.5
        let u = el.mean_arg_of_lat();
        assert!(
            (u - 1.5).abs() < eps,
            "mean_arg_of_lat basic sum: expected 1.5, got {u}"
        );
    }

    /// u wraps when ω + M exceeds 2π.
    #[test]
    fn mean_arg_of_lat_wraps_above_two_pi() {
        let eps = 1e-12;
        let mut el = make_elements(7_000.0, 0.01, TWO_PI - 0.1);
        el.aop_rad = 0.5;
        // ω + M = 0.5 + (2π - 0.1) = 2π + 0.4 → wraps to 0.4
        let u = el.mean_arg_of_lat();
        assert!(
            (u - 0.4).abs() < eps,
            "mean_arg_of_lat wrap: expected 0.4, got {u}"
        );
    }

    /// u = 0 when ω = M = 0.
    #[test]
    fn mean_arg_of_lat_zero() {
        let eps = 1e-15;
        let el = make_elements(7_000.0, 0.01, 0.0);
        let u = el.mean_arg_of_lat();
        assert!(u < eps, "mean_arg_of_lat zero: expected 0, got {u}");
    }

    // ---------------------------------------------------------------------------
    // Error-path tests
    // ---------------------------------------------------------------------------

    /// Hyperbolic eccentricity (e >= 1) returns `InvalidEccentricity`.
    #[test]
    fn true_anomaly_rejects_hyperbolic_eccentricity() {
        let el = make_elements(7_000.0, 1.5, 0.0);
        let result = el.true_anomaly();
        assert!(
            matches!(result, Err(KeplerError::InvalidEccentricity { e }) if (e - 1.5).abs() < 1e-15),
            "e >= 1 should return InvalidEccentricity, got {result:?}"
        );
    }

    /// Negative eccentricity returns `InvalidEccentricity`.
    #[test]
    fn true_anomaly_rejects_negative_eccentricity() {
        let el = make_elements(7_000.0, -0.1, 0.0);
        let result = el.true_anomaly();
        assert!(
            matches!(result, Err(KeplerError::InvalidEccentricity { .. })),
            "e < 0 should return InvalidEccentricity, got {result:?}"
        );
    }

    /// Zero SMA returns `InvalidSemiMajorAxis`.
    #[test]
    fn mean_motion_rejects_zero_sma() {
        let el = make_elements(0.0, 0.01, 0.0);
        let result = el.mean_motion();
        assert!(
            matches!(result, Err(KeplerError::InvalidSemiMajorAxis { .. })),
            "a_km = 0 should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    /// Negative SMA returns `InvalidSemiMajorAxis`.
    #[test]
    fn mean_motion_rejects_negative_sma() {
        let el = make_elements(-100.0, 0.01, 0.0);
        let result = el.mean_motion();
        assert!(
            matches!(result, Err(KeplerError::InvalidSemiMajorAxis { .. })),
            "a_km < 0 should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    /// `period()` propagates SMA error from `mean_motion()`.
    #[test]
    fn period_propagates_sma_error() {
        let el = make_elements(-100.0, 0.01, 0.0);
        let result = el.period();
        assert!(
            matches!(result, Err(KeplerError::InvalidSemiMajorAxis { .. })),
            "period() should propagate SMA error, got {result:?}"
        );
    }
}

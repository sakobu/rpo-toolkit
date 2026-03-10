//! J2 perturbation parameters for secular rate computation (Koenig Eqs. 13-16).

use serde::{Deserialize, Serialize};

use crate::constants::{J2, R_EARTH};
use crate::types::KeplerianElements;

/// Precomputed J2 perturbation parameters for a mean orbit.
///
/// Contains auxiliary quantities and secular rates needed by the
/// J2-perturbed state transition matrix (Koenig Eqs. 13-16).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct J2Params {
    /// Mean motion n = sqrt(μ/a³) (rad/s)
    pub n: f64,
    /// J2 perturbation frequency parameter κ = `(3/4)·n·J2·(R_E/p)²`
    pub kappa: f64,
    /// η = sqrt(1 - e²)
    pub eta: f64,
    /// E = 1 + η (Koenig Eq. 14)
    pub big_e: f64,
    /// F = 4 + 3η (Koenig Eq. 14) — dimensionless auxiliary
    pub big_f: f64,
    /// G = 1/(η²·(1+η)) (Koenig Eq. 14) — dimensionless auxiliary
    pub big_g: f64,
    /// Eccentricity vector x-component: ex = e·cos(ω)
    pub ex: f64,
    /// Eccentricity vector y-component: ey = e·sin(ω)
    pub ey: f64,
    /// P = 3cos²i - 1 (Koenig Eq. 15)
    pub big_p: f64,
    /// Q = 5cos²i - 1 (Koenig Eq. 15)
    pub big_q: f64,
    /// R = cos(i) (Koenig Eq. 15)
    pub big_r: f64,
    /// S = sin(2i) (Koenig Eq. 15)
    pub big_s: f64,
    /// T = sin²(i) (Koenig Eq. 15)
    pub big_t: f64,
    /// cos(i)
    pub cos_i: f64,
    /// sin(i)
    pub sin_i: f64,
    /// cos²(i)
    pub cos2_i: f64,
    /// sin²(i)
    pub sin2_i: f64,
    /// Secular rate of RAAN (rad/s): dΩ/dt = -2κR
    pub raan_dot: f64,
    /// Secular rate of argument of perigee (rad/s): dω/dt = κQ
    pub aop_dot: f64,
    /// Secular rate of mean anomaly (rad/s): dM/dt = n + κηP
    pub m_dot: f64,
    /// Semi-latus rectum p = a(1 - e²)
    pub p: f64,
}

/// Compute J2 perturbation parameters from mean Keplerian elements.
///
/// Implements Koenig Eqs. 13-16 for secular J2 rates and auxiliary quantities.
#[must_use]
#[allow(clippy::many_single_char_names, clippy::similar_names)]
pub fn compute_j2_params(mean: &KeplerianElements) -> J2Params {
    let a = mean.a_km;
    let e = mean.e;
    let i = mean.i_rad;

    let n = mean.mean_motion();
    let eta = (1.0 - e * e).sqrt();
    let p = a * (1.0 - e * e);

    let cos_i = i.cos();
    let sin_i = i.sin();
    let cos2_i = cos_i * cos_i;
    let sin2_i = sin_i * sin_i;

    // κ = (3/4)·n·J2·(R_E/p)²  (Koenig Eq. 13)
    let r_over_p = R_EARTH / p;
    let kappa = 0.75 * n * J2 * r_over_p * r_over_p;

    // Eq. 14 auxiliaries
    let big_e = 1.0 + eta;
    let big_f = 4.0 + 3.0 * eta;
    let big_g = 1.0 / (eta * eta * big_e);

    // Eccentricity vector components (Eq. A2)
    let ex = e * mean.aop_rad.cos();
    let ey = e * mean.aop_rad.sin();

    // Eq. 15 auxiliaries
    let big_p = 3.0 * cos2_i - 1.0;
    let big_q = 5.0 * cos2_i - 1.0;
    let big_r = cos_i;
    let big_s = (2.0 * i).sin(); // sin(2i)
    let big_t = sin2_i;

    // Secular rates (Koenig Eq. 13)
    let raan_dot = -2.0 * kappa * big_r;
    let aop_dot = kappa * big_q;
    let m_dot = n + kappa * eta * big_p;

    J2Params {
        n,
        kappa,
        eta,
        big_e,
        big_f,
        big_g,
        ex,
        ey,
        big_p,
        big_q,
        big_r,
        big_s,
        big_t,
        cos_i,
        sin_i,
        cos2_i,
        sin2_i,
        raan_dot,
        aop_dot,
        m_dot,
        p,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::iss_like_elements;

    #[test]
    fn iss_raan_rate() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean);

        // ISS RAAN regression rate ≈ -5.1°/day
        let raan_deg_per_day = j2p.raan_dot.to_degrees() * 86400.0;
        assert!(
            (raan_deg_per_day - (-5.1)).abs() < 0.5,
            "ISS RAAN rate should be ~-5.1 deg/day, got {raan_deg_per_day}"
        );
    }

    #[test]
    fn sun_synchronous_condition() {
        let mean = KeplerianElements {
            a_km: 7158.0,
            e: 0.001,
            i_rad: 98.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let j2p = compute_j2_params(&mean);
        let raan_deg_per_day = j2p.raan_dot.to_degrees() * 86400.0;

        assert!(
            (raan_deg_per_day - 0.9856).abs() < 0.05,
            "Sun-sync RAAN rate should be ~0.9856 deg/day, got {raan_deg_per_day}"
        );
    }

    #[test]
    fn secular_rates_signs() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean);

        assert!(j2p.raan_dot < 0.0, "RAAN should regress for prograde orbit");
        assert!(j2p.aop_dot > 0.0, "AoP should advance for ISS inclination");
        assert!(j2p.m_dot > 0.0, "Mean motion rate must be positive");
    }

    #[test]
    fn koenig_auxiliaries_correct() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean);

        // F = 4 + 3η, η ≈ 1 for near-circular → F ≈ 7
        assert!(
            (j2p.big_f - (4.0 + 3.0 * j2p.eta)).abs() < 1e-14,
            "F should be 4+3η"
        );

        // G = 1/(η²(1+η))
        let expected_g = 1.0 / (j2p.eta * j2p.eta * (1.0 + j2p.eta));
        assert!(
            (j2p.big_g - expected_g).abs() < 1e-14,
            "G should be 1/(η²(1+η))"
        );

        // P = 3cos²i - 1
        let expected_p = 3.0 * j2p.cos2_i - 1.0;
        assert!(
            (j2p.big_p - expected_p).abs() < 1e-14,
            "P should be 3cos²i - 1"
        );

        // Q = 5cos²i - 1
        let expected_q = 5.0 * j2p.cos2_i - 1.0;
        assert!(
            (j2p.big_q - expected_q).abs() < 1e-14,
            "Q should be 5cos²i - 1"
        );
    }
}

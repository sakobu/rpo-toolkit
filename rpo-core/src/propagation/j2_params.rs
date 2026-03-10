//! J2 perturbation parameters for secular rate computation (Koenig Eqs. 13-16).

use serde::{Deserialize, Serialize};

use crate::constants::{J2, R_EARTH};
use crate::propagation::propagator::PropagationError;
use crate::types::KeplerianElements;

/// Precomputed J2 perturbation parameters for a mean orbit.
///
/// Contains auxiliary quantities and secular rates needed by the
/// J2-perturbed state transition matrix (Koenig Eqs. 13-16).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct J2Params {
    /// Mean motion n = sqrt(μ/a³) (rad/s)
    pub n_rad_s: f64,
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
    pub raan_dot_rad_s: f64,
    /// Secular rate of argument of perigee (rad/s): dω/dt = κQ
    pub aop_dot_rad_s: f64,
    /// Secular rate of mean anomaly (rad/s): dM/dt = n + κηP
    pub m_dot_rad_s: f64,
    /// Semi-latus rectum p = a(1 - e²)
    pub p_km: f64,
}

/// Compute J2 perturbation parameters from mean Keplerian elements.
///
/// Implements Koenig Eqs. 13-16 for secular J2 rates and auxiliary quantities.
///
/// # Invariants
/// - `mean.a_km > 0`
/// - `0 <= mean.e < 1` (`e = 1` → `η = 0` → division by zero in `G = 1/(η²(1+η))`)
/// - `mean` must be **mean** Keplerian elements, not osculating
///
/// # Errors
/// Returns `PropagationError::InvalidEccentricity` if `e < 0` or `e >= 1`.
/// Returns `PropagationError::InvalidSemiMajorAxis` if `a_km <= 0`.
#[allow(clippy::many_single_char_names, clippy::similar_names)]
pub fn compute_j2_params(mean: &KeplerianElements) -> Result<J2Params, PropagationError> {
    if mean.e < 0.0 || mean.e >= 1.0 {
        return Err(PropagationError::InvalidEccentricity { e: mean.e });
    }
    if mean.a_km <= 0.0 {
        return Err(PropagationError::InvalidSemiMajorAxis { a_km: mean.a_km });
    }

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

    Ok(J2Params {
        n_rad_s: n,
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
        raan_dot_rad_s: raan_dot,
        aop_dot_rad_s: aop_dot,
        m_dot_rad_s: m_dot,
        p_km: p,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{
        iss_like_elements, koenig_table2_case1, koenig_table2_case2, koenig_table2_case3,
    };

    #[test]
    fn iss_raan_rate() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean).unwrap();

        // ISS RAAN regression rate ≈ -5.1°/day
        let raan_deg_per_day = j2p.raan_dot_rad_s.to_degrees() * 86400.0;
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
        let j2p = compute_j2_params(&mean).unwrap();
        let raan_deg_per_day = j2p.raan_dot_rad_s.to_degrees() * 86400.0;

        assert!(
            (raan_deg_per_day - 0.9856).abs() < 0.05,
            "Sun-sync RAAN rate should be ~0.9856 deg/day, got {raan_deg_per_day}"
        );
    }

    #[test]
    fn secular_rates_signs() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean).unwrap();

        assert!(j2p.raan_dot_rad_s < 0.0, "RAAN should regress for prograde orbit");
        assert!(j2p.aop_dot_rad_s > 0.0, "AoP should advance for ISS inclination");
        assert!(j2p.m_dot_rad_s > 0.0, "Mean motion rate must be positive");
    }

    #[test]
    fn invalid_sma_returns_error() {
        let mean = KeplerianElements {
            a_km: -100.0,
            e: 0.001,
            i_rad: 0.9,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = compute_j2_params(&mean);
        assert!(
            matches!(result, Err(PropagationError::InvalidSemiMajorAxis { .. })),
            "a_km <= 0 should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    #[test]
    fn zero_sma_returns_error() {
        let mean = KeplerianElements {
            a_km: 0.0,
            e: 0.001,
            i_rad: 0.9,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = compute_j2_params(&mean);
        assert!(
            matches!(result, Err(PropagationError::InvalidSemiMajorAxis { .. })),
            "a_km = 0 should return InvalidSemiMajorAxis, got {result:?}"
        );
    }

    /// Edge case: circular orbit (e=0) should produce valid J2 params.
    /// η=1, G=1/2, ex=ey=0 exactly.
    #[test]
    fn circular_orbit_e_zero() {
        let mean = KeplerianElements {
            a_km: 7000.0,
            e: 0.0,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let j2p = compute_j2_params(&mean).unwrap();

        // η = sqrt(1 - 0) = 1.0 exactly
        assert!(
            (j2p.eta - 1.0).abs() < 1e-15,
            "eta should be 1.0 for circular orbit, got {}", j2p.eta
        );
        // G = 1/(1·2) = 0.5
        assert!(
            (j2p.big_g - 0.5).abs() < 1e-15,
            "G should be 0.5 for circular orbit, got {}", j2p.big_g
        );
        // ex = ey = 0
        assert!(j2p.ex.abs() < 1e-15, "ex should be 0 for e=0");
        assert!(j2p.ey.abs() < 1e-15, "ey should be 0 for e=0");
    }

    /// Edge case: near-polar orbit (i≈90°).
    /// cos(i)≈0, P≈-1, Q≈-1, R≈0, S≈0, T≈1.
    #[test]
    fn near_polar_orbit() {
        let mean = KeplerianElements {
            a_km: 7000.0,
            e: 0.001,
            i_rad: 90.0_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let j2p = compute_j2_params(&mean).unwrap();

        // At i=90°: P = 3·0 - 1 = -1, Q = 5·0 - 1 = -1
        assert!(
            (j2p.big_p - (-1.0)).abs() < 1e-12,
            "P should be -1.0 at i=90°, got {}", j2p.big_p
        );
        assert!(
            (j2p.big_q - (-1.0)).abs() < 1e-12,
            "Q should be -1.0 at i=90°, got {}", j2p.big_q
        );
        // RAAN rate should be ~0 (R=cos(90°)≈0)
        assert!(
            j2p.raan_dot_rad_s.abs() < 1e-14,
            "RAAN rate should be ~0 at i=90°, got {}", j2p.raan_dot_rad_s
        );
        // AoP rate should be negative (Q = -1, κQ < 0)
        assert!(
            j2p.aop_dot_rad_s < 0.0,
            "AoP rate should be negative at i=90° (Q=-1), got {}", j2p.aop_dot_rad_s
        );
    }

    #[test]
    fn invalid_eccentricity_returns_error() {
        let mean = KeplerianElements {
            a_km: 7000.0,
            e: 1.0,
            i_rad: 0.9,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let result = compute_j2_params(&mean);
        assert!(
            matches!(result, Err(PropagationError::InvalidEccentricity { .. })),
            "e=1 should return InvalidEccentricity, got {result:?}"
        );
    }

    #[test]
    fn koenig_auxiliaries_correct() {
        let mean = iss_like_elements();
        let j2p = compute_j2_params(&mean).unwrap();

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

    // --- Paper-traced regression tests (Koenig Eqs. 13-16) ---

    /// Koenig Eqs. 13-16: J2 parameters for Table 2 Case 1 (i=30°, e=0.005).
    /// Validates exact trig-based auxiliaries P, Q, S, T and derived quantities.
    #[test]
    fn koenig_table2_case1_j2_params() {
        let chief = koenig_table2_case1();
        let j2p = compute_j2_params(&chief).unwrap();

        // eta = sqrt(1 - 0.005²) = sqrt(0.999975)
        let expected_eta = (1.0 - 0.005_f64.powi(2)).sqrt();
        assert!(
            (j2p.eta - expected_eta).abs() < 1e-12,
            "eta: got {}, expected {expected_eta}", j2p.eta
        );

        // i=30°: cos²(30°) = 3/4 exactly
        // P = 3cos²i - 1 = 5/4 = 1.25
        assert!(
            (j2p.big_p - 1.25).abs() < 1e-12,
            "P: got {}, expected 1.25", j2p.big_p
        );

        // Q = 5cos²i - 1 = 11/4 = 2.75
        assert!(
            (j2p.big_q - 2.75).abs() < 1e-12,
            "Q: got {}, expected 2.75", j2p.big_q
        );

        // S = sin(2·30°) = sin(60°) = √3/2
        let sqrt3_over_2 = 3.0_f64.sqrt() / 2.0;
        assert!(
            (j2p.big_s - sqrt3_over_2).abs() < 1e-12,
            "S: got {}, expected {sqrt3_over_2}", j2p.big_s
        );

        // T = sin²(30°) = 1/4 = 0.25
        assert!(
            (j2p.big_t - 0.25).abs() < 1e-12,
            "T: got {}, expected 0.25", j2p.big_t
        );

        // ex = e·cos(ω) = 0.005·cos(180°) = -0.005
        assert!(
            (j2p.ex - (-0.005)).abs() < 1e-12,
            "ex: got {}, expected -0.005", j2p.ex
        );

        // ey = e·sin(ω) = 0.005·sin(180°) ≈ 0
        assert!(
            j2p.ey.abs() < 1e-15,
            "ey: got {}, expected ~0", j2p.ey
        );

        // E = 1 + eta
        assert!(
            (j2p.big_e - (1.0 + expected_eta)).abs() < 1e-12,
            "E: got {}, expected {}", j2p.big_e, 1.0 + expected_eta
        );

        // F = 4 + 3·eta
        assert!(
            (j2p.big_f - (4.0 + 3.0 * expected_eta)).abs() < 1e-12,
            "F: got {}, expected {}", j2p.big_f, 4.0 + 3.0 * expected_eta
        );

        // G = 1/(eta²·(1+eta))
        let expected_g = 1.0 / (expected_eta * expected_eta * (1.0 + expected_eta));
        assert!(
            (j2p.big_g - expected_g).abs() < 1e-12,
            "G: got {}, expected {expected_g}", j2p.big_g
        );

        // kappa = (3/4)·n·J2·(R_E/p)²
        let p = chief.a_km * (1.0 - chief.e * chief.e);
        let n = chief.mean_motion();
        let r_over_p = R_EARTH / p;
        let expected_kappa = 0.75 * n * J2 * r_over_p * r_over_p;
        assert!(
            (j2p.kappa - expected_kappa).abs() < 1e-15,
            "kappa: got {}, expected {expected_kappa}", j2p.kappa
        );

        // Secular rates (Koenig Eq. 13)
        let expected_raan_dot = -2.0 * expected_kappa * j2p.cos_i;
        assert!(
            (j2p.raan_dot_rad_s - expected_raan_dot).abs() < 1e-15,
            "raan_dot_rad_s: got {}, expected {expected_raan_dot}", j2p.raan_dot_rad_s
        );

        let expected_aop_dot = expected_kappa * 2.75;
        assert!(
            (j2p.aop_dot_rad_s - expected_aop_dot).abs() < 1e-15,
            "aop_dot_rad_s: got {}, expected {expected_aop_dot}", j2p.aop_dot_rad_s
        );
    }

    /// Koenig Eqs. 13-16: J2 parameters for Table 2 Case 2 (i=1°, e=0.2).
    /// Near-equatorial with nonzero ey = 0.2·sin(120°) ≈ 0.1732.
    #[test]
    fn koenig_table2_case2_j2_params() {
        let chief = koenig_table2_case2();
        let j2p = compute_j2_params(&chief).unwrap();

        // eta = sqrt(1 - 0.04) = sqrt(0.96)
        let expected_eta = 0.96_f64.sqrt();
        assert!(
            (j2p.eta - expected_eta).abs() < 1e-12,
            "eta: got {}, expected {expected_eta}", j2p.eta
        );

        // ex = 0.2·cos(120°) = 0.2·(-0.5) = -0.1
        assert!(
            (j2p.ex - (-0.1)).abs() < 1e-12,
            "ex: got {}, expected -0.1", j2p.ex
        );

        // ey = 0.2·sin(120°) = 0.2·(√3/2)
        let expected_ey = 0.2 * 120.0_f64.to_radians().sin();
        assert!(
            (j2p.ey - expected_ey).abs() < 1e-12,
            "ey: got {}, expected {expected_ey}", j2p.ey
        );

        // ey should be substantial (nonzero aop effect)
        assert!(
            j2p.ey.abs() > 0.17,
            "ey should be ~0.1732, got {}", j2p.ey
        );

        // Near-equatorial: P ≈ 2.0, Q ≈ 4.0
        let cos2_1 = 1.0_f64.to_radians().cos().powi(2);
        let expected_p = 3.0 * cos2_1 - 1.0;
        let expected_q = 5.0 * cos2_1 - 1.0;
        assert!(
            (j2p.big_p - expected_p).abs() < 1e-12,
            "P: got {}, expected {expected_p}", j2p.big_p
        );
        assert!(
            (j2p.big_q - expected_q).abs() < 1e-12,
            "Q: got {}, expected {expected_q}", j2p.big_q
        );
    }

    /// Koenig Eqs. 13-16: J2 parameters for Table 2 Case 3 (i=45°, e=0.5).
    /// High eccentricity: exact trig at 45° gives S=1, T=0.5, P=0.5, Q=1.5.
    #[test]
    fn koenig_table2_case3_j2_params() {
        let chief = koenig_table2_case3();
        let j2p = compute_j2_params(&chief).unwrap();

        // eta = sqrt(1 - 0.25) = sqrt(3)/2
        let expected_eta = 3.0_f64.sqrt() / 2.0;
        assert!(
            (j2p.eta - expected_eta).abs() < 1e-12,
            "eta: got {}, expected {expected_eta}", j2p.eta
        );

        // i=45°: cos²(45°) = 1/2
        // P = 3·(1/2) - 1 = 0.5
        assert!(
            (j2p.big_p - 0.5).abs() < 1e-12,
            "P: got {}, expected 0.5", j2p.big_p
        );

        // Q = 5·(1/2) - 1 = 1.5
        assert!(
            (j2p.big_q - 1.5).abs() < 1e-12,
            "Q: got {}, expected 1.5", j2p.big_q
        );

        // S = sin(90°) = 1.0 exactly
        assert!(
            (j2p.big_s - 1.0).abs() < 1e-12,
            "S: got {}, expected 1.0", j2p.big_s
        );

        // T = sin²(45°) = 1/2 = 0.5
        assert!(
            (j2p.big_t - 0.5).abs() < 1e-12,
            "T: got {}, expected 0.5", j2p.big_t
        );

        // G = 1/(eta²·(1+eta)) = 1/(3/4 · (1 + √3/2))
        let eta2 = 0.75;
        let expected_g = 1.0 / (eta2 * (1.0 + expected_eta));
        assert!(
            (j2p.big_g - expected_g).abs() < 1e-12,
            "G: got {}, expected {expected_g}", j2p.big_g
        );

        // ex = 0.5·cos(60°) = 0.5·0.5 = 0.25
        assert!(
            (j2p.ex - 0.25).abs() < 1e-12,
            "ex: got {}, expected 0.25", j2p.ex
        );

        // ey = 0.5·sin(60°) = √3/4
        let expected_ey = 3.0_f64.sqrt() / 4.0;
        assert!(
            (j2p.ey - expected_ey).abs() < 1e-12,
            "ey: got {}, expected {expected_ey}", j2p.ey
        );
    }
}

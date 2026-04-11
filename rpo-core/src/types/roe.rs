//! Quasi-nonsingular relative orbital elements (Koenig/D'Amico formulation).

use nalgebra::SVector;
use serde::{Deserialize, Serialize};

/// Quasi-nonsingular relative orbital elements (Koenig Eq. 2 / D'Amico Eq. 2.2)
/// All elements are dimensionless (normalized by chief semi-major axis)
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct QuasiNonsingularROE {
    /// Relative semi-major axis: `(a_d - a_c) / a_c`
    pub da: f64,
    /// Relative mean longitude
    pub dlambda: f64,
    /// Relative eccentricity vector x-component
    pub dex: f64,
    /// Relative eccentricity vector y-component
    pub dey: f64,
    /// Relative inclination vector x-component
    pub dix: f64,
    /// Relative inclination vector y-component
    pub diy: f64,
}

impl Default for QuasiNonsingularROE {
    fn default() -> Self {
        Self {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        }
    }
}

impl QuasiNonsingularROE {
    /// Convert to a 6-element column vector [da, dlambda, dex, dey, dix, diy].
    #[must_use]
    pub fn to_vector(&self) -> SVector<f64, 6> {
        SVector::<f64, 6>::new(self.da, self.dlambda, self.dex, self.dey, self.dix, self.diy)
    }

    /// Construct from a 6-element column vector [da, dlambda, dex, dey, dix, diy].
    #[must_use]
    pub fn from_vector(v: &SVector<f64, 6>) -> Self {
        Self {
            da: v[0],
            dlambda: v[1],
            dex: v[2],
            dey: v[3],
            dix: v[4],
            diy: v[5],
        }
    }

    /// Relative eccentricity vector magnitude: `|δe| = √(δex² + δey²)` (dimensionless).
    #[must_use]
    pub fn de_magnitude(&self) -> f64 {
        (self.dex * self.dex + self.dey * self.dey).sqrt()
    }

    /// Relative inclination vector magnitude: `|δi| = √(δix² + δiy²)` (dimensionless).
    #[must_use]
    pub fn di_magnitude(&self) -> f64 {
        (self.dix * self.dix + self.diy * self.diy).sqrt()
    }

    /// Dimensionless separation metric: max(|δa|, |δex|, |δey|, |δix|).
    ///
    /// Excludes δλ and δiy per Koenig Sec. V (these can be large without
    /// violating linearization assumptions).
    #[must_use]
    pub fn dimensionless_norm(&self) -> f64 {
        self.da.abs().max(self.dex.abs()).max(self.dey.abs()).max(self.dix.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Dimensionless norm computation tolerance. The norm is a max of
    /// absolute values — exact arithmetic; 1e-15 covers f64 rounding.
    const NORM_TOL: f64 = 1e-15;

    // ---------------------------------------------------------------------------
    // QuasiNonsingularROE::dimensionless_norm
    // ---------------------------------------------------------------------------

    /// Verify that `dimensionless_norm` returns max(|δa|, |δex|, |δey|, |δix|)
    /// and specifically EXCLUDES δλ and δiy, even when those are larger.
    #[test]
    fn dimensionless_norm_excludes_dlambda_and_diy() {
        let roe = QuasiNonsingularROE {
            da: 0.001,
            dlambda: 999.0, // large — must NOT dominate
            dex: 0.002,
            dey: 0.003,
            dix: 0.004, // expected max of the four included fields
            diy: 888.0, // large — must NOT dominate
        };
        let norm = roe.dimensionless_norm();
        // Expected: max(0.001, 0.002, 0.003, 0.004) = 0.004
        let expected = 0.004_f64;
        assert!(
            (norm - expected).abs() < NORM_TOL,
            "dimensionless_norm: expected {expected}, got {norm}"
        );
    }

    /// When all included components are zero, norm = 0.
    #[test]
    fn dimensionless_norm_zero_roe() {
        let roe = QuasiNonsingularROE::default();
        assert_eq!(roe.dimensionless_norm(), 0.0, "zero ROE: expected norm = 0");
    }

    /// Norm picks the correct maximum when δa is the largest.
    #[test]
    fn dimensionless_norm_da_dominates() {
        let roe = QuasiNonsingularROE {
            da: 0.01,
            dlambda: 0.005,
            dex: 0.003,
            dey: 0.002,
            dix: 0.001,
            diy: 0.009, // just below da but diy is excluded anyway
        };
        let norm = roe.dimensionless_norm();
        assert!(
            (norm - 0.01).abs() < NORM_TOL,
            "dimensionless_norm da-dominated: expected 0.01, got {norm}"
        );
    }

    /// Norm picks δex when it is the largest of the four included fields.
    #[test]
    fn dimensionless_norm_dex_dominates() {
        let roe = QuasiNonsingularROE {
            da: 0.001,
            dlambda: 0.5,
            dex: 0.1,
            dey: 0.05,
            dix: 0.02,
            diy: 0.3,
        };
        let norm = roe.dimensionless_norm();
        assert!(
            (norm - 0.1).abs() < NORM_TOL,
            "dimensionless_norm dex-dominated: expected 0.1, got {norm}"
        );
    }

    /// Negative components: norm uses absolute values.
    #[test]
    fn dimensionless_norm_negative_components() {
        let roe = QuasiNonsingularROE {
            da: -0.005,
            dlambda: 0.0,
            dex: 0.002,
            dey: -0.003,
            dix: 0.001,
            diy: 0.0,
        };
        let norm = roe.dimensionless_norm();
        // max(0.005, 0.002, 0.003, 0.001) = 0.005
        assert!(
            (norm - 0.005).abs() < NORM_TOL,
            "dimensionless_norm negative: expected 0.005, got {norm}"
        );
    }
}

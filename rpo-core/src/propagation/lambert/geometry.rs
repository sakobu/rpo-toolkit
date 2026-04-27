//! Lambert problem geometry: chord, semi-perimeter, λ, and transfer-plane basis.
//!
//! Constructed once per solver entry point and threaded through the
//! root-finding and velocity-reconstruction stages — eliminates re-derivation
//! drift across modules.

use nalgebra::Vector3;

use super::{
    safe_sqrt_nonneg, LambertError, PositionTag, TransferDirection, COLLINEARITY_TOL,
    LAMBERT_DEGENERATE_NORM_KM,
};

/// Pre-computed geometry for a Lambert boundary problem.
///
/// All scalars and unit vectors derive from `(r1_km, r2_km, tof_s, mu_km3_s2,
/// direction)`. The solver kernels consume `lambda` and `big_t` (non-dimensional
/// TOF); the velocity reconstruction in [`super::solver`] consumes the rest.
///
/// Field names are math-domain names (paper symbols), not unit-tagged —
/// these are crate-private intermediates, not public state.
#[derive(Debug, Clone, Copy)]
pub(super) struct Geometry {
    /// Izzo's λ parameter, sign-corrected for short/long way (Eq. 7).
    pub lambda: f64,
    /// Non-dimensional time of flight `T = sqrt(2μ / s³) · tof`.
    pub big_t: f64,
    /// `sqrt(μ · s / 2)` — velocity scale for reconstruction.
    pub gamma: f64,
    /// `(r1n − r2n) / c`.
    pub rho: f64,
    /// `sqrt(1 − ρ²)`.
    pub sigma: f64,
    /// `|r1|`.
    pub r1n: f64,
    /// `|r2|`.
    pub r2n: f64,
    /// Unit vector along `r1`.
    pub ir1: Vector3<f64>,
    /// Unit vector along `r2`.
    pub ir2: Vector3<f64>,
    /// In-plane tangent at `r1` (sign-corrected for long-way transfers).
    pub it1: Vector3<f64>,
    /// In-plane tangent at `r2` (sign-corrected for long-way transfers).
    pub it2: Vector3<f64>,
}

/// Direction-independent geometry derived once from `(r1, r2, tof, μ)`.
/// Reused by both short-way and long-way flavors when [`Geometry::pair_from_inputs`]
/// is called, which halves the per-call cost of [`TransferDirection::Auto`].
#[derive(Debug, Clone, Copy)]
struct GeometryCommon {
    big_t: f64,
    gamma: f64,
    rho: f64,
    sigma: f64,
    r1n: f64,
    r2n: f64,
    ir1: Vector3<f64>,
    ir2: Vector3<f64>,
    /// `sqrt(1 − c/s)`, the magnitude of `λ`. Sign is applied per-direction.
    lambda_mag: f64,
    /// `ih × ir1` normalized — the short-way in-plane tangent at `r1`.
    /// The long-way tangent is `-it1_short`.
    it1_short: Vector3<f64>,
    /// `ih × ir2` normalized — the short-way in-plane tangent at `r2`.
    it2_short: Vector3<f64>,
}

impl Geometry {
    /// Build the geometry from raw inputs, validating scalars and the
    /// transfer plane along the way.
    ///
    /// `direction` must be `ShortWay` or `LongWay`; `Auto` is dispatched at
    /// the public-API layer (which calls into both directions and selects
    /// by Δv). When a caller needs both directions, prefer
    /// [`Geometry::pair_from_inputs`] to share the direction-independent work.
    ///
    /// # Errors
    ///
    /// - [`LambertError::NonPositiveTimeOfFlight`] — `tof_s <= 0`.
    /// - [`LambertError::NonPositiveMu`] — `mu_km3_s2 <= 0`.
    /// - [`LambertError::DegeneratePositionVector`] — `|r1|` or `|r2|`
    ///   below [`LAMBERT_DEGENERATE_NORM_KM`].
    /// - [`LambertError::CollinearGeometry`] — `|r1 × r2| / (|r1| · |r2|)`
    ///   below [`COLLINEARITY_TOL`].
    pub(super) fn from_inputs(
        r1_km: Vector3<f64>,
        r2_km: Vector3<f64>,
        tof_s: f64,
        mu_km3_s2: f64,
        direction: TransferDirection,
    ) -> Result<Self, LambertError> {
        let common = GeometryCommon::from_inputs(r1_km, r2_km, tof_s, mu_km3_s2)?;
        Ok(common.with_direction(direction))
    }

    /// Build short-way and long-way geometries from a single set of
    /// `(r1, r2, tof, μ)` inputs, sharing the direction-independent work
    /// (norms, chord, semi-perimeter, two cross products, the `|λ|` sqrt,
    /// `big_t`, `gamma`, `ρ`, `σ`).
    ///
    /// Halves the per-call geometry cost when both directions are needed
    /// (the `TransferDirection::Auto` path).
    ///
    /// # Errors
    ///
    /// Same as [`Geometry::from_inputs`]; the input checks fire on the
    /// shared inputs, not the per-direction derivation.
    pub(super) fn pair_from_inputs(
        r1_km: Vector3<f64>,
        r2_km: Vector3<f64>,
        tof_s: f64,
        mu_km3_s2: f64,
    ) -> Result<(Self, Self), LambertError> {
        let common = GeometryCommon::from_inputs(r1_km, r2_km, tof_s, mu_km3_s2)?;
        let short = common.with_direction(TransferDirection::ShortWay);
        let long = common.with_direction(TransferDirection::LongWay);
        Ok((short, long))
    }
}

impl GeometryCommon {
    #[allow(clippy::similar_names)]
    fn from_inputs(
        r1_km: Vector3<f64>,
        r2_km: Vector3<f64>,
        tof_s: f64,
        mu_km3_s2: f64,
    ) -> Result<Self, LambertError> {
        if tof_s <= 0.0 {
            return Err(LambertError::NonPositiveTimeOfFlight { tof_s });
        }
        if mu_km3_s2 <= 0.0 {
            return Err(LambertError::NonPositiveMu { mu_km3_s2 });
        }

        let r1n = r1_km.norm();
        let r2n = r2_km.norm();
        if r1n < LAMBERT_DEGENERATE_NORM_KM {
            return Err(LambertError::DegeneratePositionVector {
                which: PositionTag::R1,
                norm_km: r1n,
            });
        }
        if r2n < LAMBERT_DEGENERATE_NORM_KM {
            return Err(LambertError::DegeneratePositionVector {
                which: PositionTag::R2,
                norm_km: r2n,
            });
        }

        let c = (r2_km - r1_km).norm();
        let s = 0.5 * (r1n + r2n + c);

        let ir1 = r1_km / r1n;
        let ir2 = r2_km / r2n;
        let ih_raw = ir1.cross(&ir2);
        let sin_angle = ih_raw.norm();
        let Some(ih) = ih_raw.try_normalize(COLLINEARITY_TOL) else {
            return Err(LambertError::CollinearGeometry { sin_angle });
        };

        // Short-way tangents. Long-way tangents are the negatives of these
        // (since `ir·× ih = −ih × ir·`); applied in [`with_direction`].
        let it1_short = ih.cross(&ir1).normalize();
        let it2_short = ih.cross(&ir2).normalize();

        let lambda_mag = safe_sqrt_nonneg(1.0 - c / s);
        let big_t = (2.0 * mu_km3_s2 / (s * s * s)).sqrt() * tof_s;
        let gamma = (mu_km3_s2 * s / 2.0).sqrt();
        let rho = (r1n - r2n) / c;
        let sigma = safe_sqrt_nonneg(1.0 - rho * rho);

        Ok(Self {
            big_t,
            gamma,
            rho,
            sigma,
            r1n,
            r2n,
            ir1,
            ir2,
            lambda_mag,
            it1_short,
            it2_short,
        })
    }

    fn with_direction(self, direction: TransferDirection) -> Geometry {
        // Default convention: short-way (counter-clockwise about `ih`) with
        // θ ∈ [0, π] → λ > 0. Long-way flips both λ AND the tangent basis.
        let (lambda, it1, it2) = if matches!(direction, TransferDirection::LongWay) {
            (-self.lambda_mag, -self.it1_short, -self.it2_short)
        } else {
            (self.lambda_mag, self.it1_short, self.it2_short)
        };
        Geometry {
            lambda,
            big_t: self.big_t,
            gamma: self.gamma,
            rho: self.rho,
            sigma: self.sigma,
            r1n: self.r1n,
            r2n: self.r2n,
            ir1: self.ir1,
            ir2: self.ir2,
            it1,
            it2,
        }
    }
}

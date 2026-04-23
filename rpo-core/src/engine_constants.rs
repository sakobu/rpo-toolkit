//! Engine policy constants exposed across the Rust↔WASM boundary.
//!
//! Frontend callers consume [`engine_constants`] (via the `rpo-wasm`
//! re-export) instead of mirroring physical constants and policy
//! thresholds in TypeScript. This keeps `rpo-core` as the single source
//! of truth for values that are physics-grounded (Earth radius) or
//! algorithm-policy-grounded (the ROE linearization bound).
//!
//! Add a field here when a frontend mirror would otherwise need to be
//! hand-maintained. Do **not** add display/UI constants — those belong
//! in the frontend.

use serde::{Deserialize, Serialize};

use crate::constants::R_EARTH;
use crate::mission::config::ROE_THRESHOLD_DEFAULT;
use crate::mission::formation::LINEARIZATION_PERTURBATION_BOUND;

/// Policy and physics constants the frontend reads via WASM rather than
/// mirroring in TypeScript.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct EngineConstants {
    /// Dimensionless ROE perturbation norm bound for linearization validity
    /// (D'Amico §2.3.4). Source: [`LINEARIZATION_PERTURBATION_BOUND`].
    pub linearization_perturbation_bound: f64,
    /// WGS-84 equatorial radius (km). Source: [`R_EARTH`].
    pub earth_radius_km: f64,
    /// Recommended default δr/r threshold for proximity classification
    /// (dimensionless). Source: [`ROE_THRESHOLD_DEFAULT`].
    pub roe_threshold_default: f64,
}

/// Snapshot the engine policy/physics constants exposed to the frontend.
#[must_use]
pub fn engine_constants() -> EngineConstants {
    EngineConstants {
        linearization_perturbation_bound: LINEARIZATION_PERTURBATION_BOUND,
        earth_radius_km: R_EARTH,
        roe_threshold_default: ROE_THRESHOLD_DEFAULT,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_constants_match_canonical_sources() {
        let c = engine_constants();
        assert!((c.linearization_perturbation_bound - LINEARIZATION_PERTURBATION_BOUND).abs() < f64::EPSILON);
        assert!((c.earth_radius_km - R_EARTH).abs() < f64::EPSILON);
        assert!((c.roe_threshold_default - ROE_THRESHOLD_DEFAULT).abs() < f64::EPSILON);
    }
}

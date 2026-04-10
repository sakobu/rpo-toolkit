//! Display and alert thresholds for CLI output formatting.
//!
//! Presentation-layer thresholds only. Core algorithm tolerances live in rpo-core.

/// Monte Carlo convergence and safety thresholds for verdict/insight logic.
pub mod mc {
    /// Tolerance for treating convergence rate as exactly 1.0 (all samples converged).
    pub const CONVERGENCE_EXACT_TOL: f64 = f64::EPSILON;
    /// Convergence rate below which the verdict degrades to CAUTION.
    pub const CONVERGENCE_ALERT: f64 = 0.95;
}

/// Safety comparison thresholds.
pub mod fidelity {
    use super::insight;
    /// Ratio below which numerical safety is flagged as non-conservative vs analytical.
    /// Derived from [`insight::SIGNIFICANT_DELTA_PCT`] to keep both in sync.
    pub const NONCONSERVATIVE_RATIO: f64 = 1.0 - insight::SIGNIFICANT_DELTA_PCT / 100.0;
}

/// Rate-comparison thresholds for verdict zero-checks.
pub mod rate {
    /// Rate at or below which "no violations" is reported.
    /// Used for collision probability, e/i violation rate, keep-out violation rate.
    pub const ZERO_VIOLATIONS: f64 = 0.0;
}

/// Insight generation trigger thresholds.
pub mod insight {
    /// Flag when numerical metric is < 2x the configured safety threshold.
    pub const TIGHT_MARGIN_FACTOR: f64 = 2.0;
    /// Flag when numerical safety is >10% smaller than analytical.
    pub const SIGNIFICANT_DELTA_PCT: f64 = 10.0;
    /// Flag when MC Dv spread ratio (p95/p05) exceeds this.
    pub const DV_SPREAD_RATIO_ALERT: f64 = 1.2;
    /// Flag when per-leg RMS growth ratio exceeds this.
    pub const ERROR_GROWTH_RATIO_ALERT: f64 = 3.0;
}

/// Velocity target display thresholds.
pub mod velocity {
    /// Fraction of total velocity magnitude a single RIC component must exceed
    /// to be displayed as the dominant direction (e.g., "+1.0 I").
    /// Below this threshold, only magnitude is shown.
    pub const DOMINANT_COMPONENT_FRACTION: f64 = 0.7;

    /// Velocity magnitude (km/s) below which the vector is treated as zero for display.
    /// 1e-12 km/s = 1e-9 m/s = 1 nm/s — well below any physically meaningful maneuver.
    pub const ZERO_MAGNITUDE_KM_S: f64 = 1e-12;
}

/// Safety display thresholds.
pub mod safety {
    /// e/i separation (metres) below which phase angle is not displayed.
    /// At sub-millimetre separations the geometry is too small for the angle
    /// to carry meaningful information.
    pub const MIN_EI_SEPARATION_FOR_PHASE_DISPLAY_M: f64 = 0.05;
}

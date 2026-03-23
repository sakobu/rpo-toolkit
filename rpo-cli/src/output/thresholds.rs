//! Display and alert thresholds for CLI output formatting.
//!
//! Presentation-layer thresholds only. Core algorithm tolerances live in rpo-core.

/// Fidelity assessment thresholds.
pub mod fidelity {
    /// Position error (km) below which model is suitable for close-proximity.
    pub const CLOSE_PROXIMITY_KM: f64 = 0.05;
    /// Position error (km) below which model is suitable for safety screening.
    pub const SAFETY_SCREENING_KM: f64 = 0.2;
    /// Dv percentage above which Lambert transfer is the dominant cost.
    pub const FAR_FIELD_DV_DRIVER_PCT: f64 = 90.0;
    /// Ratio below which numerical safety is flagged as non-conservative vs analytical.
    pub const NONCONSERVATIVE_RATIO: f64 = 0.9;
}

/// Monte Carlo convergence and safety coloring thresholds.
pub mod mc {
    /// Tolerance for treating convergence rate as exactly 1.0 (all samples converged).
    pub const CONVERGENCE_EXACT_TOL: f64 = f64::EPSILON;
    /// Convergence rate below which the verdict degrades to CAUTION.
    pub const CONVERGENCE_ALERT: f64 = 0.95;
    /// Convergence rate below which we color yellow.
    pub const CONVERGENCE_WARN: f64 = 0.99;
    /// Collision probability above which we color red.
    pub const COLLISION_PROB_ALERT: f64 = 0.05;
    /// e/i violation rate above which we color red.
    pub const EI_VIOLATION_RATE_ALERT: f64 = 0.10;
    /// Mahalanobis distance below which we color red.
    pub const MAHALANOBIS_ALERT: f64 = 0.5;
    /// Mahalanobis distance below which we color yellow.
    pub const MAHALANOBIS_WARN: f64 = 1.0;
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

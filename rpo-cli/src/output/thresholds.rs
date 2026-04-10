//! Display and alert thresholds for CLI output formatting.
//!
//! Presentation-layer thresholds only. Core algorithm tolerances live in
//! `rpo-core`. Every constant in this module is named, categorized, and
//! documented with its rationale per the CLAUDE.md tolerance policy.

/// Monte Carlo convergence and safety thresholds for verdict/insight logic.
pub mod mc {
    /// Tolerance for treating the convergence rate as exactly `1.0`
    /// (all samples converged).
    ///
    /// Convergence rate is computed as `converged_samples / total_samples`
    /// where both numerator and denominator are exact small integers held
    /// in `f64`. Any drift from a true `1.0` is at the ULP scale, so
    /// `f64::EPSILON` is the tightest principled guard that distinguishes
    /// "all converged" from "one or more failed" without risking false
    /// negatives from float-equality pitfalls. CLAUDE.md forbids `==` on
    /// floats; this constant exists so the `all_converged` check in
    /// `determine_mc_verdict` can honor that rule.
    pub const CONVERGENCE_EXACT_TOL: f64 = f64::EPSILON;

    /// Convergence rate below which the top-line Monte Carlo verdict
    /// degrades from `PASS` to `CAUTION`.
    ///
    /// Empirically chosen at `0.95` (5% sample-failure tolerance): below
    /// this level, enough samples failed that the reported statistics are
    /// no longer trustworthy as a closed-loop safety estimate. The value
    /// is a heuristic — it is not derived from a confidence interval on
    /// the collision probability, so it should be re-tuned if the default
    /// sample count changes materially. Used by `determine_mc_verdict` in
    /// `common.rs`.
    pub const CONVERGENCE_ALERT: f64 = 0.95;
}

/// Rate-comparison thresholds for verdict zero-checks.
pub mod rate {
    /// Rate at or below which "no violations" is reported.
    ///
    /// `collision_probability`, `ei_violation_rate`, and
    /// `keepout_violation_rate` are all computed as integer ratios
    /// (`violations / samples`), so an exact `0.0` comparison is
    /// well-defined here and is *not* the usual float-equality
    /// anti-pattern CLAUDE.md warns against. This constant exists to
    /// document that principled exception at every call site that
    /// checks "did anything happen at all?" against the ensemble.
    pub const ZERO_VIOLATIONS: f64 = 0.0;
}

/// Insight generation trigger thresholds.
pub mod insight {
    /// Fraction-to-percent conversion factor: multiply a dimensionless
    /// ratio by this to obtain a human-readable percentage.
    ///
    /// Not a tolerance — a unit conversion constant. Centralized here so
    /// the rendering layer has a single named source for "× 100" instead
    /// of scattering the magic number across every `format!` site that
    /// renders a rate, fraction, delta, or probability.
    pub const PERCENT_PER_UNIT: f64 = 100.0;

    /// Ratio below which a numerical safety metric is considered "tight"
    /// against its configured threshold.
    ///
    /// Fires in two places: (1) the `validation_insights` tight-margin
    /// warnings that flag `num_3d` or `num_ei` sitting within `2×` of the
    /// configured minimum, and (2) the `ei_qualifier` branch that adds
    /// the "tighter at" phrasing to the overestimate insight. `2.0` is a
    /// conventional safety-of-flight cushion (one doubling of margin) and
    /// is empirical, not derived from any paper in the reference set.
    pub const TIGHT_MARGIN_FACTOR: f64 = 2.0;

    /// Fraction of the configured e/i separation threshold below which
    /// abort-case passive safety is considered collapsed.
    ///
    /// When any leg's free-drift e/i falls below
    /// `min_ei_separation_km * ABORT_EI_COLLAPSE_RATIO`, the top-line
    /// verdict surfaces the marginal abort case. Chosen at `0.1` (one
    /// order of magnitude below the enforced threshold) to flag only
    /// severe collapses, not borderline cases.
    pub const ABORT_EI_COLLAPSE_RATIO: f64 = 0.1;

    /// Percentage by which the analytical safety metric must exceed the
    /// numerical metric before the analytical result is flagged as
    /// non-conservative.
    ///
    /// Measurement convention is `(ana_km - num_km) / num_km * 100`,
    /// i.e. the overestimate is expressed as a fraction of the
    /// *numerical* (ground-truth) value so the insight message
    /// "Analytical overestimates by X%" reads naturally against the
    /// Nyx number the operator is looking at. The `10%` threshold is
    /// empirical: large enough that routine J2-vs-full-physics rounding
    /// differences do not fire the warning, small enough that any real
    /// model mismatch surfaces in the Safety Comparison table. Lives at
    /// the boundary where analytical and numerical tiers begin to
    /// disagree in an operationally meaningful way. See
    /// `analytical_overestimate_pct` in `common.rs` for the single
    /// source-of-truth computation.
    pub const SIGNIFICANT_DELTA_PCT: f64 = 10.0;

    /// Monte Carlo Δv spread ratio (`p95 / p05`) above which an
    /// informational insight is emitted.
    ///
    /// `1.2` means the 95th percentile Δv exceeds the 5th percentile by
    /// more than 20%. Below that level the ensemble is tight enough
    /// that dispersion sensitivity is unremarkable; above it, the
    /// operator should inspect dispersion magnitudes and re-tune
    /// targeting gains. Empirical, not derived.
    pub const DV_SPREAD_RATIO_ALERT: f64 = 1.2;

    /// Per-leg position-error growth ratio (`last_leg_rms /
    /// first_leg_rms`) above which the insight layer emits an
    /// extrapolation warning.
    ///
    /// Fires in two places: (1) `leg_error_growth_insight` in
    /// `insights.rs`, which analyzes `LegValidationSummary`, and (2)
    /// `determine_mc_verdict` / `waypoint_miss_growth_ratio`, which
    /// look at waypoint miss medians across the ensemble. `3.0` is
    /// chosen as the point at which linearized-ROE validity is
    /// visibly degrading over the mission horizon — a real model
    /// issue rather than accumulated numerical noise. Empirical.
    pub const ERROR_GROWTH_RATIO_ALERT: f64 = 3.0;
}

/// Velocity target display thresholds.
pub mod velocity {
    /// Minimum fraction of the total velocity magnitude one RIC
    /// component must exceed before it is displayed as the dominant
    /// direction (e.g. "+1.0 I").
    ///
    /// `0.7 ≈ cos(45°)`: a component is labeled "dominant" when its
    /// absolute value exceeds the total magnitude times the cosine of
    /// 45°, i.e. when it lies within a 45° cone of the RIC axis. Below
    /// that, the maneuver is close enough to an off-axis diagonal that
    /// no single component carries the full story and the renderer
    /// falls back to bare magnitude. The `cos(45°)` framing also means
    /// at most one component can ever be dominant (trigonometric
    /// identity), so the labeling is unambiguous by construction.
    pub const DOMINANT_COMPONENT_FRACTION: f64 = 0.7;

    /// Velocity magnitude (km/s) below which the vector is treated as
    /// zero for display purposes.
    ///
    /// `1e-12 km/s = 1e-9 m/s = 1 nm/s` — well below any physically
    /// meaningful maneuver. Exists to suppress display of floating-point
    /// noise that accumulates after multiple GVE roundtrips. Not a
    /// solver tolerance; rendering-layer only.
    pub const ZERO_MAGNITUDE_KM_S: f64 = 1e-12;
}

/// Safety display thresholds.
pub mod safety {
    /// Minimum e/i separation (metres) for which the phase angle is
    /// rendered in the safety summary.
    ///
    /// At ~5 cm separation the phase angle is numerically unstable:
    /// the underlying `atan2(dey, dex)` / `atan2(diy, dix)` cancellation
    /// dominates and the returned angle is dominated by ROE roundoff
    /// rather than actual formation geometry. Below this cutoff the
    /// renderer suppresses the angle entirely rather than display a
    /// meaningless value.
    pub const MIN_EI_SEPARATION_FOR_PHASE_DISPLAY_M: f64 = 0.05;
}

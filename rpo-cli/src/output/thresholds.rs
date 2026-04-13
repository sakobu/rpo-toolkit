//! Display and alert thresholds for CLI output formatting.
//!
//! Presentation-layer thresholds only. Core algorithm tolerances live in
//! `rpo-core`.

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
    /// negatives from float-equality pitfalls. Codebase forbids `==` on
    /// floats; this constant exists so the `all_converged` check in
    /// `determine_mc_verdict` (see `verdict.rs`) can honor that rule.
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
    /// `verdict.rs`.
    pub const CONVERGENCE_ALERT: f64 = 0.95;

    /// Open-loop 3σ / closed-loop p95 ratio below which the closed-loop
    /// retargeting-benefit callout is suppressed as not operationally
    /// interesting.
    ///
    /// Less than 2× suppression is barely a difference — either the
    /// dispersion is already small or the two propagations agree, in
    /// which case the operator can read the individual numbers from
    /// their respective tables. Empirical.
    pub const RETARGETING_SUPPRESSION_MIN_RATIO: f64 = 2.0;

    /// Seed displayed when a Monte Carlo config omits one. Arbitrary
    /// but fixed so example rendering stays deterministic.
    pub const DISPLAY_DEFAULT_SEED: u64 = 42;
}

/// Rate-comparison thresholds for verdict zero-checks.
pub mod rate {
    /// Rate at or below which "no violations" is reported.
    ///
    /// `collision_probability`, `ei_violation_rate`, and
    /// `keepout_violation_rate` are all computed as integer ratios
    /// (`violations / samples`), so an exact `0.0` comparison is
    /// well-defined here and is *not* the usual float-equality
    /// anti-pattern Codebase warns against. This constant exists to
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
    /// *numerical* (ground-truth) value so the firing boundary stays
    /// stable regardless of how the result is rendered downstream.
    /// The `10%` threshold is empirical: large enough that routine
    /// J2-vs-full-physics rounding differences do not fire the
    /// warning, small enough that any real model mismatch surfaces in
    /// the Safety Comparison table. Lives at the boundary where
    /// analytical and numerical tiers begin to disagree in an
    /// operationally meaningful way. See `analytical_overestimate`
    /// in `verdict.rs` for the single source-of-truth computation.
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
    /// `determine_mc_verdict` / `waypoint_miss_growth`, which look
    /// at waypoint miss medians across the ensemble. `3.0` is chosen
    /// as the point at which linearized-ROE validity is visibly
    /// degrading over the mission horizon — a real model issue
    /// rather than accumulated numerical noise. Empirical.
    pub const ERROR_GROWTH_RATIO_ALERT: f64 = 3.0;

    /// Final-waypoint p95 miss (metres) above which the MC report
    /// emits the "closed-loop targeting degraded" insight. A
    /// kilometer-scale miss on the arrival waypoint implies a
    /// failed approach even with COLA active, so the operator
    /// needs visibility even when the ensemble otherwise reports
    /// clean statistics. Separate from [`ERROR_GROWTH_RATIO_ALERT`]:
    /// this is an absolute-scale gate, that one is a ratio gate.
    /// Empirical; a kilometer is the smallest round number at which
    /// "missed the waypoint" becomes unambiguous regardless of mission
    /// scale.
    pub const MC_FINAL_WAYPOINT_P95_ALERT_M: f64 = 1000.0;

    /// Maximum number of insights surfaced in the above-the-fold alert
    /// box. Empirical UX limit: three keeps the alert block scan-length
    /// bounded so an operator does not need to page past it to reach
    /// the Summary table. Anything beyond three rolls into the bottom
    /// insights list, which is where exhaustive findings belong.
    pub const ALERT_BOX_MAX_ITEMS: usize = 3;
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

    /// Phase angle (degrees) at which the e and i vectors point in
    /// opposite directions. At this angle the e/i separation norm
    /// collapses because the two contributions subtract rather than
    /// add, so the norm can drop from perch-scale to transit-scale
    /// between neighbouring legs even though neither eccentricity
    /// nor inclination vector has changed in magnitude.
    pub const ANTIPARALLEL_PHASE_DEG: f64 = 180.0;

    /// Half-width (degrees) around [`ANTIPARALLEL_PHASE_DEG`] that the
    /// safety renderer annotates as "near anti-parallel". Chosen so
    /// the annotation covers the regime where the collapse is visible
    /// to a reader but the angle is not yet dominated by roundoff.
    /// Empirical; re-tune if a real case surfaces a phase just
    /// outside this window.
    pub const ANTIPARALLEL_TOLERANCE_DEG: f64 = 30.0;

    /// Tolerance (metres) for treating numerical pre-COLA and post-COLA
    /// min 3D distances as equal in the Safety Comparison footnote.
    /// Below this level the two values are within float-format and
    /// integer-metre rounding noise, and the "pre and post are
    /// identical because the primary POCA fires before the COLA burn"
    /// footnote fires. Chosen at 5 cm so single-cm render rounding
    /// does not suppress the footnote and integer-metre display does
    /// not spuriously trigger it.
    pub const POCA_PRE_POST_EQUAL_TOL_M: f64 = 0.05;
}

/// Covariance-section display and interpretation thresholds.
pub mod covariance {
    /// Nominal position magnitude (km) below which the open-loop
    /// fragility ratio is skipped as numerically meaningless.
    ///
    /// 3σ / 0 is undefined; 1 mm ≈ 1 ppm of a typical 1 km chief-deputy
    /// separation — below this, "3σ exceeds nominal" is roundoff noise,
    /// not a real operational warning.
    pub const NEAR_ZERO_NOMINAL_KM: f64 = 1.0e-6;

    /// Terminal 3σ / |nominal| ratio above which the covariance section
    /// emits the open-loop fragility warning. At exactly 1.0 the 3σ
    /// bound equals the nominal magnitude — the uncertainty ellipsoid
    /// is on the verge of swallowing the nominal point. Strict `>`:
    /// the warning does not fire at the boundary.
    pub const OPEN_LOOP_FRAGILITY_RATIO: f64 = 1.0;

    /// Mahalanobis distance below which the chief is considered
    /// "inside the deputy's 1σ uncertainty ellipsoid". Standard
    /// Gauss/χ² interpretation with 3 DOF (3σ ≈ 97.07% containment).
    pub const MAHALANOBIS_INSIDE_1SIGMA: f64 = 1.0;

    /// Mahalanobis distance below which the deputy is considered
    /// "within the 3σ ellipsoid" but not well-separated. 3σ ≈ 97.07%
    /// containment under a 3-DOF Gaussian.
    pub const MAHALANOBIS_INSIDE_3SIGMA: f64 = 3.0;
}

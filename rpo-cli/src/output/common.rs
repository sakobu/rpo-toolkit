//! Shared output helpers.

use std::borrow::Cow;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use serde::Serialize;

use rpo_core::mission::{
    assess_safety, AvoidanceManeuver, ColaConfig, EiAlignment, EnsembleStatistics, FreeDriftAnalysis,
    MonteCarloReport, SafetyConfig, SafetyRequirements, ValidationReport,
};
use rpo_core::pipeline::{
    resolve_propagator, to_propagation_model, PipelineInput, PipelineOutput, TransferResult,
};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;
use rpo_nyx::nyx_bridge::extract_dmf_rates;

use crate::error::CliError;

use super::thresholds::{insight as insight_thresh, mc as mc_thresh, rate as rate_thresh};

/// Whether analytical safety is the governing tier or a baseline for comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyTier {
    /// Analytical is the governing and only fidelity tier (mission command).
    Governing,
    /// Analytical is a baseline; Nyx full-physics validation governs.
    Baseline,
}

/// Default delta-v budget for CLI COLA overlay (km/s).
/// 10 m/s — matches the pipeline auto-COLA default in `execute.rs`.
const DEFAULT_COLA_BUDGET_KM_S: f64 = 0.01;

/// Default formation design enrichment threshold (km).
/// 100 m — a reasonable passive safety margin for close proximity operations.
const DEFAULT_ENRICHMENT_THRESHOLD_KM: f64 = 0.1;

/// CLI overlay flags that mutate a [`PipelineInput`] before execution.
///
/// Groups COLA and formation design enrichment flags to avoid
/// long argument lists in porcelain command `run()` functions.
#[derive(Debug, Clone, Default)]
pub struct OverlayFlags {
    /// Target miss distance for collision avoidance (km).
    pub cola_threshold: Option<f64>,
    /// Maximum delta-v budget for COLA (km/s).
    pub cola_budget: Option<f64>,
    /// Enable formation design enrichment.
    pub auto_enrich: bool,
    /// Custom min separation threshold (km) for formation design.
    pub auto_enrich_threshold: Option<f64>,
}

/// Apply all CLI overlay flags onto a [`PipelineInput`].
///
/// Handles COLA configuration and formation design enrichment in one call.
pub fn apply_overlays(input: &mut PipelineInput, flags: &OverlayFlags) {
    // COLA overlay
    if let Some(threshold) = flags.cola_threshold {
        input.cola = Some(ColaConfig {
            target_distance_km: threshold,
            max_dv_km_s: flags.cola_budget.unwrap_or(DEFAULT_COLA_BUDGET_KM_S),
        });
    }

    // Formation design enrichment overlay
    let separation_km = match (flags.auto_enrich_threshold, flags.auto_enrich) {
        (Some(t), _) => Some(t),
        (None, true) => Some(DEFAULT_ENRICHMENT_THRESHOLD_KM),
        (None, false) => None,
    };
    if let Some(min_separation_km) = separation_km {
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km,
            alignment: EiAlignment::default(),
        });
    }
}

/// Mission feasibility verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// All safety margins satisfied.
    Feasible,
    /// Operational safety passes; passive safety below threshold but non-governing.
    OperationallyFeasible,
    /// Safety margins violated or insufficient data.
    Caution,
}

impl Verdict {
    /// Whether the verdict indicates the mission is feasible (either fully or operationally).
    #[must_use]
    pub fn is_feasible(self) -> bool {
        matches!(self, Self::Feasible | Self::OperationallyFeasible)
    }
}

impl std::fmt::Display for Verdict {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Feasible => write!(f, "FEASIBLE"),
            Self::OperationallyFeasible => write!(f, "OPERATIONALLY FEASIBLE"),
            Self::Caution => write!(f, "CAUTION"),
        }
    }
}

/// Orthogonal annotation on a verdict — independent of [`Verdict`] so that
/// callers matching on feasibility category (`is_feasible`, primary dispatch
/// arms) stay untouched when a new qualifier is added.
///
/// The `Display` impl renders directly into the verdict header, so
/// renderers can interpolate `{qualifier}` without first checking for
/// presence: `VerdictQualifier::None` writes an empty string.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub enum VerdictQualifier {
    /// No qualifier rendered.
    #[default]
    None,
    /// Verdict is FEASIBLE / OPERATIONALLY FEASIBLE but the Nyx 3D margin
    /// is inside [`insight_thresh::TIGHT_MARGIN_FACTOR`] × the keep-out
    /// threshold, or a COLA burn missed its full-physics target.
    /// Renders as `" (AT MARGIN)"`.
    AtMargin,
}

impl std::fmt::Display for VerdictQualifier {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::None => Ok(()),
            Self::AtMargin => write!(f, " (AT MARGIN)"),
        }
    }
}

/// Verdict with contextual reason string and an optional typed qualifier.
pub struct VerdictResult {
    /// The verdict category.
    pub verdict: Verdict,
    /// Typed qualifier rendered directly after the verdict label via
    /// [`VerdictQualifier`]'s `Display` impl.
    pub qualifier: VerdictQualifier,
    /// Human-readable reason for the verdict.
    pub reason: Cow<'static, str>,
}

impl VerdictResult {
    /// Construct a verdict without a qualifier. Use the direct struct-literal
    /// form when a qualifier needs to be attached.
    fn new(verdict: Verdict, reason: Cow<'static, str>) -> Self {
        Self { verdict, qualifier: VerdictQualifier::None, reason }
    }
}

// ── Status / spinner ────────────────────────────────────────────────────

/// Print a status message to the spinner if available, otherwise to stderr.
macro_rules! status {
    ($spinner:expr, $($arg:tt)*) => {
        if let Some(s) = &$spinner {
            s.set_message(format!($($arg)*));
        } else {
            eprintln!($($arg)*);
        }
    };
}
pub(crate) use status;

/// Create an animated spinner for long-running CLI operations.
///
/// The spinner writes to stderr, so it does not interfere with stdout output.
/// Returns `None` if stderr is not a terminal (piped/redirected).
#[must_use]
pub fn create_spinner() -> Option<ProgressBar> {
    if !std::io::IsTerminal::is_terminal(&std::io::stderr()) {
        return None;
    }
    let s = ProgressBar::new_spinner();
    s.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner()),
    );
    s.enable_steady_tick(Duration::from_millis(120));
    Some(s)
}

// ── Drag / propagator resolution ────────────────────────────────────────

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via Nyx DMF
/// and prints them to stderr. Returns the propagation model and optional
/// derived drag config.
///
/// # Errors
///
/// Returns [`CliError`] if drag extraction or propagator resolution fails.
pub fn resolve_drag_and_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    input: &PipelineInput,
    spinner: Option<&ProgressBar>,
) -> Result<(PropagationModel, Option<DragConfig>), CliError> {
    let auto_drag_config = if auto_drag {
        status!(spinner, "Extracting differential drag rates via Nyx...");
        let drag = extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            chief_config,
            deputy_config,
            almanac,
        )?;
        eprintln!(
            "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
        Some(drag)
    } else {
        None
    };
    Ok(resolve_propagator(
        auto_drag_config,
        to_propagation_model(&input.propagator),
    ))
}

// ── Output helpers ─────────────────────────────────────────────────────

/// Create the output sink: a buffered file writer, or locked stdout.
fn open_output(path: Option<&Path>) -> Result<Box<dyn std::io::Write>, CliError> {
    if let Some(p) = path {
        if let Some(parent) = p.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent).map_err(|e| CliError::io(parent, e))?;
            }
        }
        let file = std::fs::File::create(p).map_err(|e| CliError::io(p, e))?;
        Ok(Box::new(std::io::BufWriter::new(file)))
    } else {
        Ok(Box::new(std::io::stdout().lock()))
    }
}

/// Write text content to a file or stdout.
///
/// If `path` is `Some`, writes to that file (creating parent directories)
/// and prints the path to stderr. If `None`, prints to stdout.
///
/// # Errors
///
/// Returns [`CliError::Io`] if directory creation or file write fails.
pub fn output_text(content: &str, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    out.write_all(content.as_bytes())
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Serialize a value as JSON directly to a file or stdout.
///
/// Writes directly to the output stream via [`serde_json::to_writer_pretty`],
/// avoiding an intermediate `String` allocation.
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails, or [`CliError::Io`]
/// if file write fails.
pub fn output_json<T: Serialize>(data: &T, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    serde_json::to_writer_pretty(&mut out, data).map_err(CliError::Serialize)?;
    out.write_all(b"\n")
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Pretty-print a value as JSON to stdout (plumbing commands).
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails.
pub fn print_json<T: Serialize>(value: &T) -> Result<(), CliError> {
    output_json(value, None)
}

// ── Unit formatting ─────────────────────────────────────────────────────

/// Conversion factor from kilometres to metres for display formatting.
pub(crate) const KM_TO_M: f64 = 1000.0;

/// ROE component below which the value is treated as effectively zero for display.
const ROE_DISPLAY_ZERO_THRESHOLD: f64 = 1e-15;

/// RIC frame component labels for display, ordered [Radial, In-track, Cross-track].
const RIC_COMPONENT_LABELS: [&str; 3] = ["R", "I", "C"];

/// Format a value stored in km/s as m/s with given decimal places.
///
/// Example: `fmt_m_s(0.5432, 1)` returns `"543.2 m/s"`.
#[must_use]
pub fn fmt_m_s(value_km_s: f64, decimals: usize) -> String {
    let m_s = value_km_s * KM_TO_M;
    format!("{m_s:.decimals$} m/s")
}

/// Format a value stored in km as meters with given decimal places.
///
/// Example: `fmt_m(0.150, 1)` returns `"150.0 m"`.
#[must_use]
pub fn fmt_m(value_km: f64, decimals: usize) -> String {
    let m = value_km * KM_TO_M;
    format!("{m:.decimals$} m")
}

/// Format a ROE component: scientific notation for non-zero, padded zero for zero.
///
/// Avoids the ugly `+0.000000e0` output for zero values.
#[must_use]
pub fn fmt_roe_component(value: f64) -> String {
    if value.abs() < ROE_DISPLAY_ZERO_THRESHOLD {
        "0".to_string()
    } else {
        format!("{value:+.6e}")
    }
}

// ── Bounded-motion formatting ────────────────────────────────────────────

/// Bounded-motion residual below which the orbit is considered effectively bounded.
/// Residuals below this are display-formatted as "bounded (residual ~ 0)".
const BOUNDED_MOTION_DISPLAY_THRESHOLD: f64 = 1e-10;

/// Format a bounded-motion residual for human display.
#[must_use]
pub fn fmt_bounded_motion_residual(residual: f64) -> String {
    if residual.abs() < BOUNDED_MOTION_DISPLAY_THRESHOLD {
        "bounded (residual ~ 0)".to_string()
    } else {
        format!("drifting (residual = {residual:.3e})")
    }
}

// ── Duration formatting ─────────────────────────────────────────────────

/// Format a duration in seconds as a human-readable string.
///
/// Returns `"Xd YYh"` for durations >= 1 day, `"Xh YYm"` for >= 1 hour,
/// `"Xm YYs"` for >= 1 minute, `"Xs"` for >= 1 second, `"< 1s"` for
/// sub-second, and `"0s"` for zero.
#[must_use]
pub fn fmt_duration(s: f64) -> String {
    // Round to nearest second to avoid fractional artifacts, then format
    // from the integer total. This prevents edge cases like "59m 60s".
    let total_s = s.max(0.0).round();
    if total_s >= 86400.0 {
        let d = (total_s / 86400.0).floor();
        let h = ((total_s - d * 86400.0) / 3600.0).floor();
        format!("{d:.0}d {h:02.0}h")
    } else if total_s >= 3600.0 {
        let h = (total_s / 3600.0).floor();
        let m = ((total_s - h * 3600.0) / 60.0).floor();
        format!("{h:.0}h {m:02.0}m")
    } else if total_s >= 60.0 {
        let m = (total_s / 60.0).floor();
        let sec = total_s - m * 60.0;
        format!("{m:.0}m {sec:02.0}s")
    } else if total_s >= 1.0 {
        format!("{total_s:.0}s")
    } else if s > 0.0 {
        "< 1s".to_string()
    } else {
        "0s".to_string()
    }
}


// ── Velocity target formatting ──────────────────────────────────────────

/// Format a velocity target vector for the waypoint table.
///
/// - Zero vector → "—"
/// - Single dominant component (≥ 70% of magnitude) → signed value + direction tag (e.g. "+1.0 I")
/// - No dominant component → magnitude only (e.g. "1.4")
///
/// All values displayed in m/s.
#[must_use]
pub fn fmt_velocity_target(v: &nalgebra::Vector3<f64>) -> String {
    use super::thresholds::velocity::{DOMINANT_COMPONENT_FRACTION, ZERO_MAGNITUDE_KM_S};

    let mag = v.norm();
    if mag < ZERO_MAGNITUDE_KM_S {
        return "\u{2014}".to_string();
    }

    let abs_components = [v.x.abs(), v.y.abs(), v.z.abs()];
    let values = [v.x, v.y, v.z];

    for (i, &abs_val) in abs_components.iter().enumerate() {
        if abs_val / mag >= DOMINANT_COMPONENT_FRACTION {
            let sign = if values[i] >= 0.0 { "+" } else { "-" };
            let val_m_s = abs_val * KM_TO_M;
            return format!("{sign}{val_m_s:.1} {}", RIC_COMPONENT_LABELS[i]);
        }
    }

    // No single dominant component — show magnitude only
    let mag_m_s = mag * KM_TO_M;
    format!("{mag_m_s:.1}")
}

// ── COLA helpers ────────────────────────────────────────────────────────

/// Compute total COLA Δv and burn count from avoidance maneuvers.
#[must_use]
pub fn cola_dv_summary(cola: Option<&[AvoidanceManeuver]>) -> Option<(f64, usize)> {
    cola.map(|maneuvers| {
        let total: f64 = maneuvers.iter().map(|m| m.fuel_cost_km_s).sum();
        (total, maneuvers.len())
    })
}

// ── MC baseline ─────────────────────────────────────────────────────────

/// Baseline mission parameters passed to MC formatters.
pub struct McBaseline {
    /// Lambert transfer Dv (km/s).
    pub lambert_dv_km_s: f64,
    /// Lambert transfer time of flight (s).
    pub lambert_tof_s: f64,
    /// Waypoint targeting Dv (km/s).
    pub waypoint_dv_km_s: f64,
    /// Waypoint phase duration (s).
    pub waypoint_duration_s: f64,
    /// Number of waypoint legs.
    pub num_legs: usize,
    /// Whether the mission is far-field (Lambert transfer required).
    pub is_far_field: bool,
}

impl McBaseline {
    /// Total Dv: Lambert transfer + waypoint targeting.
    #[must_use]
    pub fn total_dv_km_s(&self) -> f64 {
        self.lambert_dv_km_s + self.waypoint_dv_km_s
    }
}

// ── Verdict determination ───────────────────────────────────────────────

/// Determine the mission verdict from safety assessment.
///
/// Returns a [`VerdictResult`] with verdict category and human-readable reason.
/// If validation data is provided, uses numerical safety; otherwise uses analytical.
///
/// The `qualifier` field is set to `" (AT MARGIN)"` when validate-tier data
/// shows the Nyx 3D margin below [`insight_thresh::TIGHT_MARGIN_FACTOR`] × the
/// keep-out threshold, or when any COLA burn missed its full-physics target.
///
/// After the primary verdict is constructed, the free-drift analysis is
/// walked to classify abort-case passive safety per leg:
/// - legs with e/i separation below `ABORT_EI_COLLAPSE_RATIO` × threshold are
///   "essentially lost" (effectively zero),
/// - legs with e/i separation below threshold but above the collapse ratio are
///   "marginal" (just below cutoff).
///
/// Both classes are surfaced in the verdict reason so a skim reader sees the
/// per-leg distinction instead of a single soft "marginal" label.
#[must_use]
pub fn determine_verdict(
    output: &PipelineOutput,
    config: &SafetyConfig,
    validation: Option<&ValidationReport>,
) -> VerdictResult {
    let enrichment_active = output.formation_design.is_some();

    let mut verdict = if let Some(report) = validation {
        let assessment = assess_safety(&report.numerical_safety, config);
        if assessment.overall_pass {
            VerdictResult::new(
                Verdict::Feasible,
                Cow::Borrowed("safety margins satisfied, Nyx full-physics"),
            )
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult::new(
                Verdict::OperationallyFeasible,
                Cow::Borrowed(
                    "3D distance passes (Nyx); e/i below threshold (non-governing for guided V-bar targeting)",
                ),
            )
        } else {
            VerdictResult::new(
                Verdict::Caution,
                Cow::Borrowed("safety margins violated, Nyx full-physics"),
            )
        }
    } else if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, config);
        if assessment.overall_pass {
            VerdictResult::new(
                Verdict::Feasible,
                Cow::Borrowed("analytical safety margins satisfied"),
            )
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult::new(
                Verdict::OperationallyFeasible,
                Cow::Borrowed(
                    "3D distance passes; e/i below threshold (non-governing for guided V-bar targeting)",
                ),
            )
        } else {
            VerdictResult::new(
                Verdict::Caution,
                Cow::Borrowed("analytical safety margins violated"),
            )
        }
    } else {
        VerdictResult::new(
            Verdict::Feasible,
            Cow::Borrowed("no safety configured"),
        )
    };

    // Validate-tier qualifier: downgrade to "(AT MARGIN)" when the Nyx
    // margin is tight or COLA burns missed their full-physics target.
    if let Some(report) = validation {
        if verdict.verdict == Verdict::OperationallyFeasible {
            let num_3d_km = report.numerical_safety.operational.min_distance_3d_km;
            let d3d_ratio = margin_ratio(num_3d_km, config.min_distance_3d_km);
            let tight_margin = d3d_ratio < insight_thresh::TIGHT_MARGIN_FACTOR;
            let cola_missed = report
                .cola_effectiveness
                .iter()
                .any(|eff| eff.threshold_met == Some(false));
            if tight_margin || cola_missed {
                verdict.qualifier = VerdictQualifier::AtMargin;
            }
        }
    }

    if let Some(ref free_drift) = output.safety.free_drift {
        if let Some(text) = classify_abort_case_passive_safety(free_drift, config) {
            // Use a semicolon separator rather than nesting another
            // parenthesised clause. The outer verdict renderer wraps
            // `verdict.reason` in a single pair of parens, so this yields
            // a flat "(a; b; c)" list instead of "(a (b) (c))".
            verdict.reason = Cow::Owned(format!("{}; {text}", verdict.reason));
        }
    }

    verdict
}

/// Classify each free-drift leg as "marginal" (just below threshold) or
/// "essentially lost" (well below threshold), and render a single verdict
/// suffix that lists both groups with concrete separation values in
/// metres.
///
/// Classification rules (per leg):
/// - leg passes if its free-drift e/i separation is at or above
///   `config.min_ei_separation_km`
/// - leg is "essentially lost" if its separation is strictly below
///   `config.min_ei_separation_km * insight_thresh::ABORT_EI_COLLAPSE_RATIO`
///   (one order of magnitude below the enforced threshold)
/// - leg is "marginal" if its separation is below the threshold but at
///   or above the collapse cutoff
///
/// Returns `None` when (a) `config.min_ei_separation_km <= 0.0` (the
/// threshold-disabled guard) or (b) every leg satisfies the configured
/// threshold. Otherwise returns `Some(text)` where leg indices are
/// 1-based and separation values are rendered in metres to one decimal
/// place via [`render_leg_sep_list`] + [`oxford_join`].
fn classify_abort_case_passive_safety(
    free_drift: &[FreeDriftAnalysis],
    config: &SafetyConfig,
) -> Option<String> {
    let threshold_km = config.min_ei_separation_km;
    if threshold_km <= 0.0 {
        return None;
    }
    let lost_cutoff_km = threshold_km * insight_thresh::ABORT_EI_COLLAPSE_RATIO;

    // (1-based leg index, separation in metres)
    let mut marginal: Vec<(usize, f64)> = Vec::new();
    let mut lost: Vec<(usize, f64)> = Vec::new();
    for (i, fd) in free_drift.iter().enumerate() {
        let separation_km = fd.safety.passive.min_ei_separation_km;
        if separation_km >= threshold_km {
            continue;
        }
        let leg = i + 1;
        let separation_metres = separation_km * KM_TO_M;
        if separation_km < lost_cutoff_km {
            lost.push((leg, separation_metres));
        } else {
            marginal.push((leg, separation_metres));
        }
    }

    if marginal.is_empty() && lost.is_empty() {
        return None;
    }

    let mut parts: Vec<String> = Vec::new();
    if !marginal.is_empty() {
        parts.push(format!(
            "abort-case passive safety is marginal on {}",
            render_leg_sep_list(&marginal),
        ));
    }
    if !lost.is_empty() {
        let prefix = if marginal.is_empty() {
            "abort-case passive safety is essentially lost on"
        } else {
            "essentially lost on"
        };
        parts.push(format!("{prefix} {}", render_leg_sep_list(&lost)));
    }
    Some(parts.join(" and "))
}

/// Render a list of `(leg_1_based, separation_metres)` pairs as a
/// human-readable string with Oxford-comma style: `"leg 1 (99.8 m)"`,
/// `"leg 2 (0.1 m) and leg 3 (0.2 m)"`, or `"leg 2 (0.1 m), leg 3 (0.2 m),
/// and leg 4 (0.3 m)"`.
///
/// Separations MUST already be in metres — the helper does not convert
/// from km. Leg indices MUST already be 1-based. Format precision is
/// fixed at one decimal place.
fn render_leg_sep_list(items: &[(usize, f64)]) -> String {
    let formatted: Vec<String> = items
        .iter()
        .map(|(leg, separation_metres)| format!("leg {leg} ({separation_metres:.1} m)"))
        .collect();
    oxford_join(&formatted)
}

/// Join a list of pre-formatted items with Oxford-comma style:
///
/// - `[]` → `""`
/// - `[a]` → `"a"`
/// - `[a, b]` → `"a and b"`
/// - `[a, b, c, ...]` → `"a, b, ..., and last"`
///
/// Shared by the verdict-reason leg enumeration (`render_leg_sep_list`
/// above) and the COLA callout leg enumeration (`format_leg_list` in
/// `markdown_fmt/mission.rs`) so every renderer in the crate speaks the
/// same English.
#[must_use]
pub(crate) fn oxford_join(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [single] => single.clone(),
        [a, b] => format!("{a} and {b}"),
        multi => {
            let head = multi[..multi.len() - 1].join(", ");
            let last = &multi[multi.len() - 1];
            format!("{head}, and {last}")
        }
    }
}

/// First and last populated waypoint-miss medians, their growth ratio,
/// and the total waypoint count for rendering "WP1 → WPN" anchors.
///
/// Computed in one pass over `stats.waypoint_miss_km` so the MC verdict
/// renderer and the `mc_waypoint_growth_insight` stay in lockstep.
pub(crate) struct WaypointMissGrowth {
    /// `last_p50_km / first_p50_km` — dimensionless growth ratio of
    /// the median waypoint miss from the first populated waypoint to
    /// the last populated waypoint.
    pub ratio: f64,
    /// `p50` of the first populated waypoint's miss distribution, in km.
    pub first_p50_km: f64,
    /// `p50` of the last populated waypoint's miss distribution, in km.
    pub last_p50_km: f64,
    /// 1-based index of the last waypoint in `stats.waypoint_miss_km`.
    /// Matches the `WP{last_waypoint_index}` anchor used in the rendered
    /// verdict reason and the `mc_waypoint_last_p95_insight` wording.
    pub last_waypoint_index: usize,
}

/// Compute a [`WaypointMissGrowth`] for the ensemble, or `None` when
/// growth is undefined.
///
/// Returns `None` when fewer than two waypoints carry populated
/// statistics, or when the first populated median is non-positive
/// (ratio would be undefined). Callers must treat `None` as "no growth
/// signal", not "zero growth".
///
/// Used by [`determine_mc_verdict`] (to append growth text to the
/// verdict reason) and by `insights::mc_waypoint_growth_insight` (to
/// emit the matching info-level insight). Keeping the walk in one
/// place — and returning all four values from a single pass — prevents
/// the two consumers from drifting.
pub(crate) fn waypoint_miss_growth(stats: &EnsembleStatistics) -> Option<WaypointMissGrowth> {
    let mut iter = stats.waypoint_miss_km.iter().filter_map(|opt| opt.as_ref());
    let first_stats = iter.next()?;
    // `last` remains the most recent populated entry across the walk;
    // after the loop finishes it holds the final populated waypoint.
    let mut last_stats = iter.next()?;
    for stats in iter {
        last_stats = stats;
    }
    let first_p50_km = first_stats.p50;
    let last_p50_km = last_stats.p50;
    if first_p50_km <= 0.0 {
        return None;
    }
    Some(WaypointMissGrowth {
        ratio: last_p50_km / first_p50_km,
        first_p50_km,
        last_p50_km,
        last_waypoint_index: stats.waypoint_miss_km.len(),
    })
}

/// Attribution for a Monte Carlo ensemble's e/i-violation rate.
///
/// The MC verdict reason branches on this classification to distinguish
/// "nominal design already fails e/i" (every sample inherits the nominal
/// failure — not dispersion-driven) from "nominal passes, but dispersion
/// pushes some samples below the threshold" (classically dispersion-driven).
/// Encoded as an enum rather than a boolean pair so the three-way decision
/// is explicit at every match site.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum EiClassification {
    /// No samples violate the e/i threshold.
    NoViolation,
    /// Nominal e/i is already below the configured threshold, so every
    /// sample inherits the nominal failure — the ensemble is "consistent
    /// with the nominal", not "degraded under dispersion".
    NominalFailure,
    /// Nominal e/i passes the threshold but dispersion pushes some samples
    /// below it. The classic "passive abort safety degraded under
    /// dispersion" case.
    DispersionInduced,
}

impl EiClassification {
    /// Classify the e/i-violation regime for a Monte Carlo report.
    ///
    /// `ei_degraded` MUST match `stats.ei_violation_rate > 0` — it is
    /// passed in so the caller can reuse the same value for verdict
    /// category dispatch without recomputing the comparison.
    fn classify(
        report: &MonteCarloReport,
        config: &SafetyConfig,
        ei_degraded: bool,
    ) -> Self {
        if !ei_degraded {
            return Self::NoViolation;
        }
        match report.nominal_safety.as_ref() {
            Some(nominal)
                if nominal.passive.min_ei_separation_km < config.min_ei_separation_km =>
            {
                Self::NominalFailure
            }
            _ => Self::DispersionInduced,
        }
    }
}

/// Determine the MC verdict from ensemble statistics.
///
/// Three-tier logic:
/// - **FEASIBLE (100%):** all converged, no collisions, no keep-out violations.
/// - **FEASIBLE (acceptable):** convergence >= 95%, no collisions.
/// - **CAUTION:** otherwise.
///
/// When `ei_violation_rate > 0` the verdict distinguishes two cases:
/// - **Nominal already fails** (`nominal_ei < threshold`): the 100% violation
///   rate is not dispersion-driven — the ensemble inherits the nominal failure.
///   The verdict reason calls this out with "consistent with the X.X m nominal".
/// - **Nominal passes** but dispersion pushes samples below threshold: the
///   classic "degraded under dispersion" framing.
///
/// When [`waypoint_miss_growth`] exceeds
/// [`insight_thresh::ERROR_GROWTH_RATIO_ALERT`], the growth is appended to the
/// verdict reason as "waypoint-miss dispersion grows X× from WP1 (…p50) to
/// WPN (…p50)" — growth alone never flips the verdict tier. The metric name
/// is deliberately distinct from validate.md's "position error" (Nyx vs
/// analytical RIC residual) so cross-report comparisons are unambiguous.
#[must_use]
pub fn determine_mc_verdict(report: &MonteCarloReport, config: &SafetyConfig) -> VerdictResult {
    let stats = &report.statistics;
    let all_converged = (stats.convergence_rate - 1.0).abs() < mc_thresh::CONVERGENCE_EXACT_TOL;
    let no_collisions = stats.collision_probability <= rate_thresh::ZERO_VIOLATIONS;
    let no_keepout = stats.keepout_violation_rate <= rate_thresh::ZERO_VIOLATIONS;

    let ei_degraded = stats.ei_violation_rate > rate_thresh::ZERO_VIOLATIONS;
    let ei_classification = EiClassification::classify(report, config, ei_degraded);

    let base_reason: Cow<'static, str> = if all_converged && no_collisions && no_keepout {
        match ei_classification {
            EiClassification::NoViolation => Cow::Borrowed("all safety margins satisfied"),
            EiClassification::NominalFailure => {
                let nominal_ei_m = report
                    .nominal_safety
                    .as_ref()
                    .map_or(0.0, |s| s.passive.min_ei_separation_km * KM_TO_M);
                let n = f64::from(report.config.num_samples);
                let viol = (stats.ei_violation_rate * n).round();
                Cow::Owned(format!(
                    "operational safety holds; passive e/i advisory fails in \
                     {viol:.0}/{} samples, consistent with the {nominal_ei_m:.1} m nominal",
                    report.config.num_samples,
                ))
            }
            EiClassification::DispersionInduced => Cow::Borrowed(
                "operational safety holds; passive abort safety degraded under dispersion",
            ),
        }
    } else if stats.convergence_rate >= mc_thresh::CONVERGENCE_ALERT && no_collisions {
        match ei_classification {
            EiClassification::NoViolation => Cow::Borrowed("convergence acceptable, no collisions"),
            EiClassification::NominalFailure => Cow::Borrowed(
                "convergence acceptable; passive e/i advisory fails consistent with the nominal",
            ),
            EiClassification::DispersionInduced => Cow::Borrowed(
                "convergence acceptable; passive abort safety degraded under dispersion",
            ),
        }
    } else {
        Cow::Borrowed("review safety and convergence above")
    };

    let verdict_category = if !all_converged || !no_collisions || !no_keepout {
        if stats.convergence_rate >= mc_thresh::CONVERGENCE_ALERT && no_collisions {
            if ei_degraded { Verdict::OperationallyFeasible } else { Verdict::Feasible }
        } else {
            Verdict::Caution
        }
    } else if ei_degraded {
        Verdict::OperationallyFeasible
    } else {
        Verdict::Feasible
    };

    let mut result = VerdictResult::new(verdict_category, base_reason);

    if let Some(growth) = waypoint_miss_growth(stats) {
        if growth.ratio > insight_thresh::ERROR_GROWTH_RATIO_ALERT {
            let first_m = growth.first_p50_km * KM_TO_M;
            let last_m = growth.last_p50_km * KM_TO_M;
            let last_wp = growth.last_waypoint_index;
            let ratio = growth.ratio;
            result.reason = Cow::Owned(format!(
                "{} (waypoint-miss dispersion grows {ratio:.1}\u{00d7} from WP1 \
                 ({first_m:.0} m p50) to WP{last_wp} ({last_m:.0} m p50))",
                result.reason,
            ));
        }
    }

    result
}

/// Absolute and relative overestimate of an analytical safety metric
/// against its numerical (Nyx full-physics) counterpart, or `None` when
/// analytical is conservative.
///
/// Both fields are derived from the same `(ana_km, num_km)` pair so the
/// Safety Comparison table footnote, the validation insight list, and
/// the COLA analytical-overestimate insight cannot drift out of sync.
/// The `ratio` field replaces the older "overestimated by N%" phrasing
/// that readers routinely misparsed as "analytical is 1.65× Nyx" when
/// a 165% overestimate actually means 2.65× Nyx.
///
/// # Invariants
/// - When returned inside `Some(_)`, `delta_m > 0.0` and `ratio > 1.0`.
/// - `delta_m` is in metres (converted from the km inputs internally).
/// - `ratio` is the dimensionless `ana_km / num_km`.
pub(crate) struct AnalyticalOverestimate {
    /// `(ana_km - num_km) * KM_TO_M` — how many metres analytical
    /// overstates the numerical value.
    pub delta_m: f64,
    /// `ana_km / num_km` — multiplicative overestimate factor.
    pub ratio: f64,
}

/// Compute an [`AnalyticalOverestimate`] when the numerical trajectory
/// is non-conservative relative to analytical, or `None` otherwise.
///
/// Returns `None` when either input is non-positive (guards against
/// missing data and divide-by-zero) or when the overestimate stays
/// within [`insight_thresh::SIGNIFICANT_DELTA_PCT`] percent of the
/// numerical value. The threshold check uses the classical
/// `(ana - num) / num` percentage formulation so the firing boundary
/// stays stable regardless of how the result is rendered downstream.
///
/// # Arguments
/// - `ana_km` — analytical safety metric in kilometres (3D distance or
///   e/i separation).
/// - `num_km` — numerical (Nyx) safety metric in kilometres, same
///   quantity as `ana_km`.
///
/// # Single source of truth
/// Callers must never compute `delta_m`/`ratio` directly. Use the
/// struct fields so every call site stays in lockstep:
/// - `validation_insights` 3D overestimate warning (`insights.rs`)
/// - `cola_insights` analytical overestimation info (`insights.rs`)
/// - validate leg table "Analytical bias" row (`markdown_fmt/mission.rs`)
/// - Safety Comparison `\*` footnote + post-table annotation
///   (`markdown_fmt/mission.rs`, both the 3D and e/i rows)
#[must_use]
pub(crate) fn analytical_overestimate(
    ana_km: f64,
    num_km: f64,
) -> Option<AnalyticalOverestimate> {
    if ana_km <= 0.0 || num_km <= 0.0 {
        return None;
    }
    let delta_pct = (ana_km - num_km) / num_km * insight_thresh::PERCENT_PER_UNIT;
    if delta_pct <= insight_thresh::SIGNIFICANT_DELTA_PCT {
        return None;
    }
    Some(AnalyticalOverestimate {
        delta_m: (ana_km - num_km) * KM_TO_M,
        ratio: ana_km / num_km,
    })
}

/// Ratio of a measured safety metric to its configured threshold.
///
/// Returns [`f64::INFINITY`] when the threshold is non-positive, so callers
/// can render the ratio unconditionally without a divide-by-zero branch.
/// Both inputs carry the `_km` suffix by convention but the helper is
/// unit-agnostic — any two quantities with matching units work.
///
/// # Arguments
/// - `value_km` — measured safety metric (e.g. numerical 3D distance or e/i separation)
/// - `threshold_km` — configured minimum from [`SafetyConfig`]
///
/// # Returns
/// `value_km / threshold_km` when `threshold_km > 0.0`, else `f64::INFINITY`.
#[must_use]
pub(crate) fn margin_ratio(value_km: f64, threshold_km: f64) -> f64 {
    if threshold_km > 0.0 {
        value_km / threshold_km
    } else {
        f64::INFINITY
    }
}

/// Render a margin / shortfall row cell for a Passive Safety or Free-Drift table.
///
/// Single source of truth for "headroom vs deficit" wording. When the value
/// is at or above the threshold, returns `"Margin | +X.X m"`. When it falls
/// below the threshold, returns `"Shortfall | X.X m (separation Y.Y m / Z m
/// required)"`. A negative number is never labelled "Margin" — a margin below
/// zero is not a margin at all.
///
/// Both inputs are in kilometres so the helper composes with the rest of the
/// safety renderer's unit convention. The rendered row includes the leading
/// pipe and excludes the trailing newline — callers append `"\n"` as needed.
#[must_use]
pub(crate) fn margin_or_shortfall_row(value_km: f64, threshold_km: f64) -> String {
    let value_metres = value_km * KM_TO_M;
    let threshold_metres = threshold_km * KM_TO_M;
    if value_km >= threshold_km {
        format!("| Margin | +{:.1} m |", value_metres - threshold_metres)
    } else {
        format!(
            "| Shortfall | {:.1} m (separation {:.1} m / {:.0} m required) |",
            threshold_metres - value_metres,
            value_metres,
            threshold_metres,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        FormationDesignReport, FreeDriftAnalysis, MonteCarloConfig, MonteCarloMode,
        MonteCarloReport, OperationalSafety, PassiveSafety, PercentileStats,
        PerchEnrichmentResult, SafetyConfig, SafetyMetrics, WaypointMission,
    };
    use rpo_core::pipeline::SafetyAnalysis;
    use rpo_core::types::QuasiNonsingularROE;

    fn stats_with_waypoint_medians(medians: &[Option<f64>]) -> EnsembleStatistics {
        EnsembleStatistics {
            total_dv_km_s: PercentileStats::default(),
            min_rc_distance_km: None,
            min_3d_distance_km: None,
            min_ei_separation_km: None,
            waypoint_miss_km: medians
                .iter()
                .map(|m| {
                    m.map(|p50| PercentileStats {
                        p50,
                        ..Default::default()
                    })
                })
                .collect(),
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 0.0,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![],
        }
    }

    fn clean_report(stats: EnsembleStatistics) -> MonteCarloReport {
        MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 100,
                dispersions: rpo_core::mission::DispersionConfig::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.0,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 0.0,
            covariance_cross_check: None,
        }
    }

    #[test]
    fn waypoint_miss_growth_none_for_insufficient_data() {
        let stats = stats_with_waypoint_medians(&[]);
        assert!(waypoint_miss_growth(&stats).is_none());
        let stats = stats_with_waypoint_medians(&[Some(0.001)]);
        assert!(waypoint_miss_growth(&stats).is_none());
    }

    #[test]
    fn waypoint_miss_growth_skips_none_entries() {
        // First median 1 m, last median 5 m, intermediate None must not break walking.
        // last_waypoint_index follows the vec length (4), not the populated count.
        let stats =
            stats_with_waypoint_medians(&[Some(0.001), None, Some(0.003), Some(0.005)]);
        let growth = waypoint_miss_growth(&stats).expect("growth");
        assert!((growth.ratio - 5.0).abs() < 1e-12);
        assert!((growth.first_p50_km - 0.001).abs() < 1e-12);
        assert!((growth.last_p50_km - 0.005).abs() < 1e-12);
        assert_eq!(growth.last_waypoint_index, 4);
    }

    #[test]
    fn waypoint_miss_growth_none_when_first_is_zero() {
        let stats = stats_with_waypoint_medians(&[Some(0.0), Some(0.005)]);
        assert!(waypoint_miss_growth(&stats).is_none());
    }

    #[test]
    fn analytical_overestimate_firing_boundary_and_struct_values() {
        // SIGNIFICANT_DELTA_PCT = 10.0 means the predicate fires when
        //   (ana - num) / num > 0.10
        // i.e. num < ana / 1.10 ≈ 0.9091 × ana. This test pins that
        // boundary so the Option gate and the struct contents stay in
        // lockstep across every call site in the crate.
        let ana_km = 100.0;

        // Well inside the non-conservative region.
        assert!(analytical_overestimate(ana_km, 80.0).is_some());
        assert!(analytical_overestimate(ana_km, 90.0).is_some());

        // Dead band just above 0.9091 × ana — predicate must fire here.
        assert!(
            analytical_overestimate(ana_km, 90.5).is_some(),
            "num = 0.905 × ana → delta_pct ≈ 10.5% must fire",
        );
        assert!(
            analytical_overestimate(ana_km, 90.9).is_some(),
            "num = 0.909 × ana → delta_pct ≈ 10.01% must fire",
        );

        // Just above the boundary — predicate must NOT fire.
        assert!(
            analytical_overestimate(ana_km, 91.0).is_none(),
            "num = 0.910 × ana → delta_pct ≈ 9.89% must not fire",
        );
        assert!(analytical_overestimate(ana_km, 100.0).is_none());
        assert!(analytical_overestimate(ana_km, 110.0).is_none());

        // Guards — non-positive inputs must never fire.
        assert!(analytical_overestimate(0.0, 50.0).is_none());
        assert!(analytical_overestimate(-1.0, 50.0).is_none());
        assert!(analytical_overestimate(100.0, 0.0).is_none());
        assert!(analytical_overestimate(100.0, -1.0).is_none());

        // 110 vs 100 → delta_pct = 10.0 exactly. Strict inequality at
        // SIGNIFICANT_DELTA_PCT means exactly 10% does not fire.
        assert!(analytical_overestimate(110.0, 100.0).is_none());

        // Struct contents at a representative firing point.
        // 0.120 km analytical vs 0.100 km Nyx → delta = 20 m, ratio = 1.2.
        let ovr = analytical_overestimate(0.120, 0.100)
            .expect("20% overestimate must exceed the 10% threshold");
        assert!(
            (ovr.delta_m - 20.0).abs() < 1e-9,
            "delta_m must be (0.120 - 0.100) × 1000 = 20 m, got {}",
            ovr.delta_m
        );
        assert!(
            (ovr.ratio - 1.2).abs() < 1e-9,
            "ratio must be 0.120 / 0.100 = 1.2, got {}",
            ovr.ratio
        );
    }

    #[test]
    fn margin_ratio_returns_infinity_for_zero_threshold() {
        // 5 / 2 = 2.5 exact in IEEE 754, so a tight epsilon pin is fine.
        assert!((margin_ratio(5.0, 2.0) - 2.5).abs() < 1e-12);
        assert!(margin_ratio(5.0, 0.0).is_infinite());
        assert!(margin_ratio(5.0, -1.0).is_infinite());
    }

    fn default_mc_config() -> SafetyConfig {
        SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        }
    }

    #[test]
    fn determine_mc_verdict_appends_waypoint_growth_when_above_alert() {
        // First 1m, last 5m → 5.0× > 3.0 threshold.
        let stats = stats_with_waypoint_medians(&[Some(0.001), Some(0.005)]);
        let report = clean_report(stats);
        let verdict = determine_mc_verdict(&report, &default_mc_config());
        assert!(verdict.verdict.is_feasible());
        assert!(
            verdict.reason.contains("waypoint-miss dispersion"),
            "reason should include waypoint-miss dispersion text, got: {}",
            verdict.reason
        );
        assert!(
            verdict.reason.contains("5.0"),
            "reason should include 5.0 ratio, got: {}",
            verdict.reason
        );
        // The new wording includes explicit WP1 and WPN miss values.
        assert!(
            verdict.reason.contains("WP1"),
            "reason should identify WP1 anchor, got: {}",
            verdict.reason
        );
        assert!(
            verdict.reason.contains("WP2"),
            "reason should identify the last waypoint (WP2 in this fixture), got: {}",
            verdict.reason
        );
        // Legacy "position error growth" phrasing must not return — it
        // collides with validate.md's Nyx-vs-analytical residual metric.
        assert!(
            !verdict.reason.contains("position error growth"),
            "legacy phrasing must not reappear, got: {}",
            verdict.reason
        );
    }

    #[test]
    fn determine_mc_verdict_omits_growth_when_below_alert() {
        // First 1m, last 2m → 2.0× ≤ 3.0 threshold.
        let stats = stats_with_waypoint_medians(&[Some(0.001), Some(0.002)]);
        let report = clean_report(stats);
        let verdict = determine_mc_verdict(&report, &default_mc_config());
        assert!(!verdict.reason.contains("waypoint-miss dispersion"));
    }

    #[test]
    fn determine_mc_verdict_attributes_nominal_failure_when_nominal_ei_fails() {
        // Nominal e/i separation (2.2 m) is already below the 100 m threshold,
        // and every sample in the ensemble fails e/i. The verdict must NOT
        // blame "dispersion" — it should call out the nominal failure instead.
        let mut stats = stats_with_waypoint_medians(&[]);
        stats.ei_violation_rate = 1.0;
        let mut report = clean_report(stats);
        report.nominal_safety = Some(safety_metrics_with(1.0, 0.0022));

        let verdict = determine_mc_verdict(&report, &default_mc_config());
        assert!(
            verdict.reason.contains("consistent with the 2.2 m nominal"),
            "reason should attribute failure to the nominal, got: {}",
            verdict.reason
        );
        assert!(
            !verdict.reason.contains("degraded under dispersion"),
            "reason must not blame dispersion when the nominal already fails, got: {}",
            verdict.reason
        );
    }

    #[test]
    fn determine_mc_verdict_attributes_dispersion_when_nominal_passes() {
        // Nominal e/i is healthy (150 m > 100 m threshold), but some samples
        // still fail e/i — that IS dispersion-driven.
        let mut stats = stats_with_waypoint_medians(&[]);
        stats.ei_violation_rate = 0.10;
        let mut report = clean_report(stats);
        report.nominal_safety = Some(safety_metrics_with(1.0, 0.150));

        let verdict = determine_mc_verdict(&report, &default_mc_config());
        assert!(
            verdict.reason.contains("degraded under dispersion"),
            "reason should attribute failure to dispersion, got: {}",
            verdict.reason
        );
    }

    #[test]
    fn fmt_velocity_target_zero() {
        assert_eq!(fmt_velocity_target(&Vector3::zeros()), "\u{2014}");
    }

    #[test]
    fn fmt_velocity_target_dominant_positive_intrack() {
        // 100% in-track: 0.001 km/s = 1.0 m/s
        let v = Vector3::new(0.0, 0.001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "+1.0 I");
    }

    #[test]
    fn fmt_velocity_target_dominant_negative_intrack() {
        let v = Vector3::new(0.0, -0.001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "-1.0 I");
    }

    #[test]
    fn fmt_velocity_target_no_dominant() {
        // Three roughly equal components — none reaches 70%
        let v = Vector3::new(0.001, 0.001, 0.001);
        let mag_m_s = v.norm() * KM_TO_M;
        assert_eq!(fmt_velocity_target(&v), format!("{mag_m_s:.1}"));
    }

    #[test]
    fn fmt_velocity_target_dominant_crosstrack() {
        // ~98% cross-track
        let v = Vector3::new(0.0002, 0.0, 0.001);
        assert_eq!(fmt_velocity_target(&v), "+1.0 C");
    }

    #[test]
    fn fmt_velocity_target_dominant_radial() {
        let v = Vector3::new(0.002, 0.0001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "+2.0 R");
    }

    // ── Verdict helpers & tests ─────────────────────────────────────────

    /// Build a `SafetyMetrics` with the given 3D distance and e/i separation.
    fn safety_metrics_with(min_distance_3d_km: f64, min_ei_separation_km: f64) -> SafetyMetrics {
        SafetyMetrics {
            operational: OperationalSafety {
                min_rc_separation_km: min_distance_3d_km,
                min_distance_3d_km,
                min_rc_leg_index: 0,
                min_rc_elapsed_s: 0.0,
                min_rc_ric_position_km: Vector3::zeros(),
                min_3d_leg_index: 0,
                min_3d_elapsed_s: 0.0,
                min_3d_ric_position_km: Vector3::zeros(),
            },
            passive: PassiveSafety {
                min_ei_separation_km,
                de_magnitude: 0.0,
                di_magnitude: 0.0,
                ei_phase_angle_rad: 0.0,
            },
        }
    }

    /// Build a `FreeDriftAnalysis` whose passive e/i separation equals
    /// `min_ei_separation_km`. Used only for verdict tests — trajectory is
    /// empty because `determine_verdict` does not walk it.
    fn free_drift_analysis_with_ei(min_ei_separation_km: f64) -> FreeDriftAnalysis {
        FreeDriftAnalysis {
            trajectory: vec![],
            safety: safety_metrics_with(1.0, min_ei_separation_km),
            bounded_motion_residual: 0.0,
        }
    }

    /// Minimal `PipelineOutput` with enrichment active and numerical safety
    /// passing under 3D distance but failing under e/i separation — the
    /// configuration that triggers the `OperationallyFeasible` branch in
    /// `determine_verdict`.
    fn pipeline_output_passing_3d_with_enrichment() -> PipelineOutput {
        use rpo_core::mission::MissionPhase;
        use rpo_core::types::KeplerianElements;

        let placeholder_elements = KeplerianElements {
            a_km: 6786.0,
            e: 0.0001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let placeholder_roe = QuasiNonsingularROE::default();

        let mission = WaypointMission {
            legs: vec![],
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            // 3D distance passes (1 km > 0.050 threshold), e/i fails
            // (0.001 km < 0.100 threshold). Triggers OperationallyFeasible
            // when enrichment is active.
            safety: Some(safety_metrics_with(1.0, 0.001)),
            covariance: None,
            eclipse: None,
        };

        let formation_design = FormationDesignReport {
            perch: PerchEnrichmentResult::Baseline(placeholder_roe),
            waypoints: vec![],
            transit_safety: vec![],
            mission_min_ei_separation_km: None,
            drift_prediction: None,
        };

        PipelineOutput {
            phase: MissionPhase::Proximity {
                roe: placeholder_roe,
                chief_elements: placeholder_elements,
                deputy_elements: placeholder_elements,
                separation_km: 0.0,
                delta_r_over_r: 0.0,
            },
            transfer: None,
            transfer_eclipse: None,
            perch_roe: placeholder_roe,
            mission,
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            auto_drag_config: None,
            covariance: None,
            monte_carlo: None,
            safety: SafetyAnalysis {
                free_drift: None,
                poca: None,
                free_drift_poca: None,
                cola: None,
                secondary_conjunctions: None,
                cola_skipped: None,
            },
            formation_design: Some(formation_design),
        }
    }

    #[test]
    fn oxford_join_zero_one_two_three_items() {
        assert_eq!(oxford_join(&[]), "");
        assert_eq!(oxford_join(&["a".to_string()]), "a");
        assert_eq!(oxford_join(&["a".to_string(), "b".to_string()]), "a and b");
        assert_eq!(
            oxford_join(&["a".to_string(), "b".to_string(), "c".to_string()]),
            "a, b, and c",
        );
        assert_eq!(
            oxford_join(&[
                "a".to_string(),
                "b".to_string(),
                "c".to_string(),
                "d".to_string(),
            ]),
            "a, b, c, and d",
        );
    }

    #[test]
    fn render_leg_sep_list_uses_oxford_commas() {
        assert_eq!(render_leg_sep_list(&[]), "");
        assert_eq!(render_leg_sep_list(&[(1, 99.8)]), "leg 1 (99.8 m)");
        assert_eq!(
            render_leg_sep_list(&[(2, 0.1), (3, 0.2)]),
            "leg 2 (0.1 m) and leg 3 (0.2 m)",
        );
        assert_eq!(
            render_leg_sep_list(&[(1, 0.0), (2, 0.1), (3, 0.2)]),
            "leg 1 (0.0 m), leg 2 (0.1 m), and leg 3 (0.2 m)",
        );
    }

    #[test]
    fn determine_verdict_flags_abort_case_passive_safety_collapse() {
        // Build a PipelineOutput whose numerical safety passes, enrichment
        // active, but whose free_drift analysis shows leg 2 with
        // min_ei_separation_km essentially zero (0.0002 km = 0.2 m,
        // threshold = 0.100 km = 100 m, ratio = 0.002).
        let mut output = pipeline_output_passing_3d_with_enrichment();
        output.safety.free_drift = Some(vec![
            free_drift_analysis_with_ei(0.120),  // leg 1: healthy (120 m)
            free_drift_analysis_with_ei(0.0002), // leg 2: collapsed (0.2 m)
            free_drift_analysis_with_ei(0.0001), // leg 3: collapsed (0.1 m)
        ]);
        let config = SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        };

        let verdict = determine_verdict(&output, &config, None);

        assert!(
            verdict.reason.to_lowercase().contains("abort"),
            "verdict reason must surface abort-case collapse; got: {}",
            verdict.reason,
        );
        assert!(
            verdict.reason.contains("leg 2") || verdict.reason.contains("legs 2"),
            "verdict reason must identify the offending leg(s); got: {}",
            verdict.reason,
        );
    }
}

//! Shared output helpers.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use owo_colors::{OwoColorize, Stream};
use serde::Serialize;

use rpo_core::mission::{
    assess_safety, AvoidanceManeuver, ColaConfig, MonteCarloReport, PercentileStats, SafetyConfig,
    ValidationReport,
};
use rpo_core::pipeline::{
    resolve_propagator, to_propagation_model, PipelineInput, PipelineOutput, TransferResult,
};
use rpo_core::propagation::{extract_dmf_rates, DragConfig, PropagationModel};
use rpo_core::types::{QuasiNonsingularROE, SpacecraftConfig};

use crate::error::CliError;

use super::thresholds::mc as mc_thresh;

/// Whether analytical safety is the governing tier or a baseline for comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyTier {
    /// Analytical is the governing and only fidelity tier (mission command).
    Governing,
    /// Analytical is a baseline; a higher-fidelity validation tier governs.
    Baseline(ValidationTier),
}

/// The higher-fidelity validation source that governs safety margins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationTier {
    /// Nyx full-physics propagation.
    Nyx,
    /// Monte Carlo ensemble statistics.
    MonteCarlo,
}

/// Default delta-v budget for CLI COLA overlay (km/s).
/// 10 m/s — matches the pipeline auto-COLA default in `execute.rs`.
const DEFAULT_COLA_BUDGET_KM_S: f64 = 0.01;

/// Apply CLI `--cola-threshold` / `--cola-budget` flags onto a `PipelineInput`.
///
/// When `threshold` is `Some`, creates a `ColaConfig` with the given target
/// distance and budget (defaulting to 10 m/s if `budget` is `None`).
pub fn apply_cola_overlay(
    input: &mut PipelineInput,
    threshold: Option<f64>,
    budget: Option<f64>,
) {
    if let Some(threshold) = threshold {
        input.cola = Some(ColaConfig {
            target_distance_km: threshold,
            max_dv_km_s: budget.unwrap_or(DEFAULT_COLA_BUDGET_KM_S),
        });
    }
}

/// Mission feasibility verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// All safety margins satisfied.
    Feasible,
    /// Safety margins violated or insufficient data.
    Caution,
}

impl std::fmt::Display for Verdict {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Feasible => write!(f, "FEASIBLE"),
            Self::Caution => write!(f, "CAUTION"),
        }
    }
}

/// Verdict with contextual reason string.
pub struct VerdictResult {
    /// The verdict category.
    pub verdict: Verdict,
    /// Human-readable reason for the verdict.
    pub reason: &'static str,
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
/// Returns `None` if `suppress` is true (JSON and markdown modes suppress spinners).
#[must_use]
pub fn create_spinner(suppress: bool) -> Option<ProgressBar> {
    if suppress {
        return None;
    }
    let s = ProgressBar::new_spinner();
    s.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .expect("valid spinner template"),
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
        if let Some(s) = spinner {
            s.set_message("Extracting differential drag rates via Nyx...".to_string());
        } else {
            eprintln!("Extracting differential drag rates via Nyx...");
        }
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

// ── JSON output ─────────────────────────────────────────────────────────

/// Pretty-print a value as JSON to stdout.
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails.
pub fn print_json<T: Serialize>(value: &T) -> Result<(), CliError> {
    let json = serde_json::to_string_pretty(value).map_err(CliError::Serialize)?;
    println!("{json}");
    Ok(())
}

// ── Report file output ─────────────────────────────────────────────────

/// Write a file to `reports/{subdir}/{name}.{extension}`, creating directories if needed.
///
/// Prints the output path to stderr so the user knows where the file was written.
fn write_file_report(subdir: &str, name: &str, extension: &str, content: &str) -> Result<(), CliError> {
    let dir = Path::new("reports").join(subdir);
    std::fs::create_dir_all(&dir).map_err(|e| CliError::Io {
        path: dir.clone(),
        source: e,
    })?;
    let path = dir.join(format!("{name}.{extension}"));
    std::fs::write(&path, content).map_err(|e| CliError::Io {
        path: path.clone(),
        source: e,
    })?;
    eprintln!("Report written to {}", path.display());
    Ok(())
}

/// Write a markdown report to `reports/markdown/{name}.md`, creating directories if needed.
///
/// Prints the output path to stderr so the user knows where the file was written.
///
/// # Errors
///
/// Returns [`CliError::Io`] if directory creation or file write fails.
pub fn write_report(name: &str, content: &str) -> Result<(), CliError> {
    write_file_report("markdown", name, "md", content)
}

/// Write a JSON report to `reports/json/{name}.json`, creating directories if needed.
///
/// Serializes the value as pretty-printed JSON and writes to file.
/// Prints the output path to stderr.
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails, or [`CliError::Io`]
/// if directory creation or file write fails.
pub fn write_json_report<T: Serialize>(name: &str, value: &T) -> Result<(), CliError> {
    let json = serde_json::to_string_pretty(value).map_err(CliError::Serialize)?;
    write_file_report("json", name, "json", &json)
}

// ── Unit formatting ─────────────────────────────────────────────────────

/// ROE component below which the value is treated as effectively zero for display.
const ROE_DISPLAY_ZERO_THRESHOLD: f64 = 1e-15;

/// Format a value stored in km/s as m/s with given decimal places.
///
/// Example: `fmt_m_s(0.5432, 1)` returns `"543.2 m/s"`.
#[must_use]
pub fn fmt_m_s(value_km_s: f64, decimals: usize) -> String {
    let m_s = value_km_s * 1000.0;
    format!("{m_s:.decimals$} m/s")
}

/// Format a value stored in km as meters with given decimal places.
///
/// Example: `fmt_m(0.150, 1)` returns `"150.0 m"`.
#[must_use]
pub fn fmt_m(value_km: f64, decimals: usize) -> String {
    let m = value_km * 1000.0;
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
pub const BOUNDED_MOTION_DISPLAY_THRESHOLD: f64 = 1e-10;

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

/// Format a float for display, showing "N/A" for NaN (no-data sentinel).
pub fn fmt_or_na(v: f64, precision: usize) -> String {
    if v.is_nan() {
        "N/A".to_string()
    } else {
        format!("{v:.precision$}")
    }
}

// ── Section headers ─────────────────────────────────────────────────────

/// Print a top-level section header with bold cyan text and `===` underline.
pub fn print_header(title: &str) {
    println!(
        "\n{}",
        title.if_supports_color(Stream::Stdout, |v| v.cyan())
    );
    println!("{}", "=".repeat(title.len()));
}

/// Print a sub-section header with `---` underline.
pub fn print_subheader(title: &str) {
    println!("\n{title}");
    println!("{}", "-".repeat(title.len()));
}

// ── Color helpers ───────────────────────────────────────────────────────

/// Direction of threshold comparison for colored output.
#[derive(Clone, Copy)]
pub enum ThresholdDirection {
    /// Higher values are better (e.g., convergence rate).
    /// Green if `value >= warn`, yellow if `value >= alert`, red otherwise.
    HigherIsBetter,
    /// Lower values are better (e.g., shadow fraction).
    /// Red if `value > alert`, yellow if `value > warn`, no color otherwise.
    LowerIsBetter,
}

/// Print a line colored by threshold comparison.
pub fn print_colored(
    text: &str,
    value: f64,
    warn: f64,
    alert: f64,
    direction: ThresholdDirection,
) {
    match direction {
        ThresholdDirection::HigherIsBetter => {
            if value >= warn {
                println!(
                    "{}",
                    text.if_supports_color(Stream::Stdout, |v| v.green())
                );
            } else if value >= alert {
                println!(
                    "{}",
                    text.if_supports_color(Stream::Stdout, |v| v.yellow())
                );
            } else {
                println!(
                    "{}",
                    text.if_supports_color(Stream::Stdout, |v| v.red())
                );
            }
        }
        ThresholdDirection::LowerIsBetter => {
            if value > alert {
                println!(
                    "{}",
                    text.if_supports_color(Stream::Stdout, |v| v.red())
                );
            } else if value > warn {
                println!(
                    "{}",
                    text.if_supports_color(Stream::Stdout, |v| v.yellow())
                );
            } else {
                println!("{text}");
            }
        }
    }
}

// ── ROE display ─────────────────────────────────────────────────────────

/// Print all 6 ROE components with physical meaning.
pub fn print_roe(label: &str, roe: &QuasiNonsingularROE, chief_a: f64) {
    println!("  {label}:");
    let da_s = fmt_roe_component(roe.da);
    if roe.da.abs() < ROE_DISPLAY_ZERO_THRESHOLD {
        println!("    \u{03b4}a       = {da_s:>13}  (relative SMA)");
    } else {
        println!(
            "    \u{03b4}a       = {:>13}  (relative SMA, \u{2248} {:.1} km)",
            da_s,
            roe.da * chief_a
        );
    }
    println!(
        "    \u{03b4}\u{03bb}       = {:>13}  (relative mean longitude)",
        fmt_roe_component(roe.dlambda)
    );
    println!(
        "    \u{03b4}ex      = {:>13}  (relative e\u{00b7}cos \u{03c9})",
        fmt_roe_component(roe.dex)
    );
    println!(
        "    \u{03b4}ey      = {:>13}  (relative e\u{00b7}sin \u{03c9})",
        fmt_roe_component(roe.dey)
    );
    println!(
        "    \u{03b4}ix      = {:>13}  (relative inclination)",
        fmt_roe_component(roe.dix)
    );
    println!(
        "    \u{03b4}iy      = {:>13}  (relative RAAN\u{00b7}sin i)",
        fmt_roe_component(roe.diy)
    );
}

// ── Percentile stats ────────────────────────────────────────────────────

/// Print full percentile statistics (mean, std, p05/p95, min/max).
///
/// `scale` converts from stored unit to display unit (e.g., 1000.0 for km->m).
pub fn print_percentile_stats(
    indent: &str,
    stats: &PercentileStats,
    scale: f64,
    decimals: usize,
) {
    let fmt = |v: f64| fmt_or_na(v * scale, decimals);
    println!(
        "{indent}Mean: {}    Std: {}",
        fmt(stats.mean),
        fmt(stats.std_dev),
    );
    println!(
        "{indent}p05:  {}    p95: {}",
        fmt(stats.p05),
        fmt(stats.p95),
    );
    println!(
        "{indent}Min:  {}    Max: {}",
        fmt(stats.min),
        fmt(stats.max),
    );
}

/// Print compact percentile summary (p05/p50/p95 one-liner).
pub fn print_percentile_summary(
    indent: &str,
    stats: &PercentileStats,
    scale: f64,
    decimals: usize,
) {
    let fmt = |v: f64| fmt_or_na(v * scale, decimals);
    println!(
        "{indent}p05: {}    p50: {}    p95: {}",
        fmt(stats.p05),
        fmt(stats.p50),
        fmt(stats.p95),
    );
}

/// Print compact percentile summary, or "N/A" if no data.
pub fn print_optional_percentile_summary(
    indent: &str,
    stats: Option<&PercentileStats>,
    scale: f64,
    decimals: usize,
) {
    if let Some(s) = stats {
        print_percentile_summary(indent, s, scale, decimals);
    } else {
        println!("{indent}N/A (no data)");
    }
}

// ── Per-leg error table ─────────────────────────────────────────────────

/// Per-leg position error statistics (km).
// All fields carry the `_km` unit suffix per our naming rules.
// Clippy's `struct_field_names` lint fires on the shared suffix, but
// removing it would violate the project's unit-suffix convention.
#[allow(clippy::struct_field_names)]
pub struct LegErrorStats {
    /// Maximum position error (km).
    pub max_km: f64,
    /// Mean position error (km).
    pub mean_km: f64,
    /// RMS position error (km).
    pub rms_km: f64,
}

/// Compute max/mean/RMS position errors for a single leg in a single pass.
///
/// Returns `None` if `points` is empty.
#[must_use]
pub fn compute_leg_error_stats(
    points: &[rpo_core::mission::ValidationPoint],
) -> Option<LegErrorStats> {
    if points.is_empty() {
        return None;
    }
    let n = f64::from(u32::try_from(points.len()).unwrap_or(u32::MAX));
    let (max, sum, sum_sq) = points.iter().fold(
        (0.0_f64, 0.0_f64, 0.0_f64),
        |(mx, s, sq), p| {
            let e = p.position_error_km;
            (mx.max(e), s + e, sq + e * e)
        },
    );
    Some(LegErrorStats {
        max_km: max,
        mean_km: sum / n,
        rms_km: (sum_sq / n).sqrt(),
    })
}

/// Print per-leg position error breakdown in meters.
pub fn print_per_leg_errors(leg_points: &[Vec<rpo_core::mission::ValidationPoint>]) {
    if leg_points.is_empty() {
        return;
    }
    println!("\n  Per-leg position error (m):");
    println!(
        "    {:>4}  {:>8}  {:>8}  {:>8}",
        "Leg", "Max", "Mean", "RMS"
    );
    println!(
        "    {:->4}  {:->8}  {:->8}  {:->8}",
        "", "", "", ""
    );
    for (i, points) in leg_points.iter().enumerate() {
        if let Some(stats) = compute_leg_error_stats(points) {
            println!(
                "    {:>4}  {:>8.1}  {:>8.1}  {:>8.1}",
                i + 1,
                stats.max_km * 1000.0,
                stats.mean_km * 1000.0,
                stats.rms_km * 1000.0,
            );
        }
    }
}

// ── Auto-derived drag ───────────────────────────────────────────────────

/// Print auto-derived differential drag rates at the given indent level.
pub fn print_derived_drag(drag: &DragConfig, indent: &str) {
    println!("{indent}Auto-derived drag:");
    println!("{indent}  da_dot:  {:+.6e}", drag.da_dot);
    println!("{indent}  dex_dot: {:+.6e}", drag.dex_dot);
    println!("{indent}  dey_dot: {:+.6e}", drag.dey_dot);
}

// ── Rate metric ─────────────────────────────────────────────────────────

/// Whether to display a rate as a decimal probability or a percentage.
#[derive(Clone, Copy)]
pub enum RateFormat {
    /// Display as `0` for zero, `0.0000` for non-zero (e.g., collision probability).
    Probability,
    /// Display as `0.0%` (e.g., violation rate).
    Percentage,
}

/// Print a rate metric (probability or violation rate) with color.
///
/// Green if rate is 0.0, yellow if rate > 0 but below `alert_threshold`,
/// red if rate >= `alert_threshold`. If `alert_threshold` is 0.0, any non-zero
/// rate is red.
pub fn print_rate_metric(
    label: &str,
    rate: f64,
    n: f64,
    total: u32,
    format: RateFormat,
    alert_threshold: f64,
) {
    let line = match format {
        RateFormat::Probability => {
            if rate <= 0.0 {
                format!("{label}: 0 (0/{total})")
            } else {
                format!("{label}: {rate:.4} ({:.0}/{total})", (rate * n).round())
            }
        }
        RateFormat::Percentage => {
            format!(
                "{label}: {:.1}% ({:.0}/{total})",
                rate * 100.0,
                (rate * n).round()
            )
        }
    };

    if rate <= 0.0 {
        println!(
            "{}",
            line.if_supports_color(Stream::Stdout, |v| v.green())
        );
    } else if alert_threshold > 0.0 && rate < alert_threshold {
        println!(
            "{}",
            line.if_supports_color(Stream::Stdout, |v| v.yellow())
        );
    } else {
        println!(
            "{}",
            line.if_supports_color(Stream::Stdout, |v| v.red())
        );
    }
}

// ── Insight printing ────────────────────────────────────────────────────

use super::insights::{Insight, Severity};

/// Print cross-tier insights with severity-appropriate coloring.
///
/// - **Critical:** red `CRITICAL:` prefix
/// - **Warning:** yellow `WARNING:` prefix
/// - **Info:** cyan arrow prefix
pub fn print_insights(insights: &[Insight]) {
    for insight in insights {
        match insight.severity {
            Severity::Critical => {
                println!(
                    "{}",
                    format!("  Critical: {}", insight.message)
                        .if_supports_color(Stream::Stdout, |v| v.red())
                );
            }
            Severity::Warning => {
                println!(
                    "{}",
                    format!("  Warning: {}", insight.message)
                        .if_supports_color(Stream::Stdout, |v| v.yellow())
                );
            }
            Severity::Info => {
                println!(
                    "{}",
                    format!("  Insight: {}", insight.message)
                        .if_supports_color(Stream::Stdout, |v| v.cyan())
                );
            }
        }
    }
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

// ── Δv budget ───────────────────────────────────────────────────────────

/// Print the Δv budget block (Transfer / Targeting / Total) used by mission and MC summaries.
///
/// Flags when Lambert transfer dominates the budget (>90% of total) to highlight
/// that far-field transfer is the primary cost driver.
pub fn print_dv_budget(
    lambert_dv_km_s: f64,
    targeting_dv_km_s: f64,
    drag_aware: bool,
    cola_dv_km_s: Option<(f64, usize)>,
) {
    use super::thresholds::fidelity;

    let total = lambert_dv_km_s + targeting_dv_km_s;
    println!("\n  \u{0394}v Budget:");
    if total > 0.0 {
        let lambert_pct = lambert_dv_km_s / total * 100.0;
        let wp_pct = targeting_dv_km_s / total * 100.0;
        if lambert_pct > fidelity::FAR_FIELD_DV_DRIVER_PCT {
            println!(
                "    Transfer:        {}  ({lambert_pct:.1}%{})",
                fmt_m_s(lambert_dv_km_s, 1),
                " -- far-field driver".if_supports_color(Stream::Stdout, |v| v.red()),
            );
        } else {
            println!(
                "    Transfer:        {}  ({lambert_pct:.1}%)",
                fmt_m_s(lambert_dv_km_s, 1),
            );
        }
        println!(
            "    Targeting:       {}  ({wp_pct:.1}%)",
            fmt_m_s(targeting_dv_km_s, 1),
        );
    } else {
        println!("    Transfer:        {}", fmt_m_s(lambert_dv_km_s, 1));
        println!("    Targeting:       {}", fmt_m_s(targeting_dv_km_s, 1));
    }
    println!(
        "    Total:           {}",
        fmt_m_s(total, 1).if_supports_color(Stream::Stdout, |v| v.green()),
    );
    if let Some((cola_dv, num_burns)) = cola_dv_km_s {
        let burn_label = if num_burns == 1 { "burn" } else { "burns" };
        println!(
            "    COLA:            +{}  ({num_burns} {burn_label})",
            fmt_m_s(cola_dv, 2),
        );
        println!(
            "    Total (w/ COLA): {}",
            fmt_m_s(total + cola_dv, 1).if_supports_color(Stream::Stdout, |v| v.green()),
        );
    }
    if drag_aware {
        println!("    (drag-aware targeting; \u{0394}v differs slightly from analytical-only)");
    }
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
#[must_use]
pub fn determine_verdict(
    output: &PipelineOutput,
    config: &SafetyConfig,
    validation: Option<&ValidationReport>,
) -> VerdictResult {
    if let Some(report) = validation {
        let assessment = assess_safety(&report.numerical_safety, config);
        if assessment.overall_pass {
            VerdictResult { verdict: Verdict::Feasible, reason: "safety margins satisfied, Nyx full-physics" }
        } else {
            VerdictResult { verdict: Verdict::Caution, reason: "safety margins violated, Nyx full-physics" }
        }
    } else if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, config);
        if assessment.overall_pass {
            VerdictResult { verdict: Verdict::Feasible, reason: "analytical safety margins satisfied" }
        } else {
            VerdictResult { verdict: Verdict::Caution, reason: "analytical safety margins violated" }
        }
    } else {
        VerdictResult { verdict: Verdict::Feasible, reason: "no safety configured" }
    }
}

/// Determine the MC verdict from ensemble statistics.
///
/// Three-tier logic:
/// - **FEASIBLE (100%):** all converged, no collisions, no keep-out violations.
/// - **FEASIBLE (acceptable):** convergence >= 95%, no collisions.
/// - **CAUTION:** otherwise.
///
/// Returns a [`VerdictResult`] with verdict category and reason.
#[must_use]
pub fn determine_mc_verdict(report: &MonteCarloReport) -> VerdictResult {
    let stats = &report.statistics;
    let all_converged = (stats.convergence_rate - 1.0).abs() < mc_thresh::CONVERGENCE_EXACT_TOL;
    let no_collisions = stats.collision_probability <= 0.0;
    let no_keepout = stats.keepout_violation_rate <= 0.0;

    let ei_degraded = stats.ei_violation_rate > 0.0;

    if all_converged && no_collisions && no_keepout {
        VerdictResult {
            verdict: Verdict::Feasible,
            reason: if ei_degraded {
                "operationally feasible, passive abort safety degraded under dispersion"
            } else {
                "100% convergence, no collisions, operational safety holds"
            },
        }
    } else if stats.convergence_rate >= mc_thresh::CONVERGENCE_ALERT && no_collisions {
        VerdictResult {
            verdict: Verdict::Feasible,
            reason: if ei_degraded {
                "convergence acceptable, no collisions, passive abort safety degraded"
            } else {
                "convergence acceptable, no collisions"
            },
        }
    } else {
        VerdictResult {
            verdict: Verdict::Caution,
            reason: "review safety and convergence above",
        }
    }
}

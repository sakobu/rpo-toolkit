//! Shared output helpers.

use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use owo_colors::{OwoColorize, Stream};
use serde::Serialize;

use rpo_core::mission::PercentileStats;
use rpo_core::pipeline::{resolve_propagator, to_propagation_model, PipelineInput, TransferResult};
use rpo_core::propagation::{extract_dmf_rates, DragConfig, PropagationModel};
use rpo_core::types::{QuasiNonsingularROE, SpacecraftConfig};

use crate::error::CliError;

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
/// Returns `None` if `json` is true (machine-readable output suppresses spinners).
#[must_use]
pub fn create_spinner(json: bool) -> Option<ProgressBar> {
    if json {
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
        if points.is_empty() {
            continue;
        }
        let max = points
            .iter()
            .fold(0.0_f64, |acc, p| acc.max(p.position_error_km));
        let sum: f64 = points.iter().map(|p| p.position_error_km).sum();
        let sum_sq: f64 = points.iter().map(|p| p.position_error_km.powi(2)).sum();
        let n = f64::from(u32::try_from(points.len()).unwrap_or(u32::MAX));
        let mean = sum / n;
        let rms = (sum_sq / n).sqrt();
        let max_m = max * 1000.0; // km → m
        let mean_m = mean * 1000.0; // km → m
        let rms_m = rms * 1000.0; // km → m
        println!(
            "    {:>4}  {:>8.1}  {:>8.1}  {:>8.1}",
            i + 1, max_m, mean_m, rms_m,
        );
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

//! Shared output helpers.

use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use serde::Serialize;

use rpo_core::mission::PercentileStats;
use rpo_core::pipeline::{resolve_propagator, to_propagation_model, PipelineInput, TransferResult};
use rpo_core::propagation::{extract_dmf_rates, DragConfig, PropagationModel};
use rpo_core::types::{QuasiNonsingularROE, SpacecraftConfig};

use crate::error::CliError;

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

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via nyx DMF
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
            s.set_message("Extracting differential drag rates via nyx...".to_string());
        } else {
            eprintln!("Extracting differential drag rates via nyx...");
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

/// Print all 6 ROE components with physical meaning.
pub fn print_roe(label: &str, roe: &QuasiNonsingularROE, chief_a: f64) {
    println!("  {label}:");
    println!(
        "    δa       = {:+.6e}  (relative SMA, ≈ {:.4} km)",
        roe.da,
        roe.da * chief_a
    );
    println!("    δλ       = {:+.6e}  (relative mean longitude)", roe.dlambda);
    println!("    δex      = {:+.6e}  (relative e·cos ω)", roe.dex);
    println!("    δey      = {:+.6e}  (relative e·sin ω)", roe.dey);
    println!("    δix      = {:+.6e}  (relative inclination)", roe.dix);
    println!("    δiy      = {:+.6e}  (relative RAAN·sin i)", roe.diy);
}

/// Format a float for display, showing "N/A" for NaN (no-data sentinel).
pub fn fmt_or_na(v: f64, precision: usize) -> String {
    if v.is_nan() {
        "N/A".to_string()
    } else {
        format!("{v:.precision$}")
    }
}

/// Print full percentile statistics (mean, std, p05/p95, min/max).
pub fn print_percentile_stats(indent: &str, stats: &PercentileStats) {
    println!(
        "{indent}Mean:   {}    Std: {}",
        fmt_or_na(stats.mean, 6),
        fmt_or_na(stats.std_dev, 6),
    );
    println!(
        "{indent}p05:    {}    p95: {}",
        fmt_or_na(stats.p05, 6),
        fmt_or_na(stats.p95, 6),
    );
    println!(
        "{indent}Min:    {}    Max: {}",
        fmt_or_na(stats.min, 6),
        fmt_or_na(stats.max, 6),
    );
}

/// Print compact percentile summary (p05/p50/p95 one-liner).
pub fn print_percentile_summary(indent: &str, stats: &PercentileStats) {
    println!(
        "{indent}p05: {}    p50: {}    p95: {}",
        fmt_or_na(stats.p05, 4),
        fmt_or_na(stats.p50, 4),
        fmt_or_na(stats.p95, 4),
    );
}

/// Print compact percentile summary, or "N/A" if no data.
pub fn print_optional_percentile_summary(indent: &str, stats: Option<&PercentileStats>) {
    if let Some(s) = stats {
        print_percentile_summary(indent, s);
    } else {
        println!("{indent}N/A (no data)");
    }
}

/// Print per-leg position error breakdown.
pub fn print_per_leg_errors(leg_points: &[Vec<rpo_core::mission::ValidationPoint>]) {
    if leg_points.is_empty() {
        return;
    }
    println!("\n  Per-leg position error (km):");
    println!(
        "    {:>4}  {:>10}  {:>10}  {:>10}",
        "Leg", "Max", "Mean", "RMS"
    );
    println!(
        "    {:->4}  {:->10}  {:->10}  {:->10}",
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
        println!(
            "    {:>4}  {:>10.4}  {:>10.4}  {:>10.4}",
            i + 1,
            max,
            mean,
            rms
        );
    }
}

/// Format a duration in seconds as a human-readable string.
///
/// Returns `"Xh Ym"` for durations >= 1 hour, `"Xm Ys"` otherwise.
#[must_use]
pub fn fmt_duration(s: f64) -> String {
    let s = s.max(0.0);
    if s >= 3600.0 {
        let h = (s / 3600.0).floor();
        let m = ((s - h * 3600.0) / 60.0).round();
        format!("{h:.0}h {m:.0}m")
    } else {
        let m = (s / 60.0).floor();
        let sec = (s - m * 60.0).round();
        format!("{m:.0}m {sec:.0}s")
    }
}

/// Format a duration in seconds as mm:ss.
#[must_use]
pub fn fmt_mmss(s: f64) -> String {
    let total_secs = s.round().max(0.0);
    let m = (total_secs / 60.0).floor();
    let sec = total_secs - m * 60.0;
    format!("{m:.0}:{sec:02.0}")
}

/// Print a top-level section header with bold cyan text and `===` underline.
pub fn print_header(title: &str) {
    use owo_colors::{OwoColorize, Stream};
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

/// Print a line colored by threshold comparison.
///
/// When `higher_is_better` is `true` (e.g., convergence rate):
///   green if `value >= warn`, yellow if `value >= alert`, red otherwise.
///
/// When `higher_is_better` is `false` (e.g., shadow fraction):
///   red if `value > alert`, yellow if `value > warn`, no color otherwise.
pub fn print_colored(text: &str, value: f64, warn: f64, alert: f64, higher_is_better: bool) {
    use owo_colors::{OwoColorize, Stream};
    if higher_is_better && value >= warn {
        println!(
            "{}",
            text.if_supports_color(Stream::Stdout, |v| v.green())
        );
    } else if higher_is_better && value >= alert {
        println!(
            "{}",
            text.if_supports_color(Stream::Stdout, |v| v.yellow())
        );
    } else if higher_is_better {
        println!(
            "{}",
            text.if_supports_color(Stream::Stdout, |v| v.red())
        );
    } else if value > alert {
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

/// Print auto-derived differential drag rates.
pub fn print_derived_drag(drag: &rpo_core::propagation::DragConfig) {
    println!("\nAuto-derived differential drag (nyx DMF):");
    println!("  da_dot:  {:+.6e}", drag.da_dot);
    println!("  dex_dot: {:+.6e}", drag.dex_dot);
    println!("  dey_dot: {:+.6e}", drag.dey_dot);
}

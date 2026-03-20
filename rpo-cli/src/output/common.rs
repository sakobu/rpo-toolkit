//! Shared output helpers.

use serde::Serialize;

use rpo_core::mission::PercentileStats;
use rpo_core::types::QuasiNonsingularROE;

use crate::error::CliError;

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

/// Print auto-derived differential drag rates.
pub fn print_derived_drag(drag: &rpo_core::propagation::DragConfig) {
    println!("\nAuto-derived differential drag (nyx DMF):");
    println!("  da_dot:  {:+.6e}", drag.da_dot);
    println!("  dex_dot: {:+.6e}", drag.dex_dot);
    println!("  dey_dot: {:+.6e}", drag.dey_dot);
}

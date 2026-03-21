//! Monte Carlo report formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{MonteCarloMode, MonteCarloReport};

use super::common::{
    fmt_duration, print_colored, print_header, print_optional_percentile_summary,
    print_percentile_stats,
};

/// Collision probability threshold above which we color red.
const COLLISION_PROB_ALERT: f64 = 0.05;

/// Convergence rate threshold below which we color red.
const CONVERGENCE_RATE_ALERT: f64 = 0.95;

/// Convergence rate threshold below which we color yellow.
const CONVERGENCE_RATE_WARN: f64 = 0.99;

/// Violation rate threshold above which we color red for e/i separation.
const EI_VIOLATION_RATE_ALERT: f64 = 0.10;

/// Mahalanobis distance threshold below which we color red.
const MAHALANOBIS_ALERT: f64 = 0.5;

/// Mahalanobis distance threshold below which we color yellow.
const MAHALANOBIS_WARN: f64 = 1.0;

/// Print human-readable Monte Carlo ensemble report.
#[allow(clippy::too_many_lines)]
pub fn print_mc_report(report: &MonteCarloReport, lambert_dv_km_s: f64) {
    let title = format!(
        "Full-Physics Monte Carlo Analysis ({})",
        report.config.mode
    );
    print_header(&title);

    println!(
        "Config: {} samples, seed={}",
        report.config.num_samples,
        report.config.seed.unwrap_or(42),
    );
    if lambert_dv_km_s > 0.0 {
        let total = lambert_dv_km_s + report.nominal_dv_km_s;
        println!(
            "Nominal Δv (waypoint):  {:.6} km/s",
            report.nominal_dv_km_s
        );
        println!("Lambert transfer Δv:    {lambert_dv_km_s:.4} km/s");
        println!(
            "Total mission Δv:       {}",
            format!("{total:.4} km/s").if_supports_color(Stream::Stdout, |v| v.green()),
        );
    } else {
        println!("Nominal Δv: {:.6} km/s", report.nominal_dv_km_s);
    }
    println!("Wall time:  {}\n", fmt_duration(report.elapsed_wall_s));

    let stats = &report.statistics;

    println!("Δv Distribution (km/s):");
    print_percentile_stats("  ", &stats.total_dv_km_s);

    let n = f64::from(report.config.num_samples);

    println!("\nOperational Safety:");
    print_rate_metric(
        "  Collision Probability",
        stats.collision_probability,
        n,
        report.config.num_samples,
        RateFormat::Probability,
        COLLISION_PROB_ALERT,
    );
    print_rate_metric(
        "  Keep-out violations",
        stats.keepout_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        0.0,
    );
    println!("  Min R/C distance (km):");
    print_optional_percentile_summary("    ", stats.min_rc_distance_km.as_ref());
    println!("  Min 3D distance (km):");
    print_optional_percentile_summary("    ", stats.min_3d_distance_km.as_ref());

    println!("\nPassive / Abort Safety:");
    print_rate_metric(
        "  e/i separation violations",
        stats.ei_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        EI_VIOLATION_RATE_ALERT,
    );
    println!("  Min e/i separation (km):");
    print_optional_percentile_summary("    ", stats.min_ei_separation_km.as_ref());
    if stats.ei_violation_rate > 0.0 {
        println!("  Note: e/i separation is an orbit-averaged passive safety metric (D'Amico Eq. 2.22)");
        println!("        derived for free-drift relative motion. In actively guided waypoint missions,");
        println!("        high violation rates can occur even when operational safety constraints remain");
        println!("        satisfied. Treat e/i here as an abort/contingency indicator unless explicitly");
        println!("        enforced during targeting.");
    }

    // Convergence rate
    let conv_str = format!(
        "\nConvergence Rate: {:.1}% ({:.0} / {})",
        stats.convergence_rate * 100.0,
        (stats.convergence_rate * n).round(),
        report.config.num_samples,
    );
    print_colored(
        &conv_str,
        stats.convergence_rate,
        CONVERGENCE_RATE_WARN,
        CONVERGENCE_RATE_ALERT,
        true,
    );
    println!("Failures: {}", report.num_failures);

    if !stats.waypoint_miss_km.is_empty() {
        println!("\nPer-Waypoint Miss (km):");
        for (i, wp_stats) in stats.waypoint_miss_km.iter().enumerate() {
            if let Some(s) = wp_stats {
                println!("    WP{}:  p50: {:.4}    p95: {:.4}", i + 1, s.p50, s.p95,);
            } else {
                println!("    WP{}:  N/A", i + 1);
            }
        }
    }

    if let Some(ref cv) = report.covariance_cross_check {
        println!("\nCovariance Cross-Check:");
        if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
            println!(
                "  Terminal 3σ box containment: {:.1}% of closed-loop MC samples",
                cv.terminal_3sigma_containment * 100.0,
            );
            println!("    (compared against open-loop covariance prediction)");
        } else {
            println!(
                "  Terminal 3σ box containment: {:.1}% of samples",
                cv.terminal_3sigma_containment * 100.0,
            );
        }
        let sigma_str = format!(
            "  Sigma ratio (R/I/C): {:.2} / {:.2} / {:.2}",
            cv.sigma_ratio_ric.x, cv.sigma_ratio_ric.y, cv.sigma_ratio_ric.z,
        );
        let all_below_one = cv.sigma_ratio_ric.x < 1.0
            && cv.sigma_ratio_ric.y < 1.0
            && cv.sigma_ratio_ric.z < 1.0;
        if all_below_one {
            println!(
                "{}",
                sigma_str.if_supports_color(Stream::Stdout, |v| v.green())
            );
        } else {
            println!(
                "{}",
                sigma_str.if_supports_color(Stream::Stdout, |v| v.yellow())
            );
        }
        if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
            println!(
                "    (Closed Loop: retargeting suppresses dispersion relative to open-loop covariance; values << 1 expected)"
            );
        }
        let maha_str = format!(
            "  Closest Mahalanobis approach over mission: {:.2}",
            cv.min_mahalanobis_distance,
        );
        print_colored(
            &maha_str,
            cv.min_mahalanobis_distance,
            MAHALANOBIS_WARN,
            MAHALANOBIS_ALERT,
            true,
        );
        println!(
            "    (Open-loop covariance diagnostic only; <1 means the nominal trajectory enters the 1σ covariance envelope)"
        );
    }
}

/// Whether to display a rate as a decimal probability or a percentage.
#[derive(Clone, Copy)]
enum RateFormat {
    /// Display as `0.0000` (e.g., collision probability).
    Probability,
    /// Display as `0.0%` (e.g., violation rate).
    Percentage,
}

/// Print a rate metric (probability or violation rate) with color.
///
/// Green if rate is 0.0, yellow if rate > 0 but below `alert_threshold`,
/// red if rate >= `alert_threshold`. If `alert_threshold` is 0.0, any non-zero
/// rate is red.
fn print_rate_metric(
    label: &str,
    rate: f64,
    n: f64,
    total: u32,
    format: RateFormat,
    alert_threshold: f64,
) {
    let line = match format {
        RateFormat::Probability => {
            format!("{label}: {rate:.4} ({:.0} / {total})", (rate * n).round())
        }
        RateFormat::Percentage => {
            format!(
                "{label}: {:.1}% ({:.0} / {total})",
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

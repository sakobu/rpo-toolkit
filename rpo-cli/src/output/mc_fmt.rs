//! Monte Carlo report formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{MonteCarloMode, MonteCarloReport, SafetyConfig};
use rpo_core::propagation::DragConfig;

use super::common::{
    fmt_duration, fmt_m_s, print_colored, print_derived_drag, print_header,
    print_optional_percentile_summary, print_percentile_stats, print_rate_metric,
    print_subheader, RateFormat, ThresholdDirection,
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

/// Tolerance for treating convergence rate as exactly 1.0 (all samples converged).
const CONVERGENCE_RATE_EXACT_TOL: f64 = f64::EPSILON;

/// Print the compact baseline mission context for MC output.
pub fn print_mc_baseline(
    lambert_dv_km_s: f64,
    lambert_tof_s: f64,
    waypoint_dv_km_s: f64,
    waypoint_duration_s: f64,
    num_legs: usize,
    is_far_field: bool,
) {
    print_subheader("Baseline");
    let class = if is_far_field {
        "FAR-FIELD"
    } else {
        "PROXIMITY"
    };
    println!("  Classification:  {class}");
    if lambert_dv_km_s > 0.0 {
        println!(
            "  Transfer \u{0394}v:     {}  ({})",
            fmt_m_s(lambert_dv_km_s, 1),
            fmt_duration(lambert_tof_s),
        );
    }
    println!("  Waypoint legs:   {num_legs}");
    println!(
        "  Targeting \u{0394}v:    {}  ({})",
        fmt_m_s(waypoint_dv_km_s, 1),
        fmt_duration(waypoint_duration_s),
    );
    let total = lambert_dv_km_s + waypoint_dv_km_s;
    println!(
        "  Total \u{0394}v:        {}",
        fmt_m_s(total, 1).if_supports_color(Stream::Stdout, |v| v.green()),
    );
}

/// Print human-readable Monte Carlo ensemble report.
#[allow(clippy::too_many_lines)]
pub fn print_mc_report(
    report: &MonteCarloReport,
    lambert_dv_km_s: f64,
    derived_drag: Option<&DragConfig>,
) {
    let mode_str = match report.config.mode {
        MonteCarloMode::ClosedLoop => "closed-loop",
        MonteCarloMode::OpenLoop => "open-loop",
    };
    let section_title = format!(
        "Monte Carlo ({} samples, {mode_str}, seed={})",
        report.config.num_samples,
        report.config.seed.unwrap_or(42),
    );
    print_subheader(&section_title);

    // Header stats
    if lambert_dv_km_s > 0.0 {
        let total = lambert_dv_km_s + report.nominal_dv_km_s;
        println!(
            "  Nominal \u{0394}v:     {} (waypoint)",
            fmt_m_s(report.nominal_dv_km_s, 1),
        );
        println!("  Transfer \u{0394}v:    {}", fmt_m_s(lambert_dv_km_s, 1));
        println!(
            "  Total \u{0394}v:       {}",
            fmt_m_s(total, 1).if_supports_color(Stream::Stdout, |v| v.green()),
        );
    } else {
        println!(
            "  Nominal \u{0394}v:     {}",
            fmt_m_s(report.nominal_dv_km_s, 1),
        );
    }
    println!("  Wall time:      {}\n", fmt_duration(report.elapsed_wall_s));

    let stats = &report.statistics;
    let n = f64::from(report.config.num_samples);

    // Δv distribution
    println!("  \u{0394}v Distribution (m/s):");
    print_percentile_stats("    ", &stats.total_dv_km_s, 1000.0, 2);

    // Operational Safety
    println!("\n  Operational Safety:");
    print_rate_metric(
        "    Collision probability",
        stats.collision_probability,
        n,
        report.config.num_samples,
        RateFormat::Probability,
        COLLISION_PROB_ALERT,
    );
    print_rate_metric(
        "    Keep-out violations",
        stats.keepout_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        0.0,
    );
    println!("    R/C-plane distance (m):");
    print_optional_percentile_summary("      ", stats.min_rc_distance_km.as_ref(), 1000.0, 1);
    println!("    Min 3D distance (m):");
    print_optional_percentile_summary("      ", stats.min_3d_distance_km.as_ref(), 1000.0, 1);

    // Passive Safety
    println!("\n  Passive Safety:");
    print_rate_metric(
        "    e/i violations",
        stats.ei_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        EI_VIOLATION_RATE_ALERT,
    );
    println!("    Min e/i separation (m):");
    print_optional_percentile_summary("      ", stats.min_ei_separation_km.as_ref(), 1000.0, 1);
    if stats.ei_violation_rate > 0.0 {
        println!("    Note: e/i is a free-drift passive safety metric. High violation rates are");
        println!("          common in guided missions while operational safety holds.");
    }

    // Convergence
    let conv_str = format!(
        "\n  Convergence: {:.1}% ({:.0}/{}), {} failures",
        stats.convergence_rate * 100.0,
        (stats.convergence_rate * n).round(),
        report.config.num_samples,
        report.num_failures,
    );
    print_colored(
        &conv_str,
        stats.convergence_rate,
        CONVERGENCE_RATE_WARN,
        CONVERGENCE_RATE_ALERT,
        ThresholdDirection::HigherIsBetter,
    );

    // Per-Waypoint Miss
    if !stats.waypoint_miss_km.is_empty() {
        println!("\n  Per-Waypoint Miss (m):");
        for (i, wp_stats) in stats.waypoint_miss_km.iter().enumerate() {
            if let Some(s) = wp_stats {
                let p50_m = s.p50 * 1000.0; // km → m
                let p95_m = s.p95 * 1000.0; // km → m
                println!(
                    "    WP {}:  p50: {p50_m:.1}    p95: {p95_m:.1}",
                    i + 1,
                );
            } else {
                println!("    WP {}:  N/A", i + 1);
            }
        }
    }

    // Diagnostics
    let has_cov = report.covariance_cross_check.is_some();
    let has_drag = derived_drag.is_some();
    if has_cov || has_drag {
        println!("\n  Diagnostics:");

        if let Some(ref cv) = report.covariance_cross_check {
            println!("    Covariance cross-check:");
            if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
                println!(
                    "      3\u{03c3} containment: {:.1}%",
                    cv.terminal_3sigma_containment * 100.0,
                );
            } else {
                println!(
                    "      3\u{03c3} containment: {:.1}% of samples",
                    cv.terminal_3sigma_containment * 100.0,
                );
            }
            let sigma_str = format!(
                "      Sigma ratio (R / I / C): {:.2} / {:.2} / {:.2}",
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
                    "        (closed-loop: dispersion suppressed by retargeting)"
                );
            }
            let maha_str = format!(
                "      Mahalanobis distance: {:.2}",
                cv.min_mahalanobis_distance,
            );
            print_colored(
                &maha_str,
                cv.min_mahalanobis_distance,
                MAHALANOBIS_WARN,
                MAHALANOBIS_ALERT,
                ThresholdDirection::HigherIsBetter,
            );
            println!(
                "        (< 1 = nominal within 1\u{03c3})"
            );
        }

        if let Some(drag) = derived_drag {
            print_derived_drag(drag, "    ");
        }
    }
}

/// Print Monte Carlo summary block.
pub fn print_mc_summary(
    report: &MonteCarloReport,
    lambert_dv_km_s: f64,
    safety_config: &SafetyConfig,
) {
    let stats = &report.statistics;

    print_header("Summary");

    // Verdict
    let all_converged = (stats.convergence_rate - 1.0).abs() < CONVERGENCE_RATE_EXACT_TOL;
    let no_collisions = stats.collision_probability <= 0.0;
    let no_keepout = stats.keepout_violation_rate <= 0.0;

    let verdict = if all_converged && no_collisions && no_keepout {
        "FEASIBLE (100% convergence, no collisions, operational safety holds)"
    } else if stats.convergence_rate >= CONVERGENCE_RATE_ALERT && no_collisions {
        "FEASIBLE (convergence acceptable, no collisions)"
    } else {
        "CAUTION (review safety and convergence above)"
    };

    let is_feasible = (all_converged && no_collisions && no_keepout)
        || (stats.convergence_rate >= CONVERGENCE_RATE_ALERT && no_collisions);

    if is_feasible {
        println!(
            "  Verdict:         {}",
            verdict.if_supports_color(Stream::Stdout, |v| v.green())
        );
    } else {
        println!(
            "  Verdict:         {}",
            verdict.if_supports_color(Stream::Stdout, |v| v.yellow())
        );
    }

    // Δv Budget
    let total = lambert_dv_km_s + report.nominal_dv_km_s;
    println!("\n  \u{0394}v Budget:");
    if total > 0.0 {
        let transfer_pct = lambert_dv_km_s / total * 100.0;
        let targeting_pct = report.nominal_dv_km_s / total * 100.0;
        println!(
            "    Transfer:        {}  ({transfer_pct:.1}%)",
            fmt_m_s(lambert_dv_km_s, 1),
        );
        println!(
            "    Targeting:       {}  ({targeting_pct:.1}%)",
            fmt_m_s(report.nominal_dv_km_s, 1),
        );
    } else {
        println!("    Transfer:        {}", fmt_m_s(lambert_dv_km_s, 1));
        println!(
            "    Targeting:       {}",
            fmt_m_s(report.nominal_dv_km_s, 1),
        );
    }
    println!(
        "    Total:           {}",
        fmt_m_s(total, 1).if_supports_color(Stream::Stdout, |v| v.green()),
    );

    // Safety
    println!("\n  Safety:");
    let n = f64::from(report.config.num_samples);
    print_rate_metric(
        "    Collision probability",
        stats.collision_probability,
        n,
        report.config.num_samples,
        RateFormat::Probability,
        COLLISION_PROB_ALERT,
    );
    if let Some(ref d3) = stats.min_3d_distance_km {
        let p05_m = d3.p05 * 1000.0; // km → m
        let thresh_m = safety_config.min_distance_3d_km * 1000.0; // km → m
        println!(
            "    Min 3D distance (p05): {p05_m:.1} m  (threshold: {thresh_m:.0} m)",
        );
    }
    print_rate_metric(
        "    e/i violations",
        stats.ei_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        EI_VIOLATION_RATE_ALERT,
    );

    // Convergence
    println!("\n  Convergence:");
    println!(
        "    Rate:                {:.1}% ({:.0}/{})",
        stats.convergence_rate * 100.0,
        (stats.convergence_rate * n).round(),
        report.config.num_samples,
    );
    if let Some(ref cv) = report.covariance_cross_check {
        println!(
            "    3\u{03c3} containment:      {:.1}%",
            cv.terminal_3sigma_containment * 100.0,
        );
    }
}

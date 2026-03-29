//! Monte Carlo report formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{MonteCarloMode, MonteCarloReport, SafetyConfig};
use rpo_core::propagation::DragConfig;

use super::common::{
    determine_mc_verdict, fmt_duration, fmt_m_s, print_colored, print_derived_drag,
    print_dv_budget, print_header, print_optional_percentile_summary, print_percentile_stats,
    print_rate_metric, print_subheader, RateFormat, ThresholdDirection, Verdict,
};
use super::thresholds::mc as mc_thresh;

/// Print human-readable Monte Carlo ensemble report.
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

    print_mc_header_stats(report, lambert_dv_km_s);

    let stats = &report.statistics;
    let n = f64::from(report.config.num_samples);

    // Δv distribution
    println!("  \u{0394}v Distribution (m/s):");
    print_percentile_stats("    ", &stats.total_dv_km_s, 1000.0, 2);

    // Convergence (before safety for operator priority)
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
        mc_thresh::CONVERGENCE_WARN,
        mc_thresh::CONVERGENCE_ALERT,
        ThresholdDirection::HigherIsBetter,
    );

    print_mc_safety_stats(stats, report);

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

    print_mc_diagnostics_human(report, derived_drag);
}

/// Print MC header: nominal/transfer/total Dv and wall time.
fn print_mc_header_stats(report: &MonteCarloReport, lambert_dv_km_s: f64) {
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
}

/// Print operational and passive safety stats.
fn print_mc_safety_stats(
    stats: &rpo_core::mission::EnsembleStatistics,
    report: &MonteCarloReport,
) {
    let n = f64::from(report.config.num_samples);

    println!("\n  Operational Safety:");
    print_rate_metric(
        "    Collision probability",
        stats.collision_probability,
        n,
        report.config.num_samples,
        RateFormat::Probability,
        mc_thresh::COLLISION_PROB_ALERT,
    );
    print_rate_metric(
        "    Keep-out violations",
        stats.keepout_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        0.0,
    );
    println!("    Min R/C-plane distance (m):");
    print_optional_percentile_summary("      ", stats.min_rc_distance_km.as_ref(), 1000.0, 1);
    println!("    Min 3D distance (m):");
    print_optional_percentile_summary("      ", stats.min_3d_distance_km.as_ref(), 1000.0, 1);

    println!("\n  Passive Safety:");
    print_rate_metric(
        "    e/i violations",
        stats.ei_violation_rate,
        n,
        report.config.num_samples,
        RateFormat::Percentage,
        mc_thresh::EI_VIOLATION_RATE_ALERT,
    );
    println!("    Min e/i separation (m):");
    print_optional_percentile_summary("      ", stats.min_ei_separation_km.as_ref(), 1000.0, 1);
    if stats.ei_violation_rate > 0.0 {
        println!(
            "    Note: e/i is a free-drift passive safety metric; high violation rates are"
        );
        println!("          common in guided missions while operational safety holds.");
    }
}

/// Print diagnostics: covariance cross-check and auto-derived drag.
fn print_mc_diagnostics_human(report: &MonteCarloReport, derived_drag: Option<&DragConfig>) {
    let has_cov = report.covariance_cross_check.is_some();
    let has_drag = derived_drag.is_some();
    if !has_cov && !has_drag {
        return;
    }

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
            mc_thresh::MAHALANOBIS_WARN,
            mc_thresh::MAHALANOBIS_ALERT,
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

/// Print Monte Carlo summary block.
pub fn print_mc_summary(
    report: &MonteCarloReport,
    lambert_dv_km_s: f64,
    safety_config: &SafetyConfig,
    drag_aware: bool,
) {
    let stats = &report.statistics;

    print_header("Summary");

    let vr = determine_mc_verdict(report);
    let verdict_line = format!("{} ({})", vr.verdict, vr.reason);
    if vr.verdict == Verdict::Feasible {
        println!(
            "  Verdict:         {}",
            verdict_line.if_supports_color(Stream::Stdout, |v| v.green())
        );
    } else {
        println!(
            "  Verdict:         {}",
            verdict_line.if_supports_color(Stream::Stdout, |v| v.yellow())
        );
    }

    print_dv_budget(lambert_dv_km_s, report.nominal_dv_km_s, drag_aware);

    // Safety
    println!("\n  Safety (Monte Carlo):");
    let n = f64::from(report.config.num_samples);
    print_rate_metric(
        "    Collision probability",
        stats.collision_probability,
        n,
        report.config.num_samples,
        RateFormat::Probability,
        mc_thresh::COLLISION_PROB_ALERT,
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
        mc_thresh::EI_VIOLATION_RATE_ALERT,
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

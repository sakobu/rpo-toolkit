//! Monte Carlo report formatting.

use rpo_core::mission::{MonteCarloMode, MonteCarloReport};

use super::common::{
    print_optional_percentile_summary, print_percentile_stats,
};

/// Print human-readable Monte Carlo ensemble report.
#[allow(clippy::too_many_lines)]
pub fn print_mc_report(report: &MonteCarloReport, lambert_dv_km_s: f64) {
    println!(
        "Full-Physics Monte Carlo Analysis ({:?})",
        report.config.mode
    );
    println!("===============================================\n");

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
        println!("Total mission Δv:       {total:.4} km/s");
    } else {
        println!("Nominal Δv: {:.6} km/s", report.nominal_dv_km_s);
    }
    println!("Wall time:  {:.1}s\n", report.elapsed_wall_s);

    let stats = &report.statistics;

    println!("Δv Distribution (km/s):");
    print_percentile_stats("  ", &stats.total_dv_km_s);

    let n = f64::from(report.config.num_samples);

    println!("\nOperational Safety:");
    println!(
        "  Collision Probability: {:.4} ({:.0} / {})",
        stats.collision_probability,
        (stats.collision_probability * n).round(),
        report.config.num_samples,
    );
    if stats.keepout_violation_rate > 0.0 {
        println!(
            "  Keep-out sphere violations: {:.1}% ({:.0} / {})",
            stats.keepout_violation_rate * 100.0,
            (stats.keepout_violation_rate * n).round(),
            report.config.num_samples,
        );
    } else {
        println!(
            "  Keep-out sphere violations: 0.0% (0 / {})",
            report.config.num_samples
        );
    }
    println!("  Min R/C distance (km):");
    print_optional_percentile_summary("    ", stats.min_rc_distance_km.as_ref());
    println!("  Min 3D distance (km):");
    print_optional_percentile_summary("    ", stats.min_3d_distance_km.as_ref());

    println!("\nPassive / Abort Safety:");
    if stats.ei_violation_rate > 0.0 {
        println!(
            "  e/i separation violations: {:.1}% ({:.0} / {})",
            stats.ei_violation_rate * 100.0,
            (stats.ei_violation_rate * n).round(),
            report.config.num_samples,
        );
    } else {
        println!(
            "  e/i separation violations: 0.0% (0 / {})",
            report.config.num_samples
        );
    }
    println!("  Min e/i separation (km):");
    print_optional_percentile_summary("    ", stats.min_ei_separation_km.as_ref());
    if stats.ei_violation_rate > 0.0 {
        println!("  Note: e/i separation is an orbit-averaged passive safety metric (D'Amico Eq. 2.22)");
        println!("        derived for free-drift relative motion. In actively guided waypoint missions,");
        println!("        high violation rates can occur even when operational safety constraints remain");
        println!("        satisfied. Treat e/i here as an abort/contingency indicator unless explicitly");
        println!("        enforced during targeting.");
    }

    println!(
        "\nConvergence Rate: {:.1}% ({:.0} / {})",
        stats.convergence_rate * 100.0,
        (stats.convergence_rate * n).round(),
        report.config.num_samples,
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
        println!(
            "  Sigma ratio (R/I/C): {:.2} / {:.2} / {:.2}",
            cv.sigma_ratio_ric.x, cv.sigma_ratio_ric.y, cv.sigma_ratio_ric.z,
        );
        if matches!(report.config.mode, MonteCarloMode::ClosedLoop) {
            println!(
                "    (ClosedLoop: retargeting suppresses dispersion relative to open-loop covariance; values << 1 expected)"
            );
        }
        println!(
            "  Closest Mahalanobis approach over mission: {:.2}",
            cv.min_mahalanobis_distance,
        );
        println!(
            "    (Open-loop covariance diagnostic only; <1 means the nominal trajectory enters the 1σ covariance envelope)"
        );
    }
}

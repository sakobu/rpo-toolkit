//! Safety analysis formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{
    assess_safety, ClosestApproach, FreeDriftAnalysis, MissionConfig, RcContext, SafetyConfig,
    SafetyMetrics, ValidationReport,
};

use super::common::{fmt_bounded_motion_residual, fmt_duration, fmt_m, print_subheader};
use super::thresholds::fidelity;

/// Print safety analysis results with pass/fail assessment.
pub fn print_safety_analysis(safety: &SafetyMetrics, config: &SafetyConfig) {
    let assessment = assess_safety(safety, config);

    // --- Operational Safety ---
    println!("  Operational:");
    print!("    3D distance:          ");
    print_pass_fail(assessment.distance_3d_pass);
    println!(
        "    Min 3D distance:      {}  (threshold: {})",
        fmt_m(safety.operational.min_distance_3d_km, 1),
        fmt_m(config.min_distance_3d_km, 0),
    );
    print_margin(
        safety.operational.min_distance_3d_km,
        config.min_distance_3d_km,
    );
    println!(
        "      at leg {}, t = {}",
        safety.operational.min_3d_leg_index + 1,
        fmt_duration(safety.operational.min_3d_elapsed_s),
    );
    println!(
        "      RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_3d_ric_position_km.x,
        safety.operational.min_3d_ric_position_km.y,
        safety.operational.min_3d_ric_position_km.z,
    );

    match assessment.rc_context {
        RcContext::AlongTrackDominated { along_track_km } => {
            println!(
                "    Min R/C-plane distance: {}  (along-track dominated, V-bar at {along_track_km:.1} km)",
                fmt_m(safety.operational.min_rc_separation_km, 1),
            );
        }
        RcContext::RadialCrossTrack => {
            println!(
                "    Min R/C-plane distance: {}",
                fmt_m(safety.operational.min_rc_separation_km, 1),
            );
        }
    }
    println!(
        "      at leg {}, t = {}",
        safety.operational.min_rc_leg_index + 1,
        fmt_duration(safety.operational.min_rc_elapsed_s),
    );
    println!(
        "      RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_rc_ric_position_km.x,
        safety.operational.min_rc_ric_position_km.y,
        safety.operational.min_rc_ric_position_km.z,
    );

    // --- Passive Safety ---
    println!("  Passive Safety:");
    print!("    e/i separation:       ");
    print_pass_fail(assessment.ei_separation_pass);
    println!(
        "    Min e/i separation:   {}  (threshold: {})",
        fmt_m(safety.passive.min_ei_separation_km, 1),
        fmt_m(config.min_ei_separation_km, 0),
    );
    print_margin(
        safety.passive.min_ei_separation_km,
        config.min_ei_separation_km,
    );
    println!(
        "    e/i phase angle:      {:.2}\u{00b0}",
        safety.passive.ei_phase_angle_rad.to_degrees()
    );

    print!("\n  Overall:                ");
    print_pass_fail(assessment.overall_pass);
}

/// Print PASS or FAIL with appropriate color.
fn print_pass_fail(pass: bool) {
    if pass {
        println!("{}", "PASS".if_supports_color(Stream::Stdout, |v| v.green()));
    } else {
        println!("{}", "FAIL".if_supports_color(Stream::Stdout, |v| v.red()));
    }
}

/// Print margin above threshold with color: green if >2x, yellow if 1-2x.
///
/// Values are stored in km; displayed in meters.
fn print_margin(value_km: f64, threshold_km: f64) {
    let margin_km = value_km - threshold_km;
    if margin_km > 0.0 {
        let ratio = value_km / threshold_km;
        let margin_str = format!(
            "    Margin:               +{} margin",
            fmt_m(margin_km, 1),
        );
        if ratio > 2.0 {
            println!(
                "{}",
                margin_str.if_supports_color(Stream::Stdout, |v| v.green())
            );
        } else {
            println!(
                "{}",
                margin_str.if_supports_color(Stream::Stdout, |v| v.yellow())
            );
        }
    }
}

/// Print compact safety metrics with threshold context (meters, integer precision).
pub fn print_safety_summary(label: &str, safety: &SafetyMetrics, config: &SafetyConfig) {
    println!("\n  Safety ({label}):");
    println!(
        "    Min 3D distance:     {}  (threshold: {})",
        fmt_m(safety.operational.min_distance_3d_km, 0),
        fmt_m(config.min_distance_3d_km, 0),
    );
    println!(
        "    Min e/i separation:  {}  (threshold: {})",
        fmt_m(safety.passive.min_ei_separation_km, 0),
        fmt_m(config.min_ei_separation_km, 0),
    );
}

/// Format a non-conservative direction flag for safety comparison.
fn nonconservative_flag(analytical: f64, numerical: f64) -> &'static str {
    if numerical < analytical * fidelity::NONCONSERVATIVE_RATIO {
        "  *"
    } else {
        ""
    }
}

/// Print safety comparison table (analytical vs numerical from validation report).
///
/// All values displayed in meters.
pub fn print_safety_comparison(report: &ValidationReport, config: &MissionConfig) {
    let num = &report.numerical_safety;
    println!("\n  Safety Comparison (Analytical vs Numerical):");
    if let Some(ref ana) = report.analytical_safety {
        let sc = config.safety.unwrap_or_default();
        // All values converted km → m for display
        let ana_rc_m = ana.operational.min_rc_separation_km * 1000.0;
        let num_rc_m = num.operational.min_rc_separation_km * 1000.0;
        let ana_d3_m = ana.operational.min_distance_3d_km * 1000.0;
        let num_d3_m = num.operational.min_distance_3d_km * 1000.0;
        let thresh_d3_m = sc.min_distance_3d_km * 1000.0;
        let ana_ei_m = ana.passive.min_ei_separation_km * 1000.0;
        let num_ei_m = num.passive.min_ei_separation_km * 1000.0;
        let thresh_ei_m = sc.min_ei_separation_km * 1000.0;

        println!(
            "    {:>30}  {:>12}  {:>12}  {:>10}",
            "", "Analytical", "Numerical", "Threshold"
        );
        println!(
            "    {:>30}  {:>12.1}  {:>12.1}  {:>10}",
            "Min R/C-plane distance (m)", ana_rc_m, num_rc_m, "-"
        );
        let d3_flag = nonconservative_flag(
            ana.operational.min_distance_3d_km,
            num.operational.min_distance_3d_km,
        );
        println!(
            "    {:>30}  {:>12.1}  {:>12.1}  {:>10.0}{d3_flag}",
            "3D distance (m)", ana_d3_m, num_d3_m, thresh_d3_m,
        );
        let ei_flag = nonconservative_flag(
            ana.passive.min_ei_separation_km,
            num.passive.min_ei_separation_km,
        );
        println!(
            "    {:>30}  {:>12.1}  {:>12.1}  {:>10.0}{ei_flag}",
            "e/i separation (m)", ana_ei_m, num_ei_m, thresh_ei_m,
        );
        if !d3_flag.is_empty() || !ei_flag.is_empty() {
            println!("  * = numerical margin >10% smaller than analytical");
        }
    } else {
        // All values converted km → m for display
        let num_rc_m = num.operational.min_rc_separation_km * 1000.0;
        let num_d3_m = num.operational.min_distance_3d_km * 1000.0;
        let num_ei_m = num.passive.min_ei_separation_km * 1000.0;
        println!("    (no analytical safety available)");
        println!(
            "    {:>30}  {:>12.1}",
            "Min R/C-plane distance (m)", num_rc_m,
        );
        println!(
            "    {:>30}  {:>12.1}",
            "3D distance (m)", num_d3_m,
        );
        println!(
            "    {:>30}  {:>12.1}",
            "e/i separation (m)", num_ei_m,
        );
    }
}

/// Print free-drift (abort-case) analysis per leg.
///
/// Shows operational (3D distance, R/C) and passive (e/i separation) safety
/// as separate pass/fail checks, matching the nominal safety display style.
/// Includes a bounded-motion diagnostic per leg.
pub fn print_free_drift_analysis(analyses: &[FreeDriftAnalysis], config: &SafetyConfig) {
    print_subheader("Free-Drift Safety (abort case)");
    for (i, analysis) in analyses.iter().enumerate() {
        let s = &analysis.safety;
        let assessment = assess_safety(s, config);
        println!("  Leg {}:", i + 1);

        print!("    Operational:          ");
        print_pass_fail(assessment.distance_3d_pass);
        println!(
            "      Min 3D distance:    {}  (threshold: {})",
            fmt_m(s.operational.min_distance_3d_km, 1),
            fmt_m(config.min_distance_3d_km, 0),
        );
        match assessment.rc_context {
            RcContext::AlongTrackDominated { along_track_km } => {
                println!(
                    "      Min R/C distance:   {}  (along-track dominated, V-bar at {along_track_km:.1} km)",
                    fmt_m(s.operational.min_rc_separation_km, 1),
                );
            }
            RcContext::RadialCrossTrack => {
                println!(
                    "      Min R/C distance:   {}",
                    fmt_m(s.operational.min_rc_separation_km, 1),
                );
            }
        }

        print!("    Passive:              ");
        print_pass_fail(assessment.ei_separation_pass);
        println!(
            "      e/i separation:     {}  (threshold: {})",
            fmt_m(s.passive.min_ei_separation_km, 1),
            fmt_m(config.min_ei_separation_km, 0),
        );
        println!(
            "      Phase angle:        {:.2}\u{00b0}",
            s.passive.ei_phase_angle_rad.to_degrees(),
        );

        println!("    Bounded-motion:       {}", fmt_bounded_motion_residual(analysis.bounded_motion_residual));
    }
}

/// Print refined closest-approach (POCA) results per leg.
///
/// Shows the closest approach for each leg — one line per leg, not one per
/// POCA event. The mission-wide global minimum is labeled "(global min)".
/// Diverging legs (empty vec) are noted. Multi-POCA legs show a count.
pub fn print_poca_analysis(label: &str, poca_per_leg: &[Vec<ClosestApproach>]) {
    print_subheader(label);
    for (leg_idx, pocas) in poca_per_leg.iter().enumerate() {
        if pocas.is_empty() {
            println!("  Leg {}: no close approach (diverging)", leg_idx + 1);
            continue;
        }

        // Print the closest POCA for this leg (already sorted ascending)
        let closest = &pocas[0];
        let global_marker = if closest.is_global_minimum {
            " (global min)"
        } else {
            ""
        };

        println!("  Leg {}{global_marker}:", leg_idx + 1);
        println!(
            "    Distance:  {} at t = {}",
            fmt_m(closest.distance_km, 1),
            fmt_duration(closest.elapsed_s),
        );
        println!(
            "    Position:  [{:.4}, {:.4}, {:.4}] km (RIC)",
            closest.position_ric_km.x,
            closest.position_ric_km.y,
            closest.position_ric_km.z,
        );
    }
}

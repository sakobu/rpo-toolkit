//! Safety analysis formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{
    assess_safety, MissionConfig, RcContext, SafetyConfig, SafetyMetrics, ValidationReport,
};

use super::common::fmt_duration;

/// Ratio threshold for flagging non-conservative analytical predictions.
const NONCONSERVATIVE_RATIO: f64 = 0.9;

/// Print safety analysis results with pass/fail assessment.
///
/// `sample_interval_s` is the approximate time between trajectory samples
/// (computed from `total_duration / legs / trajectory_steps`).
pub fn print_safety_analysis(
    safety: &SafetyMetrics,
    config: &SafetyConfig,
    sample_interval_s: f64,
) {
    let assessment = assess_safety(safety, config);

    let interval_label = if sample_interval_s > 0.0 {
        format!("analytical, sampled every ~{sample_interval_s:.0} s")
    } else {
        "analytical".to_string()
    };

    println!("\nSafety Analysis ({interval_label}):");

    // --- Operational Safety ---
    println!("  Operational:");
    print!("    3D distance:          ");
    print_pass_fail(assessment.distance_3d_pass);
    println!(
        "    Min 3D distance:      {:.4} km  (threshold: {:.2} km)",
        safety.operational.min_distance_3d_km, config.min_distance_3d_km
    );
    print_margin(
        safety.operational.min_distance_3d_km,
        config.min_distance_3d_km,
    );
    println!(
        "      Leg: {}  Time: {} (from WP start)",
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
                "    Min R/C plane dist:   {:.4} km  (along-track dominated: {:.2} km along-track)",
                safety.operational.min_rc_separation_km, along_track_km,
            );
        }
        RcContext::RadialCrossTrack => {
            println!(
                "    Min R/C plane dist:   {:.4} km",
                safety.operational.min_rc_separation_km
            );
        }
    }
    println!(
        "      Leg: {}  Time: {} (from WP start)",
        safety.operational.min_rc_leg_index + 1,
        fmt_duration(safety.operational.min_rc_elapsed_s),
    );
    println!(
        "      RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_rc_ric_position_km.x,
        safety.operational.min_rc_ric_position_km.y,
        safety.operational.min_rc_ric_position_km.z,
    );

    // --- Abort Safety ---
    println!("  Abort (free-drift e/i bound):");
    print!("    e/i separation:       ");
    print_pass_fail(assessment.ei_separation_pass);
    println!(
        "    Min e/i separation:   {:.4} km  (threshold: {:.2} km)",
        safety.passive.min_ei_separation_km, config.min_ei_separation_km
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
fn print_margin(value: f64, threshold: f64) {
    let margin = value - threshold;
    if margin > 0.0 {
        let ratio = value / threshold;
        let margin_str = format!("    Margin:               +{margin:.4} km above threshold");
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

/// Print compact safety metrics with threshold context.
pub fn print_safety_summary(label: &str, safety: &SafetyMetrics, config: &SafetyConfig) {
    println!("\n  Safety ({label}):");
    println!(
        "    Min 3D distance:     {:.3} km  (threshold: {:.2} km)",
        safety.operational.min_distance_3d_km, config.min_distance_3d_km,
    );
    println!(
        "    Abort safety (e/i):  {:.3} km  (threshold: {:.2} km)",
        safety.passive.min_ei_separation_km, config.min_ei_separation_km,
    );
}

/// Format a non-conservative direction flag for safety comparison.
fn nonconservative_flag(analytical: f64, numerical: f64) -> &'static str {
    if numerical < analytical * NONCONSERVATIVE_RATIO {
        "  *"
    } else {
        ""
    }
}

/// Print safety comparison table (analytical vs numerical from validation report).
pub fn print_safety_comparison(report: &ValidationReport, config: &MissionConfig) {
    let num = &report.numerical_safety;
    println!("\n  Safety Comparison (Analytical vs Numerical):");
    if let Some(ref ana) = report.analytical_safety {
        let sc = config.safety.unwrap_or_default();
        println!(
            "    {:>20}  {:>12}  {:>12}  {:>8}",
            "", "Analytical", "Numerical", "Thresh."
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8}",
            "R/C plane (km)",
            ana.operational.min_rc_separation_km,
            num.operational.min_rc_separation_km,
            "N/A"
        );
        let d3_flag = nonconservative_flag(
            ana.operational.min_distance_3d_km,
            num.operational.min_distance_3d_km,
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}{}",
            "3D dist (km)",
            ana.operational.min_distance_3d_km,
            num.operational.min_distance_3d_km,
            sc.min_distance_3d_km,
            d3_flag,
        );
        let ei_flag = nonconservative_flag(
            ana.passive.min_ei_separation_km,
            num.passive.min_ei_separation_km,
        );
        println!(
            "    {:>20}  {:>12.4}  {:>12.4}  {:>8.2}{}",
            "e/i sep (km)",
            ana.passive.min_ei_separation_km,
            num.passive.min_ei_separation_km,
            sc.min_ei_separation_km,
            ei_flag,
        );
        if !d3_flag.is_empty() || !ei_flag.is_empty() {
            println!("  * = numerical margin is >10% smaller than analytical");
        }
    } else {
        println!("    (no analytical safety available)");
        println!(
            "    {:>20}  {:>12.4}",
            "R/C plane (km)", num.operational.min_rc_separation_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "3D dist (km)", num.operational.min_distance_3d_km
        );
        println!(
            "    {:>20}  {:>12.4}",
            "e/i sep (km)", num.passive.min_ei_separation_km
        );
    }
}

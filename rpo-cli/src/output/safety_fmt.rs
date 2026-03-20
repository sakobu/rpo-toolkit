//! Safety analysis formatting.

use rpo_core::mission::{
    assess_safety, MissionConfig, RcContext, SafetyConfig, SafetyMetrics, ValidationReport,
};

/// Ratio threshold for flagging non-conservative analytical predictions.
const NONCONSERVATIVE_RATIO: f64 = 0.9;

/// Print safety analysis results with pass/fail assessment.
pub fn print_safety_analysis(safety: &SafetyMetrics, config: &SafetyConfig) {
    let assessment = assess_safety(safety, config);

    println!("\nOperational Safety (analytical, sampled every ~21 s):");
    println!(
        "  3D distance constraint:   {}",
        if assessment.distance_3d_pass {
            "PASS"
        } else {
            "FAIL"
        }
    );
    println!(
        "  Min 3D distance:          {:.4} km  (threshold: {:.2} km)",
        safety.operational.min_distance_3d_km, config.min_distance_3d_km
    );
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min, from WP start)",
        safety.operational.min_3d_leg_index + 1,
        safety.operational.min_3d_elapsed_s,
        safety.operational.min_3d_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_3d_ric_position_km.x,
        safety.operational.min_3d_ric_position_km.y,
        safety.operational.min_3d_ric_position_km.z,
    );

    match assessment.rc_context {
        RcContext::AlongTrackDominated { along_track_km } => {
            println!(
                "  Min R/C plane dist:       {:.4} km  (along-track dominated: {:.2} km along-track)",
                safety.operational.min_rc_separation_km, along_track_km,
            );
        }
        RcContext::RadialCrossTrack => {
            println!(
                "  Min R/C plane dist:       {:.4} km",
                safety.operational.min_rc_separation_km
            );
        }
    }
    println!(
        "    Leg: {}  Time: {:.1} s ({:.2} min, from WP start)",
        safety.operational.min_rc_leg_index + 1,
        safety.operational.min_rc_elapsed_s,
        safety.operational.min_rc_elapsed_s / 60.0,
    );
    println!(
        "    RIC: [{:.4}, {:.4}, {:.4}] km",
        safety.operational.min_rc_ric_position_km.x,
        safety.operational.min_rc_ric_position_km.y,
        safety.operational.min_rc_ric_position_km.z,
    );

    println!("\nAbort Safety (free-drift e/i bound):");
    println!(
        "  e/i separation constraint: {}",
        if assessment.ei_separation_pass {
            "PASS"
        } else {
            "FAIL"
        }
    );
    println!(
        "  Min e/i separation:       {:.4} km  (threshold: {:.2} km)",
        safety.passive.min_ei_separation_km, config.min_ei_separation_km
    );
    let margin = safety.passive.min_ei_separation_km - config.min_ei_separation_km;
    if margin > 0.0 {
        println!(
            "  Margin:                   +{margin:.4} km above threshold"
        );
    }
    println!(
        "  e/i phase angle:          {:.2}\u{00b0}",
        safety.passive.ei_phase_angle_rad.to_degrees()
    );

    println!(
        "\n  Overall:                  {}",
        if assessment.overall_pass {
            "PASS"
        } else {
            "FAIL"
        }
    );
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

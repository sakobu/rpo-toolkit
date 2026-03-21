//! Eclipse formatting.

use rpo_core::mission::EclipseValidation;
use rpo_core::types::MissionEclipseData;

use super::common::{fmt_duration, print_colored, ThresholdDirection};

/// Shadow fraction threshold above which we color yellow.
const SHADOW_FRACTION_WARN: f64 = 0.25;

/// Shadow fraction threshold above which we color red.
const SHADOW_FRACTION_ALERT: f64 = 0.50;

/// Print eclipse summary content (intervals, durations, shadow fraction).
///
/// The caller is responsible for printing the section header.
pub fn print_eclipse_summary(eclipse: &MissionEclipseData) {
    println!(
        "  Shadow intervals:   {}",
        eclipse.summary.intervals.len()
    );
    let frac = eclipse.summary.time_in_shadow_fraction;
    let frac_str = format!(
        "  Total shadow time:  {} ({:.1}% of waypoint phase)",
        fmt_duration(eclipse.summary.total_shadow_duration_s),
        frac * 100.0,
    );
    print_colored(&frac_str, frac, SHADOW_FRACTION_WARN, SHADOW_FRACTION_ALERT, ThresholdDirection::LowerIsBetter);
    if eclipse.summary.max_shadow_duration_s > 0.0 {
        println!(
            "  Max single eclipse: {}",
            fmt_duration(eclipse.summary.max_shadow_duration_s),
        );
    }
}

/// Print eclipse validation comparison (analytical vs ANISE).
pub fn print_eclipse_validation(ev: &EclipseValidation, max_eclipse_duration_s: f64) {
    println!("\n  Eclipse Validation (Analytical vs ANISE):");
    println!(
        "    Sun direction error:  max {:.4}\u{00b0}, mean {:.4}\u{00b0}",
        ev.max_sun_direction_error_rad.to_degrees(),
        ev.mean_sun_direction_error_rad.to_degrees(),
    );
    println!(
        "    Eclipse intervals:    {} analytical, {} numerical, {} matched",
        ev.analytical_interval_count, ev.numerical_interval_count, ev.matched_interval_count,
    );
    if !ev.interval_comparisons.is_empty() {
        println!(
            "    Timing error:         max {:.1} s, mean {:.1} s (entry/exit)",
            ev.max_timing_error_s, ev.mean_timing_error_s,
        );
        if max_eclipse_duration_s > 0.0 {
            let pct = ev.max_timing_error_s / max_eclipse_duration_s * 100.0;
            println!(
                "      ({pct:.1}% of max eclipse; acceptable for mission planning)",
            );
        }
    }
    if ev.unmatched_interval_count > 0 {
        println!("    Unmatched intervals:  {}", ev.unmatched_interval_count,);
    }
}

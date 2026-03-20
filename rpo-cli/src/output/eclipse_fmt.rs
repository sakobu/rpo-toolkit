//! Eclipse formatting.

use rpo_core::mission::EclipseValidation;
use rpo_core::types::MissionEclipseData;

/// Print eclipse summary (intervals, durations, shadow fraction).
pub fn print_eclipse_summary(eclipse: &MissionEclipseData) {
    println!("\nEclipse Summary:");
    println!(
        "  Shadow intervals:   {}",
        eclipse.summary.intervals.len()
    );
    println!(
        "  Total shadow time:  {:.0} s ({:.1}% of waypoint phase)",
        eclipse.summary.total_shadow_duration_s,
        eclipse.summary.time_in_shadow_fraction * 100.0,
    );
    if eclipse.summary.max_shadow_duration_s > 0.0 {
        println!(
            "  Max single eclipse: {:.0} s ({:.1} min)",
            eclipse.summary.max_shadow_duration_s,
            eclipse.summary.max_shadow_duration_s / 60.0,
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
                "      ({pct:.1}% of max eclipse; adequate for planning, not power-critical analysis)",
            );
        }
    }
    if ev.unmatched_interval_count > 0 {
        println!("    Unmatched intervals:  {}", ev.unmatched_interval_count,);
    }
}

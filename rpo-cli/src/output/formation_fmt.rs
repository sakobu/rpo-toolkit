//! Formation design output formatting (human-readable + markdown).

use std::fmt::Write;

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{
    DriftCompensationStatus, EiAlignment, EnrichmentMode, FormationDesignReport,
    PerchEnrichmentResult, PerchFallbackReason,
};

use super::common::{fmt_duration, fmt_m, print_subheader};

// ---------------------------------------------------------------------------
// Human-readable (terminal) output
// ---------------------------------------------------------------------------

/// Print perch enrichment results after the Perch ROE section.
///
/// Shows enrichment status, alignment, delta-e/delta-i magnitudes,
/// and resulting minimum R/C separation. For fallback cases, shows
/// the reason enrichment could not be applied.
pub fn print_perch_enrichment(report: &FormationDesignReport) {
    print_subheader("Perch Enrichment");
    match &report.perch {
        PerchEnrichmentResult::Enriched(safe_perch) => {
            println!(
                "  Status:             {}",
                "ENRICHED".if_supports_color(Stream::Stdout, |v| v.green()),
            );
            println!("  Alignment:          {}", format_alignment(safe_perch.alignment));
            println!("  \u{03b4}e magnitude:       {}", fmt_m(safe_perch.de_magnitude_km, 1));
            println!("  \u{03b4}i magnitude:       {}", fmt_m(safe_perch.di_magnitude_km, 1));
            println!("  Min R/C separation: {}", fmt_m(safe_perch.min_rc_separation_km, 1));
        }
        PerchEnrichmentResult::Fallback { reason, .. } => {
            println!(
                "  Status:             {}",
                "FALLBACK".if_supports_color(Stream::Stdout, |v| v.yellow()),
            );
            println!("  Reason:             {}", format_fallback_reason(reason));
        }
    }
}

/// Print per-waypoint enrichment advisory table after the Waypoint Targeting table.
///
/// This is advisory data comparing baseline (minimum-norm) e/i separation
/// against the enriched (safe) alternative for each waypoint.
pub fn print_waypoint_enrichment(report: &FormationDesignReport) {
    if report.waypoints.is_empty() {
        return;
    }
    print_subheader("Waypoint Enrichment (advisory)");
    println!(
        "  {:>4}  {:>12}  {:>12}  {:>7}  {:<15}",
        "Leg", "Baseline e/i", "Enriched e/i", "Phase", "Mode",
    );
    println!(
        "  {:-<4}  {:-<12}  {:-<12}  {:-<7}  {:-<15}",
        "", "", "", "", "",
    );
    for (i, wp) in report.waypoints.iter().enumerate() {
        match wp {
            Some(enriched) => {
                println!(
                    "  {:>4}  {:>12}  {:>12}  {:>6.1}\u{00b0}  {:<15}",
                    i + 1,
                    fmt_m(enriched.baseline_ei.min_separation_km, 1),
                    fmt_m(enriched.enriched_ei.min_separation_km, 1),
                    enriched.enriched_ei.phase_angle_rad.to_degrees(),
                    format_enrichment_mode(enriched.mode),
                );
            }
            None => {
                println!(
                    "  {:>4}  {:>12}  {:>12}  {:>7}  {:<15}",
                    i + 1, "N/A", "N/A", "-", "failed",
                );
            }
        }
    }
}

/// Print per-leg transit safety with PASS/FAIL after the Safety section.
///
/// Shows minimum e/i separation, timing, phase angle, and drift compensation
/// status for each leg. Includes mission-wide minimum at the bottom.
pub fn print_transit_safety(report: &FormationDesignReport) {
    if report.transit_safety.is_empty() {
        return;
    }
    print_subheader("Formation Transit Safety");
    println!(
        "  {:>4}  {:>11}  {:>9}  {:>7}  {:>10}  Status",
        "Leg", "Min e/i sep", "At", "Phase", "Drift comp",
    );
    println!(
        "  {:-<4}  {:-<11}  {:-<9}  {:-<7}  {:-<10}  {:-<6}",
        "", "", "", "", "", "",
    );
    for (i, ts) in report.transit_safety.iter().enumerate() {
        match ts {
            Some(t) => {
                let status = if t.satisfies_requirement {
                    "PASS".if_supports_color(Stream::Stdout, |v| v.green()).to_string()
                } else {
                    "FAIL".if_supports_color(Stream::Stdout, |v| v.red()).to_string()
                };
                let drift = match t.drift_compensation {
                    DriftCompensationStatus::Applied => "applied",
                    DriftCompensationStatus::Skipped => "skipped",
                };
                println!(
                    "  {:>4}  {:>11}  {:>9}  {:>6.1}\u{00b0}  {:>10}  {}",
                    i + 1,
                    fmt_m(t.min_ei_separation_km, 1),
                    fmt_duration(t.min_elapsed_s),
                    t.min_phase_angle_rad.to_degrees(),
                    drift,
                    status,
                );
            }
            None => {
                println!(
                    "  {:>4}  {:>11}  {:>9}  {:>7}  {:>10}  -",
                    i + 1, "N/A", "-", "-", "-",
                );
            }
        }
    }

    // Mission-wide minimum
    match report.mission_min_ei_separation_km {
        Some(min_km) => {
            println!("\n  Mission min e/i separation: {}", fmt_m(min_km, 1));
        }
        None => {
            println!("\n  Mission min e/i separation: N/A");
        }
    }
}

// ---------------------------------------------------------------------------
// Markdown output
// ---------------------------------------------------------------------------

/// Write perch enrichment section to markdown output.
pub fn write_perch_enrichment_md(out: &mut String, report: &FormationDesignReport) {
    let _ = writeln!(out, "## Perch Enrichment\n");
    match &report.perch {
        PerchEnrichmentResult::Enriched(safe_perch) => {
            let _ = writeln!(out, "| Parameter | Value |");
            let _ = writeln!(out, "| --- | --- |");
            let _ = writeln!(out, "| Status | ENRICHED |");
            let _ = writeln!(out, "| Alignment | {} |", format_alignment(safe_perch.alignment));
            let _ = writeln!(
                out,
                "| \u{03b4}e magnitude | {} |",
                fmt_m(safe_perch.de_magnitude_km, 1),
            );
            let _ = writeln!(
                out,
                "| \u{03b4}i magnitude | {} |",
                fmt_m(safe_perch.di_magnitude_km, 1),
            );
            let _ = writeln!(
                out,
                "| Min R/C separation | {} |",
                fmt_m(safe_perch.min_rc_separation_km, 1),
            );
        }
        PerchEnrichmentResult::Fallback { reason, .. } => {
            let _ = writeln!(out, "| Parameter | Value |");
            let _ = writeln!(out, "| --- | --- |");
            let _ = writeln!(out, "| Status | FALLBACK |");
            let _ = writeln!(out, "| Reason | {} |", format_fallback_reason(reason));
        }
    }
    let _ = writeln!(out);
}

/// Write waypoint enrichment advisory table to markdown output.
pub fn write_waypoint_enrichment_md(out: &mut String, report: &FormationDesignReport) {
    if report.waypoints.is_empty() {
        return;
    }
    let _ = writeln!(out, "## Waypoint Enrichment (Advisory)\n");
    let _ = writeln!(out, "| Leg | Baseline e/i | Enriched e/i | Phase | Mode |");
    let _ = writeln!(out, "| --- | --- | --- | --- | --- |");
    for (i, wp) in report.waypoints.iter().enumerate() {
        match wp {
            Some(enriched) => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {} | {:.1}\u{00b0} | {} |",
                    i + 1,
                    fmt_m(enriched.baseline_ei.min_separation_km, 1),
                    fmt_m(enriched.enriched_ei.min_separation_km, 1),
                    enriched.enriched_ei.phase_angle_rad.to_degrees(),
                    format_enrichment_mode(enriched.mode),
                );
            }
            None => {
                let _ = writeln!(out, "| {} | N/A | N/A | - | failed |", i + 1);
            }
        }
    }
    let _ = writeln!(out);
}

/// Write transit safety table to markdown output.
pub fn write_transit_safety_md(out: &mut String, report: &FormationDesignReport) {
    if report.transit_safety.is_empty() {
        return;
    }
    let _ = writeln!(out, "## Formation Transit Safety\n");
    let _ = writeln!(
        out,
        "| Leg | Min e/i sep | At | Phase | Drift comp | Status |",
    );
    let _ = writeln!(out, "| --- | --- | --- | --- | --- | --- |");
    for (i, ts) in report.transit_safety.iter().enumerate() {
        match ts {
            Some(t) => {
                let status = if t.satisfies_requirement {
                    "\u{2705} PASS"
                } else {
                    "\u{274c} FAIL"
                };
                let drift = match t.drift_compensation {
                    DriftCompensationStatus::Applied => "applied",
                    DriftCompensationStatus::Skipped => "skipped",
                };
                let _ = writeln!(
                    out,
                    "| {} | {} | {} | {:.1}\u{00b0} | {} | {} |",
                    i + 1,
                    fmt_m(t.min_ei_separation_km, 1),
                    fmt_duration(t.min_elapsed_s),
                    t.min_phase_angle_rad.to_degrees(),
                    drift,
                    status,
                );
            }
            None => {
                let _ = writeln!(out, "| {} | N/A | - | - | - | - |", i + 1);
            }
        }
    }

    // Mission-wide minimum
    match report.mission_min_ei_separation_km {
        Some(min_km) => {
            let _ = writeln!(out, "\n**Mission min e/i separation:** {}\n", fmt_m(min_km, 1));
        }
        None => {
            let _ = writeln!(out, "\n**Mission min e/i separation:** N/A\n");
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Format e/i alignment strategy for display.
fn format_alignment(alignment: EiAlignment) -> &'static str {
    match alignment {
        EiAlignment::Parallel => "Parallel",
        EiAlignment::AntiParallel => "Anti-Parallel",
        EiAlignment::Auto => "Auto",
    }
}

/// Format enrichment mode for display.
fn format_enrichment_mode(mode: EnrichmentMode) -> &'static str {
    match mode {
        EnrichmentMode::PositionOnly => "position-only",
        EnrichmentMode::VelocityConstrained => "vel-constrained",
    }
}

/// Format a perch fallback reason as a concise human-readable string.
fn format_fallback_reason(reason: &PerchFallbackReason) -> String {
    match reason {
        PerchFallbackReason::SingularGeometry { mean_arg_lat_rad } => {
            format!(
                "singular geometry at mean arg lat = {:.2}\u{00b0}",
                mean_arg_lat_rad.to_degrees(),
            )
        }
        PerchFallbackReason::SeparationUnachievable {
            requested_km,
            achievable_km,
        } => {
            format!(
                "requested {} exceeds achievable {}",
                fmt_m(*requested_km, 1),
                fmt_m(*achievable_km, 1),
            )
        }
        PerchFallbackReason::InvalidChiefElements { detail } => {
            format!("invalid chief elements: {detail}")
        }
        PerchFallbackReason::SafetyAnalysis { detail } => {
            format!("safety analysis failed: {detail}")
        }
        PerchFallbackReason::Propagation { detail } => {
            format!("propagation failed: {detail}")
        }
        PerchFallbackReason::KeplerFailure { detail } => {
            format!("kepler failure: {detail}")
        }
        PerchFallbackReason::InsufficientSampling {
            total_samples,
            required_per_orbit,
        } => {
            format!(
                "insufficient sampling: {total_samples} samples < {required_per_orbit} required/orbit"
            )
        }
    }
}

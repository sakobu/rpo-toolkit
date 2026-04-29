//! Formation design markdown output formatting.

use std::fmt::Write;

use rpo_core::mission::{
    EiAlignment, EnrichmentMode, EnrichmentSuggestion, FormationDesignReport,
};

use crate::output::fmt::{fmt_duration, fmt_m, fmt_roe_component};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Write the complete Formation Design section to markdown output.
///
/// Groups perch enrichment, waypoint enrichment advisory, and transit safety
/// under a single `## Formation Design` heading with narrative context.
pub fn write_formation_design_md(out: &mut String, report: &FormationDesignReport) {
    let _ = writeln!(out, "## Formation Design\n");

    write_perch_enrichment_md(out, report);

    // Consolidated e/i context block — emitted once when enrichment is active
    if matches!(report.perch, EnrichmentSuggestion::Enriched { .. }) {
        let _ = writeln!(
            out,
            "> **e/i context:** V-bar perch geometry has zero e/i separation by construction. \
             Perch enrichment establishes separation at the departure state, but waypoint \
             targeting preserves RIC position, not e/i geometry \u{2014} transit degradation is \
             expected. For guided operations, operational safety (3D distance) is the \
             governing constraint. e/i metrics are reported for completeness and free-drift \
             contingency assessment.\n",
        );
    }

    write_waypoint_enrichment_md(out, report);
    write_transit_safety_md(out, report);
}

/// Write a condensed Formation Design summary for validate/MC reports.
///
/// Emits perch status, mission-wide minimum e/i, and the worst transit e/i —
/// enough for an operator to confirm the design unchanged from the mission tier
/// without repeating ~50 lines of detail. Points at the mission report for the
/// full breakdown.
pub fn write_formation_design_condensed_md(out: &mut String, report: &FormationDesignReport) {
    let _ = writeln!(out, "## Formation Design\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");

    // Perch status summary
    let perch_line = match &report.perch {
        EnrichmentSuggestion::Enriched { safe_perch, .. } => format!(
            "ENRICHED ({}, \u{03b4}e = {}, \u{03b4}i = {})",
            format_alignment(safe_perch.alignment),
            fmt_m(safe_perch.de_magnitude_km, 1),
            fmt_m(safe_perch.di_magnitude_km, 1),
        ),
        EnrichmentSuggestion::Baseline { .. } => "NOT APPLIED (baseline)".to_string(),
    };
    let _ = writeln!(out, "| Perch | {perch_line} |");

    // Worst transit e/i across all legs (look at transit_safety)
    let worst_transit: Option<(usize, f64)> = report
        .transit_safety
        .iter()
        .enumerate()
        .filter_map(|(i, ts)| ts.as_ref().map(|t| (i + 1, t.min_ei_separation_km)))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    if let Some((leg, min_km)) = worst_transit {
        let _ = writeln!(out, "| Min transit e/i | {} (leg {leg}) |", fmt_m(min_km, 1));
    }

    // Mission-wide min e/i
    if let Some(min_km) = report.mission_min_ei_separation_km {
        let _ = writeln!(out, "| Mission min e/i | {} |", fmt_m(min_km, 1));
    }

    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "> Full formation design detail is available in the `mission` report.\n",
    );
}

// ---------------------------------------------------------------------------
// Markdown helpers
// ---------------------------------------------------------------------------

/// Write perch enrichment section to markdown output (### heading).
fn write_perch_enrichment_md(out: &mut String, report: &FormationDesignReport) {
    let _ = writeln!(out, "### Perch Enrichment\n");
    match &report.perch {
        EnrichmentSuggestion::Enriched { safe_perch, .. } => {
            let _ = writeln!(out, "| Parameter | Value |");
            let _ = writeln!(out, "| --- | --- |");
            let _ = writeln!(out, "| Status | ENRICHED |");
            let _ = writeln!(
                out,
                "| Alignment | {} |",
                format_alignment(safe_perch.alignment),
            );
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
            if let Some(ref pred) = report.drift_prediction {
                let _ = writeln!(
                    out,
                    "| Predicted mid-transit e/i | {} ({:.1}\u{00b0}) |",
                    fmt_m(pred.predicted_min_ei_km, 1),
                    pred.predicted_phase_angle_rad.to_degrees(),
                );
            }
            let _ = writeln!(out);

            // Before/after ROE comparison
            let b = &safe_perch.baseline_roe;
            let e = &safe_perch.roe;
            let _ = writeln!(out, "**Baseline \u{2192} Enriched ROE:**\n");
            let _ = writeln!(out, "| Element | Baseline | Enriched |");
            let _ = writeln!(out, "| --- | --- | --- |");
            for (label, bv, ev) in [
                ("\u{03b4}a", b.da, e.da),
                ("\u{03b4}\u{03bb}", b.dlambda, e.dlambda),
                ("\u{03b4}ex", b.dex, e.dex),
                ("\u{03b4}ey", b.dey, e.dey),
                ("\u{03b4}ix", b.dix, e.dix),
                ("\u{03b4}iy", b.diy, e.diy),
            ] {
                let _ = writeln!(
                    out,
                    "| {} | {} | {} |",
                    label,
                    fmt_roe_component(bv),
                    fmt_roe_component(ev),
                );
            }
        }
        EnrichmentSuggestion::Baseline { .. } => {
            let _ = writeln!(out, "| Parameter | Value |");
            let _ = writeln!(out, "| --- | --- |");
            let _ = writeln!(out, "| Status | NOT APPLIED (baseline) |");
        }
    }
    let _ = writeln!(out);
}

/// Write waypoint enrichment advisory table to markdown output (### heading).
fn write_waypoint_enrichment_md(out: &mut String, report: &FormationDesignReport) {
    if report.waypoints.is_empty() {
        return;
    }
    let _ = writeln!(out, "### Waypoint Enrichment (Advisory)\n");
    let _ = writeln!(
        out,
        "> Not applied to targeting. See e/i context above.\n",
    );
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

/// Write transit safety table to markdown output (### heading).
fn write_transit_safety_md(out: &mut String, report: &FormationDesignReport) {
    if report.transit_safety.is_empty() {
        return;
    }
    let _ = writeln!(out, "### Transit Safety\n");
    let _ = writeln!(
        out,
        "> Guided waypoint targeting enforces RIC position and velocity at each waypoint; \
         it does not constrain e/i geometry. Near-zero transit e/i on intermediate legs is a \
         property of this targeting mode, not a COLA failure. See e/i context above for the \
         operational interpretation.\n",
    );
    let _ = writeln!(
        out,
        "| Leg | Min e/i sep | At | Phase | Status |",
    );
    let _ = writeln!(out, "| --- | --- | --- | --- | --- |");
    for (i, ts) in report.transit_safety.iter().enumerate() {
        match ts {
            Some(t) => {
                let status = if t.satisfies_requirement {
                    "\u{2705}"
                } else {
                    "\u{26a0}\u{fe0f}"
                };
                let _ = writeln!(
                    out,
                    "| {} | {} | {} | {:.1}\u{00b0} | {} |",
                    i + 1,
                    fmt_m(t.min_ei_separation_km, 1),
                    fmt_duration(t.min_elapsed_s),
                    t.min_phase_angle_rad.to_degrees(),
                    status,
                );
            }
            None => {
                let _ = writeln!(out, "| {} | N/A | - | - | - |", i + 1);
            }
        }
    }

    // Mission-wide minimum
    match report.mission_min_ei_separation_km {
        Some(min_km) => {
            let _ = writeln!(
                out,
                "\n**Mission min e/i separation:** {}\n",
                fmt_m(min_km, 1),
            );
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


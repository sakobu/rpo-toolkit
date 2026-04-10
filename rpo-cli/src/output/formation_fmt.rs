//! Formation design markdown output formatting.

use std::fmt::Write;

use rpo_core::mission::{
    EiAlignment, EnrichmentMode, FormationDesignReport,
    PerchEnrichmentResult, PerchFallbackReason,
};

use super::common::{fmt_duration, fmt_m, fmt_roe_component};

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
    if matches!(report.perch, PerchEnrichmentResult::Enriched(_)) {
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

// ---------------------------------------------------------------------------
// Markdown helpers
// ---------------------------------------------------------------------------

/// Write perch enrichment section to markdown output (### heading).
fn write_perch_enrichment_md(out: &mut String, report: &FormationDesignReport) {
    let _ = writeln!(out, "### Perch Enrichment\n");
    match &report.perch {
        PerchEnrichmentResult::Enriched(safe_perch) => {
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
        PerchEnrichmentResult::Baseline(_) => {
            let _ = writeln!(out, "| Parameter | Value |");
            let _ = writeln!(out, "| --- | --- |");
            let _ = writeln!(out, "| Status | NOT APPLIED (baseline) |");
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

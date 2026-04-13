//! Per-tier Recommendations section writer.
//!
//! Emits a short action-oriented block at the bottom of each report,
//! tailored to what the tier reveals:
//!
//! - Mission tier: "run validate", "review free-drift", eclipse overlap
//! - Validate tier: COLA remedies, sensitivity follow-up (`mc`)
//! - MC tier: design-level guidance based on dispersion signals
//!
//! Conditional recommendations are driven by the pre-computed insight list
//! so a single source of truth powers both the alert box (top) and the
//! Recommendations block (bottom).

use std::fmt::Write;

use rpo_core::mission::MonteCarloReport;
use rpo_core::pipeline::{PipelineInput, PipelineOutput};

use crate::output::fmt::KM_TO_M;
use crate::output::insights::{Insight, Severity};
use crate::output::thresholds::rate as rate_thresh;

/// Write the Recommendations section for a mission report.
///
/// # Arguments
/// - `out` — output buffer; the section markdown is appended.
/// - `output` — pipeline output used for COLA / eclipse detection.
/// - `input` — pipeline input (read for `input.base.cola` target).
pub(crate) fn write_mission_recommendations(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
) {
    let mut items: Vec<String> = Vec::new();

    items.push(
        "Run `validate --auto-drag` to confirm safety margins under full-physics propagation."
            .to_string(),
    );

    // Check for COLA application → follow-up advice
    if let (Some(cola), Some(cola_config)) = (&output.safety.cola, &input.base.cola) {
        let target_m = cola_config.target_distance_km * KM_TO_M;
        let any_miss = cola
            .iter()
            .any(|m| m.post_avoidance_poca_km < cola_config.target_distance_km);
        if any_miss {
            items.push(format!(
                "COLA analytical solver did not reach the {target_m:.0} m target \
                 \u{2014} redesign waypoint offsets or raise COLA budget, then re-run `mission`.",
            ));
        }
    }

    // Eclipse overlap with burns
    if let Some(ref eclipse) = output.mission.eclipse {
        if !eclipse.summary.intervals.is_empty() {
            items.push(
                "Check \"Burns in Shadow\" under Eclipse \u{2014} burns firing in umbra may \
                 have thermal/attitude implications.".to_string(),
            );
        }
    }

    items.push(
        "Review Free-Drift Safety (abort case) for each leg before uplink.".to_string(),
    );

    emit_recommendations(out, &items);
}

/// Write the Recommendations section for a validate report.
///
/// # Arguments
/// - `out` — output buffer; the section markdown is appended.
/// - `output` — pipeline output used for COLA / eclipse detection.
/// - `insights` — the full insight list; text-matched to drive optional items.
pub(crate) fn write_validate_recommendations(
    out: &mut String,
    output: &PipelineOutput,
    insights: &[Insight],
) {
    let mut items: Vec<String> = Vec::new();

    // COLA effectiveness — if any insight mentions a COLA threshold miss, add remedy
    let cola_missed = insights.iter().any(|i| {
        matches!(i.severity, Severity::Critical)
            && i.message.contains("COLA burn")
            && i.message.contains("did not achieve target")
    });
    if cola_missed {
        items.push(
            "COLA burn did not achieve target separation under full physics \
             \u{2014} consider increasing waypoint offsets or raising the COLA target."
                .to_string(),
        );
    }

    // Always recommend moving to MC for sensitivity
    items.push(
        "Run `mc --auto-drag` to assess sensitivity to navigation and maneuver execution errors."
            .to_string(),
    );

    // Eclipse overlap reminder (same as mission)
    if let Some(ref eclipse) = output.mission.eclipse {
        if !eclipse.summary.intervals.is_empty() {
            items.push(
                "Re-verify burn-in-shadow overlaps against the post-COLA schedule \
                 under Eclipse.".to_string(),
            );
        }
    }

    emit_recommendations(out, &items);
}

/// Write the Recommendations section for a Monte Carlo report.
///
/// # Arguments
/// - `out` — output buffer; the section markdown is appended.
/// - `report` — the Monte Carlo report carrying ensemble statistics.
/// - `insights` — the full insight list; text-matched to drive optional items.
pub(crate) fn write_mc_recommendations(
    out: &mut String,
    report: &MonteCarloReport,
    insights: &[Insight],
) {
    let mut items: Vec<String> = Vec::new();
    let stats = &report.statistics;

    // Collision probability > 0 → urgent redesign
    if stats.collision_probability > rate_thresh::ZERO_VIOLATIONS {
        items.push(
            "Redesign waypoint geometry \u{2014} collisions detected in the MC ensemble."
                .to_string(),
        );
    }

    // e/i nominal-inherited failure → design-level guidance
    let nominal_ei_inherited = insights
        .iter()
        .any(|i| i.message.contains("MC ensemble inherits this"));
    if nominal_ei_inherited {
        items.push(
            "Nominal e/i design fails the threshold \u{2014} free-drift contingency \
             requires a larger e/i offset in the baseline, not more dispersion margin."
                .to_string(),
        );
    }

    // Convergence failures
    if stats.convergence_rate < 1.0 {
        items.push(
            "Convergence below 100% \u{2014} inspect failing samples under Diagnostics \
             and tighten waypoint tolerances."
                .to_string(),
        );
    }

    // Default item (always included, last)
    items.push(
        "Review dispersion envelope growth across legs for operational margin adequacy."
            .to_string(),
    );

    emit_recommendations(out, &items);
}

/// Emit the Recommendations block header and numbered list.
///
/// # Arguments
/// - `out` — output buffer; the section markdown is appended.
/// - `items` — the numbered recommendations; empty list renders nothing.
fn emit_recommendations(out: &mut String, items: &[String]) {
    if items.is_empty() {
        return;
    }
    let _ = writeln!(out, "## Recommendations\n");
    for (i, item) in items.iter().enumerate() {
        let _ = writeln!(out, "{}. {item}", i + 1);
    }
    let _ = writeln!(out);
}

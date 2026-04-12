//! COLA (collision avoidance) section writers.

use std::fmt::Write;

use rpo_core::mission::ValidationReport;
use rpo_core::pipeline::{PipelineInput, PipelineOutput};

use crate::output::fmt::KM_TO_M;
use crate::output::report::helpers::ReportContext;

use super::safety::format_leg_list;

/// Map a correction type to its display label.
fn correction_type_label(ct: rpo_core::mission::CorrectionType) -> &'static str {
    match ct {
        rpo_core::mission::CorrectionType::InPlane => "in-plane",
        rpo_core::mission::CorrectionType::CrossTrack => "cross-track",
        rpo_core::mission::CorrectionType::Combined => "combined",
    }
}

/// Write all COLA-related sections (main COLA, secondary conjunctions, skipped legs).
pub(crate) fn write_cola_sections(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
    context: ReportContext,
) {
    if let Some(ref cola) = output.safety.cola {
        let target_distance_km = input.base.cola.as_ref().map(|c| c.target_distance_km);
        write_cola_section(out, cola, target_distance_km, context);
    }
    if let Some(ref secondary) = output.safety.secondary_conjunctions {
        write_secondary_conjunction_section(out, secondary);
    }
    if let Some(ref skipped) = output.safety.cola_skipped {
        write_cola_skipped_section(out, skipped);
    }
}

/// A COLA maneuver whose analytical post-burn POCA fell below the
/// configured target distance. Collected during table rendering so the
/// callout beneath the table can quote concrete per-leg values (achieved
/// POCA and fuel cost) instead of a generic "leg(s) X" hedge.
struct OffendingCola {
    /// 1-based leg index, matching the rendered table's Leg column.
    leg_1_based: usize,
    /// Achieved post-burn POCA in metres (analytical — full-physics
    /// margins are typically smaller).
    achieved_poca_m: f64,
    /// Analytical fuel cost for the burn in m/s.
    fuel_cost_m_s: f64,
}

/// Write the COLA maneuver table with per-leg Dv, POCA, and sub-threshold callout.
fn write_cola_section(
    out: &mut String,
    maneuvers: &[rpo_core::mission::AvoidanceManeuver],
    target_distance_km: Option<f64>,
    context: ReportContext,
) {
    let _ = writeln!(out, "## Collision Avoidance (Post-Baseline Adjustment)\n");
    let _ = writeln!(
        out,
        "> Avoidance maneuvers computed after baseline targeting to increase closest-approach distance.\n",
    );
    if maneuvers.is_empty() {
        let _ = writeln!(out, "No POCA violations requiring avoidance.\n");
        return;
    }
    let _ = writeln!(
        out,
        "| Leg | \u{0394}v (km/s) | u (rad) | Post-COLA POCA | Cost (m/s) | Type |",
    );
    let _ = writeln!(
        out,
        "|-----|-----------|---------|---------------|------------|------|",
    );
    // Track offending legs with their achieved POCA and fuel cost so the
    // callout below can quote per-leg values instead of a generic "leg(s) X"
    // hedge that obscures how much margin was actually lost.
    let mut offending: Vec<OffendingCola> = Vec::new();
    for m in maneuvers {
        let correction = correction_type_label(m.correction_type);
        let poca_cell = match target_distance_km {
            Some(target_km) if m.post_avoidance_poca_km < target_km => {
                offending.push(OffendingCola {
                    leg_1_based: m.leg_index + 1,
                    achieved_poca_m: m.post_avoidance_poca_km * KM_TO_M,
                    fuel_cost_m_s: m.fuel_cost_km_s * KM_TO_M,
                });
                format!(
                    "{:.1} m \u{26a0}\u{fe0f} (target: {:.0} m)",
                    m.post_avoidance_poca_km * KM_TO_M,
                    target_km * KM_TO_M,
                )
            }
            _ => format!("{:.1} m", m.post_avoidance_poca_km * KM_TO_M),
        };
        let _ = writeln!(
            out,
            "| {} | [{:.6}, {:.6}, {:.6}] | {:.4} | {} | {:.2} | {} |",
            m.leg_index + 1,
            m.dv_ric_km_s.x,
            m.dv_ric_km_s.y,
            m.dv_ric_km_s.z,
            m.maneuver_location_rad,
            poca_cell,
            m.fuel_cost_km_s * KM_TO_M,
            correction,
        );
    }
    let _ = writeln!(out);
    if !offending.is_empty() {
        let target_m = target_distance_km.unwrap_or(0.0) * KM_TO_M;
        let (leg_desc, per_leg_suffix) = match offending.as_slice() {
            [OffendingCola { leg_1_based, achieved_poca_m, fuel_cost_m_s }] => (
                format!("leg {leg_1_based}"),
                format!(" (achieved {achieved_poca_m:.1} m at cost {fuel_cost_m_s:.2} m/s)"),
            ),
            multi => {
                let leg_list = format_leg_list(
                    &multi.iter().map(|o| o.leg_1_based).collect::<Vec<_>>(),
                );
                (leg_list, String::new())
            }
        };
        let _ = writeln!(
            out,
            "> **Analytical COLA solver did not reach the {target_m:.0} m target on \
             {leg_desc}**{per_leg_suffix}. Full-physics propagation typically reduces these \
             margins further \u{2014} run `validate` before relying on this burn.\n",
        );
    }
    let note = match context {
        ReportContext::Mission => {
            "> Post-COLA POCA distances are analytical (J2 STM). Full-physics propagation \
             typically reduces these margins \u{2014} run `validate` to confirm effectiveness.\n"
        }
        ReportContext::Validate => {
            "> Post-COLA POCA distances are analytical (J2 STM). See Safety Comparison below \
             for full-physics effectiveness.\n"
        }
    };
    let _ = writeln!(out, "{note}");
}

/// Write the secondary conjunctions table (COLA-induced close approaches on other legs).
fn write_secondary_conjunction_section(
    out: &mut String,
    violations: &[rpo_core::mission::SecondaryViolation],
) {
    let _ = writeln!(out, "## Secondary Conjunctions\n");
    if violations.is_empty() {
        let _ = writeln!(out, "No secondary conjunctions detected.\n");
        return;
    }
    let _ = writeln!(
        out,
        "| COLA Leg | Violated Leg | Distance (m) | Elapsed (s) | Position (RIC km) |",
    );
    let _ = writeln!(
        out,
        "|----------|-------------|-------------|-------------|-------------------|",
    );
    for sv in violations {
        let _ = writeln!(
            out,
            "| {} | {} | {:.1} | {:.1} | [{:.4}, {:.4}, {:.4}] |",
            sv.original_leg_index + 1,
            sv.violated_leg_index + 1,
            sv.poca.distance_km * KM_TO_M,
            sv.poca.elapsed_s,
            sv.poca.position_ric_km.x,
            sv.poca.position_ric_km.y,
            sv.poca.position_ric_km.z,
        );
    }
    let _ = writeln!(out);
}

/// Write the skipped-legs table (legs where COLA was attempted but failed).
fn write_cola_skipped_section(out: &mut String, skipped: &[rpo_core::mission::SkippedLeg]) {
    let _ = writeln!(out, "## COLA Skipped Legs\n");
    let _ = writeln!(out, "| Leg | Reason |");
    let _ = writeln!(out, "|-----|--------|");
    for s in skipped {
        let _ = writeln!(out, "| {} | {} |", s.leg_index + 1, s.error_message);
    }
    let _ = writeln!(out);
}

/// Write COLA validation detail (burns injected into Nyx propagation).
pub(crate) fn write_cola_validation_detail(
    out: &mut String,
    output: &PipelineOutput,
    report: &ValidationReport,
) {
    let maneuvers = match output.safety.cola.as_deref() {
        Some(m) if !m.is_empty() && report.cola_validated() => m,
        _ => return,
    };

    let has_effectiveness = !report.cola_effectiveness.is_empty();

    let _ = writeln!(out, "### COLA Burns Injected into Nyx Propagation\n");

    if has_effectiveness {
        let _ = writeln!(
            out,
            "| Leg | \u{0394}v (m/s) | Type | Analytical Post-COLA POCA | Nyx Post-Burn POCA | Threshold Met |",
        );
        let _ = writeln!(
            out,
            "| --- | -------- | ---- | ------------------------- | ------------------ | ------------- |",
        );
    } else {
        let _ = writeln!(
            out,
            "| Leg | \u{0394}v (m/s) | Type | Analytical Post-COLA POCA |",
        );
        let _ = writeln!(out, "| --- | -------- | ---- | ------------------------- |");
    }

    for m in maneuvers {
        let correction = correction_type_label(m.correction_type);

        if has_effectiveness {
            let eff = report
                .cola_effectiveness
                .iter()
                .find(|e| e.leg_index == m.leg_index);
            let nyx_min = eff.map_or_else(
                || "\u{2014}".to_string(),
                |e| format!("{:.1} m", e.nyx_post_cola_min_distance_km * KM_TO_M),
            );
            let threshold = eff
                .and_then(|e| e.threshold_met)
                .map_or("\u{2014}", |met| if met { "\u{2705}" } else { "\u{274c}" });

            let _ = writeln!(
                out,
                "| {} | {:.2} | {} | {:.1} m | {} | {} |",
                m.leg_index + 1,
                m.fuel_cost_km_s * KM_TO_M,
                correction,
                m.post_avoidance_poca_km * KM_TO_M,
                nyx_min,
                threshold,
            );
        } else {
            let _ = writeln!(
                out,
                "| {} | {:.2} | {} | {:.1} m |",
                m.leg_index + 1,
                m.fuel_cost_km_s * KM_TO_M,
                correction,
                m.post_avoidance_poca_km * KM_TO_M,
            );
        }
    }
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "> Nyx propagated both the pre-COLA baseline and the post-COLA trajectory; \
         the Safety Comparison table above shows both side-by-side.\n",
    );
}

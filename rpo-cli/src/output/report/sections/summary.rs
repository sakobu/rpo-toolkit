//! Summary block writer.

use std::fmt::Write;

use rpo_core::mission::{assess_safety, SafetyConfig, ValidationReport};
use rpo_core::pipeline::PipelineOutput;

use crate::output::fmt::{fmt_duration, fmt_m, fmt_m_s, KM_TO_M};
use crate::output::report::helpers::{status_emoji, write_cola_callout};
use crate::output::verdict::{analytical_overestimate, VerdictResult};

/// Write the Mission Summary header block.
pub(crate) fn write_summary_block_mission(
    out: &mut String,
    verdict_result: &VerdictResult,
    output: &PipelineOutput,
    config: &SafetyConfig,
    validation: Option<&ValidationReport>,
) {
    let _ = writeln!(out, "# Mission Summary\n");
    let _ = writeln!(
        out,
        "**Verdict: {}{}** ({}) | {} total \u{0394}v | {} duration | {} waypoints\n",
        verdict_result.verdict,
        verdict_result.qualifier,
        verdict_result.reason,
        fmt_m_s(output.total_dv_km_s, 1),
        fmt_duration(output.total_duration_s),
        output.mission.legs.len(),
    );

    // Safety summary table
    let safety = if let Some(report) = validation {
        Some(&report.numerical_safety)
    } else {
        output.mission.safety.as_ref()
    };

    if let Some(safety) = safety {
        let assessment = assess_safety(safety, config);
        let tier = if validation.is_some() {
            " (Nyx)"
        } else {
            " (analytical)"
        };
        let _ = writeln!(out, "| Metric | Value | Threshold | Status |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min 3D distance{tier} | {} | {} | {} |",
            fmt_m(safety.operational.min_distance_3d_km, 0),
            fmt_m(config.min_distance_3d_km, 0),
            status_emoji(assessment.distance_3d_pass),
        );
        let ei_status = if assessment.ei_separation_pass {
            status_emoji(true)
        } else if output.formation_design.is_some() {
            "\u{26a0}\u{fe0f} BELOW THRESHOLD"
        } else {
            status_emoji(false)
        };
        let ei_label = if output.formation_design.is_some() {
            "Min guided-trajectory e/i"
        } else {
            "Min e/i separation"
        };
        let _ = writeln!(
            out,
            "| {ei_label}{tier} | {} | {} | {} |",
            fmt_m(safety.passive.min_ei_separation_km, 0),
            fmt_m(config.min_ei_separation_km, 0),
            ei_status,
        );
        if let Some(report) = validation {
            let _ = writeln!(out, "| Convergence | 100% | \u{2014} | \u{2705} |",);
            let _ = writeln!(
                out,
                "| Max position error | {} | \u{2014} | \u{2014} |",
                fmt_m(report.max_position_error_km, 0),
            );
            // Analytical bias row: absolute delta + ratio (not a single percentage,
            // which readers routinely misparse as "analytical is 1.65× Nyx" when it
            // is actually 2.6× Nyx). Uses the same helper as `validation_insights`
            // so the summary row and the insight list stay in lockstep.
            if let Some(ref ana) = report.analytical_safety {
                let ana_3d = ana.operational.min_distance_3d_km;
                let num_3d = safety.operational.min_distance_3d_km;
                if let Some(ovr) = analytical_overestimate(ana_3d, num_3d) {
                    let _ = writeln!(
                        out,
                        "| Analytical bias | +{:.0} m optimistic (analytical {:.0} m vs \
                         Nyx {:.0} m; analytical \u{2248} {:.1}\u{00d7} Nyx) | \u{2014} | \
                         \u{26a0}\u{fe0f} |",
                        ovr.delta_m,
                        ana_3d * KM_TO_M,
                        num_3d * KM_TO_M,
                        ovr.ratio,
                    );
                }
            }
        }
        let _ = writeln!(out);
    }
    write_cola_callout(out, output);
    if output.auto_drag_config.is_some() {
        let _ = writeln!(
            out,
            "> Drag-aware targeting; \u{0394}v differs slightly from analytical-only (`mission`) results.\n",
        );
    }
}

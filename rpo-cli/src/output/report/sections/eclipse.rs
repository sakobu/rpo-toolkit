//! Eclipse section writers.

use std::fmt::Write;

use hifitime::Epoch;
use rpo_core::pipeline::PipelineOutput;

use crate::output::fmt::{fmt_duration, fmt_epoch_rounded};
use crate::output::thresholds::insight as insight_thresh;

/// Write the Eclipse section.
pub(crate) fn write_eclipse_section(out: &mut String, output: &PipelineOutput) {
    if let Some(ref eclipse) = output.mission.eclipse {
        let _ = writeln!(out, "## Eclipse\n");
        let _ = writeln!(out, "| Parameter | Value |");
        let _ = writeln!(out, "| --- | --- |");
        let _ = writeln!(
            out,
            "| Shadow intervals | {} |",
            eclipse.summary.intervals.len(),
        );
        let _ = writeln!(
            out,
            "| Total shadow time | {} ({:.1}% of waypoint phase) |",
            fmt_duration(eclipse.summary.total_shadow_duration_s),
            eclipse.summary.time_in_shadow_fraction * insight_thresh::PERCENT_PER_UNIT,
        );
        let _ = writeln!(
            out,
            "| Max single eclipse | {} |",
            fmt_duration(eclipse.summary.max_shadow_duration_s),
        );

        // Combined transfer + waypoint eclipse
        if let Some(ref te) = output.transfer_eclipse {
            let total_shadow =
                te.summary.total_shadow_duration_s + eclipse.summary.total_shadow_duration_s;
            let lambert_tof_s = output.transfer.as_ref().map_or(0.0, |t| t.tof_s);
            let total_duration = lambert_tof_s + output.mission.total_duration_s;
            if total_duration > 0.0 {
                let _ = writeln!(
                    out,
                    "| Combined (transfer + waypoint) | {} ({:.1}% of full mission) |",
                    fmt_duration(total_shadow),
                    total_shadow / total_duration * insight_thresh::PERCENT_PER_UNIT,
                );
            }
        }
        let _ = writeln!(out);

        // Burns-in-shadow check: cross-reference maneuver epochs with
        // eclipse intervals so an operator can identify thermal/attitude
        // concerns at a glance.
        write_burns_in_shadow(out, output);
    }
}

/// Write the burns-in-shadow sub-section.
///
/// Cross-references every burn in the consolidated mission timeline
/// (from [`super::super::helpers::collect_burns`]) against the eclipse
/// intervals and flags any burns that fire in umbra.
fn write_burns_in_shadow(out: &mut String, output: &PipelineOutput) {
    let intervals = output
        .mission
        .eclipse
        .as_ref()
        .map(|e| &e.summary.intervals[..])
        .unwrap_or_default();

    let burns = super::super::helpers::collect_burns(output);
    if burns.is_empty() {
        return;
    }

    let shadow_status = |epoch: Epoch| -> &'static str {
        for iv in intervals {
            if epoch >= iv.start && epoch <= iv.end {
                return "Umbra";
            }
        }
        "Sunlit"
    };

    let any_in_shadow = burns.iter().any(|b| shadow_status(b.epoch) == "Umbra");

    let _ = writeln!(out, "### Burns in Shadow\n");
    if !any_in_shadow {
        let _ = writeln!(out, "No burns fire during shadow intervals.\n");
        return;
    }

    let _ = writeln!(out, "| Burn | Epoch | Status |");
    let _ = writeln!(out, "| --- | --- | --- |");
    for burn in &burns {
        let status = shadow_status(burn.epoch);
        let status_cell = if status == "Umbra" {
            "\u{26a0}\u{fe0f} Umbra"
        } else {
            "Sunlit"
        };
        let _ = writeln!(
            out,
            "| {} | {} | {} |",
            burn.kind,
            fmt_epoch_rounded(burn.epoch),
            status_cell,
        );
    }
    let _ = writeln!(out);
}

/// Write the Eclipse Validation sub-section (analytical vs ANISE).
pub(crate) fn write_eclipse_validation(
    out: &mut String,
    ev: &rpo_core::mission::EclipseValidation,
    output: &PipelineOutput,
) {
    let _ = writeln!(out, "### Eclipse Validation (Analytical vs ANISE)\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Sun direction error | max {:.4}\u{00b0}, mean {:.4}\u{00b0} |",
        ev.max_sun_direction_error_rad.to_degrees(),
        ev.mean_sun_direction_error_rad.to_degrees(),
    );
    let _ = writeln!(
        out,
        "| Eclipse intervals | {} analytical, {} numerical, {} matched |",
        ev.analytical_interval_count, ev.numerical_interval_count, ev.matched_interval_count,
    );

    if !ev.interval_comparisons.is_empty() {
        let max_eclipse_s = output
            .mission
            .eclipse
            .as_ref()
            .map_or(0.0, |e| e.summary.max_shadow_duration_s);
        let _ = writeln!(
            out,
            "| Timing error | max {:.1} s, mean {:.1} s (entry/exit) |",
            ev.max_timing_error_s, ev.mean_timing_error_s,
        );
        if max_eclipse_s > 0.0 {
            let _ = writeln!(
                out,
                "| | {:.1}% of max eclipse; acceptable for mission planning |",
                ev.max_timing_error_s / max_eclipse_s * insight_thresh::PERCENT_PER_UNIT,
            );
        }
    }

    let _ = writeln!(out);
}

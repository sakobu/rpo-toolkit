//! Waypoint section writer.

use std::fmt::Write;

use rpo_core::pipeline::{PipelineInput, PipelineOutput};

use crate::output::fmt::{fmt_duration, fmt_velocity_target, KM_TO_M};

/// Write the Waypoint Targeting section.
pub(crate) fn write_waypoint_section(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
) {
    let _ = writeln!(
        out,
        "## Waypoint Targeting ({} legs)\n",
        output.mission.legs.len(),
    );

    let _ = writeln!(
        out,
        "| Leg | TOF | \u{0394}v1 (m/s) | \u{0394}v2 (m/s) | Total (m/s) | v_target | Label |",
    );
    let _ = writeln!(out, "| --- | --- | --- | --- | --- | --- | --- |");

    let mut total_dv = 0.0;
    for (i, leg) in output.mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        total_dv += leg.total_dv_km_s;
        let v_target = fmt_velocity_target(&leg.target_velocity_ric_km_s);
        let label = input
            .base.waypoints
            .get(i)
            .and_then(|wp| wp.label.as_deref())
            .unwrap_or("-");
        let _ = writeln!(
            out,
            "| {} | {} | {:.2} | {:.2} | {:.2} | {} | {} |",
            i + 1,
            fmt_duration(leg.tof_s),
            dv1 * KM_TO_M,
            dv2 * KM_TO_M,
            leg.total_dv_km_s * KM_TO_M,
            v_target,
            label,
        );
    }
    let _ = writeln!(
        out,
        "| **Total** | **{}** | | | **{:.2}** | | |",
        fmt_duration(output.mission.total_duration_s),
        total_dv * KM_TO_M,
    );
    let _ = writeln!(out);
}

//! Transfer section writer.

use std::fmt::Write;

use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::mission::MissionPhase;

use crate::output::fmt::{fmt_duration, fmt_m_s, fmt_roe_component};
use crate::output::thresholds::insight as insight_thresh;

/// Write the Transfer section including Lambert solution and perch ROE.
pub(crate) fn write_transfer_section(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
) {
    let _ = writeln!(out, "## Transfer\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");

    match &output.phase {
        MissionPhase::Proximity {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            let _ = writeln!(out, "| Classification | PROXIMITY |");
            let _ = writeln!(out, "| ECI separation | {separation_km:.1} km |");
            let _ = writeln!(out, "| \u{03b4}r/r (relative separation ratio) | {delta_r_over_r:.3e} |");
        }
        MissionPhase::FarField {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            let _ = writeln!(
                out,
                "| Classification | FAR-FIELD (Lambert transfer required) |",
            );
            let _ = writeln!(out, "| ECI separation | {separation_km:.1} km |");
            let _ = writeln!(out, "| \u{03b4}r/r (relative separation ratio) | {delta_r_over_r:.3e} |");
        }
    }
    let _ = writeln!(out);

    if let Some(ref lambert) = output.transfer {
        write_lambert_solution(out, lambert, input);
    }

    if let Some(ref te) = output.transfer_eclipse {
        let _ = writeln!(out, "### Transfer Eclipse\n");
        let _ = writeln!(out, "| Parameter | Value |");
        let _ = writeln!(out, "| --- | --- |");
        let _ = writeln!(
            out,
            "| Shadow intervals | {} |",
            te.summary.intervals.len(),
        );
        if te.summary.total_shadow_duration_s > 0.0 {
            let _ = writeln!(
                out,
                "| Shadow time | {} ({:.1}% of transfer) |",
                fmt_duration(te.summary.total_shadow_duration_s),
                te.summary.time_in_shadow_fraction * insight_thresh::PERCENT_PER_UNIT,
            );
        }
        let _ = writeln!(out);
    }

    write_perch_roe(out, &output.perch_roe, output.formation_design.is_some());
}

/// Write the Lambert solution sub-section.
pub(crate) fn write_lambert_solution(
    out: &mut String,
    lambert: &rpo_core::propagation::LambertTransfer,
    input: &PipelineInput,
) {
    let _ = writeln!(out, "### Lambert Solution\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Total \u{0394}v | {} |",
        fmt_m_s(lambert.total_dv_km_s, 1),
    );
    let _ = writeln!(
        out,
        "| Departure \u{0394}v | {} |",
        fmt_m_s(lambert.departure_dv_eci_km_s.norm(), 1),
    );
    let _ = writeln!(
        out,
        "| Arrival \u{0394}v | {} |",
        fmt_m_s(lambert.arrival_dv_eci_km_s.norm(), 1),
    );
    let _ = writeln!(out, "| TOF | {} |", fmt_duration(lambert.tof_s));
    let _ = writeln!(out, "| Direction | {} |", lambert.direction);
    if input.lambert_config.revolutions > 0 {
        let _ = writeln!(
            out,
            "| Revolutions | {} |",
            input.lambert_config.revolutions,
        );
    }
    let v_circ = (rpo_core::constants::MU_EARTH
        / lambert.departure_state.position_eci_km.norm())
    .sqrt();
    let _ = writeln!(
        out,
        "| \u{0394}v/v_circ | {:.1}% |",
        lambert.total_dv_km_s / v_circ * insight_thresh::PERCENT_PER_UNIT,
    );
    let _ = writeln!(out);
}

/// Render the perch ROE table.
///
/// `is_enriched` should be `true` when the caller knows the pipeline has
/// overwritten `output.perch_roe` with an enriched state (i.e.,
/// `output.formation_design.is_some()`), so the heading matches the
/// rendered values.
pub(crate) fn write_perch_roe(
    out: &mut String,
    roe: &rpo_core::types::QuasiNonsingularROE,
    is_enriched: bool,
) {
    let heading = if is_enriched {
        "### Enriched Perch ROE\n"
    } else {
        "### Perch ROE\n"
    };
    let _ = writeln!(out, "{heading}");
    let _ = writeln!(out, "| Element | Value | Description |");
    let _ = writeln!(out, "| --- | --- | --- |");
    let _ = writeln!(
        out,
        "| \u{03b4}a | {} | Relative SMA |",
        fmt_roe_component(roe.da),
    );
    let _ = writeln!(
        out,
        "| \u{03b4}\u{03bb} | {} | Relative mean longitude |",
        fmt_roe_component(roe.dlambda),
    );
    let _ = writeln!(
        out,
        "| \u{03b4}ex | {} | Relative e\u{00b7}cos \u{03c9} |",
        fmt_roe_component(roe.dex),
    );
    let _ = writeln!(
        out,
        "| \u{03b4}ey | {} | Relative e\u{00b7}sin \u{03c9} |",
        fmt_roe_component(roe.dey),
    );
    let _ = writeln!(
        out,
        "| \u{03b4}ix | {} | Relative inclination |",
        fmt_roe_component(roe.dix),
    );
    let _ = writeln!(
        out,
        "| \u{03b4}iy | {} | Relative RAAN\u{00b7}sin i |",
        fmt_roe_component(roe.diy),
    );
    let _ = writeln!(out);
}


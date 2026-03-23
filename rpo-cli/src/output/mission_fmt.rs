//! Mission phase + validation formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{MissionPhase, ValidationReport};
use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::TransferEclipseData;

use super::common::{
    determine_verdict, fmt_duration, fmt_m, fmt_m_s, print_derived_drag, print_header,
    print_per_leg_errors, print_roe, print_subheader, SafetyTier, Verdict,
};
use super::thresholds::fidelity;
use super::eclipse_fmt::{print_eclipse_summary, print_eclipse_validation};
use super::safety_fmt::{print_safety_analysis, print_safety_comparison, print_safety_summary};

/// Print the common Transfer + Waypoint Targeting output shared by mission/validate.
pub fn print_mission_human(
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
    title: &str,
    tier: SafetyTier,
) {
    print_header(title);

    print_transfer_human(output, input);

    // Perch ROE
    let chief_a = match &output.phase {
        MissionPhase::Proximity {
            chief_elements, ..
        }
        | MissionPhase::FarField {
            chief_elements, ..
        } => chief_elements.a_km,
    };
    println!();
    print_roe("Perch ROE", &output.perch_roe, chief_a);

    print_waypoint_table(output, input, propagator, auto_drag);

    // ── Safety ────────────────────────────────────────────────
    if let Some(ref safety) = output.mission.safety {
        let sc = input.config.safety.unwrap_or_default();
        match tier {
            SafetyTier::NyxValidated => print_subheader("Analytical Safety"),
            SafetyTier::Analytical => print_subheader("Safety"),
        }
        print_safety_analysis(safety, &sc);
    }

    // ── Eclipse ───────────────────────────────────────────────
    if let Some(ref eclipse) = output.mission.eclipse {
        print_subheader("Eclipse");
        print_eclipse_summary(eclipse);

        // Combined mission eclipse: transfer + waypoint phases
        if let Some(ref te) = output.transfer_eclipse {
            let total_shadow =
                te.summary.total_shadow_duration_s + eclipse.summary.total_shadow_duration_s;
            let lambert_tof_s = output.transfer.as_ref().map_or(0.0, |t| t.tof_s);
            let total_duration = lambert_tof_s + output.mission.total_duration_s;
            if total_duration > 0.0 {
                println!(
                    "  Combined (transfer + waypoint): {} ({:.1}% of full mission)",
                    fmt_duration(total_shadow),
                    total_shadow / total_duration * 100.0,
                );
            }
        }
    }
}

/// Print the Transfer section: classification, Lambert details, transfer eclipse.
fn print_transfer_human(output: &PipelineOutput, input: &PipelineInput) {
    print_subheader("Transfer");
    match &output.phase {
        MissionPhase::Proximity {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            println!(
                "  Classification: {}",
                "PROXIMITY".if_supports_color(Stream::Stdout, |v| v.green())
            );
            println!("  ECI separation:  {separation_km:.1} km");
            println!("  \u{03b4}r/r:            {delta_r_over_r:.3e}");
        }
        MissionPhase::FarField {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            println!(
                "  Classification: {} (Lambert transfer required)",
                "FAR-FIELD".if_supports_color(Stream::Stdout, |v| v.yellow())
            );
            println!("  ECI separation:  {separation_km:.1} km");
            println!("  \u{03b4}r/r:            {delta_r_over_r:.3e}");
        }
    }

    if let Some(ref lambert) = output.transfer {
        println!("\n  Lambert:");
        println!("    Total \u{0394}v:     {}", fmt_m_s(lambert.total_dv_km_s, 1));
        println!(
            "    Departure:    {}",
            fmt_m_s(lambert.departure_dv_eci_km_s.norm(), 1)
        );
        println!(
            "    Arrival:      {}",
            fmt_m_s(lambert.arrival_dv_eci_km_s.norm(), 1)
        );
        println!(
            "    TOF:          {}",
            fmt_duration(lambert.tof_s)
        );
        println!(
            "    Direction:    {}",
            lambert.direction
        );
        if input.lambert_config.revolutions > 0 {
            println!("    Revolutions:  {}", input.lambert_config.revolutions);
        }
        let v_circ = (rpo_core::constants::MU_EARTH
            / lambert.departure_state.position_eci_km.norm())
        .sqrt();
        println!(
            "    \u{0394}v/v_circ:    {:.1}%",
            lambert.total_dv_km_s / v_circ * 100.0
        );
    }

    if let Some(ref te) = output.transfer_eclipse {
        print_transfer_eclipse(te);
    }
}

/// Print the Waypoint Targeting table.
fn print_waypoint_table(
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
) {
    let wp_title = format!(
        "Waypoint Targeting ({} legs)",
        output.mission.legs.len()
    );
    print_subheader(&wp_title);
    let prop_label = match propagator {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => "J2+Drag STM (auto-derived)",
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    };
    println!("  Propagator: {prop_label}");
    println!(
        "  {:>4}  {:>10}  {:>10}  {:>10}  {:>11}  Label",
        "Leg", "TOF", "\u{0394}v1 (m/s)", "\u{0394}v2 (m/s)", "Total (m/s)"
    );
    println!(
        "  {:-<4}  {:-<10}  {:-<10}  {:-<10}  {:-<11}  {:-<30}",
        "", "", "", "", "", ""
    );
    let mut total_dv = 0.0;
    for (i, leg) in output.mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        total_dv += leg.total_dv_km_s;
        let label = input
            .waypoints
            .get(i)
            .and_then(|wp| wp.label.as_deref())
            .unwrap_or("-");
        println!(
            "  {:>4}  {:>10}  {:>10.2}  {:>10.2}  {:>11.2}  {}",
            i + 1,
            fmt_duration(leg.tof_s),
            dv1 * 1000.0,
            dv2 * 1000.0,
            leg.total_dv_km_s * 1000.0,
            label,
        );
    }
    println!(
        "  {:-<4}  {:-<10}  {:-<10}  {:-<10}  {:-<11}",
        "", "", "", "", ""
    );
    println!(
        "  {:>4}  {:>10}  {:>10}  {:>10}  {:>11.2}",
        "Total",
        fmt_duration(output.mission.total_duration_s),
        "",
        "",
        total_dv * 1000.0,
    );
}

/// Print the mission verdict with Δv breakdown, safety summary, and model confidence.
pub fn print_mission_verdict(
    output: &PipelineOutput,
    input: &PipelineInput,
    validation: Option<&ValidationReport>,
) {
    let lambert_dv_km_s = output
        .transfer
        .as_ref()
        .map_or(0.0, |t| t.total_dv_km_s);
    let waypoint_dv_km_s = output.mission.total_dv_km_s;
    let total_dv_km_s = output.total_dv_km_s;
    let lambert_tof_s = output.transfer.as_ref().map_or(0.0, |t| t.tof_s);
    let total_duration_s = output.total_duration_s;

    print_header("Summary");

    let sc = input.config.safety.unwrap_or_default();
    let vr = determine_verdict(output, &sc, validation);
    let verdict_line = format!("{} ({})", vr.verdict, vr.reason);
    if vr.verdict == Verdict::Feasible {
        println!(
            "  Verdict:         {}",
            verdict_line.if_supports_color(Stream::Stdout, |v| v.green())
        );
    } else {
        println!(
            "  Verdict:         {}",
            verdict_line.if_supports_color(Stream::Stdout, |v| v.yellow())
        );
    }

    println!("\n  \u{0394}v Budget:");
    if total_dv_km_s > 0.0 {
        let lambert_pct = lambert_dv_km_s / total_dv_km_s * 100.0;
        let wp_pct = waypoint_dv_km_s / total_dv_km_s * 100.0;
        if lambert_pct > fidelity::FAR_FIELD_DV_DRIVER_PCT {
            println!(
                "    Transfer:        {}  ({lambert_pct:.1}%{})",
                fmt_m_s(lambert_dv_km_s, 1),
                " -- far-field driver".if_supports_color(Stream::Stdout, |v| v.red()),
            );
        } else {
            println!(
                "    Transfer:        {}  ({lambert_pct:.1}%)",
                fmt_m_s(lambert_dv_km_s, 1),
            );
        }
        println!(
            "    Targeting:       {}  ({wp_pct:.1}%)",
            fmt_m_s(waypoint_dv_km_s, 1),
        );
    } else {
        println!("    Transfer:        {}", fmt_m_s(lambert_dv_km_s, 1));
        println!("    Targeting:       {}", fmt_m_s(waypoint_dv_km_s, 1));
    }
    println!(
        "    Total:           {}",
        fmt_m_s(total_dv_km_s, 1)
            .if_supports_color(Stream::Stdout, |v| v.green()),
    );
    if output.auto_drag_config.is_some() {
        println!("    (drag-aware targeting; Δv differs slightly from analytical-only)");
    }

    println!(
        "\n  Duration:          {} ({} transfer + {} proximity)",
        fmt_duration(total_duration_s),
        fmt_duration(lambert_tof_s),
        fmt_duration(output.mission.total_duration_s),
    );

    if let Some(report) = validation {
        print_safety_summary("Nyx", &report.numerical_safety, &sc);
    } else if let Some(ref safety) = output.mission.safety {
        print_safety_summary("analytical", safety, &sc);
    }

    // Closest approach quick-scan line
    let safety_source = validation
        .map(|r| &r.numerical_safety)
        .or(output.mission.safety.as_ref());
    if let Some(safety) = safety_source {
        let source = if validation.is_some() { "Nyx" } else { "analytical" };
        println!(
            "  Closest approach:  leg {} at {} ({source})",
            safety.operational.min_3d_leg_index + 1,
            fmt_duration(safety.operational.min_3d_elapsed_s),
        );
    }

    print_fidelity(input, validation);
}

/// Print fidelity assessment based on validation data or propagator choice.
fn print_fidelity(input: &PipelineInput, validation: Option<&ValidationReport>) {
    if let Some(report) = validation {
        println!("\n  Fidelity:");
        println!(
            "    Position error: max {}, mean {}",
            fmt_m(report.max_position_error_km, 0),
            fmt_m(report.mean_position_error_km, 0),
        );
        if report.max_position_error_km < fidelity::CLOSE_PROXIMITY_KM {
            println!("    Suitable for:   mission planning, safety screening, close-proximity operations");
        } else if report.max_position_error_km < fidelity::SAFETY_SCREENING_KM {
            println!(
                "    Suitable for:   mission planning, safety screening"
            );
        } else {
            println!("    Suitable for:   preliminary mission planning");
        }
        println!("    Not suitable for: operational truth analysis");
    } else {
        let prop_label = match &input.propagator {
            rpo_core::pipeline::PropagatorChoice::J2Drag { .. } => "J2+Drag STM",
            rpo_core::pipeline::PropagatorChoice::J2 => "J2 STM",
        };
        println!("\n  Fidelity:          analytical only ({prop_label})");
    }
}

/// Print validation details (position errors, spacecraft configs, safety comparison).
pub fn print_validation_details(
    output: &PipelineOutput,
    input: &PipelineInput,
    report: &ValidationReport,
    samples_per_leg: u32,
    derived_drag: Option<&DragConfig>,
) {
    let val_title = format!("Nyx Validation ({samples_per_leg} samples/leg)");
    print_subheader(&val_title);

    // 1. Position error summary
    println!("  Position error (analytical vs Nyx full-physics):");
    println!("    Max:  {}", fmt_m(report.max_position_error_km, 1));
    println!("    Mean: {}", fmt_m(report.mean_position_error_km, 1));
    println!("    RMS:  {}", fmt_m(report.rms_position_error_km, 1));
    println!(
        "    Max velocity error: {}",
        fmt_m_s(report.max_velocity_error_km_s, 3),
    );

    // 2. Numerical Safety (Nyx) with PASS/FAIL
    let sc = input.config.safety.unwrap_or_default();
    println!("\n  Numerical Safety (Nyx):");
    print_safety_analysis(&report.numerical_safety, &sc);

    // 3. Safety Comparison (Analytical vs Numerical)
    print_safety_comparison(report, &input.config);
    println!(
        "  (analytical: {} steps/leg; numerical: {} Nyx samples/leg)",
        input.config.targeting.trajectory_steps, samples_per_leg,
    );

    // 4. Eclipse Validation
    if let Some(ref ev) = report.eclipse_validation {
        let max_eclipse_s = output
            .mission
            .eclipse
            .as_ref()
            .map_or(0.0, |e| e.summary.max_shadow_duration_s);
        print_eclipse_validation(ev, max_eclipse_s);
    }

    // 5. Per-leg position error
    print_per_leg_errors(&report.leg_points);

    // 6. Spacecraft
    println!("\n  Spacecraft:");
    println!(
        "    Chief:  {:.0} kg, drag area {:.2} m\u{00b2}, Cd {:.1}",
        report.chief_config.dry_mass_kg,
        report.chief_config.drag_area_m2,
        report.chief_config.coeff_drag,
    );
    println!(
        "    Deputy: {:.0} kg, drag area {:.2} m\u{00b2}, Cd {:.1}",
        report.deputy_config.dry_mass_kg,
        report.deputy_config.drag_area_m2,
        report.deputy_config.coeff_drag,
    );

    // 7. Auto-derived drag
    if let Some(drag) = derived_drag {
        println!();
        print_derived_drag(drag, "  ");
    }
}

/// Print transfer eclipse info.
fn print_transfer_eclipse(te: &TransferEclipseData) {
    println!("\n  Transfer Eclipse:");
    println!(
        "    Shadow intervals: {}",
        te.summary.intervals.len()
    );
    if te.summary.total_shadow_duration_s > 0.0 {
        println!(
            "    Shadow time:      {} ({:.1}% of transfer)",
            fmt_duration(te.summary.total_shadow_duration_s),
            te.summary.time_in_shadow_fraction * 100.0,
        );
    } else {
        println!("    No shadow during transfer");
    }
}

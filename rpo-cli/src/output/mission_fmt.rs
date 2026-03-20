//! Mission phase + validation formatting.

use owo_colors::{OwoColorize, Stream};
use rpo_core::mission::{MissionPhase, ValidationReport};
use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::TransferEclipseData;

use super::common::print_roe;
use super::eclipse_fmt::{print_eclipse_summary, print_eclipse_validation};
use super::safety_fmt::{print_safety_analysis, print_safety_comparison, print_safety_summary};

use crate::output::common::{print_derived_drag, print_per_leg_errors};

/// Position error threshold (km) below which model is suitable for close-proximity.
const FIDELITY_CLOSE_PROXIMITY_KM: f64 = 0.05;

/// Position error threshold (km) below which model is suitable for safety screening.
const FIDELITY_SAFETY_SCREENING_KM: f64 = 0.2;

/// Δv percentage threshold above which Lambert transfer is the dominant cost.
const FAR_FIELD_DV_DRIVER_PCT: f64 = 90.0;

/// Print the common Phase 1 + Phase 2 output shared by mission/validate handlers.
#[allow(clippy::too_many_lines)]
pub fn print_mission_human(
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
) {
    println!("End-to-End Mission");
    println!("==================\n");

    // Phase 1: Classification & Transfer
    println!("Phase 1: Classification & Transfer");
    println!("-----------------------------------");
    match &output.phase {
        MissionPhase::Proximity {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            println!("  Classification: PROXIMITY");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
        MissionPhase::FarField {
            separation_km,
            delta_r_over_r,
            ..
        } => {
            println!("  Classification: FAR-FIELD (Lambert transfer required)");
            println!("  ECI separation:  {separation_km:.4} km");
            println!("  δr/r:            {delta_r_over_r:.6e}");
        }
    }

    if let Some(ref lambert) = output.transfer {
        println!("\n  Lambert Transfer:");
        println!("    Total Δv:     {:.4} km/s", lambert.total_dv_km_s);
        println!(
            "    Departure Δv: {:.4} km/s",
            lambert.departure_dv_eci_km_s.norm()
        );
        println!(
            "    Arrival Δv:   {:.4} km/s",
            lambert.arrival_dv_eci_km_s.norm()
        );
        println!(
            "    TOF:          {:.1} s ({:.2} min)",
            lambert.tof_s,
            lambert.tof_s / 60.0
        );
        println!("    Direction:    {:?}", lambert.direction);
        if input.lambert_config.revolutions > 0 {
            println!("    Revolutions:  {}", input.lambert_config.revolutions);
        }
        let v_circ = (rpo_core::constants::MU_EARTH
            / lambert.departure_state.position_eci_km.norm())
        .sqrt();
        println!(
            "    Δv/v_circ:    {:.1}%",
            lambert.total_dv_km_s / v_circ * 100.0
        );
    }

    if let Some(ref te) = output.transfer_eclipse {
        print_transfer_eclipse(te);
    }

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

    // Phase 2: Waypoint Targeting
    println!(
        "\n\nPhase 2: Waypoint Targeting ({} legs)",
        output.mission.legs.len()
    );
    println!("-----------------------------------");
    let prop_label = match propagator {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => "J2+Drag STM (auto-derived)",
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    };
    println!("  Propagator: {prop_label}");
    println!(
        "  {:>4}  {:>10}  {:>12}  {:>12}  {:>12}  Profile",
        "Leg", "TOF (s)", "Δv1 (km/s)", "Δv2 (km/s)", "Total (km/s)"
    );
    println!(
        "  {:-<4}  {:-<10}  {:-<12}  {:-<12}  {:-<12}  {:-<30}",
        "", "", "", "", "", ""
    );
    for (i, leg) in output.mission.legs.iter().enumerate() {
        let dv1 = leg.departure_maneuver.dv_ric_km_s.norm();
        let dv2 = leg.arrival_maneuver.dv_ric_km_s.norm();
        let label = input.waypoints.get(i).and_then(|wp| wp.label.as_deref()).unwrap_or("-");
        println!(
            "  {:>4}  {:>10.1}  {:>12.6}  {:>12.6}  {:>12.6}  {}",
            i + 1,
            leg.tof_s,
            dv1,
            dv2,
            leg.total_dv_km_s,
            label,
        );
    }

    // Safety
    if let Some(ref safety) = output.mission.safety {
        let sc = input.config.safety.unwrap_or_default();
        print_safety_analysis(safety, &sc);
    }

    if let Some(ref eclipse) = output.mission.eclipse {
        print_eclipse_summary(eclipse);

        // Combined mission eclipse: transfer + waypoint phases
        if let Some(ref te) = output.transfer_eclipse {
            let total_shadow =
                te.summary.total_shadow_duration_s + eclipse.summary.total_shadow_duration_s;
            let lambert_tof_s = output.transfer.as_ref().map_or(0.0, |t| t.tof_s);
            let total_duration = lambert_tof_s + output.mission.total_duration_s;
            if total_duration > 0.0 {
                println!(
                    "  Combined (transfer + waypoint): {:.0} s ({:.1}% of full mission)",
                    total_shadow,
                    total_shadow / total_duration * 100.0,
                );
            }
        }
    }
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

    println!("\nMission Summary");
    println!("===============");

    let sc = input.config.safety.unwrap_or_default();
    if let Some(report) = validation {
        let num_assessment =
            rpo_core::mission::assess_safety(&report.numerical_safety, &sc);
        if num_assessment.overall_pass {
            println!("  Verdict:               {}", "FEASIBLE (safety metrics remain above threshold under nyx full-physics)".if_supports_color(Stream::Stdout, |v| v.green()));
        } else {
            println!("  Verdict:               {}", "CAUTION (numerical safety below threshold)".if_supports_color(Stream::Stdout, |v| v.yellow()));
        }
    } else if let Some(ref safety) = output.mission.safety {
        let ana_assessment = rpo_core::mission::assess_safety(safety, &sc);
        if ana_assessment.overall_pass {
            println!("  Verdict:               {}", "FEASIBLE (analytical safety above threshold)".if_supports_color(Stream::Stdout, |v| v.green()));
        } else {
            println!("  Verdict:               {}", "CAUTION (analytical safety below threshold)".if_supports_color(Stream::Stdout, |v| v.yellow()));
        }
    }

    println!("\n  Δv Budget:");
    if total_dv_km_s > 0.0 {
        let lambert_pct = lambert_dv_km_s / total_dv_km_s * 100.0;
        let wp_pct = waypoint_dv_km_s / total_dv_km_s * 100.0;
        if lambert_pct > FAR_FIELD_DV_DRIVER_PCT {
            println!(
                "    Lambert transfer:    {lambert_dv_km_s:.4} km/s  ({lambert_pct:.1}%{})",
                " -- far-field driver".if_supports_color(Stream::Stdout, |v| v.red()),
            );
        } else {
            println!(
                "    Lambert transfer:    {lambert_dv_km_s:.4} km/s  ({lambert_pct:.1}%)",
            );
        }
        println!(
            "    Waypoint targeting:  {waypoint_dv_km_s:.4} km/s  ({wp_pct:.1}%)",
        );
    } else {
        println!("    Lambert transfer:    {lambert_dv_km_s:.4} km/s");
        println!("    Waypoint targeting:  {waypoint_dv_km_s:.6} km/s");
    }
    println!(
        "    Total:               {}",
        format!("{total_dv_km_s:.4} km/s").if_supports_color(Stream::Stdout, |v| v.green()),
    );

    println!(
        "\n  Duration:              {:.1} min ({:.1} min transfer + {:.1} min proximity)",
        total_duration_s / 60.0,
        lambert_tof_s / 60.0,
        output.mission.total_duration_s / 60.0,
    );

    if let Some(report) = validation {
        print_safety_summary("nyx", &report.numerical_safety, &sc);
    } else if let Some(ref safety) = output.mission.safety {
        print_safety_summary("analytical", safety, &sc);
    }

    if let Some(report) = validation {
        println!("\n  Model Fidelity:");
        println!(
            "    Position error: max {:.3} km, mean {:.3} km",
            report.max_position_error_km, report.mean_position_error_km,
        );
        if report.max_position_error_km < FIDELITY_CLOSE_PROXIMITY_KM {
            println!("    Suitable for:  interactive planning, preliminary safety screening, close proximity");
        } else if report.max_position_error_km < FIDELITY_SAFETY_SCREENING_KM {
            println!(
                "    Suitable for:  interactive planning, preliminary safety screening"
            );
        } else {
            println!("    Suitable for:  preliminary mission planning only");
        }
        println!("    Not for:       high-confidence operational truth analysis");
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
    println!(
        "\n\nPhase 3: Nyx Validation ({samples_per_leg} samples/leg)"
    );
    println!("-----------------------------------");
    println!("  Position error (analytical vs nyx full-physics):");
    println!("    Max:  {:.6} km", report.max_position_error_km);
    println!("    Mean: {:.6} km", report.mean_position_error_km);
    println!("    RMS:  {:.6} km", report.rms_position_error_km);
    println!(
        "    Max velocity error: {:.6e} km/s",
        report.max_velocity_error_km_s
    );

    print_per_leg_errors(&report.leg_points);

    println!("\n  Spacecraft configs:");
    println!(
        "    Chief:  {:.0} kg, drag area {:.2} m², Cd {:.1}",
        report.chief_config.dry_mass_kg,
        report.chief_config.drag_area_m2,
        report.chief_config.coeff_drag,
    );
    println!(
        "    Deputy: {:.0} kg, drag area {:.2} m², Cd {:.1}",
        report.deputy_config.dry_mass_kg,
        report.deputy_config.drag_area_m2,
        report.deputy_config.coeff_drag,
    );

    print_safety_comparison(report, &input.config);
    println!(
        "  (analytical: {} steps/leg; numerical: {} nyx samples/leg)",
        input.config.targeting.trajectory_steps, samples_per_leg,
    );

    if let Some(ref ev) = report.eclipse_validation {
        let max_eclipse_s = output
            .mission
            .eclipse
            .as_ref()
            .map_or(0.0, |e| e.summary.max_shadow_duration_s);
        print_eclipse_validation(ev, max_eclipse_s);
    }

    if let Some(drag) = derived_drag {
        print_derived_drag(drag);
    }
}

/// Print transfer eclipse info.
fn print_transfer_eclipse(te: &TransferEclipseData) {
    println!("\n  Transfer Eclipse:");
    println!(
        "    Deputy shadow intervals: {}",
        te.summary.intervals.len()
    );
    if te.summary.total_shadow_duration_s > 0.0 {
        println!(
            "    Deputy shadow time: {:.0} s ({:.1}% of transfer)",
            te.summary.total_shadow_duration_s,
            te.summary.time_in_shadow_fraction * 100.0,
        );
    } else {
        println!("    No shadow during transfer");
    }
}

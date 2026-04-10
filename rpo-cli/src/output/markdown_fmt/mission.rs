use std::fmt::Write;

use rpo_core::mission::{
    assess_safety, FreeDriftAnalysis, MissionPhase, RcContext, SafetyAssessment, SafetyConfig,
    SafetyMetrics, ValidationReport,
};
use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::propagation::{DragConfig, PropagationModel};

use crate::output::common::{
    analytical_overestimate, determine_verdict, fmt_bounded_motion_residual, fmt_duration, fmt_m,
    fmt_m_s, fmt_roe_component, fmt_velocity_target, margin_or_shortfall_row, margin_ratio,
    KM_TO_M, SafetyTier, VerdictResult,
};
use crate::output::formation_fmt::write_formation_design_md;
use crate::output::insights;

use super::helpers::{
    propagator_label, status_emoji, write_cola_callout, write_drag_table, write_insights,
    ReportContext,
};
use crate::output::thresholds::{insight as insight_thresh, safety as safety_thresh};

/// How to render the overall passive safety result.
///
/// Determined from the verdict + enrichment state before entering the safety
/// section. Replaces two interdependent boolean flags (`enrichment_active`,
/// `verdict_feasible`) with an explicit enum per CLAUDE.md "enums over boolean
/// flags" rule.
#[derive(Debug, Clone, Copy)]
enum PassiveSafetyOutcome {
    /// All safety checks pass (e/i included).
    Pass,
    /// 3D distance governs; e/i is advisory (enrichment active, verdict feasible).
    AdvisoryEi,
    /// Safety checks fail.
    Fail,
}

/// Render a list of 1-based leg numbers with correct grammar and Oxford
/// commas. Single-element variant is singular (`"leg 3"`); multi-element
/// variants are plural (`"legs 2 and 3"`, `"legs 1, 2, and 3"`).
///
/// Delegates the Oxford-comma join to
/// [`crate::output::common::oxford_join`] so this renderer stays in sync
/// with the verdict-reason leg enumeration in `common.rs`.
fn format_leg_list(legs: &[usize]) -> String {
    match legs {
        [] => String::new(),
        [single] => format!("leg {single}"),
        multi => {
            let numbers: Vec<String> = multi.iter().map(ToString::to_string).collect();
            format!("legs {}", crate::output::common::oxford_join(&numbers))
        }
    }
}

/// Determine the passive safety rendering outcome from the assessment and context.
fn passive_safety_outcome(
    assessment: &SafetyAssessment,
    enrichment_active: bool,
    vr: &VerdictResult,
) -> PassiveSafetyOutcome {
    if assessment.overall_pass {
        PassiveSafetyOutcome::Pass
    } else if enrichment_active && vr.verdict.is_feasible() {
        PassiveSafetyOutcome::AdvisoryEi
    } else {
        PassiveSafetyOutcome::Fail
    }
}

/// Generate a complete markdown report for the `mission` command.
#[must_use]
pub fn mission_to_markdown(
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
) -> String {
    let mut out = String::with_capacity(4096);

    let sc = input.config.safety.unwrap_or_default();
    let enrichment_active = output.formation_design.is_some();

    // Summary block at top
    let vr = determine_verdict(output, &sc, None);
    write_summary_block_mission(&mut out, &vr, output, &sc, None);

    write_transfer_section(&mut out, output, input);

    write_waypoint_section(&mut out, output, input, propagator, auto_drag);

    write_maneuver_schedule(&mut out, output);

    if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, &sc);
        let outcome = passive_safety_outcome(&assessment, enrichment_active, &vr);
        write_safety_section(&mut out, safety, &sc, SafetyTier::Governing, outcome);
    }

    if let Some(ref fd) = output.formation_design {
        write_formation_design_md(&mut out, fd);
    }

    if let Some(ref poca) = output.safety.poca {
        write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }

    if let Some(ref fd) = output.safety.free_drift {
        write_free_drift_section(&mut out, fd, &sc);
    }

    if let Some(ref fd_poca) = output.safety.free_drift_poca {
        write_poca_section(
            &mut out,
            "Free-Drift Closest Approach (Brent-refined)",
            fd_poca,
        );
    }

    write_cola_sections(&mut out, output, input, ReportContext::Mission);

    write_eclipse_section(&mut out, output);

    out
}

/// Validation-specific parameters for markdown formatting.
pub struct ValidationContext<'a> {
    /// Propagation model used for analytical targeting.
    pub propagator: &'a PropagationModel,
    /// Whether drag was auto-derived from spacecraft properties.
    pub auto_drag: bool,
    /// Number of nyx sample points per leg.
    pub samples_per_leg: u32,
    /// Auto-derived drag config, if any.
    pub derived_drag: Option<&'a DragConfig>,
}

/// Generate a complete markdown report for the `validate` command.
#[must_use]
pub fn validation_to_markdown(
    output: &PipelineOutput,
    input: &PipelineInput,
    report: &ValidationReport,
    ctx: &ValidationContext<'_>,
) -> String {
    let mut out = String::with_capacity(8192);

    let sc = input.config.safety.unwrap_or_default();
    let enrichment_active = output.formation_design.is_some();

    let vr = determine_verdict(output, &sc, Some(report));
    write_summary_block_mission(&mut out, &vr, output, &sc, Some(report));

    let prop_label = propagator_label(ctx.propagator, ctx.auto_drag);
    let _ = writeln!(
        out,
        "> \u{0394}v values below reflect the analytical targeting plan ({prop_label}). \
         Nyx full-physics validation governs safety margins \u{2014} \
         it does not recompute maneuvers.\n",
    );

    write_transfer_section(&mut out, output, input);

    write_waypoint_section(&mut out, output, input, ctx.propagator, ctx.auto_drag);

    write_maneuver_schedule(&mut out, output);

    if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, &sc);
        let outcome = passive_safety_outcome(&assessment, enrichment_active, &vr);
        write_safety_section(&mut out, safety, &sc, SafetyTier::Baseline, outcome);
    }

    if let Some(ref fd) = output.formation_design {
        write_formation_design_md(&mut out, fd);
    }

    if let Some(ref poca) = output.safety.poca {
        write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }

    if let Some(ref fd) = output.safety.free_drift {
        write_free_drift_section(&mut out, fd, &sc);
    }

    if let Some(ref fd_poca) = output.safety.free_drift_poca {
        write_poca_section(
            &mut out,
            "Free-Drift Closest Approach (Brent-refined)",
            fd_poca,
        );
    }

    write_cola_sections(&mut out, output, input, ReportContext::Validate);

    write_eclipse_section(&mut out, output);
    write_validation_section(
        &mut out,
        output,
        input,
        report,
        ctx.samples_per_leg,
        ctx.derived_drag,
    );

    // Insights
    let mut insight_lines = insights::validation_insights(report, &sc);
    if let (Some(cola), Some(cola_config)) = (&output.safety.cola, &input.cola) {
        insight_lines.extend(insights::cola_analytical_miss_insights(
            cola,
            cola_config.target_distance_km,
        ));
    }
    write_insights(&mut out, &insight_lines);

    out
}

// ── Private helpers ──────────────────────────────────────────────

// ── Maneuver schedule ───────────────────────────────────────────

/// Maneuver type for the consolidated schedule table.
enum ScheduleEntryKind {
    /// Lambert transfer departure impulse (ECI frame).
    LambertDeparture,
    /// Lambert transfer arrival impulse (ECI frame).
    LambertArrival,
    /// Waypoint-leg departure impulse (RIC frame).
    WaypointDeparture { leg: usize },
    /// Waypoint-leg arrival impulse (RIC frame).
    WaypointArrival { leg: usize },
    /// Collision avoidance maneuver (RIC frame).
    Cola { leg: usize },
}

impl std::fmt::Display for ScheduleEntryKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LambertDeparture => write!(f, "Lambert dep"),
            Self::LambertArrival => write!(f, "Lambert arr"),
            Self::WaypointDeparture { leg } => write!(f, "WP{} dep", leg + 1),
            Self::WaypointArrival { leg } => write!(f, "WP{} arr", leg + 1),
            Self::Cola { leg } => write!(f, "COLA (L{})", leg + 1),
        }
    }
}

/// A single row in the consolidated maneuver schedule table.
struct ScheduleEntry {
    /// Burn epoch (UTC).
    epoch: hifitime::Epoch,
    /// Maneuver classification.
    kind: ScheduleEntryKind,
    /// RIC Dv components (`None` for Lambert burns which are ECI-only).
    dv_ric_km_s: Option<nalgebra::Vector3<f64>>,
    /// Total Dv magnitude (km/s).
    dv_mag_km_s: f64,
}

/// Which report the maneuver schedule is being rendered into.
///
/// Controls both the section heading and whether COLA rows carry a
/// reference-marker suffix. Encoded as an enum so the mission / MC
/// dispatch is explicit and cannot be accidentally mixed (e.g. mission
/// heading with MC marker, or vice versa).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(super) enum ScheduleVariant {
    /// Standard mission.md schedule. No COLA-row marker.
    Mission,
    /// mc.md schedule. COLA rows carry a `†` suffix because Monte Carlo
    /// samples propagate the pre-COLA baseline — the burn is shown for
    /// reference only, not executed in the ensemble.
    MonteCarlo,
}

impl ScheduleVariant {
    /// Full section heading line, including the leading `##`.
    fn section_title(self) -> &'static str {
        match self {
            Self::Mission => "## Maneuver Schedule",
            Self::MonteCarlo => "## Nominal Maneuver Schedule (reference)",
        }
    }

    /// Optional suffix appended to COLA rows' Type cell. `None` means
    /// no marker; callers match the presence of the marker to the
    /// footnote they emit beneath the table.
    fn cola_marker(self) -> Option<&'static str> {
        match self {
            Self::Mission => None,
            Self::MonteCarlo => Some("\u{2020}"),
        }
    }
}

/// Write a consolidated chronological maneuver schedule table in the
/// default mission-report variant. Thin wrapper around
/// [`write_maneuver_schedule_with`].
pub(super) fn write_maneuver_schedule(out: &mut String, output: &PipelineOutput) {
    write_maneuver_schedule_with(out, output, ScheduleVariant::Mission);
}

/// Write a consolidated chronological maneuver schedule table.
///
/// Collects Lambert, waypoint, and COLA burns from the pipeline output,
/// sorts by epoch, and renders a single table.
///
/// # Arguments
/// - `out` — output buffer; the rendered table is appended.
/// - `output` — pipeline output carrying `transfer`, `mission.legs`,
///   and `safety.cola` maneuvers.
/// - `variant` — mission vs MC dispatch (see [`ScheduleVariant`]).
pub(super) fn write_maneuver_schedule_with(
    out: &mut String,
    output: &PipelineOutput,
    variant: ScheduleVariant,
) {
    let cola_count = output.safety.cola.as_ref().map_or(0, Vec::len);
    let capacity = output.mission.legs.len() * 2
        + if output.transfer.is_some() { 2 } else { 0 }
        + cola_count;
    let mut entries: Vec<ScheduleEntry> = Vec::with_capacity(capacity);

    // Lambert burns (ECI Dv — no RIC decomposition)
    if let Some(ref transfer) = output.transfer {
        entries.push(ScheduleEntry {
            epoch: transfer.departure_state.epoch,
            kind: ScheduleEntryKind::LambertDeparture,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.departure_dv_eci_km_s.norm(),
        });
        entries.push(ScheduleEntry {
            epoch: transfer.arrival_state.epoch,
            kind: ScheduleEntryKind::LambertArrival,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.arrival_dv_eci_km_s.norm(),
        });
    }

    // Waypoint departure / arrival burns (RIC Dv)
    for (i, leg) in output.mission.legs.iter().enumerate() {
        entries.push(ScheduleEntry {
            epoch: leg.departure_maneuver.epoch,
            kind: ScheduleEntryKind::WaypointDeparture { leg: i },
            dv_ric_km_s: Some(leg.departure_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.departure_maneuver.dv_ric_km_s.norm(),
        });
        entries.push(ScheduleEntry {
            epoch: leg.arrival_maneuver.epoch,
            kind: ScheduleEntryKind::WaypointArrival { leg: i },
            dv_ric_km_s: Some(leg.arrival_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.arrival_maneuver.dv_ric_km_s.norm(),
        });
    }

    // COLA burns (RIC Dv)
    if let Some(ref cola) = output.safety.cola {
        for m in cola {
            entries.push(ScheduleEntry {
                epoch: m.epoch,
                kind: ScheduleEntryKind::Cola { leg: m.leg_index },
                dv_ric_km_s: Some(m.dv_ric_km_s),
                dv_mag_km_s: m.dv_ric_km_s.norm(),
            });
        }
    }

    if entries.is_empty() {
        return;
    }

    entries.sort_by_key(|e| e.epoch);

    let _ = writeln!(out, "{}\n", variant.section_title());
    let _ = writeln!(
        out,
        "| # | Epoch (UTC) | Type | \u{0394}vR (m/s) | \u{0394}vI (m/s) | \u{0394}vC (m/s) | \\|\u{0394}v\\| (m/s) |",
    );
    let _ = writeln!(
        out,
        "|---|-------------|------|-----------|-----------|-----------|------------|",
    );

    let cola_marker = variant.cola_marker();
    for (i, entry) in entries.iter().enumerate() {
        // Mark COLA rows with the configured suffix (e.g. `"†"`) so mc.md can
        // visually flag the row as "not executed in Monte Carlo samples".
        let type_cell = match (&entry.kind, cola_marker) {
            (ScheduleEntryKind::Cola { .. }, Some(mark)) => {
                format!("{kind} {mark}", kind = entry.kind)
            }
            _ => format!("{}", entry.kind),
        };
        match entry.dv_ric_km_s {
            Some(v) => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | {:.2} | {:.2} | {:.2} | {:.1} |",
                    i + 1,
                    entry.epoch,
                    v.x * KM_TO_M,
                    v.y * KM_TO_M,
                    v.z * KM_TO_M,
                    entry.dv_mag_km_s * KM_TO_M,
                );
            }
            None => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | \u{2014} | \u{2014} | \u{2014} | {:.1} |",
                    i + 1,
                    entry.epoch,
                    entry.dv_mag_km_s * KM_TO_M,
                );
            }
        }
    }
    let _ = writeln!(out);
}

fn write_summary_block_mission(
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

fn write_transfer_section(out: &mut String, output: &PipelineOutput, input: &PipelineInput) {
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
            let _ = writeln!(out, "| \u{03b4}r/r | {delta_r_over_r:.3e} |");
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
            let _ = writeln!(out, "| \u{03b4}r/r | {delta_r_over_r:.3e} |");
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

fn write_lambert_solution(
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
fn write_perch_roe(
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

fn write_waypoint_section(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
) {
    let _ = writeln!(
        out,
        "## Waypoint Targeting ({} legs)\n",
        output.mission.legs.len(),
    );
    let _ = writeln!(
        out,
        "**Propagator:** {}\n",
        propagator_label(propagator, auto_drag),
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
            .waypoints
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

fn write_safety_section(
    out: &mut String,
    safety: &SafetyMetrics,
    config: &SafetyConfig,
    tier: SafetyTier,
    outcome: PassiveSafetyOutcome,
) {
    let assessment = assess_safety(safety, config);

    match tier {
        SafetyTier::Baseline => {
            let _ = writeln!(out, "## Safety (Analytical Baseline)\n");
            let _ = writeln!(
                out,
                "> Governing full-physics margins from Nyx are shown in the \
                 Summary table above and in Safety Comparison below.\n",
            );
        }
        SafetyTier::Governing => {
            let _ = writeln!(out, "## Safety\n");
        }
    }
    let _ = writeln!(out, "### Operational\n");
    let _ = writeln!(out, "| Check | Result |");
    let _ = writeln!(out, "| --- | --- |");

    let d3d_pass = if assessment.distance_3d_pass {
        "**PASS**"
    } else {
        "**FAIL**"
    };
    let _ = writeln!(out, "| 3D distance | {d3d_pass} |");
    let _ = writeln!(
        out,
        "| Min 3D distance | {} (threshold: {}) |",
        fmt_m(safety.operational.min_distance_3d_km, 1),
        fmt_m(config.min_distance_3d_km, 0),
    );
    let _ = writeln!(
        out,
        "{}",
        margin_or_shortfall_row(safety.operational.min_distance_3d_km, config.min_distance_3d_km),
    );
    let _ = writeln!(
        out,
        "| \u{2014} at | leg {}, t = {} |",
        safety.operational.min_3d_leg_index + 1,
        fmt_duration(safety.operational.min_3d_elapsed_s),
    );
    let ric = safety.operational.min_3d_ric_position_km;
    let _ = writeln!(
        out,
        "| \u{2014} RIC | [{:.4}, {:.4}, {:.4}] km |",
        ric[0], ric[1], ric[2],
    );

    // R/C plane
    let rc_km = safety.operational.min_rc_separation_km;
    let rc_ric = safety.operational.min_rc_ric_position_km;
    match assessment.rc_context {
        RcContext::AlongTrackDominated { along_track_km } => {
            let _ = writeln!(
                out,
                "| Min R/C-plane distance | {} (along-track dominated, V-bar at {along_track_km:.1} km) |",
                fmt_m(rc_km, 1),
            );
        }
        RcContext::RadialCrossTrack => {
            let _ = writeln!(
                out,
                "| Min R/C-plane distance | {} |",
                fmt_m(rc_km, 1),
            );
        }
    }
    let _ = writeln!(
        out,
        "| \u{2014} at | leg {}, t = {} |",
        safety.operational.min_rc_leg_index + 1,
        fmt_duration(safety.operational.min_rc_elapsed_s),
    );
    let _ = writeln!(
        out,
        "| \u{2014} RIC | [{:.4}, {:.4}, {:.4}] km |",
        rc_ric[0], rc_ric[1], rc_ric[2],
    );
    let _ = writeln!(out);

    write_passive_safety_md(out, safety, config, &assessment, outcome);
}

fn write_passive_safety_md(
    out: &mut String,
    safety: &SafetyMetrics,
    config: &SafetyConfig,
    assessment: &SafetyAssessment,
    outcome: PassiveSafetyOutcome,
) {
    let _ = writeln!(out, "### Passive Safety\n");
    let _ = writeln!(out, "| Check | Result |");
    let _ = writeln!(out, "| --- | --- |");
    let ei_pass = if assessment.ei_separation_pass {
        "**PASS**"
    } else {
        "**FAIL**"
    };
    let _ = writeln!(out, "| e/i separation | {ei_pass} |");
    let _ = writeln!(
        out,
        "| Min e/i separation | {} (threshold: {}) |",
        fmt_m(safety.passive.min_ei_separation_km, 1),
        fmt_m(config.min_ei_separation_km, 0),
    );
    let _ = writeln!(
        out,
        "{}",
        margin_or_shortfall_row(safety.passive.min_ei_separation_km, config.min_ei_separation_km),
    );
    // Phase angle is meaningless when e/i separation is effectively zero.
    // When it IS rendered and the angle is near ±180° we annotate it as
    // "near anti-parallel" so a reader can see that the e and i vectors
    // oppose (which is why the norm can collapse from perch-scale to
    // transit-scale separation).
    if safety.passive.min_ei_separation_km * KM_TO_M >= safety_thresh::MIN_EI_SEPARATION_FOR_PHASE_DISPLAY_M {
        let phase_deg = safety.passive.ei_phase_angle_rad.to_degrees();
        let anti_parallel = (phase_deg.abs() - safety_thresh::ANTIPARALLEL_PHASE_DEG).abs()
            < safety_thresh::ANTIPARALLEL_TOLERANCE_DEG;
        if anti_parallel {
            let _ = writeln!(
                out,
                "| e/i phase angle | {phase_deg:.2}\u{00b0} (near anti-parallel \u{2014} \
                 e and i vectors oppose, so the norm collapses) |",
            );
        } else {
            let _ = writeln!(
                out,
                "| e/i phase angle | {phase_deg:.2}\u{00b0} |",
            );
        }
    }
    let _ = writeln!(out);

    match outcome {
        PassiveSafetyOutcome::Pass => {
            let _ = writeln!(out, "### Overall: **PASS**\n");
        }
        PassiveSafetyOutcome::AdvisoryEi => {
            let _ = writeln!(
                out,
                "> Operationally **PASS** (3D distance governs). \
                 e/i separation is advisory \u{2014} see Formation Design.\n",
            );
        }
        PassiveSafetyOutcome::Fail => {
            let _ = writeln!(out, "### Overall: **FAIL**\n");
        }
    }
}

fn write_free_drift_section(
    out: &mut String,
    analyses: &[FreeDriftAnalysis],
    config: &SafetyConfig,
) {
    let _ = writeln!(out, "## Free-Drift Safety (abort case)\n");
    let _ = writeln!(
        out,
        "Per-leg analysis: what happens if the departure burn is skipped.\n",
    );

    for (i, analysis) in analyses.iter().enumerate() {
        let s = &analysis.safety;
        let assessment = assess_safety(s, config);

        let _ = writeln!(out, "### Leg {}\n", i + 1);
        let _ = writeln!(out, "| Check | Result |");
        let _ = writeln!(out, "| --- | --- |");

        let d3d_pass = if assessment.distance_3d_pass {
            "**PASS**"
        } else {
            "**FAIL**"
        };
        let _ = writeln!(out, "| Operational (3D distance) | {d3d_pass} |");
        let _ = writeln!(
            out,
            "| Min 3D distance | {} (threshold: {}) |",
            fmt_m(s.operational.min_distance_3d_km, 1),
            fmt_m(config.min_distance_3d_km, 0),
        );

        let rc_km = s.operational.min_rc_separation_km;
        match assessment.rc_context {
            RcContext::AlongTrackDominated { along_track_km } => {
                let _ = writeln!(
                    out,
                    "| Min R/C distance | {} (along-track dominated, V-bar at {along_track_km:.1} km) |",
                    fmt_m(rc_km, 1),
                );
            }
            RcContext::RadialCrossTrack => {
                let _ = writeln!(
                    out,
                    "| Min R/C distance | {} |",
                    fmt_m(rc_km, 1),
                );
            }
        }

        let ei_pass = if assessment.ei_separation_pass {
            "**PASS**"
        } else {
            "**FAIL**"
        };
        let _ = writeln!(out, "| Passive (e/i separation) | {ei_pass} |");
        let _ = writeln!(
            out,
            "| Min e/i separation | {} (threshold: {}) |",
            fmt_m(s.passive.min_ei_separation_km, 1),
            fmt_m(config.min_ei_separation_km, 0),
        );
        // Phase angle is meaningless when e/i separation is effectively zero
        if s.passive.min_ei_separation_km * KM_TO_M >= 0.05 {
            let _ = writeln!(
                out,
                "| e/i phase angle | {:.2}\u{00b0} |",
                s.passive.ei_phase_angle_rad.to_degrees(),
            );
        }

        let _ = writeln!(
            out,
            "| Bounded-motion | {} |",
            fmt_bounded_motion_residual(analysis.bounded_motion_residual),
        );
        let _ = writeln!(out);
    }
}

fn write_poca_section(
    out: &mut String,
    title: &str,
    poca_per_leg: &[Vec<rpo_core::mission::ClosestApproach>],
) {
    let _ = writeln!(out, "## {title}\n");
    for (leg_idx, pocas) in poca_per_leg.iter().enumerate() {
        if pocas.is_empty() {
            let _ = writeln!(
                out,
                "**Leg {}**: no close approach (diverging)\n",
                leg_idx + 1,
            );
            continue;
        }
        let closest = &pocas[0];
        let marker = if closest.is_global_minimum {
            " (global min)"
        } else {
            ""
        };
        let _ = writeln!(out, "**Leg {}**{marker}:\n", leg_idx + 1);
        let _ = writeln!(out, "| Metric | Value |");
        let _ = writeln!(out, "|--------|-------|");
        let _ = writeln!(
            out,
            "| Distance | {:.1} m at t = {:.1} s |",
            closest.distance_km * KM_TO_M,
            closest.elapsed_s,
        );
        let _ = writeln!(
            out,
            "| Position (RIC) | [{:.4}, {:.4}, {:.4}] km |",
            closest.position_ric_km.x,
            closest.position_ric_km.y,
            closest.position_ric_km.z,
        );
        let _ = writeln!(out);
    }
}

fn write_cola_sections(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
    context: ReportContext,
) {
    if let Some(ref cola) = output.safety.cola {
        let target_distance_km = input.cola.as_ref().map(|c| c.target_distance_km);
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
        let correction = match m.correction_type {
            rpo_core::mission::CorrectionType::InPlane => "in-plane",
            rpo_core::mission::CorrectionType::CrossTrack => "cross-track",
            rpo_core::mission::CorrectionType::Combined => "combined",
        };
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

fn write_cola_skipped_section(out: &mut String, skipped: &[rpo_core::mission::SkippedLeg]) {
    let _ = writeln!(out, "## COLA Skipped Legs\n");
    let _ = writeln!(out, "| Leg | Reason |");
    let _ = writeln!(out, "|-----|--------|");
    for s in skipped {
        let _ = writeln!(out, "| {} | {} |", s.leg_index + 1, s.error_message);
    }
    let _ = writeln!(out);
}

fn write_eclipse_section(out: &mut String, output: &PipelineOutput) {
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
    }
}

fn write_validation_section(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
    report: &ValidationReport,
    samples_per_leg: u32,
    derived_drag: Option<&DragConfig>,
) {
    let _ = writeln!(
        out,
        "## Nyx Validation ({samples_per_leg} samples/leg)\n",
    );

    // Position error
    let _ = writeln!(out, "### Position Error (Analytical vs Nyx Full-Physics)\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Max | {} |",
        fmt_m(report.max_position_error_km, 1),
    );
    let _ = writeln!(
        out,
        "| Mean | {} |",
        fmt_m(report.mean_position_error_km, 1),
    );
    let _ = writeln!(
        out,
        "| RMS | {} |",
        fmt_m(report.rms_position_error_km, 1),
    );
    let _ = writeln!(
        out,
        "| Max velocity error | {} |",
        fmt_m_s(report.max_velocity_error_km_s, 3),
    );
    let _ = writeln!(out);

    // Per-leg error from pre-computed summaries
    if !report.leg_summaries.is_empty() {
        let _ = writeln!(out, "### Per-Leg Position Error\n");
        let _ = writeln!(out, "| Leg | Max (m) | Mean (m) | RMS (m) |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        for (i, summary) in report.leg_summaries.iter().enumerate() {
            if summary.num_points > 0 {
                let _ = writeln!(
                    out,
                    "| {} | {:.1} | {:.1} | {:.1} |",
                    i + 1,
                    summary.max_position_error_km * KM_TO_M,
                    summary.mean_position_error_km * KM_TO_M,
                    summary.rms_position_error_km * KM_TO_M,
                );
            }
        }
        let _ = writeln!(out);
    }

    // Spacecraft
    let _ = writeln!(out, "### Spacecraft\n");
    let _ = writeln!(out, "| Property | Chief | Deputy |");
    let _ = writeln!(out, "| --- | --- | --- |");
    let _ = writeln!(
        out,
        "| Mass | {:.0} kg | {:.0} kg |",
        report.chief_config.dry_mass_kg, report.deputy_config.dry_mass_kg,
    );
    let _ = writeln!(
        out,
        "| Drag area | {:.2} m\u{00b2} | {:.2} m\u{00b2} |",
        report.chief_config.drag_area_m2, report.deputy_config.drag_area_m2,
    );
    let _ = writeln!(
        out,
        "| Cd | {:.1} | {:.1} |",
        report.chief_config.coeff_drag, report.deputy_config.coeff_drag,
    );
    let _ = writeln!(out);

    // Safety comparison
    write_safety_comparison(out, report, &input.config);

    // COLA validation detail (post-validation section, before eclipse)
    write_cola_validation_detail(out, output, report);

    // Eclipse validation
    if let Some(ref ev) = report.eclipse_validation {
        write_eclipse_validation(out, ev, output);
    }

    // Auto-derived drag
    if let Some(drag) = derived_drag {
        write_drag_table(out, drag);
    }
}

fn write_cola_validation_detail(
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
        let correction = match m.correction_type {
            rpo_core::mission::CorrectionType::InPlane => "in-plane",
            rpo_core::mission::CorrectionType::CrossTrack => "cross-track",
            rpo_core::mission::CorrectionType::Combined => "combined",
        };

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

/// Safety comparison numbers (analytical + numerical) in a single bundle.
///
/// Avoids threading seven floats through helper function signatures. The
/// `_km` suffixes are kept on every field per the project's mandatory unit
/// naming rules (see `CLAUDE.md#naming-rules`), which means clippy's
/// `struct_field_names` lint fires here and is allowed locally.
#[allow(clippy::struct_field_names)]
struct SafetyComparisonValues {
    ana_rc_km: f64,
    ana_3d_km: f64,
    ana_ei_km: f64,
    num_rc_km: f64,
    num_3d_km: f64,
    num_ei_km: f64,
}

impl SafetyComparisonValues {
    fn from_report(report: &ValidationReport) -> Self {
        Self {
            ana_rc_km: report
                .analytical_safety
                .as_ref()
                .map_or(0.0, |s| s.operational.min_rc_separation_km),
            ana_3d_km: report
                .analytical_safety
                .as_ref()
                .map_or(0.0, |s| s.operational.min_distance_3d_km),
            ana_ei_km: report
                .analytical_safety
                .as_ref()
                .map_or(0.0, |s| s.passive.min_ei_separation_km),
            num_rc_km: report.numerical_safety.operational.min_rc_separation_km,
            num_3d_km: report.numerical_safety.operational.min_distance_3d_km,
            num_ei_km: report.numerical_safety.passive.min_ei_separation_km,
        }
    }
}

fn write_safety_comparison(
    out: &mut String,
    report: &ValidationReport,
    config: &rpo_core::mission::MissionConfig,
) {
    let sc = config.safety.unwrap_or_default();
    let v = SafetyComparisonValues::from_report(report);

    // Prefer pre-COLA numerical safety as the analytical comparison baseline —
    // analytical propagation does not model COLA burns, so pre-COLA is the
    // apples-to-apples pair. When no COLA burn was applied, fall back to the
    // single numerical baseline. Mirrors `validation_insights` in insights.rs
    // so the rendered markdown and the insight list agree on both the delta
    // percentage and the "Nyx" label.
    let (cmp_3d, cmp_ei, annotation_mode) = match &report.pre_cola_numerical_safety {
        Some(pre) => (
            pre.operational.min_distance_3d_km,
            pre.passive.min_ei_separation_km,
            SafetyAnnotationMode::PreCola,
        ),
        None => (v.num_3d_km, v.num_ei_km, SafetyAnnotationMode::Standard),
    };

    let noncons_3d = analytical_overestimate(v.ana_3d_km, cmp_3d).is_some();
    let noncons_ei = analytical_overestimate(v.ana_ei_km, cmp_ei).is_some();

    if let Some(pre_cola) = &report.pre_cola_numerical_safety {
        write_safety_comparison_with_cola(out, &v, pre_cola, &sc, noncons_3d, noncons_ei);
    } else {
        write_safety_comparison_without_cola(out, &v, &sc, noncons_3d, noncons_ei);
    }

    write_safety_annotations(out, &SafetyAnnotationCtx {
        noncons_ei,
        ana_3d_km: v.ana_3d_km,
        num_3d_km: cmp_3d,
        config: &sc,
        mode: annotation_mode,
    });
}

/// 3-column safety comparison table (analytical / pre-COLA / post-COLA).
fn write_safety_comparison_with_cola(
    out: &mut String,
    v: &SafetyComparisonValues,
    pre_cola: &rpo_core::mission::SafetyMetrics,
    sc: &rpo_core::mission::SafetyConfig,
    noncons_3d: bool,
    noncons_ei: bool,
) {
    let _ = writeln!(
        out,
        "### Safety Comparison (Analytical vs Numerical + COLA)\n",
    );
    let _ = writeln!(
        out,
        "> Numerical columns: pre-COLA is the baseline Nyx trajectory without COLA impulses;\n\
         > post-COLA reflects the trajectory with COLA burns injected.\n",
    );
    let _ = writeln!(
        out,
        "| Metric | Analytical | Numerical (pre-COLA) | Numerical (post-COLA) | Threshold |",
    );
    let _ = writeln!(out, "| --- | --- | --- | --- | --- |");

    let pre_rc = pre_cola.operational.min_rc_separation_km;
    let pre_3d = pre_cola.operational.min_distance_3d_km;
    let pre_ei = pre_cola.passive.min_ei_separation_km;

    let _ = writeln!(
        out,
        "| Min R/C-plane distance (m) | {:.1} | {:.1} | {:.1} | \u{2014} |",
        v.ana_rc_km * KM_TO_M, pre_rc * KM_TO_M, v.num_rc_km * KM_TO_M,
    );
    let _ = writeln!(
        out,
        "| 3D distance (m) | {:.1} | {:.1} | {:.1} | {:.0}{} |",
        v.ana_3d_km * KM_TO_M, pre_3d * KM_TO_M, v.num_3d_km * KM_TO_M,
        sc.min_distance_3d_km * KM_TO_M, if noncons_3d { " \\*" } else { "" },
    );
    let _ = writeln!(
        out,
        "| e/i separation (m) | {:.1} | {:.1} | {:.1} | {:.0}{} |",
        v.ana_ei_km * KM_TO_M, pre_ei * KM_TO_M, v.num_ei_km * KM_TO_M,
        sc.min_ei_separation_km * KM_TO_M, if noncons_ei { " \\*" } else { "" },
    );
    let _ = writeln!(out);

    // Explanatory footnote: pre-COLA and post-COLA 3D distances are
    // identical whenever the primary POCA occurs before the COLA burn
    // fires. Up to the burn epoch the two trajectories are identical, so
    // the overall minimum is unchanged. Without this note, a reader
    // comparing the pre-COLA column (70.8 m) against the post-burn POCA
    // in the COLA Burns table (e.g. 121.9 m) thinks the report is
    // internally inconsistent.
    let pre_post_equal =
        (pre_3d - v.num_3d_km).abs() * KM_TO_M < safety_thresh::POCA_PRE_POST_EQUAL_TOL_M;
    if pre_post_equal {
        let _ = writeln!(
            out,
            "> Numerical pre-COLA and post-COLA min 3D are identical because the primary POCA \
             occurs before the COLA burn fires \u{2014} up to the burn epoch the two \
             trajectories are identical, so the overall minimum is unchanged. The COLA burn \
             targets a later conjunction; its post-burn effectiveness is reported separately \
             in \"COLA Burns Injected into Nyx Propagation\" below.\n",
        );
    }
}

/// 2-column safety comparison table (analytical / numerical), used when no
/// COLA burn was injected.
fn write_safety_comparison_without_cola(
    out: &mut String,
    v: &SafetyComparisonValues,
    sc: &rpo_core::mission::SafetyConfig,
    noncons_3d: bool,
    noncons_ei: bool,
) {
    let _ = writeln!(out, "### Safety Comparison (Analytical vs Numerical)\n");
    let _ = writeln!(out, "| Metric | Analytical | Numerical | Threshold |");
    let _ = writeln!(out, "| --- | --- | --- | --- |");

    let _ = writeln!(
        out,
        "| Min R/C-plane distance (m) | {:.1} | {:.1} | \u{2014} |",
        v.ana_rc_km * KM_TO_M, v.num_rc_km * KM_TO_M,
    );
    let _ = writeln!(
        out,
        "| 3D distance (m) | {:.1} | {:.1} | {:.0}{} |",
        v.ana_3d_km * KM_TO_M, v.num_3d_km * KM_TO_M,
        sc.min_distance_3d_km * KM_TO_M, if noncons_3d { " \\*" } else { "" },
    );
    let _ = writeln!(
        out,
        "| e/i separation (m) | {:.1} | {:.1} | {:.0}{} |",
        v.ana_ei_km * KM_TO_M, v.num_ei_km * KM_TO_M,
        sc.min_ei_separation_km * KM_TO_M, if noncons_ei { " \\*" } else { "" },
    );
    let _ = writeln!(out);
}

/// Whether safety annotations reference a COLA trajectory.
enum SafetyAnnotationMode {
    /// No COLA burns — Nyx label is just "Nyx".
    Standard,
    /// COLA burns injected — analytical comparison uses the pre-COLA baseline
    /// (apples-to-apples, since analytical propagation does not model COLA).
    /// Nyx label includes "pre-COLA".
    PreCola,
}

/// Context for writing non-conservative footnotes and 3D overestimation
/// annotations. `noncons_3d` is intentionally omitted — the annotation
/// function recomputes `analytical_overestimate(ana_3d_km, num_3d_km)`
/// as its single source of truth.
struct SafetyAnnotationCtx<'a> {
    noncons_ei: bool,
    ana_3d_km: f64,
    num_3d_km: f64,
    config: &'a rpo_core::mission::SafetyConfig,
    mode: SafetyAnnotationMode,
}

/// Write the non-conservative footnote and the 3D overestimation annotation.
///
/// Footnote: when the 3D value is non-conservative, quote the actual margin
/// reduction (analytical margin vs Nyx margin) as an absolute percentage so
/// an 85%-level reduction is not hidden behind the prior "`>10% smaller`"
/// phrasing that understated it by almost an order of magnitude.
///
/// Annotation: show the absolute overestimate delta (`+117 m`) AND the
/// analytical/Nyx ratio (`≈ 2.6× Nyx`). Readers routinely misparse the
/// older "overestimated by 165%" phrasing as "analytical is 1.65× Nyx",
/// losing a factor of ~1.6 in their mental model.
fn write_safety_annotations(out: &mut String, ctx: &SafetyAnnotationCtx<'_>) {
    let threshold_m = ctx.config.min_distance_3d_km * KM_TO_M;
    let ana_3d_m = ctx.ana_3d_km * KM_TO_M;
    let num_3d_m = ctx.num_3d_km * KM_TO_M;
    let ana_margin_m = ana_3d_m - threshold_m;
    let nyx_margin_m = num_3d_m - threshold_m;

    // Single source of truth. `ctx.noncons_3d` is derived from this same
    // helper on the same inputs by the caller — we re-compute here so the
    // Option acts as both the predicate and the struct-carrying value for
    // this function, avoiding a stringly-coupled boolean path.
    let overestimate = analytical_overestimate(ctx.ana_3d_km, ctx.num_3d_km);

    match (overestimate.as_ref(), ctx.noncons_ei) {
        (Some(ovr), _) if ana_margin_m > 0.0 => {
            let reduction_pct = ((ana_margin_m - nyx_margin_m) / ana_margin_m)
                * insight_thresh::PERCENT_PER_UNIT;
            let _ = writeln!(
                out,
                "\\* Full-physics 3D margin is {nyx_margin_m:.0} m (vs {ana_margin_m:.0} m \
                 analytical) \u{2014} a {reduction_pct:.0}% reduction. Analytical 3D distance \
                 is approximately {:.1}\u{00d7} Nyx. Use the Nyx column as the governing value.",
                ovr.ratio,
            );
        }
        // e/i is flagged but 3D reduction wording does not apply — emit a
        // terser footnote so the `\*` marker in the table has a referent.
        // Fires only when 3D is consistent but e/i disagrees.
        (None, true) => {
            let _ = writeln!(
                out,
                "\\* Numerical e/i margin is materially smaller than the analytical value \
                 \u{2014} see the insight list below.",
            );
        }
        _ => {}
    }

    if let Some(ovr) = overestimate {
        let thr_ratio = margin_ratio(ctx.num_3d_km, ctx.config.min_distance_3d_km);
        let nyx_label = match ctx.mode {
            SafetyAnnotationMode::PreCola => "Nyx pre-COLA trajectory",
            SafetyAnnotationMode::Standard => "Nyx",
        };
        let _ = writeln!(
            out,
            "\n> Analytical overestimates min 3D distance: {ana_3d_m:.0} m analytical vs \
             {num_3d_m:.0} m {nyx_label} (+{:.0} m, ~{:.1}\u{00d7}). The Nyx value \
             governs for operations \u{2014} it sits {nyx_margin_m:.0} m above the \
             {threshold_m:.0} m keep-out ({thr_ratio:.1}\u{00d7} threshold).\n",
            ovr.delta_m,
            ovr.ratio,
        );
    }
}

fn write_eclipse_validation(
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        MissionConfig, OperationalSafety, PassiveSafety, SafetyConfig, SafetyMetrics,
        ValidationReport,
    };
    use rpo_core::pipeline::to_propagation_model;
    use rpo_nyx::pipeline::execute_mission;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("examples")
    }

    /// Build a minimal [`SafetyMetrics`] with only the fields exercised by
    /// `write_safety_comparison`: 3D distance and e/i separation.
    fn make_safety(min_3d_km: f64, min_ei_km: f64) -> SafetyMetrics {
        SafetyMetrics {
            operational: OperationalSafety {
                min_rc_separation_km: 0.0,
                min_distance_3d_km: min_3d_km,
                min_rc_leg_index: 0,
                min_rc_elapsed_s: 0.0,
                min_rc_ric_position_km: Vector3::zeros(),
                min_3d_leg_index: 0,
                min_3d_elapsed_s: 0.0,
                min_3d_ric_position_km: Vector3::zeros(),
            },
            passive: PassiveSafety {
                min_ei_separation_km: min_ei_km,
                de_magnitude: 0.0,
                di_magnitude: 0.0,
                ei_phase_angle_rad: 0.0,
            },
        }
    }

    /// Build a synthetic [`ValidationReport`] where pre-COLA vs post-COLA
    /// denominators produce distinct rounded percentages, matching the
    /// fixture used in `insights::tests::validation_report_with_pre_cola`.
    fn validation_report_with_pre_cola(
        analytical_3d_km: f64,
        pre_cola_3d_km: f64,
        post_cola_3d_km: f64,
    ) -> ValidationReport {
        ValidationReport {
            leg_points: vec![],
            max_position_error_km: 0.1,
            mean_position_error_km: 0.05,
            rms_position_error_km: 0.07,
            max_velocity_error_km_s: 0.001,
            analytical_safety: Some(make_safety(analytical_3d_km, 0.170)),
            numerical_safety: make_safety(post_cola_3d_km, 0.170),
            pre_cola_numerical_safety: Some(make_safety(pre_cola_3d_km, 0.170)),
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness: vec![],
            leg_summaries: vec![],
        }
    }

    /// Build a synthetic [`ValidationReport`] without a pre-COLA baseline,
    /// so the fallback 2-column layout is exercised.
    fn validation_report_without_pre_cola(
        analytical_3d_km: f64,
        numerical_3d_km: f64,
    ) -> ValidationReport {
        ValidationReport {
            leg_points: vec![],
            max_position_error_km: 0.1,
            mean_position_error_km: 0.05,
            rms_position_error_km: 0.07,
            max_velocity_error_km_s: 0.001,
            analytical_safety: Some(make_safety(analytical_3d_km, 0.170)),
            numerical_safety: make_safety(numerical_3d_km, 0.170),
            pre_cola_numerical_safety: None,
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness: vec![],
            leg_summaries: vec![],
        }
    }

    /// Mission config with a safety block whose thresholds match
    /// `insights::tests::default_config` so the two test suites stay in sync.
    fn mission_config_with_safety() -> MissionConfig {
        MissionConfig {
            safety: Some(SafetyConfig {
                min_distance_3d_km: 0.050,
                min_ei_separation_km: 0.100,
            }),
            ..MissionConfig::default()
        }
    }

    #[test]
    fn mission_safety_comparison_uses_pre_cola_when_available() {
        // Mirrors the insight test in `insights::tests::
        // validation_insights_compares_analytical_against_pre_cola_when_available`.
        //   analytical = 0.200 km
        //   pre-COLA  (correct): 200 - 120 =  80 m, 200 / 120 ≈ 1.7×
        //   post-COLA (wrong):   200 -  80 = 120 m, 200 /  80 = 2.5×
        // The comparison-table annotation must use the pre-COLA baseline and
        // absolute-delta + ratio wording.
        let report = validation_report_with_pre_cola(
            /* analytical_3d_km = */ 0.200,
            /* pre_cola_3d_km  = */ 0.120,
            /* post_cola_3d_km = */ 0.080,
        );
        let config = mission_config_with_safety();

        let mut md = String::new();
        write_safety_comparison(&mut md, &report, &config);

        assert!(
            md.contains("Nyx pre-COLA trajectory"),
            "expected 'Nyx pre-COLA trajectory' label, got:\n{md}",
        );
        assert!(
            md.contains("+80 m"),
            "expected '+80 m' absolute delta (pre-COLA baseline), got:\n{md}",
        );
        assert!(
            md.contains("~1.7"),
            "expected '~1.7×' ratio (pre-COLA baseline), got:\n{md}",
        );
        assert!(
            !md.contains("+120 m"),
            "must not report +120 m delta (post-COLA baseline), got:\n{md}",
        );
        assert!(
            !md.contains("~2.5"),
            "must not report 2.5× ratio (post-COLA baseline), got:\n{md}",
        );
        assert!(
            !md.contains("post-COLA trajectory"),
            "must not use 'post-COLA trajectory' label when pre-COLA is available, got:\n{md}",
        );
        // Legacy phrasings must not leak back in.
        assert!(
            !md.contains("Numerical margin") && !md.contains("margin) governs"),
            "legacy 'Numerical margin ... governs' phrasing must not reappear, got:\n{md}",
        );
        assert!(
            !md.contains("overestimated by"),
            "legacy 'overestimated by N%' phrasing must not reappear, got:\n{md}",
        );
    }

    #[test]
    fn mission_safety_comparison_fallback_without_pre_cola() {
        // No pre-COLA baseline — the 2-column layout and "Nyx" (plain) label
        // must be used. analytical = 200 m, Nyx = 100 m →
        //   absolute delta = 100 m, ratio = 2.0×
        let report = validation_report_without_pre_cola(0.200, 0.100);
        let config = mission_config_with_safety();

        let mut md = String::new();
        write_safety_comparison(&mut md, &report, &config);

        // Plain "Nyx" label (no "pre-COLA"/"post-COLA") followed by the
        // absolute delta parenthetical in the new template.
        assert!(
            md.contains("vs 100 m Nyx (+100 m"),
            "expected plain 'Nyx' label with absolute delta, got:\n{md}",
        );
        assert!(
            !md.contains("pre-COLA"),
            "must not mention pre-COLA in fallback, got:\n{md}",
        );
        assert!(
            !md.contains("post-COLA"),
            "must not mention post-COLA in fallback, got:\n{md}",
        );
        assert!(
            md.contains("~2.0"),
            "expected '~2.0×' ratio, got:\n{md}",
        );
    }

    /// Load `examples/mission.json` and drive the real pipeline, returning
    /// the `(input, output, propagator)` tuple used by report-rendering tests.
    ///
    /// Mirrors the pattern in `mission_markdown_contains_expected_sections`.
    /// The default `examples/mission.json` does not enable enrichment, so
    /// `output.formation_design` is `None` on return.
    fn load_example_mission() -> (PipelineInput, PipelineOutput, PropagationModel) {
        let input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.propagator);
        (input, output, propagator)
    }

    /// Build a minimal synthetic [`rpo_core::mission::FormationDesignReport`]
    /// sufficient to toggle `output.formation_design.is_some()` in rendering
    /// tests. Uses the `Baseline` perch variant with empty leg vectors so
    /// `write_formation_design_md` stays on its cheapest code path.
    fn synthetic_formation_design_report() -> rpo_core::mission::FormationDesignReport {
        rpo_core::mission::FormationDesignReport {
            perch: rpo_core::mission::PerchEnrichmentResult::Baseline(
                rpo_core::types::QuasiNonsingularROE::default(),
            ),
            waypoints: Vec::new(),
            transit_safety: Vec::new(),
            mission_min_ei_separation_km: None,
            drift_prediction: None,
        }
    }

    #[test]
    fn perch_roe_heading_says_enriched_when_formation_design_present() {
        let (input, mut output, propagator) = load_example_mission();
        output.formation_design = Some(synthetic_formation_design_report());

        let md = mission_to_markdown(&output, &input, &propagator, false);

        assert!(
            md.contains("### Enriched Perch ROE"),
            "heading must reflect that the rendered values are enriched when formation_design is present; got:\n{md}",
        );
        assert!(
            !md.contains("\n### Perch ROE\n"),
            "unqualified heading must not appear when enrichment is active; got:\n{md}",
        );
    }

    #[test]
    fn perch_roe_heading_stays_bare_when_formation_design_absent() {
        let (input, output, propagator) = load_example_mission();
        assert!(
            output.formation_design.is_none(),
            "baseline example must not enable enrichment",
        );

        let md = mission_to_markdown(&output, &input, &propagator, false);

        assert!(
            md.contains("### Perch ROE"),
            "baseline heading stays when no enrichment; got:\n{md}",
        );
        assert!(
            !md.contains("### Enriched Perch ROE"),
            "must not claim enrichment when None; got:\n{md}",
        );
    }

    #[test]
    fn mission_markdown_contains_expected_sections() {
        let input: PipelineInput =
            serde_json::from_str(
                &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
            )
            .unwrap();
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.propagator);
        let md = mission_to_markdown(&output, &input, &propagator, false);

        assert!(md.contains("# Mission Summary"), "missing summary header");
        assert!(md.contains("**Verdict:"), "missing verdict");
        assert!(md.contains("## Transfer"), "missing transfer section");
        assert!(
            md.contains("## Waypoint Targeting"),
            "missing waypoint section",
        );
        assert!(md.contains("## Safety"), "missing safety section");
        assert!(md.contains("## Eclipse"), "missing eclipse section");
        // No duplicate summary: only `# Mission Summary` at the top, no `# Summary` at bottom
        let h1_count = md.lines().filter(|l| l.starts_with("# ")).count();
        assert_eq!(
            h1_count, 1,
            "expected exactly one H1 heading, got {h1_count}",
        );
        // Verify summary table has emoji status
        assert!(
            md.contains("\u{2705} PASS"),
            "missing pass emoji in summary table",
        );
        // Verify detailed safety uses text PASS/FAIL (not emoji)
        assert!(
            md.contains("| 3D distance | **PASS** |"),
            "missing text PASS in safety detail",
        );
    }

    // ── Report-wording regression tests ─────────────────────────────
    //
    // These lock the load-bearing phrasing from the 2026-04-10 CLI report
    // wording cleanup. Each test asserts both the presence of the NEW
    // wording and the absence of the LEGACY wording so regressions either
    // direction surface immediately.

    /// End-to-end render from `examples/mission.json` with enrichment
    /// enabled via `safety_requirements`. Returns the rendered markdown
    /// along with the input/output/propagator tuple for further assertions.
    fn render_mission_with_enrichment() -> String {
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();
        input.safety_requirements = Some(rpo_core::mission::SafetyRequirements {
            min_separation_km: 0.100,
            alignment: rpo_core::mission::EiAlignment::default(),
        });
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.propagator);
        mission_to_markdown(&output, &input, &propagator, false)
    }

    #[test]
    fn mission_report_transit_safety_footnote_names_the_mechanism() {
        let md = render_mission_with_enrichment();
        assert!(
            md.contains("Guided waypoint targeting enforces RIC position and velocity at each waypoint"),
            "transit safety footnote must name the RIC-targeting mechanism; got:\n{md}"
        );
        assert!(
            !md.contains("FAIL results below are expected for guided operations"),
            "legacy 'FAIL expected' hand-wave must not return; got:\n{md}"
        );
    }

    #[test]
    fn mission_report_passive_safety_renders_shortfall_row_when_below_threshold() {
        let md = render_mission_with_enrichment();
        // V-bar perch geometry has zero e/i separation by construction — the
        // Passive Safety table must render a Shortfall row, never a negative
        // Margin row.
        assert!(
            md.contains("| Shortfall |"),
            "Passive Safety must render a Shortfall row for sub-threshold e/i; got:\n{md}"
        );
        assert!(
            !md.contains("| Margin | -"),
            "negative margins must render as Shortfall, not 'Margin | -X.X m'; got:\n{md}"
        );
    }

    #[test]
    fn mission_report_cola_callout_quotes_per_leg_values_without_leg_hedge() {
        // examples/mission.json is configured to miss the 300 m COLA target
        // on leg 3, so the callout must fire.
        let md = render_mission_with_enrichment();
        assert!(
            md.contains("Analytical COLA solver did not reach"),
            "COLA callout must fire on the leg-3 miss; got:\n{md}"
        );
        assert!(
            !md.contains("leg(s)"),
            "codegen hedge 'leg(s)' must not appear; got:\n{md}"
        );
        assert!(
            !md.contains("will compound this deficit"),
            "legacy 'will compound' forecast must not appear; got:\n{md}"
        );
        assert!(
            md.contains("run `validate` before relying on this burn"),
            "callout must point the operator at `validate`; got:\n{md}"
        );
    }

    /// Build a synthetic [`ValidationReport`] mimicking the audit dataset:
    /// analytical 188 m 3D, Nyx 71 m 3D, against a 50 m keep-out. When
    /// `cola_miss` is true, injects a COLA effectiveness entry flagged as
    /// threshold-not-met so the `(AT MARGIN)` qualifier and the CRITICAL
    /// insight fire.
    fn synthetic_validation_report(tight_margin: bool, cola_miss: bool) -> ValidationReport {
        let nyx_3d_km = if tight_margin { 0.071 } else { 0.150 };
        let cola_effectiveness = if cola_miss {
            vec![rpo_core::mission::ColaEffectivenessEntry {
                leg_index: 2,
                analytical_post_cola_poca_km: Some(0.281),
                nyx_post_cola_min_distance_km: 0.122,
                target_distance_km: Some(0.300),
                threshold_met: Some(false),
            }]
        } else {
            vec![]
        };
        ValidationReport {
            leg_points: vec![],
            max_position_error_km: 0.300,
            mean_position_error_km: 0.170,
            rms_position_error_km: 0.181,
            max_velocity_error_km_s: 0.0003,
            analytical_safety: Some(make_safety(0.188, 0.002)),
            numerical_safety: make_safety(nyx_3d_km, 0.018),
            pre_cola_numerical_safety: Some(make_safety(nyx_3d_km, 0.018)),
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness,
            leg_summaries: vec![],
        }
    }

    /// Load `examples/mission.json` with enrichment enabled so that the
    /// `(AT MARGIN)` verdict qualifier path is reachable (the qualifier only
    /// downgrades from `OperationallyFeasible`, which in turn requires
    /// enrichment to be active when e/i fails).
    fn load_example_mission_with_enrichment() -> (PipelineInput, PipelineOutput, PropagationModel) {
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();
        input.safety_requirements = Some(rpo_core::mission::SafetyRequirements {
            min_separation_km: 0.100,
            alignment: rpo_core::mission::EiAlignment::default(),
        });
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.propagator);
        (input, output, propagator)
    }

    #[test]
    fn validate_report_emits_at_margin_qualifier_on_tight_margin_and_cola_miss() {
        let (input, output, propagator) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ true);
        let ctx = ValidationContext {
            propagator: &propagator,
            auto_drag: true,
            samples_per_leg: 50,
            derived_drag: None,
        };
        let md = validation_to_markdown(&output, &input, &report, &ctx);

        assert!(
            md.contains("OPERATIONALLY FEASIBLE (AT MARGIN)"),
            "verdict must carry '(AT MARGIN)' qualifier; got:\n{md}"
        );
        // Legacy column name must never render, regardless of whether the
        // COLA validation detail section is emitted (it gates on post_cola
        // leg points which the synthetic fixture does not populate).
        assert!(
            !md.contains("Nyx Post-COLA Min"),
            "legacy 'Nyx Post-COLA Min' column header must be removed; got:\n{md}"
        );
    }

    #[test]
    fn validate_report_safety_comparison_uses_absolute_delta_and_ratio() {
        let (input, output, propagator) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ false);
        let ctx = ValidationContext {
            propagator: &propagator,
            auto_drag: true,
            samples_per_leg: 50,
            derived_drag: None,
        };
        let md = validation_to_markdown(&output, &input, &report, &ctx);

        // 188 analytical - 71 Nyx = 117 m delta, 188/71 ≈ 2.6×.
        assert!(
            md.contains("+117 m"),
            "annotation must show '+117 m' absolute delta; got:\n{md}"
        );
        assert!(
            md.contains("~2.6"),
            "annotation must show '~2.6×' ratio; got:\n{md}"
        );
        assert!(
            !md.contains("Numerical margin >"),
            "legacy '>10% smaller' footnote must not return; got:\n{md}"
        );
        assert!(
            !md.contains("Numerical margin ("),
            "legacy 'Numerical margin (...× threshold) governs' phrasing must not return; got:\n{md}"
        );
    }

    #[test]
    fn validate_report_adds_pre_post_cola_equality_footnote() {
        let (input, output, propagator) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ true);
        let ctx = ValidationContext {
            propagator: &propagator,
            auto_drag: true,
            samples_per_leg: 50,
            derived_drag: None,
        };
        let md = validation_to_markdown(&output, &input, &report, &ctx);

        assert!(
            md.contains("pre-COLA and post-COLA min 3D are identical"),
            "Safety Comparison must explain pre/post equality; got:\n{md}"
        );
    }
}

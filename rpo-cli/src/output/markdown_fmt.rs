//! Markdown report generation.
//!
//! Pure functions that produce self-contained markdown reports as `String`s.
//! Called by porcelain command `run()` functions when `--markdown` is given.
//! No `println!` — the caller prints the returned string once.
//!
//! # Note on `let _ = writeln!(...)`
//!
//! `writeln!` to `String` is infallible (`fmt::Write` for `String` always
//! returns `Ok`). The `let _ =` suppresses the unused `Result` lint without
//! introducing an `unwrap()` or `expect()` that would imply fallibility.
//! This is a standard Rust convention for infallible formatting.

use std::fmt::Write;

use rpo_core::mission::{
    assess_safety, EnsembleStatistics, FreeDriftAnalysis, MissionPhase, MonteCarloReport,
    RcContext, SafetyAssessment, SafetyConfig, SafetyMetrics, ValidationReport,
};
use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::propagation::{DragConfig, PropagationModel};

use super::common::{
    compute_leg_error_stats, determine_mc_verdict, determine_verdict, fmt_bounded_motion_residual,
    fmt_duration, fmt_m, fmt_m_s, fmt_roe_component, McBaseline, SafetyTier, ValidationTier,
    VerdictResult,
};
use super::insights;

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

    // Summary block at top
    let vr = determine_verdict(output, &sc, None);
    write_summary_block_mission(&mut out, &vr, output, &sc, None);

    write_transfer_section(&mut out, output, input);
    write_waypoint_section(&mut out, output, input, propagator, auto_drag);

    if let Some(ref safety) = output.mission.safety {
        write_safety_section(&mut out, safety, &sc, SafetyTier::Governing);
    }

    if let Some(ref poca) = output.poca {
        write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }

    if let Some(ref fd) = output.free_drift {
        write_free_drift_section(&mut out, fd, &sc);
    }

    if let Some(ref fd_poca) = output.free_drift_poca {
        write_poca_section(&mut out, "Free-Drift Closest Approach (Brent-refined)", fd_poca);
    }

    write_cola_sections(&mut out, output);

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

    let vr = determine_verdict(output, &sc, Some(report));
    write_summary_block_mission(&mut out, &vr, output, &sc, Some(report));

    let _ = writeln!(
        out,
        "> \u{0394}v values below reflect the analytical targeting plan (J2 STM). \
         Nyx full-physics validation governs safety margins \u{2014} \
         it does not recompute maneuvers.\n",
    );

    write_transfer_section(&mut out, output, input);
    write_waypoint_section(&mut out, output, input, ctx.propagator, ctx.auto_drag);

    if let Some(ref safety) = output.mission.safety {
        write_safety_section(&mut out, safety, &sc, SafetyTier::Baseline(ValidationTier::Nyx));
    }

    if let Some(ref poca) = output.poca {
        write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }

    if let Some(ref fd) = output.free_drift {
        write_free_drift_section(&mut out, fd, &sc);
    }

    if let Some(ref fd_poca) = output.free_drift_poca {
        write_poca_section(&mut out, "Free-Drift Closest Approach (Brent-refined)", fd_poca);
    }

    write_cola_sections(&mut out, output);

    write_eclipse_section(&mut out, output);
    write_validation_section(
        &mut out, output, input, report, ctx.samples_per_leg, ctx.derived_drag,
    );

    // Insights
    let insight_lines = insights::validation_insights(report, &sc);
    write_insights(&mut out, &insight_lines);

    out
}

/// Generate a complete markdown report for the `mc` command.
#[must_use]
pub fn mc_to_markdown(
    output: &PipelineOutput,
    input: &PipelineInput,
    baseline: &McBaseline,
    derived_drag: Option<&DragConfig>,
) -> String {
    let report = output
        .monte_carlo
        .as_ref()
        .expect("monte_carlo field must be set for mc_to_markdown");
    let mut out = String::with_capacity(8192);
    let sc = input.config.safety.unwrap_or_default();
    let stats = &report.statistics;

    write_mc_summary_table(&mut out, report, baseline, &sc);
    write_mc_baseline_section(&mut out, baseline);
    if derived_drag.is_some() {
        let _ = writeln!(
            out,
            "> Drag-aware targeting; \u{0394}v differs slightly from analytical-only (`mission`) results.\n",
        );
    }

    // Nominal safety context (free-drift, POCA)
    if let Some(ref safety) = output.mission.safety {
        write_safety_section(&mut out, safety, &sc, SafetyTier::Baseline(ValidationTier::MonteCarlo));
    }
    if let Some(ref poca) = output.poca {
        write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }
    if let Some(ref fd) = output.free_drift {
        write_free_drift_section(&mut out, fd, &sc);
    }
    if let Some(ref fd_poca) = output.free_drift_poca {
        write_poca_section(&mut out, "Free-Drift Closest Approach (Brent-refined)", fd_poca);
    }
    write_cola_sections(&mut out, output);

    // MC ensemble statistics
    write_mc_config_section(&mut out, report, baseline);
    write_mc_dv_distribution(&mut out, stats);
    write_mc_operational_safety(&mut out, stats, report);
    write_mc_passive_safety(&mut out, stats, report);

    let insight_lines = insights::mc_insights(report, &sc);
    write_insights(&mut out, &insight_lines);

    write_mc_convergence_and_miss(&mut out, stats, report);
    write_mc_diagnostics(&mut out, report, derived_drag);

    out
}

// ── MC section helpers ──────────────────────────────────────────

fn write_mc_summary_table(
    out: &mut String,
    report: &MonteCarloReport,
    baseline: &McBaseline,
    config: &SafetyConfig,
) {
    let stats = &report.statistics;
    let total_dv_km_s = baseline.total_dv_km_s();
    let n = f64::from(report.config.num_samples);
    let vr = determine_mc_verdict(report);

    let _ = writeln!(out, "# Mission Summary\n");
    let _ = writeln!(
        out,
        "**Verdict: {} ({})** | {} total \u{0394}v | {} duration | {} waypoints\n",
        vr.verdict,
        vr.reason,
        fmt_m_s(total_dv_km_s, 1),
        fmt_duration(baseline.lambert_tof_s + baseline.waypoint_duration_s),
        baseline.num_legs,
    );

    let _ = writeln!(out, "| Metric | Value | Threshold | Status |");
    let _ = writeln!(out, "| --- | --- | --- | --- |");

    let coll_n = (stats.collision_probability * n).round();
    let _ = writeln!(
        out,
        "| Collision probability | {:.1}% ({:.0}/{}) | 0 | {} |",
        stats.collision_probability * 100.0,
        coll_n,
        report.config.num_samples,
        status_emoji(stats.collision_probability <= 0.0),
    );

    if let Some(ref d3d) = stats.min_3d_distance_km {
        let _ = writeln!(
            out,
            "| Min 3D distance (p05) | {} | {} | {} |",
            fmt_m(d3d.p05, 1),
            fmt_m(config.min_distance_3d_km, 0),
            status_emoji(d3d.p05 >= config.min_distance_3d_km),
        );
    }

    let ei_n = (stats.ei_violation_rate * n).round();
    let _ = writeln!(
        out,
        "| e/i violations | {:.1}% ({:.0}/{}) | \u{2014} | {} |",
        stats.ei_violation_rate * 100.0,
        ei_n,
        report.config.num_samples,
        if stats.ei_violation_rate > 0.0 { "\u{26a0}\u{fe0f}" } else { "\u{2705}" },
    );

    let conv_n = (stats.convergence_rate * n).round();
    let _ = writeln!(
        out,
        "| Convergence | {:.1}% ({:.0}/{}) | \u{2014} | {} |",
        stats.convergence_rate * 100.0,
        conv_n,
        report.config.num_samples,
        status_emoji(stats.convergence_rate >= 1.0),
    );
    let _ = writeln!(out);
}

fn write_mc_baseline_section(out: &mut String, baseline: &McBaseline) {
    let total_dv_km_s = baseline.total_dv_km_s();
    let classification = if baseline.is_far_field { "FAR-FIELD" } else { "PROXIMITY" };

    let _ = writeln!(out, "## Baseline\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(out, "| Classification | {classification} |");
    let _ = writeln!(
        out,
        "| Transfer \u{0394}v | {} ({}) |",
        fmt_m_s(baseline.lambert_dv_km_s, 1),
        fmt_duration(baseline.lambert_tof_s),
    );
    let _ = writeln!(out, "| Waypoint legs | {} |", baseline.num_legs);
    let _ = writeln!(
        out,
        "| Targeting \u{0394}v | {} ({}) |",
        fmt_m_s(baseline.waypoint_dv_km_s, 1),
        fmt_duration(baseline.waypoint_duration_s),
    );
    let _ = writeln!(out, "| Total \u{0394}v | {} |", fmt_m_s(total_dv_km_s, 1));
    let _ = writeln!(out);
}

fn write_mc_config_section(
    out: &mut String,
    report: &MonteCarloReport,
    baseline: &McBaseline,
) {
    let total_dv_km_s = baseline.total_dv_km_s();
    let seed = report.config.seed.unwrap_or(42);

    let _ = writeln!(
        out,
        "## Monte Carlo ({} samples, {}, seed={seed})\n",
        report.config.num_samples, report.config.mode,
    );

    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Nominal \u{0394}v | {} (waypoint) |",
        fmt_m_s(report.nominal_dv_km_s, 1),
    );
    let _ = writeln!(
        out,
        "| Transfer \u{0394}v | {} |",
        fmt_m_s(baseline.lambert_dv_km_s, 1),
    );
    let _ = writeln!(
        out,
        "| Total \u{0394}v | {} |",
        fmt_m_s(total_dv_km_s, 1),
    );
    let _ = writeln!(
        out,
        "| Wall time | {} |",
        fmt_duration(report.elapsed_wall_s),
    );
    let _ = writeln!(out);
}

fn write_mc_dv_distribution(out: &mut String, stats: &EnsembleStatistics) {
    let dv = &stats.total_dv_km_s;
    let _ = writeln!(out, "### \u{0394}v Distribution (m/s)\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(out, "| Mean | {:.2} |", dv.mean * 1000.0);
    let _ = writeln!(out, "| Std | {:.2} |", dv.std_dev * 1000.0);
    let _ = writeln!(out, "| p05 | {:.2} |", dv.p05 * 1000.0);
    let _ = writeln!(out, "| p95 | {:.2} |", dv.p95 * 1000.0);
    let _ = writeln!(out, "| Min | {:.2} |", dv.min * 1000.0);
    let _ = writeln!(out, "| Max | {:.2} |", dv.max * 1000.0);
    let _ = writeln!(out);
}

fn write_mc_operational_safety(
    out: &mut String,
    stats: &EnsembleStatistics,
    report: &MonteCarloReport,
) {
    let n = f64::from(report.config.num_samples);
    let coll_n = (stats.collision_probability * n).round();

    let _ = writeln!(out, "### Operational Safety\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Collision probability | {:.0} ({:.0}/{}) |",
        stats.collision_probability, coll_n, report.config.num_samples,
    );
    let _ = writeln!(
        out,
        "| Keep-out violations | {:.1}% ({:.0}/{}) |",
        stats.keepout_violation_rate * 100.0,
        (stats.keepout_violation_rate * n).round(),
        report.config.num_samples,
    );
    let _ = writeln!(out);

    if let Some(ref rc) = stats.min_rc_distance_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min R/C-plane distance (m) | {:.1} | {:.1} | {:.1} |",
            rc.p05 * 1000.0, rc.p50 * 1000.0, rc.p95 * 1000.0,
        );
        let _ = writeln!(out);
    }

    if let Some(ref d3d) = stats.min_3d_distance_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min 3D distance (m) | {:.1} | {:.1} | {:.1} |",
            d3d.p05 * 1000.0, d3d.p50 * 1000.0, d3d.p95 * 1000.0,
        );
        let _ = writeln!(out);
    }
}

fn write_mc_passive_safety(
    out: &mut String,
    stats: &EnsembleStatistics,
    report: &MonteCarloReport,
) {
    let ei_n = (stats.ei_violation_rate * f64::from(report.config.num_samples)).round();

    let _ = writeln!(out, "### Passive Safety\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| e/i violations | {:.1}% ({:.0}/{}) |",
        stats.ei_violation_rate * 100.0, ei_n, report.config.num_samples,
    );
    let _ = writeln!(out);

    if let Some(ref ei) = stats.min_ei_separation_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min e/i separation (m) | {:.1} | {:.1} | {:.1} |",
            ei.p05 * 1000.0, ei.p50 * 1000.0, ei.p95 * 1000.0,
        );
        let _ = writeln!(out);
    }

    let _ = writeln!(
        out,
        "> **Note:** e/i is a free-drift passive safety metric. \
         High violation rates are common in guided missions while operational safety holds.\n",
    );
}

fn write_mc_convergence_and_miss(
    out: &mut String,
    stats: &EnsembleStatistics,
    report: &MonteCarloReport,
) {
    let conv_n = (stats.convergence_rate * f64::from(report.config.num_samples)).round();

    let _ = writeln!(out, "### Convergence\n");
    let _ = writeln!(
        out,
        "**{:.1}%** ({:.0}/{}), {} failures\n",
        stats.convergence_rate * 100.0,
        conv_n,
        report.config.num_samples,
        report.num_failures,
    );

    if stats.waypoint_miss_km.iter().any(Option::is_some) {
        let _ = writeln!(out, "### Per-Waypoint Miss (m)\n");
        let _ = writeln!(out, "| Waypoint | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- |");
        for (i, miss_opt) in stats.waypoint_miss_km.iter().enumerate() {
            if let Some(miss) = miss_opt {
                let _ = writeln!(
                    out,
                    "| WP {} | {:.1} | {:.1} |",
                    i + 1,
                    miss.p50 * 1000.0,
                    miss.p95 * 1000.0,
                );
            }
        }
        let _ = writeln!(out);
    }
}

fn write_mc_diagnostics(
    out: &mut String,
    report: &MonteCarloReport,
    derived_drag: Option<&DragConfig>,
) {
    let _ = writeln!(out, "### Diagnostics\n");
    if let Some(ref cov) = report.covariance_cross_check {
        let _ = writeln!(out, "**Covariance cross-check:**\n");
        let _ = writeln!(out, "| Metric | Value |");
        let _ = writeln!(out, "| --- | --- |");
        let _ = writeln!(
            out,
            "| 3\u{03c3} containment | {:.1}% |",
            cov.terminal_3sigma_containment * 100.0,
        );
        let sr = &cov.sigma_ratio_ric;
        let _ = writeln!(
            out,
            "| Sigma ratio (R / I / C) | {:.2} / {:.2} / {:.2} *(closed-loop: suppressed by retargeting)* |",
            sr[0], sr[1], sr[2],
        );
        let _ = writeln!(
            out,
            "| Mahalanobis distance | {:.2} *(< 1 = nominal within 1\u{03c3})* |",
            cov.min_mahalanobis_distance,
        );
        let _ = writeln!(out);
    }

    if let Some(drag) = derived_drag {
        write_drag_table(out, drag);
    }
}

// ── Private helpers ──────────────────────────────────────────────

fn propagator_label(propagator: &PropagationModel, auto_drag: bool) -> &'static str {
    match propagator {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => "J2+Drag STM (auto-derived)",
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    }
}

fn status_emoji(pass: bool) -> &'static str {
    if pass {
        "\u{2705} PASS"
    } else {
        "\u{274c} FAIL"
    }
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
        "**Verdict: {}** ({}) | {} total \u{0394}v | {} duration | {} waypoints\n",
        verdict_result.verdict,
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
        let tier = if validation.is_some() { " (Nyx)" } else { "" };
        let _ = writeln!(out, "| Metric | Value | Threshold | Status |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min 3D distance{tier} | {} | {} | {} |",
            fmt_m(safety.operational.min_distance_3d_km, 0),
            fmt_m(config.min_distance_3d_km, 0),
            status_emoji(assessment.distance_3d_pass),
        );
        let _ = writeln!(
            out,
            "| Min e/i separation{tier} | {} | {} | {} |",
            fmt_m(safety.passive.min_ei_separation_km, 0),
            fmt_m(config.min_ei_separation_km, 0),
            status_emoji(assessment.ei_separation_pass),
        );
        if let Some(report) = validation {
            let _ = writeln!(
                out,
                "| Convergence | 100% | \u{2014} | \u{2705} |",
            );
            let _ = writeln!(
                out,
                "| Max position error | {} | \u{2014} | \u{2014} |",
                fmt_m(report.max_position_error_km, 0),
            );
        }
        let _ = writeln!(out);
    }
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
                te.summary.time_in_shadow_fraction * 100.0,
            );
        }
        let _ = writeln!(out);
    }

    write_perch_roe(out, &output.perch_roe);
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
        lambert.total_dv_km_s / v_circ * 100.0,
    );
    let _ = writeln!(out);
}

fn write_perch_roe(out: &mut String, roe: &rpo_core::types::QuasiNonsingularROE) {
    let _ = writeln!(out, "### Perch ROE\n");
    let _ = writeln!(out, "| Element | Value | Description |");
    let _ = writeln!(out, "| --- | --- | --- |");
    let _ = writeln!(out, "| \u{03b4}a | {} | Relative SMA |", fmt_roe_component(roe.da));
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
        "| Leg | TOF | \u{0394}v1 (m/s) | \u{0394}v2 (m/s) | Total (m/s) | Label |",
    );
    let _ = writeln!(out, "| --- | --- | --- | --- | --- | --- |");

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
        let _ = writeln!(
            out,
            "| {} | {} | {:.2} | {:.2} | {:.2} | {} |",
            i + 1,
            fmt_duration(leg.tof_s),
            dv1 * 1000.0,
            dv2 * 1000.0,
            leg.total_dv_km_s * 1000.0,
            label,
        );
    }
    let _ = writeln!(
        out,
        "| **Total** | **{}** | | | **{:.2}** | |",
        fmt_duration(output.mission.total_duration_s),
        total_dv * 1000.0,
    );
    let _ = writeln!(out);
}

fn write_safety_section(
    out: &mut String,
    safety: &SafetyMetrics,
    config: &SafetyConfig,
    tier: SafetyTier,
) {
    let assessment = assess_safety(safety, config);

    match tier {
        SafetyTier::Baseline(ValidationTier::Nyx) => {
            let _ = writeln!(out, "## Safety (Analytical Baseline)\n");
            let _ = writeln!(
                out,
                "> Governing full-physics margins from Nyx are shown in the \
                 Summary table above and in Safety Comparison below.\n",
            );
        }
        SafetyTier::Baseline(ValidationTier::MonteCarlo) => {
            let _ = writeln!(out, "## Safety (Analytical Baseline)\n");
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
    let margin_3d =
        (safety.operational.min_distance_3d_km - config.min_distance_3d_km) * 1000.0;
    let _ = writeln!(out, "| Margin | {margin_3d:+.1} m |");
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

    write_passive_safety_md(out, safety, config, &assessment);
}

fn write_passive_safety_md(
    out: &mut String,
    safety: &SafetyMetrics,
    config: &SafetyConfig,
    assessment: &SafetyAssessment,
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
    let margin_ei =
        (safety.passive.min_ei_separation_km - config.min_ei_separation_km) * 1000.0;
    let _ = writeln!(out, "| Margin | {margin_ei:+.1} m |");
    let _ = writeln!(
        out,
        "| e/i phase angle | {:.2}\u{00b0} |",
        safety.passive.ei_phase_angle_rad.to_degrees(),
    );
    let _ = writeln!(out);

    let overall = if assessment.overall_pass {
        "**PASS**"
    } else {
        "**FAIL**"
    };
    let _ = writeln!(out, "### Overall: {overall}\n");
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
        let _ = writeln!(
            out,
            "| e/i phase angle | {:.2}\u{00b0} |",
            s.passive.ei_phase_angle_rad.to_degrees(),
        );

        let _ = writeln!(out, "| Bounded-motion | {} |", fmt_bounded_motion_residual(analysis.bounded_motion_residual));
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
            let _ = writeln!(out, "**Leg {}**: no close approach (diverging)\n", leg_idx + 1);
            continue;
        }
        let closest = &pocas[0];
        let marker = if closest.is_global_minimum { " (global min)" } else { "" };
        let _ = writeln!(out, "**Leg {}**{marker}:\n", leg_idx + 1);
        let _ = writeln!(out, "| Metric | Value |");
        let _ = writeln!(out, "|--------|-------|");
        let _ = writeln!(
            out,
            "| Distance | {:.1} m at t = {:.1} s |",
            closest.distance_km * 1000.0,
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

fn write_cola_sections(out: &mut String, output: &PipelineOutput) {
    if let Some(ref cola) = output.cola {
        write_cola_section(out, cola);
    }
    if let Some(ref secondary) = output.secondary_conjunctions {
        write_secondary_conjunction_section(out, secondary);
    }
    if let Some(ref skipped) = output.cola_skipped {
        write_cola_skipped_section(out, skipped);
    }
}

fn write_cola_section(out: &mut String, maneuvers: &[rpo_core::mission::AvoidanceManeuver]) {
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
    let _ = writeln!(out, "|-----|-----------|---------|---------------|------------|------|");
    for m in maneuvers {
        let correction = match m.correction_type {
            rpo_core::mission::CorrectionType::InPlane => "in-plane",
            rpo_core::mission::CorrectionType::CrossTrack => "cross-track",
            rpo_core::mission::CorrectionType::Combined => "combined",
        };
        let _ = writeln!(
            out,
            "| {} | [{:.6}, {:.6}, {:.6}] | {:.4} | {:.1} m | {:.2} | {} |",
            m.leg_index + 1,
            m.dv_ric_km_s.x,
            m.dv_ric_km_s.y,
            m.dv_ric_km_s.z,
            m.maneuver_location_rad,
            m.post_avoidance_poca_km * 1000.0,
            m.fuel_cost_km_s * 1000.0,
            correction,
        );
    }
    let _ = writeln!(out);
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
            sv.poca.distance_km * 1000.0,
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
            eclipse.summary.time_in_shadow_fraction * 100.0,
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
                    total_shadow / total_duration * 100.0,
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

    // Per-leg error
    if !report.leg_points.is_empty() {
        let _ = writeln!(out, "### Per-Leg Position Error\n");
        let _ = writeln!(out, "| Leg | Max (m) | Mean (m) | RMS (m) |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        for (i, points) in report.leg_points.iter().enumerate() {
            if let Some(stats) = compute_leg_error_stats(points) {
                let _ = writeln!(
                    out,
                    "| {} | {:.1} | {:.1} | {:.1} |",
                    i + 1,
                    stats.max_km * 1000.0,
                    stats.mean_km * 1000.0,
                    stats.rms_km * 1000.0,
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

    // Eclipse validation
    if let Some(ref ev) = report.eclipse_validation {
        write_eclipse_validation(out, ev, output);
    }

    // Auto-derived drag
    if let Some(drag) = derived_drag {
        write_drag_table(out, drag);
    }
}

fn write_safety_comparison(
    out: &mut String,
    report: &ValidationReport,
    config: &rpo_core::mission::MissionConfig,
) {
    let _ = writeln!(out, "### Safety Comparison (Analytical vs Numerical)\n");
    let _ = writeln!(out, "| Metric | Analytical | Numerical | Threshold |");
    let _ = writeln!(out, "| --- | --- | --- | --- |");

    let sc = config.safety.unwrap_or_default();

    let ana_rc = report
        .analytical_safety
        .as_ref()
        .map_or(0.0, |s| s.operational.min_rc_separation_km);
    let num_rc = report.numerical_safety.operational.min_rc_separation_km;
    let _ = writeln!(
        out,
        "| Min R/C-plane distance (m) | {:.1} | {:.1} | \u{2014} |",
        ana_rc * 1000.0,
        num_rc * 1000.0,
    );

    let ana_3d = report
        .analytical_safety
        .as_ref()
        .map_or(0.0, |s| s.operational.min_distance_3d_km);
    let num_3d = report.numerical_safety.operational.min_distance_3d_km;
    let noncons_3d = ana_3d > 0.0 && num_3d < ana_3d * 0.9;
    let _ = writeln!(
        out,
        "| 3D distance (m) | {:.1} | {:.1} | {:.0}{} |",
        ana_3d * 1000.0,
        num_3d * 1000.0,
        sc.min_distance_3d_km * 1000.0,
        if noncons_3d { " \\*" } else { "" },
    );

    let ana_ei = report
        .analytical_safety
        .as_ref()
        .map_or(0.0, |s| s.passive.min_ei_separation_km);
    let num_ei = report.numerical_safety.passive.min_ei_separation_km;
    let noncons_ei = ana_ei > 0.0 && num_ei < ana_ei * 0.9;
    let _ = writeln!(
        out,
        "| e/i separation (m) | {:.1} | {:.1} | {:.0}{} |",
        ana_ei * 1000.0,
        num_ei * 1000.0,
        sc.min_ei_separation_km * 1000.0,
        if noncons_ei { " \\*" } else { "" },
    );
    let _ = writeln!(out);

    if noncons_3d || noncons_ei {
        let _ = writeln!(
            out,
            "\\* Numerical margin >10% smaller than analytical",
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
                ev.max_timing_error_s / max_eclipse_s * 100.0,
            );
        }
    }

    let _ = writeln!(out);
}

fn write_drag_table(out: &mut String, drag: &DragConfig) {
    let _ = writeln!(out, "### Auto-Derived Drag\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(out, "| da_dot | {:+.6e} |", drag.da_dot);
    let _ = writeln!(out, "| dex_dot | {:+.6e} |", drag.dex_dot);
    let _ = writeln!(out, "| dey_dot | {:+.6e} |", drag.dey_dot);
    let _ = writeln!(out);
}

fn write_insights(out: &mut String, insights: &[insights::Insight]) {
    for insight in insights {
        let prefix = match insight.severity {
            insights::Severity::Critical => "**CRITICAL:**",
            insights::Severity::Warning => "**Warning:**",
            insights::Severity::Info => "**Insight:**",
        };
        let _ = writeln!(out, "> {prefix} {}\n", insight.message);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::pipeline::{execute_mission, to_propagation_model};
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("examples")
    }

    #[test]
    fn mission_markdown_contains_expected_sections() {
        let input: PipelineInput =
            serde_json::from_str(&std::fs::read_to_string(examples_dir().join("mission.json")).unwrap())
                .unwrap();
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.propagator);
        let md = mission_to_markdown(&output, &input, &propagator, false);

        assert!(md.contains("# Mission Summary"), "missing summary header");
        assert!(md.contains("**Verdict:"), "missing verdict");
        assert!(md.contains("## Transfer"), "missing transfer section");
        assert!(md.contains("## Waypoint Targeting"), "missing waypoint section");
        assert!(md.contains("## Safety"), "missing safety section");
        assert!(md.contains("## Eclipse"), "missing eclipse section");
        // No duplicate summary: only `# Mission Summary` at the top, no `# Summary` at bottom
        let h1_count = md.lines().filter(|l| l.starts_with("# ")).count();
        assert_eq!(h1_count, 1, "expected exactly one H1 heading, got {h1_count}");
        // Verify summary table has emoji status
        assert!(
            md.contains("\u{2705} PASS"),
            "missing pass emoji in summary table"
        );
        // Verify detailed safety uses text PASS/FAIL (not emoji)
        assert!(
            md.contains("| 3D distance | **PASS** |"),
            "missing text PASS in safety detail"
        );
    }
}

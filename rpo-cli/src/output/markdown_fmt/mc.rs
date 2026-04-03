use std::fmt::Write;

use rpo_core::mission::{assess_safety, EnsembleStatistics, MonteCarloReport, SafetyConfig};
use rpo_core::pipeline::PipelineOutput;
use rpo_core::propagation::DragConfig;

use crate::output::common::{
    cola_dv_summary, determine_mc_verdict, fmt_duration, fmt_m, fmt_m_s, McBaseline, VerdictResult,
};
use crate::output::formation_fmt::write_formation_design_md;
use crate::output::insights;
use crate::output::thresholds::rate as rate_thresh;

use super::helpers::{status_emoji, write_cola_callout, write_drag_table, write_insights};

/// Generate a complete markdown report for the `mc` command.
#[must_use]
pub fn mc_to_markdown(
    output: &PipelineOutput,
    input: &rpo_core::pipeline::PipelineInput,
    report: &MonteCarloReport,
    baseline: &McBaseline,
    derived_drag: Option<&DragConfig>,
) -> String {
    let mut out = String::with_capacity(8192);
    let sc = input.config.safety.unwrap_or_default();
    let stats = &report.statistics;

    let mc_vr = determine_mc_verdict(report);
    write_mc_summary_table(&mut out, report, baseline, &sc, &mc_vr);

    write_cola_callout(&mut out, output);

    if output.safety.cola.as_ref().is_some_and(|c| !c.is_empty()) {
        let _ = writeln!(
            out,
            "> **Note:** MC samples use the baseline (pre-COLA) trajectory. \
             COLA burns are not injected into Monte Carlo dispersions.\n",
        );
    }

    write_mc_baseline_section(&mut out, baseline, output, &sc);
    if derived_drag.is_some() {
        let _ = writeln!(
            out,
            "> Drag-aware targeting; \u{0394}v differs slightly from analytical-only (`mission`) results.\n",
        );
    }

    if let Some(ref fd) = output.formation_design {
        write_formation_design_md(&mut out, fd);
    }

    // MC ensemble statistics
    write_mc_config_section(&mut out, report, baseline);
    write_mc_dv_distribution(&mut out, stats);
    write_mc_operational_safety(&mut out, stats, report);
    write_mc_convergence_and_miss(&mut out, stats, report);
    write_mc_passive_safety(&mut out, stats, report);
    if output.formation_design.is_some() {
        let _ = writeln!(
            out,
            "> See e/i context in Formation Design.\n",
        );
    }

    let insight_lines = insights::mc_insights(report, &sc);
    write_insights(&mut out, &insight_lines);

    write_mc_diagnostics(&mut out, report, derived_drag);

    out
}

// ── MC section helpers ──────────────────────────────────────────

fn write_mc_summary_table(
    out: &mut String,
    report: &MonteCarloReport,
    baseline: &McBaseline,
    config: &SafetyConfig,
    vr: &VerdictResult,
) {
    let stats = &report.statistics;
    let total_dv_km_s = baseline.total_dv_km_s();
    let n = f64::from(report.config.num_samples);

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
        status_emoji(stats.collision_probability <= rate_thresh::ZERO_VIOLATIONS),
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
        if stats.ei_violation_rate > rate_thresh::ZERO_VIOLATIONS {
            "\u{26a0}\u{fe0f} DEGRADED"
        } else {
            "\u{2705} PASS"
        },
    );

    let conv_n = (stats.convergence_rate * n).round();
    let _ = writeln!(
        out,
        "| Convergence | {:.1}% ({:.0}/{}) | \u{2014} | {} |",
        stats.convergence_rate * 100.0,
        conv_n,
        report.config.num_samples,
        status_emoji(
            (stats.convergence_rate - 1.0).abs() < crate::output::thresholds::mc::CONVERGENCE_EXACT_TOL,
        ),
    );
    let _ = writeln!(out);
}

fn write_mc_baseline_section(
    out: &mut String,
    baseline: &McBaseline,
    output: &PipelineOutput,
    config: &SafetyConfig,
) {
    let total_dv_km_s = baseline.total_dv_km_s();
    let classification = if baseline.is_far_field {
        "FAR-FIELD"
    } else {
        "PROXIMITY"
    };

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

    // Analytical safety baseline (collapsed from validate tier)
    let mut figures: Vec<String> = Vec::new();

    if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, config);
        let d3d_status = if assessment.distance_3d_pass {
            "PASS"
        } else {
            "FAIL"
        };
        figures.push(format!(
            "min 3D distance {} ({d3d_status})",
            fmt_m(safety.operational.min_distance_3d_km, 0),
        ));

        let enrichment_active = output.formation_design.is_some();
        let ei_status = if assessment.ei_separation_pass {
            "PASS"
        } else if enrichment_active {
            "below threshold"
        } else {
            "FAIL"
        };
        figures.push(format!(
            "min e/i separation {} ({ei_status})",
            fmt_m(safety.passive.min_ei_separation_km, 1),
        ));
    }

    if let Some((cola_dv, num_burns)) = cola_dv_summary(output.safety.cola.as_deref()) {
        let burn_label = if num_burns == 1 { "burn" } else { "burns" };
        figures.push(format!(
            "COLA {} ({num_burns} {burn_label})",
            fmt_m_s(cola_dv, 2),
        ));
    }

    let _ = writeln!(
        out,
        "> Analytical baseline and formation-design sections match the `validate` tier. \
         Run `validate` for full-physics Nyx results.",
    );
    if !figures.is_empty() {
        let _ = writeln!(out, "> Key figures: {}.", figures.join(", "));
    }
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
            rc.p05 * 1000.0,
            rc.p50 * 1000.0,
            rc.p95 * 1000.0,
        );
        let _ = writeln!(out);
    }

    if let Some(ref d3d) = stats.min_3d_distance_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min 3D distance (m) | {:.1} | {:.1} | {:.1} |",
            d3d.p05 * 1000.0,
            d3d.p50 * 1000.0,
            d3d.p95 * 1000.0,
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
        stats.ei_violation_rate * 100.0,
        ei_n,
        report.config.num_samples,
    );
    let _ = writeln!(out);

    if let Some(ref ei) = stats.min_ei_separation_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min e/i separation (m) | {:.1} | {:.1} | {:.1} |",
            ei.p05 * 1000.0,
            ei.p50 * 1000.0,
            ei.p95 * 1000.0,
        );
        let _ = writeln!(out);
    }

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

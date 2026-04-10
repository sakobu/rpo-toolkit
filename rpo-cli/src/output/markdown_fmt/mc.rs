use std::fmt::Write;

use rpo_core::mission::{assess_safety, EnsembleStatistics, MonteCarloReport, SafetyConfig};
use rpo_core::pipeline::PipelineOutput;
use rpo_core::propagation::DragConfig;

use crate::output::common::{
    cola_dv_summary, determine_mc_verdict, fmt_duration, fmt_m, fmt_m_s, KM_TO_M, McBaseline,
    VerdictResult,
};
use crate::output::formation_fmt::write_formation_design_md;
use crate::output::insights;
use crate::output::thresholds::{insight as insight_thresh, rate as rate_thresh};

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

    let mc_vr = determine_mc_verdict(report, &sc);
    write_mc_summary_table(&mut out, report, baseline, &sc, &mc_vr);

    write_cola_callout(&mut out, output);

    if output.safety.cola.as_ref().is_some_and(|c| !c.is_empty()) {
        let _ = writeln!(
            out,
            "> **Note:** MC samples propagate the baseline (pre-COLA) trajectory \
             \u{2014} COLA burns are NOT injected. The safety statistics below describe the \
             mission *before* avoidance maneuvers are applied. They are the correct view for \
             assessing \"what happens if COLA is skipped or ineffective\", but they do NOT \
             reflect the nominal maneuver schedule shown above (which includes the COLA row). \
             For a full-physics pre- vs post-COLA comparison on the nominal trajectory, run \
             `validate`.\n",
        );
    }

    write_mc_baseline_section(&mut out, baseline, output, &sc);
    // MC uses the MonteCarlo schedule variant (different title, COLA rows
    // carry a `†` reference marker). The footnote below is mc-specific
    // because it explains why the marker is there.
    super::mission::write_maneuver_schedule_with(
        &mut out,
        output,
        super::mission::ScheduleVariant::MonteCarlo,
    );
    let has_cola = output.safety.cola.as_ref().is_some_and(|c| !c.is_empty());
    if has_cola {
        let _ = writeln!(
            out,
            "> \u{2020} COLA row shown for reference only. Monte Carlo samples propagate the \
             pre-COLA baseline trajectory; this burn is NOT executed in the MC dispersions. The \
             safety statistics below therefore describe the mission WITHOUT avoidance \
             maneuvers.\n",
        );
    }
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
        "**Verdict: {}{}** ({}) | {} total \u{0394}v | {} duration | {} waypoints\n",
        vr.verdict,
        vr.qualifier,
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
        stats.collision_probability * insight_thresh::PERCENT_PER_UNIT,
        coll_n,
        report.config.num_samples,
        status_emoji(stats.collision_probability <= rate_thresh::ZERO_VIOLATIONS),
    );

    if let Some(ref d3d) = stats.min_3d_distance_km {
        let _ = writeln!(
            out,
            "| Min 3D distance p05 (analytical) | {} | {} | {} |",
            fmt_m(d3d.p05, 1),
            fmt_m(config.min_distance_3d_km, 0),
            status_emoji(d3d.p05 >= config.min_distance_3d_km),
        );
    }

    let ei_n = (stats.ei_violation_rate * n).round();
    let _ = writeln!(
        out,
        "| e/i violations | {:.1}% ({:.0}/{}) | \u{2014} | {} |",
        stats.ei_violation_rate * insight_thresh::PERCENT_PER_UNIT,
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
        stats.convergence_rate * insight_thresh::PERCENT_PER_UNIT,
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
    // Cross-report interpretability: readers comparing this report's p05
    // against validate.md's Nyx min would otherwise conclude the two disagree.
    // MC runs against the J2+Drag analytical STM, not Nyx full physics.
    let _ = writeln!(
        out,
        "> Propagator: J2+Drag STM (analytical). MC statistics are NOT full-physics Nyx \
         \u{2014} use `validate` for full-physics margins.\n",
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
    let _ = writeln!(out, "| Mean | {:.2} |", dv.mean * KM_TO_M);
    let _ = writeln!(out, "| Std | {:.2} |", dv.std_dev * KM_TO_M);
    let _ = writeln!(out, "| p05 | {:.2} |", dv.p05 * KM_TO_M);
    let _ = writeln!(out, "| p95 | {:.2} |", dv.p95 * KM_TO_M);
    let _ = writeln!(out, "| Min | {:.2} |", dv.min * KM_TO_M);
    let _ = writeln!(out, "| Max | {:.2} |", dv.max * KM_TO_M);
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
        stats.keepout_violation_rate * insight_thresh::PERCENT_PER_UNIT,
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
            rc.p05 * KM_TO_M,
            rc.p50 * KM_TO_M,
            rc.p95 * KM_TO_M,
        );
        let _ = writeln!(out);
    }

    if let Some(ref d3d) = stats.min_3d_distance_km {
        let _ = writeln!(out, "| Metric | p05 | p50 | p95 |");
        let _ = writeln!(out, "| --- | --- | --- | --- |");
        let _ = writeln!(
            out,
            "| Min 3D distance (m) | {:.1} | {:.1} | {:.1} |",
            d3d.p05 * KM_TO_M,
            d3d.p50 * KM_TO_M,
            d3d.p95 * KM_TO_M,
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

    let _ = writeln!(out, "### Passive Safety (Abort-Case e/i, MC Ensemble)\n");
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| e/i violations | {:.1}% ({:.0}/{}) |",
        stats.ei_violation_rate * insight_thresh::PERCENT_PER_UNIT,
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
            ei.p05 * KM_TO_M,
            ei.p50 * KM_TO_M,
            ei.p95 * KM_TO_M,
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
        stats.convergence_rate * insight_thresh::PERCENT_PER_UNIT,
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
                    miss.p50 * KM_TO_M,
                    miss.p95 * KM_TO_M,
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
    // Skip the section entirely when there is nothing to render. After Task 7
    // gated the covariance cross-check on `OpenLoop` mode, closed-loop runs
    // can end up with both fields absent -- emitting an empty "### Diagnostics"
    // heading is a narrative regression.
    if report.covariance_cross_check.is_none() && derived_drag.is_none() {
        return;
    }
    let _ = writeln!(out, "### Diagnostics\n");
    if let Some(ref cov) = report.covariance_cross_check {
        let _ = writeln!(out, "**Covariance cross-check:**\n");
        let _ = writeln!(out, "| Metric | Value |");
        let _ = writeln!(out, "| --- | --- |");
        let _ = writeln!(
            out,
            "| 3\u{03c3} containment | {:.1}% |",
            cov.terminal_3sigma_containment * insight_thresh::PERCENT_PER_UNIT,
        );
        let sr = &cov.sigma_ratio_ric;
        let _ = writeln!(
            out,
            "| Sigma ratio (R / I / C) | {:.2} / {:.2} / {:.2} |",
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        AvoidanceManeuver, ColaConfig, CorrectionType, MonteCarloConfig, MonteCarloMode,
        PercentileStats,
    };
    use rpo_core::pipeline::PipelineInput;
    use rpo_nyx::pipeline::execute_mission;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("examples")
    }

    /// Build a minimal [`MonteCarloReport`] with enough structure for the
    /// `mc_to_markdown` renderer: one sample's worth of ensemble statistics,
    /// closed-loop mode, no covariance cross-check.
    fn make_mc_report() -> MonteCarloReport {
        let stats = EnsembleStatistics {
            total_dv_km_s: PercentileStats::default(),
            min_rc_distance_km: None,
            min_3d_distance_km: None,
            min_ei_separation_km: None,
            waypoint_miss_km: vec![],
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 0.0,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![],
        };
        MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 10,
                dispersions: rpo_core::mission::DispersionConfig::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 10,
            },
            nominal_dv_km_s: 0.008,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 1.0,
            covariance_cross_check: None,
        }
    }

    /// Minimal [`McBaseline`] with arbitrary but realistic-looking scalar values.
    fn make_mc_baseline() -> McBaseline {
        McBaseline {
            lambert_dv_km_s: 0.0,
            lambert_tof_s: 0.0,
            waypoint_dv_km_s: 0.008,
            waypoint_duration_s: 12_600.0,
            num_legs: 3,
            is_far_field: false,
        }
    }

    /// Build a synthetic [`AvoidanceManeuver`] whose `post_avoidance_poca_km`
    /// is deliberately below the target so that the analytical-miss insight
    /// *would* fire if the (now-deleted) `cola_analytical_miss_insights` call
    /// were still present in the MC renderer.
    fn make_subthreshold_maneuver() -> AvoidanceManeuver {
        AvoidanceManeuver {
            epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2026, 1, 1),
            dv_ric_km_s: Vector3::new(0.0, 0.0001, 0.0),
            maneuver_location_rad: 0.0,
            post_avoidance_poca_km: 0.050,
            fuel_cost_km_s: 0.0001,
            correction_type: CorrectionType::InPlane,
            leg_index: 1,
        }
    }

    /// Regression test for Task 3 of the CLI report audit.
    ///
    /// The MC renderer previously appended [`insights::cola_analytical_miss_insights`]
    /// to its insight list whenever both `output.safety.cola` and `input.cola`
    /// were set. That warning is narratively wrong in `mc.md`: MC samples
    /// propagate the pre-COLA baseline, so a warning *about* the analytical
    /// COLA solver belongs in the mission renderer, not here. This test asserts
    /// that the two phrases produced by `cola_analytical_miss_insights` do not
    /// appear in the MC markdown even when the output would otherwise trigger
    /// them.
    #[test]
    fn mc_markdown_does_not_leak_cola_analytical_miss_warning() {
        // Load a real mission input (mission.json already has a `cola` block)
        // and run execute_mission so we get a fully-populated PipelineOutput.
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();

        let mut output = execute_mission(&input).unwrap();

        // Force-inject a sub-threshold COLA maneuver so the analytical-miss
        // insight *would* fire if the renderer still called it. This keeps
        // the test independent of whether mission.json's nominal geometry
        // actually needs avoidance.
        output.safety.cola = Some(vec![make_subthreshold_maneuver()]);

        // Ensure `input.cola.target_distance_km` is comfortably above the
        // injected 0.050 km post-avoidance POCA so the insight is unambiguously
        // triggered (pre-fix).
        input.cola = Some(ColaConfig {
            target_distance_km: 0.300,
            max_dv_km_s: 0.010,
        });

        let report = make_mc_report();
        let baseline = make_mc_baseline();

        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            !md.contains("COLA burn on leg"),
            "MC report must not emit the analytical COLA miss warning \u{2014} \
             MC never applies COLA to samples. Got:\n{md}",
        );
        assert!(
            !md.contains("solver failed before full-physics validation"),
            "MC report must not emit the analytical solver-failure phrase. Got:\n{md}",
        );
    }

    // ── Report-wording regression tests ─────────────────────────────
    //
    // These lock the load-bearing phrasing from the 2026-04-10 CLI report
    // wording cleanup (mc.md findings).

    /// Build a `SafetyMetrics` fixture with the given 3D distance and e/i
    /// separation. Used by the wording regression tests below to synthesize
    /// `nominal_safety` values that trigger the nominal-vs-dispersion
    /// branching in `determine_mc_verdict`.
    fn make_safety_fixture(
        min_3d_km: f64,
        min_ei_km: f64,
    ) -> rpo_core::mission::SafetyMetrics {
        rpo_core::mission::SafetyMetrics {
            operational: rpo_core::mission::OperationalSafety {
                min_rc_separation_km: min_3d_km,
                min_distance_3d_km: min_3d_km,
                min_rc_leg_index: 0,
                min_rc_elapsed_s: 0.0,
                min_rc_ric_position_km: Vector3::zeros(),
                min_3d_leg_index: 0,
                min_3d_elapsed_s: 0.0,
                min_3d_ric_position_km: Vector3::zeros(),
            },
            passive: rpo_core::mission::PassiveSafety {
                min_ei_separation_km: min_ei_km,
                de_magnitude: 0.0,
                di_magnitude: 0.0,
                ei_phase_angle_rad: 0.0,
            },
        }
    }

    /// Build a [`MonteCarloReport`] fixture with a failing nominal e/i, the
    /// 100% violation-rate ensemble, and waypoint miss medians that match
    /// the real audit dataset (WP1 = 90 m p50, WP3 = 381 m p50, 1379 m p95).
    fn mc_report_with_nominal_failure() -> MonteCarloReport {
        let stats = EnsembleStatistics {
            total_dv_km_s: PercentileStats {
                p05: 0.00811,
                p50: 0.00830,
                p95: 0.00854,
                mean: 0.00831,
                std_dev: 0.00014,
                min: 0.00806,
                max: 0.00877,
                ..PercentileStats::default()
            },
            min_rc_distance_km: Some(PercentileStats {
                p05: 0.0155,
                p50: 0.0789,
                p95: 0.1296,
                ..PercentileStats::default()
            }),
            min_3d_distance_km: Some(PercentileStats {
                p05: 0.1316,
                p50: 0.2367,
                p95: 0.5285,
                ..PercentileStats::default()
            }),
            min_ei_separation_km: Some(PercentileStats {
                p05: 0.0,
                p50: 0.0068,
                p95: 0.0320,
                ..PercentileStats::default()
            }),
            waypoint_miss_km: vec![
                Some(PercentileStats { p05: 0.0, p50: 0.0895, p95: 0.2317, ..PercentileStats::default() }),
                Some(PercentileStats { p05: 0.0, p50: 0.1634, p95: 0.5286, ..PercentileStats::default() }),
                Some(PercentileStats { p05: 0.0, p50: 0.3807, p95: 1.3787, ..PercentileStats::default() }),
            ],
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 1.0,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![],
        };
        MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 100,
                dispersions: rpo_core::mission::DispersionConfig::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.00831,
            // Nominal e/i = 2.2 m → already fails the 100 m threshold.
            nominal_safety: Some(make_safety_fixture(0.188, 0.0022)),
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 71.0,
            covariance_cross_check: None,
        }
    }

    fn mc_baseline_realistic() -> McBaseline {
        McBaseline {
            lambert_dv_km_s: 0.5432,
            lambert_tof_s: 3600.0,
            waypoint_dv_km_s: 0.0082,
            waypoint_duration_s: 12_600.0,
            num_legs: 3,
            is_far_field: true,
        }
    }

    /// Set up an `(input, output)` pair based on `examples/mission.json`
    /// with a COLA maneuver force-injected so the Note block and `†`
    /// marker fire. Safety config carries the 100 m e/i threshold.
    fn mission_with_cola_injected() -> (PipelineInput, rpo_core::pipeline::PipelineOutput) {
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();
        input.cola = Some(ColaConfig {
            target_distance_km: 0.300,
            max_dv_km_s: 0.010,
        });
        let mut output = execute_mission(&input).unwrap();
        // Force-inject a COLA maneuver regardless of whether the nominal
        // geometry triggers one, so the Note block and `†` marker always
        // fire in these wording tests.
        output.safety.cola = Some(vec![AvoidanceManeuver {
            epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2024, 1, 1),
            dv_ric_km_s: Vector3::new(-0.000_688, 0.0, 0.000_468),
            maneuver_location_rad: 1.79,
            post_avoidance_poca_km: 0.281,
            fuel_cost_km_s: 0.000_83,
            correction_type: CorrectionType::Combined,
            leg_index: 2,
        }]);
        (input, output)
    }

    #[test]
    fn mc_report_verdict_attributes_nominal_failure_not_dispersion() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("consistent with the 2.2 m nominal"),
            "MC verdict must attribute e/i failure to the nominal; got:\n{md}"
        );
        assert!(
            !md.contains("degraded under dispersion"),
            "MC verdict must not blame dispersion when the nominal already fails; got:\n{md}"
        );
        assert!(
            md.contains("MC ensemble inherits this"),
            "e/i insight must name nominal inheritance; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_uses_waypoint_miss_dispersion_metric_name() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("waypoint-miss dispersion"),
            "MC verdict must use 'waypoint-miss dispersion' metric name; got:\n{md}"
        );
        assert!(
            !md.contains("position error growth"),
            "legacy 'position error growth' phrasing must not return; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_renames_schedule_header_and_marks_cola_row() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("## Nominal Maneuver Schedule (reference)"),
            "Maneuver Schedule header must be renamed in mc.md; got:\n{md}"
        );
        assert!(
            md.contains("COLA (L3) \u{2020}"),
            "COLA row must carry the `†` reference marker; got:\n{md}"
        );
        assert!(
            md.contains("COLA row shown for reference only"),
            "mc.md must carry the COLA footnote; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_note_block_strengthens_wording() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("COLA burns are NOT injected"),
            "Note block must emphasize NOT injected; got:\n{md}"
        );
        assert!(
            !md.contains("treat MC safety margins as **optimistic**"),
            "legacy 'optimistic' framing must not return; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_config_header_carries_propagator_subtitle() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("Propagator: J2+Drag STM (analytical)"),
            "MC config header must carry the propagator subtitle; got:\n{md}"
        );
        assert!(
            md.contains("Min 3D distance p05 (analytical)"),
            "MC summary row must label the propagator; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_passive_safety_header_scoped_to_ensemble_abort_case() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            md.contains("### Passive Safety (Abort-Case e/i, MC Ensemble)"),
            "MC passive safety header must be scoped to abort-case ensemble; got:\n{md}"
        );
    }

    #[test]
    fn mc_report_emits_final_waypoint_p95_over_one_km_insight() {
        let (input, output) = mission_with_cola_injected();
        let report = mc_report_with_nominal_failure();
        let baseline = mc_baseline_realistic();
        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        // WP3 p95 = 1.3787 km → triggers the new insight.
        assert!(
            md.contains("WP3 p95 miss is 1.38 km"),
            "mc.md must emit the 'final waypoint p95 > 1 km' insight; got:\n{md}"
        );
        assert!(
            md.contains("re-targeting gate before the WP3 arrival burn"),
            "insight must suggest a re-targeting gate; got:\n{md}"
        );
    }

    /// Regression test for Fix 1 of the final CLI report audit polish pass.
    ///
    /// After Task 7 gated the covariance cross-check on `MonteCarloMode::OpenLoop`,
    /// closed-loop runs started producing `report.covariance_cross_check = None`.
    /// The `write_mc_diagnostics` helper still unconditionally emitted
    /// `### Diagnostics\n`, so closed-loop reports with no derived drag ended up
    /// with a dangling empty heading right before whatever came next (or the end
    /// of the document).
    ///
    /// This test builds a fully synthetic closed-loop fixture via
    /// [`make_mc_report`] and [`make_mc_baseline`], passes `derived_drag = None`,
    /// and asserts the Diagnostics section is omitted entirely.
    #[test]
    fn mc_markdown_has_no_empty_diagnostics_heading_in_closed_loop() {
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mc.json")).unwrap(),
        )
        .unwrap();
        // Strip any COLA config so the renderer doesn't inject unrelated callouts.
        input.cola = None;

        let mut output = execute_mission(&input).unwrap();
        output.safety.cola = None;

        let report = make_mc_report(); // closed-loop, covariance_cross_check = None
        let baseline = make_mc_baseline();

        let md = mc_to_markdown(&output, &input, &report, &baseline, None);

        assert!(
            !md.contains("### Diagnostics\n\n###"),
            "closed-loop mc.md must not have a dangling empty Diagnostics heading \
             followed immediately by another sub-heading; got:\n{md}",
        );
        assert!(
            !md.contains("### Diagnostics"),
            "Diagnostics section must be omitted entirely when both the covariance \
             cross-check and the derived-drag table are absent; got:\n{md}",
        );
    }
}

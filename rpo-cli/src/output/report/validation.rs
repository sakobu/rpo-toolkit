//! Validation report generation.

use std::fmt::Write;

use rpo_core::mission::{assess_safety, ValidationReport};
use rpo_core::pipeline::{PipelineInput, PipelineOutput};

use crate::output::fmt::{fmt_m, fmt_m_s, KM_TO_M};
use crate::output::insights;
use crate::output::verdict::{determine_verdict, SafetyTier};

use super::helpers::{
    emit_alert_box, write_insights, ReportContext,
};
use super::sections::{
    cola, configuration, eclipse, formation::write_formation_design_condensed_md, free_drift,
    recommendations, safety, schedule, summary, transfer, waypoints,
};

/// Validation-specific parameters for markdown formatting.
pub struct ValidationContext {
    /// Whether drag was auto-derived from spacecraft properties.
    pub auto_drag: bool,
    /// Number of nyx sample points per leg.
    pub samples_per_leg: u32,
}

/// Generate a complete markdown report for the `validate` command.
#[must_use]
pub fn validation_to_markdown(
    output: &PipelineOutput,
    input: &PipelineInput,
    report: &ValidationReport,
    ctx: &ValidationContext,
) -> String {
    let mut out = String::with_capacity(8192);

    let sc = input.base.config.safety.unwrap_or_default();
    let enrichment_active = output.formation_design.is_some();

    let vr = determine_verdict(output, &sc, Some(report));
    summary::write_summary_block_mission(&mut out, &vr, output, &sc, Some(report));

    // Compute insights once, then partition: top-of-report alert gets the most
    // urgent items, bottom insights list gets the rest. No insight is shown twice.
    let mut insight_lines = insights::validation_insights(report, &sc);
    if let (Some(cola_maneuvers), Some(cola_config)) = (&output.safety.cola, &input.base.cola) {
        insight_lines.extend(insights::cola_analytical_miss_insights(
            cola_maneuvers,
            cola_config.target_distance_km,
        ));
    }
    let bottom_items = emit_alert_box(&mut out, insight_lines.clone());

    let prop_label = configuration::propagator_label(input, output, ctx.auto_drag);
    configuration::write_configuration_section(&mut out, input, output, prop_label);
    let _ = writeln!(
        out,
        "> \u{0394}v values below reflect the analytical targeting plan ({prop_label}). \
         Nyx full-physics validation governs safety margins \u{2014} \
         it does not recompute maneuvers.\n",
    );

    transfer::write_transfer_section(&mut out, output, input);

    waypoints::write_waypoint_section(&mut out, output, input);

    schedule::write_maneuver_schedule(&mut out, output);

    if let Some(ref safety_metrics) = output.mission.safety {
        let assessment = assess_safety(safety_metrics, &sc);
        let outcome = safety::passive_safety_outcome(&assessment, enrichment_active, &vr);
        safety::write_safety_section(
            &mut out,
            safety_metrics,
            &sc,
            SafetyTier::Baseline,
            outcome,
        );
    }

    if let Some(ref fd) = output.formation_design {
        write_formation_design_condensed_md(&mut out, fd);
    }

    if let Some(ref poca) = output.safety.poca {
        free_drift::write_poca_section(&mut out, "Closest Approach (Brent-refined)", poca);
    }

    if let Some(ref fd) = output.safety.free_drift {
        free_drift::write_free_drift_section(&mut out, fd, &sc);
    }

    if let Some(ref fd_poca) = output.safety.free_drift_poca {
        free_drift::write_poca_section(
            &mut out,
            "Free-Drift Closest Approach (Brent-refined)",
            fd_poca,
        );
    }

    cola::write_cola_sections(&mut out, output, input, ReportContext::Validate);

    eclipse::write_eclipse_section(&mut out, output);
    write_validation_section(
        &mut out,
        output,
        input,
        report,
        ctx.samples_per_leg,
    );

    // Covariance propagation is intentionally NOT rendered in validate.md.
    // The validate tier's central claim is "analytical is non-conservative vs
    // Nyx" (see the Safety Comparison and bias callouts). Rendering an
    // analytical-STM covariance prediction in a report that just finished
    // telling the reader not to trust the analytical STM is a category
    // mismatch. The covariance section lives only in mc.md.

    // Bottom insights: items NOT already surfaced in the alert box.
    // Recommendations get the full list so they can reason about every finding.
    write_insights(&mut out, &bottom_items);

    recommendations::write_validate_recommendations(&mut out, output, &insight_lines);

    out
}

// ── Validation-only section ─────────────────────────────────────────

/// Write the Nyx validation section (position error, per-leg error, spacecraft, safety comparison, COLA detail, eclipse).
fn write_validation_section(
    out: &mut String,
    output: &PipelineOutput,
    input: &PipelineInput,
    report: &ValidationReport,
    samples_per_leg: u32,
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
    safety::write_safety_comparison(out, report, &input.base.config);

    // COLA validation detail (post-validation section, before eclipse)
    cola::write_cola_validation_detail(out, output, report);

    // Eclipse validation
    if let Some(ref ev) = report.eclipse_validation {
        eclipse::write_eclipse_validation(out, ev, output);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        MissionConfig, OperationalSafety, PassiveSafety, SafetyConfig, SafetyMetrics,
        ValidationReport,
    };
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
        safety::write_safety_comparison(&mut md, &report, &config);

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
        safety::write_safety_comparison(&mut md, &report, &config);

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

    /// Build a synthetic [`ValidationReport`] with a tight-margin geometry:
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
    fn load_example_mission_with_enrichment() -> (PipelineInput, PipelineOutput) {
        let mut input: PipelineInput = serde_json::from_str(
            &std::fs::read_to_string(examples_dir().join("mission.json")).unwrap(),
        )
        .unwrap();
        input.base.safety_requirements = Some(rpo_core::mission::SafetyRequirements {
            min_separation_km: 0.100,
            alignment: rpo_core::mission::EiAlignment::default(),
        });
        let output = execute_mission(&input).unwrap();
        (input, output)
    }

    #[test]
    fn validate_report_emits_at_margin_qualifier_on_tight_margin_and_cola_miss() {
        let (input, output) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ true);
        let ctx = ValidationContext {
            auto_drag: true,
            samples_per_leg: 50,
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
        let (input, output) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ false);
        let ctx = ValidationContext {
            auto_drag: true,
            samples_per_leg: 50,
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
        let (input, output) = load_example_mission_with_enrichment();
        let report = synthetic_validation_report(/* tight */ true, /* cola_miss */ true);
        let ctx = ValidationContext {
            auto_drag: true,
            samples_per_leg: 50,
        };
        let md = validation_to_markdown(&output, &input, &report, &ctx);

        assert!(
            md.contains("pre-COLA and post-COLA min 3D are identical"),
            "Safety Comparison must explain pre/post equality; got:\n{md}"
        );
    }
}

//! Mission report generation.

use rpo_core::mission::assess_safety;
use rpo_core::pipeline::{PipelineInput, PipelineOutput};
use rpo_core::propagation::PropagationModel;

use crate::output::verdict::{determine_verdict, SafetyTier};

use super::helpers::ReportContext;
use super::sections::{
    cola, eclipse, formation::write_formation_design_md, free_drift, safety, schedule, summary,
    transfer, waypoints,
};

/// Generate a complete markdown report for the `mission` command.
#[must_use]
pub fn mission_to_markdown(
    output: &PipelineOutput,
    input: &PipelineInput,
    propagator: &PropagationModel,
    auto_drag: bool,
) -> String {
    let mut out = String::with_capacity(4096);

    let sc = input.base.config.safety.unwrap_or_default();
    let enrichment_active = output.formation_design.is_some();

    // Summary block at top
    let vr = determine_verdict(output, &sc, None);
    summary::write_summary_block_mission(&mut out, &vr, output, &sc, None);

    transfer::write_transfer_section(&mut out, output, input);

    waypoints::write_waypoint_section(&mut out, output, input, propagator, auto_drag);

    schedule::write_maneuver_schedule(&mut out, output);

    if let Some(ref safety_metrics) = output.mission.safety {
        let assessment = assess_safety(safety_metrics, &sc);
        let outcome = safety::passive_safety_outcome(&assessment, enrichment_active, &vr);
        safety::write_safety_section(
            &mut out,
            safety_metrics,
            &sc,
            SafetyTier::Governing,
            outcome,
        );
    }

    if let Some(ref fd) = output.formation_design {
        write_formation_design_md(&mut out, fd);
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

    cola::write_cola_sections(&mut out, output, input, ReportContext::Mission);

    eclipse::write_eclipse_section(&mut out, output);

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use rpo_core::pipeline::to_propagation_model;
    use rpo_nyx::pipeline::execute_mission;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("examples")
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
        let propagator = to_propagation_model(&input.base.propagator);
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
        let propagator = to_propagation_model(&input.base.propagator);
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
        input.base.safety_requirements = Some(rpo_core::mission::SafetyRequirements {
            min_separation_km: 0.100,
            alignment: rpo_core::mission::EiAlignment::default(),
        });
        let output = execute_mission(&input).unwrap();
        let propagator = to_propagation_model(&input.base.propagator);
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
}

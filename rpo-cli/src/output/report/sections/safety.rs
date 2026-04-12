//! Safety section writers (operational, passive, and safety comparison).

use std::fmt::Write;

use rpo_core::mission::{
    assess_safety, RcContext, SafetyAssessment, SafetyConfig, SafetyMetrics, ValidationReport,
};

use crate::output::fmt::{fmt_m, oxford_join, KM_TO_M};
use crate::output::thresholds::{insight as insight_thresh, safety as safety_thresh};
use crate::output::verdict::{
    analytical_overestimate, margin_or_shortfall_row, margin_ratio, SafetyTier, VerdictResult,
};

/// How to render the overall passive safety result.
///
/// Determined from the verdict + enrichment state before entering the safety
/// section. Replaces two interdependent boolean flags (`enrichment_active`,
/// `verdict_feasible`) with an explicit enum per Codebase "enums over boolean
/// flags" rule.
#[derive(Debug, Clone, Copy)]
pub(crate) enum PassiveSafetyOutcome {
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
/// [`crate::output::fmt::oxford_join`] so this renderer stays in sync
/// with the verdict-reason leg enumeration in `verdict.rs`.
pub(crate) fn format_leg_list(legs: &[usize]) -> String {
    match legs {
        [] => String::new(),
        [single] => format!("leg {single}"),
        multi => {
            let numbers: Vec<String> = multi.iter().map(ToString::to_string).collect();
            format!("legs {}", oxford_join(&numbers))
        }
    }
}

/// Determine the passive safety rendering outcome from the assessment and context.
pub(crate) fn passive_safety_outcome(
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

/// Write the Safety section (operational + passive sub-sections).
pub(crate) fn write_safety_section(
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
        crate::output::fmt::fmt_duration(safety.operational.min_3d_elapsed_s),
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
        crate::output::fmt::fmt_duration(safety.operational.min_rc_elapsed_s),
    );
    let _ = writeln!(
        out,
        "| \u{2014} RIC | [{:.4}, {:.4}, {:.4}] km |",
        rc_ric[0], rc_ric[1], rc_ric[2],
    );
    let _ = writeln!(out);

    write_passive_safety_md(out, safety, config, &assessment, outcome);
}

/// Write the Passive Safety sub-section.
pub(crate) fn write_passive_safety_md(
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

// ── Safety comparison (validation-only) ─────────────────────────────

/// Safety comparison numbers (analytical + numerical) in a single bundle.
///
/// Avoids threading seven floats through helper function signatures. The
/// `_km` suffixes are kept on every field per the project's mandatory unit
/// naming rules (see `Codebase#naming-rules`), which means clippy's
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

/// Write the Safety Comparison section (analytical vs numerical).
pub(crate) fn write_safety_comparison(
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

//! Free-drift and POCA section writers.

use std::fmt::Write;

use rpo_core::mission::{assess_safety, FreeDriftAnalysis, RcContext, SafetyConfig};

use crate::output::fmt::{fmt_bounded_motion_residual, fmt_m, fmt_ric_position, KM_TO_M};
use crate::output::thresholds::safety as safety_thresh;

/// Write the Free-Drift Safety section (abort case).
pub(crate) fn write_free_drift_section(
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
        if s.passive.min_ei_separation_km * KM_TO_M >= safety_thresh::MIN_EI_SEPARATION_FOR_PHASE_DISPLAY_M {
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

/// Write the POCA (closest approach) section.
pub(crate) fn write_poca_section(
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
            "| Position (RIC) | {} |",
            fmt_ric_position(&closest.position_ric_km),
        );
        let _ = writeln!(out);
    }
}

//! Covariance Propagation section writer.
//!
//! Renders the `MissionCovarianceReport` that the pipeline computes whenever
//! `navigation_accuracy` is present in the input. Linear (Φ P Φᵀ) prediction
//! of how navigation uncertainty + maneuver execution error grow across legs.
//!
//! Computed by `rpo-core::propagation::covariance::propagate` (see
//! `rpo-core/src/propagation/covariance/propagate.rs`). The underlying
//! 6×6 / 9×9 state transition matrices follow Koenig Eq. A6 and
//! Appendix D (Koenig, Guffanti, D'Amico 2017, JGCD). The Mahalanobis
//! distance in this section follows the standard definition
//! `d² = xᵀ P⁻¹ x` with `x` the RIC-projected nominal offset.
//!
//! Complements the Monte Carlo ensemble, which shows empirical dispersion.

use std::fmt::Write;

use rpo_core::propagation::{MissionCovarianceReport, NavigationAccuracy};

use crate::output::fmt::{fmt_distance_auto, fmt_smart_f64, KM_TO_M, MM_PER_KM};
use crate::output::thresholds::covariance as cov_thresh;

/// Write the `## Covariance Propagation` section.
pub(crate) fn write_covariance_section(out: &mut String, report: &MissionCovarianceReport) {
    let _ = writeln!(out, "## Covariance Propagation\n");

    // Top-level metrics
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(
        out,
        "| Navigation accuracy (R/I/C) | {} |",
        fmt_navigation_accuracy(&report.navigation_accuracy),
    );
    if let Some(ref unc) = report.maneuver_uncertainty {
        let _ = writeln!(
            out,
            "| Maneuver uncertainty | {:.1}% magnitude, {:.2}\u{00b0} pointing |",
            unc.magnitude_sigma * 100.0,
            unc.pointing_sigma_rad.to_degrees(),
        );
    }
    let _ = writeln!(
        out,
        "| Mission-wide max 3\u{03c3} position | {} |",
        fmt_distance_auto(report.max_sigma3_position_km),
    );
    let term_sig = &report.terminal_sigma3_position_ric_km;
    let _ = writeln!(
        out,
        "| Terminal 3\u{03c3} position (R/I/C) | {} / {} / {} |",
        fmt_distance_auto(term_sig[0]),
        fmt_distance_auto(term_sig[1]),
        fmt_distance_auto(term_sig[2]),
    );
    let term_pos = &report.terminal_position_ric_km;
    let _ = writeln!(
        out,
        "| Terminal nominal position (R/I/C) | [{:.2}, {:.2}, {:.2}] km |",
        term_pos[0], term_pos[1], term_pos[2],
    );
    let _ = writeln!(
        out,
        "| Min Mahalanobis distance | {:.2} ({}) |",
        report.min_mahalanobis_distance,
        mahalanobis_interpretation(report.min_mahalanobis_distance),
    );
    let _ = writeln!(out);

    // Per-leg growth table
    if !report.legs.is_empty() {
        let _ = writeln!(out, "### Per-Leg Growth\n");
        let _ = writeln!(out, "| Leg | Max 3\u{03c3} position | Min Mahalanobis |");
        let _ = writeln!(out, "| --- | --- | --- |");
        for (i, leg) in report.legs.iter().enumerate() {
            let _ = writeln!(
                out,
                "| {} | {} | {:.2} |",
                i + 1,
                fmt_distance_auto(leg.max_sigma3_position_km),
                leg.min_mahalanobis_distance,
            );
        }
        let _ = writeln!(out);
    }

    // Open-loop fragility warning: when the terminal 3σ bound on any axis
    // exceeds the nominal position on that axis, the deputy's 3σ uncertainty
    // box extends past where the deputy is supposed to be. For closed-loop
    // operations that's tolerable (retargeting pulls things back); for
    // open-loop operations it means the plan breaks under the stated
    // uncertainty. Make the operational consequence explicit so an FDO
    // doesn't have to compute the ratio in their head.
    if let Some(open_loop_warning) = open_loop_fragility_warning(report) {
        let _ = writeln!(out, "{open_loop_warning}\n");
    }

    let _ = writeln!(
        out,
        "> Linear covariance propagation (\u{03a6} P \u{03a6}\u{1d40}) through the nominal \
         trajectory. Reflects how navigation uncertainty and maneuver execution errors grow \
         across legs. Compare the terminal 3\u{03c3} bounds against the Monte Carlo dispersion \
         envelope (if available) for empirical agreement.\n",
    );
}

/// Return an open-loop-fragility warning blockquote when any axis's terminal
/// 3σ bound exceeds the nominal position magnitude on that axis. Returns
/// `None` when the terminal 3σ is contained by the nominal on every axis.
///
/// # Invariants
/// - Axes with `|nominal| < cov_thresh::NEAR_ZERO_NOMINAL_KM` are skipped:
///   `3σ / 0` is undefined and 1 mm is 1 ppm of a 1 km separation, so
///   "3σ exceeds near-zero nominal" is roundoff noise, not a warning.
/// - The firing boundary is strict `>`: a ratio of exactly 1.0 does NOT
///   fire (the 3σ bound equals the nominal magnitude — borderline).
/// - Reports the single worst-offending axis only; a multi-axis overrun
///   is implied by being part of the same pattern.
/// - Returns `None` when every axis either passes the ratio test or has a
///   near-zero nominal.
fn open_loop_fragility_warning(report: &MissionCovarianceReport) -> Option<String> {
    let axis_labels = ["radial", "in-track", "cross-track"];
    let term_sig = &report.terminal_sigma3_position_ric_km;
    let term_pos = &report.terminal_position_ric_km;

    let mut worst: Option<(usize, f64)> = None;
    for i in 0..3 {
        let nominal = term_pos[i].abs();
        if nominal < cov_thresh::NEAR_ZERO_NOMINAL_KM {
            continue;
        }
        let ratio = term_sig[i] / nominal;
        if ratio > cov_thresh::OPEN_LOOP_FRAGILITY_RATIO
            && worst.is_none_or(|(_, r)| ratio > r)
        {
            worst = Some((i, ratio));
        }
    }

    let (axis_idx, ratio) = worst?;
    let sigma = fmt_distance_auto(term_sig[axis_idx]);
    let nominal = fmt_distance_auto(term_pos[axis_idx].abs());
    Some(format!(
        "> \u{26a0}\u{fe0f} **Open-loop fragility:** terminal 3\u{03c3} {axis} ({sigma}) \
         exceeds the nominal {axis} position ({nominal}) by {ratio:.1}\u{00d7}. Open-loop \
         operations under the stated navigation accuracy would place the deputy well beyond \
         the nominal trajectory on this axis. Closed-loop retargeting is required to \
         constrain the dispersion.",
        axis = axis_labels[axis_idx],
    ))
}

/// Format navigation accuracy as "position X / Y / Z m; velocity a / b / c mm/s".
///
/// Uses smart precision (no trailing `.0`) so default whole-number magnitudes
/// like `100 mm/s` don't render as noisy `100.0 mm/s` triples.
fn fmt_navigation_accuracy(nav: &NavigationAccuracy) -> String {
    let pr = fmt_smart_f64(nav.position_sigma_ric_km[0] * KM_TO_M, 1);
    let pi = fmt_smart_f64(nav.position_sigma_ric_km[1] * KM_TO_M, 1);
    let pc = fmt_smart_f64(nav.position_sigma_ric_km[2] * KM_TO_M, 1);
    let vr = fmt_smart_f64(nav.velocity_sigma_ric_km_s[0] * MM_PER_KM, 1);
    let vi = fmt_smart_f64(nav.velocity_sigma_ric_km_s[1] * MM_PER_KM, 1);
    let vc = fmt_smart_f64(nav.velocity_sigma_ric_km_s[2] * MM_PER_KM, 1);
    format!(
        "position {pr} / {pi} / {pc} m; velocity {vr} / {vi} / {vc} mm/s",
    )
}

/// One-line interpretation of a Mahalanobis-distance scalar.
///
/// Flight-dynamics operators read "Mahalanobis distance" as "how many sigmas
/// of separation does the deputy have from the chief in covariance space".
/// A bare `0.73` is opaque; making the operational consequence explicit is
/// not. Specifically, `< 1σ` means the chief lies INSIDE the deputy's 1σ
/// uncertainty ellipsoid — a collision-risk-adjacent state that an operator
/// should see spelled out, not have to derive.
///
/// Tier boundaries follow the standard χ² with 3 DOF interpretation
/// (3σ ≈ 97.07% containment under a zero-mean Gaussian).
///
/// # Arguments
/// - `distance` — unitless Mahalanobis scalar, assumed `>= 0`.
///
/// # Invariants
/// - Boundary at exactly `cov_thresh::MAHALANOBIS_INSIDE_1SIGMA` (1.0): the
///   `< 1.0` branch is NOT taken, so the mid tier owns the point.
/// - Boundary at exactly `cov_thresh::MAHALANOBIS_INSIDE_3SIGMA` (3.0): the
///   `< 3.0` branch is NOT taken, so the well-separated tier owns the point.
fn mahalanobis_interpretation(distance: f64) -> String {
    if distance < cov_thresh::MAHALANOBIS_INSIDE_1SIGMA {
        "\u{26a0}\u{fe0f} chief is inside the deputy's 1\u{03c3} uncertainty ellipsoid at closest"
            .to_string()
    } else if distance < cov_thresh::MAHALANOBIS_INSIDE_3SIGMA {
        format!(
            "deputy is {distance:.1}\u{03c3} from chief at closest \
             (chief within 3\u{03c3} ellipsoid)"
        )
    } else {
        format!("deputy is well-separated ({distance:.1}\u{03c3} from chief)")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{SMatrix, Vector3};
    use rpo_core::propagation::{CovarianceState, LegCovarianceReport};
    use rpo_core::types::Matrix6;

    fn stub_state() -> CovarianceState {
        CovarianceState {
            epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2024, 1, 1),
            elapsed_s: 0.0,
            covariance_roe: Matrix6::zeros(),
            covariance_ric_position_km2: SMatrix::<f64, 3, 3>::zeros(),
            sigma3_position_ric_km: Vector3::zeros(),
            mahalanobis_distance: 0.0,
        }
    }

    fn stub_report() -> MissionCovarianceReport {
        MissionCovarianceReport {
            legs: vec![
                LegCovarianceReport {
                    states: vec![stub_state()],
                    max_sigma3_position_km: 5.2,
                    min_mahalanobis_distance: 1.2,
                },
                LegCovarianceReport {
                    states: vec![stub_state()],
                    max_sigma3_position_km: 12.8,
                    min_mahalanobis_distance: 0.9,
                },
                LegCovarianceReport {
                    states: vec![stub_state()],
                    max_sigma3_position_km: 26.08,
                    min_mahalanobis_distance: 0.73,
                },
            ],
            navigation_accuracy: NavigationAccuracy {
                position_sigma_ric_km: Vector3::new(0.1, 0.1, 0.1),
                velocity_sigma_ric_km_s: Vector3::new(0.0001, 0.0001, 0.0001),
            },
            maneuver_uncertainty: Some(rpo_core::propagation::ManeuverUncertainty {
                magnitude_sigma: 0.01,
                pointing_sigma_rad: 0.01745,
            }),
            max_sigma3_position_km: 26.08,
            min_mahalanobis_distance: 0.73,
            terminal_position_ric_km: Vector3::new(0.51, 4.98, 0.50),
            terminal_sigma3_position_ric_km: Vector3::new(1.47, 26.08, 0.31),
        }
    }

    #[test]
    fn write_covariance_section_has_expected_headings() {
        let report = stub_report();
        let mut out = String::new();
        write_covariance_section(&mut out, &report);

        assert!(
            out.contains("## Covariance Propagation"),
            "missing top-level heading; got:\n{out}",
        );
        assert!(
            out.contains("### Per-Leg Growth"),
            "missing per-leg growth heading; got:\n{out}",
        );
        assert!(
            out.contains("Navigation accuracy (R/I/C)"),
            "missing navigation accuracy row; got:\n{out}",
        );
        assert!(
            out.contains("Maneuver uncertainty"),
            "missing maneuver uncertainty row; got:\n{out}",
        );
    }

    #[test]
    fn write_covariance_section_renders_per_leg_rows() {
        let report = stub_report();
        let mut out = String::new();
        write_covariance_section(&mut out, &report);

        // All three legs with their rounded 3σ values
        assert!(out.contains("| 1 | 5.20 km | 1.20 |"), "leg 1 row missing; got:\n{out}");
        assert!(out.contains("| 2 | 12.80 km | 0.90 |"), "leg 2 row missing; got:\n{out}");
        assert!(out.contains("| 3 | 26.08 km | 0.73 |"), "leg 3 row missing; got:\n{out}");
    }

    #[test]
    fn write_covariance_section_renders_mission_wide_metrics() {
        let report = stub_report();
        let mut out = String::new();
        write_covariance_section(&mut out, &report);

        assert!(
            out.contains("Mission-wide max 3\u{03c3} position | 26.08 km"),
            "mission-wide max 3σ missing; got:\n{out}",
        );
        assert!(
            out.contains("Min Mahalanobis distance | 0.73"),
            "min Mahalanobis missing; got:\n{out}",
        );
        assert!(
            out.contains("chief is inside the deputy's 1\u{03c3}"),
            "Mahalanobis operator-facing interpretation missing; got:\n{out}",
        );
    }

    #[test]
    fn write_covariance_section_auto_switches_units_for_sub_km_sigmas() {
        let mut report = stub_report();
        // Cross-track sigma at 0.31 km should render as "310 m"
        report.terminal_sigma3_position_ric_km = Vector3::new(1.47, 26.08, 0.31);
        let mut out = String::new();
        write_covariance_section(&mut out, &report);

        assert!(
            out.contains("1.47 km / 26.08 km / 310 m"),
            "expected adaptive km/m rendering; got:\n{out}",
        );
    }

    #[test]
    fn open_loop_fragility_warning_fires_when_sigma_exceeds_nominal() {
        let report = stub_report();
        // Terminal nominal: [0.51, 4.98, 0.50] km
        // Terminal 3σ:     [1.47, 26.08, 0.31] km
        // Ratios:           2.88, 5.24,  0.62
        // Worst: in-track at 5.24×
        let warning = open_loop_fragility_warning(&report).expect("warning should fire");
        assert!(warning.contains("in-track"), "axis label missing; got: {warning}");
        assert!(warning.contains("5.2\u{00d7}"), "ratio missing; got: {warning}");
        assert!(
            warning.contains("\u{26a0}\u{fe0f}"),
            "warning emoji missing; got: {warning}"
        );
    }

    #[test]
    fn open_loop_fragility_warning_silent_when_sigma_contained() {
        let mut report = stub_report();
        // Shrink terminal 3σ to well below nominal on every axis
        report.terminal_sigma3_position_ric_km = Vector3::new(0.05, 0.3, 0.05);
        assert!(open_loop_fragility_warning(&report).is_none());
    }

    #[test]
    fn open_loop_fragility_warning_skips_near_zero_nominal() {
        let mut report = stub_report();
        // Near-zero radial nominal: should not trigger warning from radial
        report.terminal_position_ric_km = Vector3::new(1.0e-10, 4.98, 0.50);
        // Shrink in-track and cross-track to safe values
        report.terminal_sigma3_position_ric_km = Vector3::new(0.001, 1.0, 0.1);
        // Radial ratio would be 0.001 / 1e-10 = huge, but we should skip it.
        // In-track: 1.0 / 4.98 ≈ 0.2 (safe). Cross-track: 0.1 / 0.5 = 0.2 (safe).
        // No warning should fire.
        assert!(open_loop_fragility_warning(&report).is_none());
    }

    #[test]
    fn write_covariance_section_includes_fragility_warning_for_stub_report() {
        let report = stub_report();
        let mut out = String::new();
        write_covariance_section(&mut out, &report);
        assert!(
            out.contains("Open-loop fragility:"),
            "fragility warning missing from rendered section; got:\n{out}",
        );
    }

    #[test]
    fn mahalanobis_interpretation_tiers() {
        // < 1σ: operator-facing "chief inside ellipsoid" form
        let tight = mahalanobis_interpretation(0.5);
        assert!(tight.contains("chief is inside"), "got: {tight}");
        assert!(tight.contains("1\u{03c3} uncertainty ellipsoid"), "got: {tight}");

        // 1-3σ: intermediate wording
        let mid = mahalanobis_interpretation(1.8);
        assert!(mid.contains("1.8\u{03c3}"), "got: {mid}");

        // ≥ 3σ: well-separated
        let far = mahalanobis_interpretation(3.5);
        assert!(far.contains("well-separated"), "got: {far}");
    }

    /// Pin the `# Invariants` claim that exactly 1.0 routes to the mid
    /// tier (strict `<` boundary — the 1σ branch is NOT taken) and that
    /// exactly 3.0 routes to the well-separated tier.
    #[test]
    fn mahalanobis_interpretation_boundary_ownership() {
        let at_one = mahalanobis_interpretation(1.0);
        assert!(
            !at_one.contains("chief is inside"),
            "exactly 1.0 must NOT route to tight tier; got: {at_one}",
        );
        assert!(
            at_one.contains("1.0\u{03c3}"),
            "exactly 1.0 must route to mid tier; got: {at_one}",
        );

        let at_three = mahalanobis_interpretation(3.0);
        assert!(
            at_three.contains("well-separated"),
            "exactly 3.0 must route to well-separated tier; got: {at_three}",
        );
        assert!(
            !at_three.contains("within 3\u{03c3} ellipsoid"),
            "exactly 3.0 must NOT route to mid tier; got: {at_three}",
        );
    }

    /// Pin the `# Invariants` claim that exactly `ratio = 1.0` does NOT
    /// fire the fragility warning (strict `>` against
    /// `cov_thresh::OPEN_LOOP_FRAGILITY_RATIO`).
    #[test]
    fn open_loop_fragility_warning_exact_ratio_does_not_fire() {
        let mut report = stub_report();
        // Radial and cross-track nominals set to zero → `< NEAR_ZERO_NOMINAL_KM`
        // causes those axes to be skipped entirely. In-track nominal and
        // sigma are both 4.98 → ratio = 1.0 exactly. Strict `>` → no fire.
        report.terminal_position_ric_km = Vector3::new(0.0, 4.98, 0.0);
        report.terminal_sigma3_position_ric_km = Vector3::new(0.0, 4.98, 0.0);
        assert!(
            open_loop_fragility_warning(&report).is_none(),
            "ratio exactly 1.0 must not fire (strict > boundary)",
        );
    }
}

//! Cross-tier insight generation.
//!
//! Pure functions that analyze mission data across fidelity tiers
//! and return structured [`Insight`] values with [`Severity`] levels.
//! Used by both terminal and markdown output paths.

use rpo_core::mission::{
    EnsembleStatistics, LegValidationSummary, MonteCarloReport, SafetyConfig, ValidationReport,
};

use super::common::{waypoint_miss_growth_ratio, KM_TO_M};
use super::thresholds::insight as insight_thresh;

/// Severity level for cross-tier insights.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// Requires immediate attention (collisions, convergence failure).
    Critical,
    /// Worth noting but not blocking (tight margins, e/i violations).
    Warning,
    /// Informational observations (error growth, Dv spread).
    Info,
}

/// A cross-tier insight with severity and message.
#[derive(Debug, Clone)]
pub struct Insight {
    /// How urgent this insight is.
    pub severity: Severity,
    /// Human-readable insight text (no severity prefix — renderer adds that).
    pub message: String,
}

/// Generate insights from validation comparison (analytical vs numerical).
///
/// Returns a list of [`Insight`] values with appropriate severity levels.
/// Each insight message is a complete sentence suitable for display as a
/// blockquote or colored callout.
///
/// When `report.pre_cola_numerical_safety` is populated, both the
/// analytical-vs-numerical overestimate comparison AND the downstream
/// tight-margin warnings use the pre-COLA baseline. The pre-COLA values
/// are the apples-to-apples comparison because analytical propagation
/// does not model COLA burns.
#[must_use]
pub fn validation_insights(
    report: &ValidationReport,
    config: &SafetyConfig,
) -> Vec<Insight> {
    let mut insights = Vec::new();

    // Prefer pre-COLA numerical safety for the analytical comparison — analytical
    // does not model COLA, so the pre-COLA baseline is the apples-to-apples pair.
    // When no COLA burn was applied, `pre_cola_numerical_safety` is `None` and we
    // fall back to the single numerical baseline.
    let (num_3d, num_ei, nyx_label) = match &report.pre_cola_numerical_safety {
        Some(pre) => (
            pre.operational.min_distance_3d_km,
            pre.passive.min_ei_separation_km,
            "Nyx pre-COLA trajectory",
        ),
        None => (
            report.numerical_safety.operational.min_distance_3d_km,
            report.numerical_safety.passive.min_ei_separation_km,
            "Nyx",
        ),
    };

    // Track whether the 3D overestimate insight already covered e/i margins
    let mut covered_ei = false;

    // Compare analytical vs numerical 3D distance
    if let Some(ref analytical) = report.analytical_safety {
        let ana_3d = analytical.operational.min_distance_3d_km;
        if ana_3d > 0.0 {
            let delta_pct = (ana_3d - num_3d) / num_3d * 100.0;
            if delta_pct > insight_thresh::SIGNIFICANT_DELTA_PCT {
                // Compute margin ratios: numerical value / threshold
                let d3d_ratio = if config.min_distance_3d_km > 0.0 {
                    num_3d / config.min_distance_3d_km
                } else {
                    f64::INFINITY
                };
                let ei_ratio = if config.min_ei_separation_km > 0.0 {
                    num_ei / config.min_ei_separation_km
                } else {
                    f64::INFINITY
                };

                let ei_qualifier = if ei_ratio < insight_thresh::TIGHT_MARGIN_FACTOR {
                    "tighter at"
                } else {
                    ""
                };
                let ei_part = if ei_qualifier.is_empty() {
                    format!("e/i margin is {ei_ratio:.1}\u{00d7}")
                } else {
                    format!("e/i margin is {ei_qualifier} {ei_ratio:.1}\u{00d7}")
                };

                insights.push(Insight {
                    severity: Severity::Warning,
                    message: format!(
                        "Analytical overestimates 3D distance by {delta_pct:.0}% relative to {nyx_label} \
                         ({:.0}m \u{2192} {:.0}m). \
                         Numerical 3D margin is {d3d_ratio:.1}\u{00d7} threshold; {ei_part}.",
                        ana_3d * KM_TO_M,
                        num_3d * KM_TO_M,
                    ),
                });
                covered_ei = true;
            }
        }
    }

    if let Some(growth_insight) = leg_error_growth_insight(&report.leg_summaries) {
        insights.push(growth_insight);
    }

    // Tight margin warning: numerical 3D < 2x threshold
    if num_3d < config.min_distance_3d_km * insight_thresh::TIGHT_MARGIN_FACTOR
        && num_3d >= config.min_distance_3d_km
    {
        insights.push(Insight {
            severity: Severity::Warning,
            message: format!(
                "Numerical 3D distance ({:.0}m) has <2\u{00d7} margin over threshold ({:.0}m). \
                 Consider increasing waypoint offsets.",
                num_3d * KM_TO_M,
                config.min_distance_3d_km * KM_TO_M,
            ),
        });
    }

    // Tight margin warning: numerical e/i < 2x threshold (skip if already covered above)
    if !covered_ei
        && num_ei < config.min_ei_separation_km * insight_thresh::TIGHT_MARGIN_FACTOR
        && num_ei >= config.min_ei_separation_km
    {
        insights.push(Insight {
            severity: Severity::Warning,
            message: format!(
                "Numerical e/i separation ({:.0}m) has <2\u{00d7} margin over threshold ({:.0}m).",
                num_ei * KM_TO_M,
                config.min_ei_separation_km * KM_TO_M,
            ),
        });
    }

    cola_effectiveness_insights(&report.cola_effectiveness, &mut insights);

    insights
}

/// Warn when the analytical COLA solver left any leg's POCA below the
/// configured target distance.
///
/// Distinct from [`cola_effectiveness_insights`], which only fires on
/// Nyx-confirmed failure (`eff.threshold_met == Some(false)`). This
/// function emits a WARNING *before* full-physics validation runs, so
/// mission-tier reports (which never see Nyx output) still surface
/// analytical-only failures.
///
/// Returns a vec of WARNING insights, one per offending leg. Empty when
/// all analytical POCAs meet or exceed `target_distance_km`, or when
/// `maneuvers` is empty.
#[must_use]
pub fn cola_analytical_miss_insights(
    maneuvers: &[rpo_core::mission::AvoidanceManeuver],
    target_distance_km: f64,
) -> Vec<Insight> {
    let mut insights = Vec::new();
    for m in maneuvers {
        if m.post_avoidance_poca_km < target_distance_km {
            insights.push(Insight {
                severity: Severity::Warning,
                message: format!(
                    "COLA burn on leg {} analytical POCA {:.0}m below target {:.0}m \
                     \u{2014} solver failed before full-physics validation.",
                    m.leg_index + 1,
                    m.post_avoidance_poca_km * KM_TO_M,
                    target_distance_km * KM_TO_M,
                ),
            });
        }
    }
    insights
}

/// COLA effectiveness insights: threshold failures (critical) and analytical overestimation (info).
fn cola_effectiveness_insights(
    effectiveness: &[rpo_core::mission::ColaEffectivenessEntry],
    insights: &mut Vec<Insight>,
) {
    for eff in effectiveness {
        // Threshold failure (critical)
        if eff.threshold_met == Some(false) {
            let nyx_m = eff.nyx_post_cola_min_distance_km * KM_TO_M;
            let target_m = eff.target_distance_km.unwrap_or(0.0) * KM_TO_M;
            insights.push(Insight {
                severity: Severity::Critical,
                message: format!(
                    "COLA burn on leg {} did not achieve target separation under full physics: \
                     nyx min distance {nyx_m:.0}m vs {target_m:.0}m threshold.",
                    eff.leg_index + 1,
                ),
            });
        }

        // Analytical overestimation (info)
        if let Some(ana) = eff.analytical_post_cola_poca_km {
            let nyx_min = eff.nyx_post_cola_min_distance_km;
            if nyx_min > 0.0 {
                let overestimate_pct = (ana - nyx_min) / nyx_min * 100.0;
                if overestimate_pct > insight_thresh::SIGNIFICANT_DELTA_PCT {
                    insights.push(Insight {
                        severity: Severity::Info,
                        message: format!(
                            "COLA analytical POCA ({:.0}m) overestimates Nyx post-COLA minimum \
                             ({:.0}m) by {overestimate_pct:.0}%. \
                             Analytical COLA effectiveness estimates are non-conservative.",
                            ana * KM_TO_M,
                            nyx_min * KM_TO_M,
                        ),
                    });
                }
            }
        }
    }
}

/// Check per-leg position error growth and produce an extrapolation insight.
///
/// Uses pre-computed [`LegValidationSummary`] to avoid redundant post-COLA
/// filtering of raw points.
fn leg_error_growth_insight(
    summaries: &[LegValidationSummary],
) -> Option<Insight> {
    if summaries.len() < 2 {
        return None;
    }

    let leg_rms: Vec<f64> = summaries
        .iter()
        .map(|s| s.rms_position_error_km)
        .collect();

    let first_rms = leg_rms[0];
    let last_rms = leg_rms[leg_rms.len() - 1];
    if first_rms <= 0.0 || last_rms / first_rms <= insight_thresh::ERROR_GROWTH_RATIO_ALERT {
        return None;
    }

    let growth_ratio = last_rms / first_rms;
    let rms_strs: Vec<String> = leg_rms
        .iter()
        .map(|r| format!("{:.0}m", r * KM_TO_M))
        .collect();

    // Extrapolate next leg from last two legs' ratio
    let next_rms = if leg_rms.len() >= 2 {
        let second_to_last = leg_rms[leg_rms.len() - 2];
        if second_to_last > 0.0 {
            last_rms * (last_rms / second_to_last)
        } else {
            last_rms * growth_ratio
        }
    } else {
        last_rms * growth_ratio
    };
    let next_leg = leg_rms.len() + 1;

    let num_legs = leg_rms.len();
    Some(Insight {
        severity: Severity::Info,
        message: format!(
            "Position error grows ~{growth_ratio:.0}\u{00d7} from leg 1 to \
             leg {num_legs} ({rms} RMS). At this rate, leg {next_leg} would \
             approach ~{:.0}m error \u{2014} consider running `validate` for \
             missions beyond {num_legs} legs at this fidelity.",
            next_rms * KM_TO_M,
            rms = rms_strs.join(" \u{2192} "),
        ),
    })
}

/// Generate insights from Monte Carlo results.
///
/// Returns a list of [`Insight`] values with appropriate severity levels.
#[must_use]
pub fn mc_insights(
    report: &MonteCarloReport,
    config: &SafetyConfig,
) -> Vec<Insight> {
    let mut insights = Vec::new();
    let stats = &report.statistics;

    // Critical: collision probability > 0
    if stats.collision_probability > 0.0 {
        let n = (stats.collision_probability * f64::from(report.config.num_samples)).round();
        insights.push(Insight {
            severity: Severity::Critical,
            message: format!(
                "{:.1}% collision probability ({:.0}/{} samples violate {:.0}m keep-out). \
                 Review waypoint design urgently.",
                stats.collision_probability * 100.0,
                n,
                report.config.num_samples,
                config.min_distance_3d_km * KM_TO_M,
            ),
        });
    }

    // e/i violation rate > 0%
    if stats.ei_violation_rate > 0.0 {
        let has_collisions = stats.collision_probability > 0.0;
        let has_keepout = stats.keepout_violation_rate > 0.0;

        // Include p05 worst-case separation when available
        let p05_part = stats
            .min_ei_separation_km
            .as_ref()
            .map(|ei| {
                format!(
                    " \u{2014} 5th-percentile separation is {:.1}m against {:.0}m threshold",
                    ei.p05 * KM_TO_M,
                    config.min_ei_separation_km * KM_TO_M,
                )
            })
            .unwrap_or_default();

        if has_collisions || has_keepout {
            insights.push(Insight {
                severity: Severity::Critical,
                message: format!(
                    "{:.0}% e/i violation rate{p05_part}. Operational safety is also compromised.",
                    stats.ei_violation_rate * 100.0,
                ),
            });
        } else {
            insights.push(Insight {
                severity: Severity::Warning,
                message: format!(
                    "{:.0}% e/i violation rate{p05_part}. \
                     Operational safety holds (0 collisions, 0 keep-out violations). \
                     If free-drift contingency is required, consider increasing \
                     R-bar or cross-track offsets in waypoint design.",
                    stats.ei_violation_rate * 100.0,
                ),
            });
        }
    }

    // Convergence < 100%
    if stats.convergence_rate < 1.0 {
        let fail_pct = (1.0 - stats.convergence_rate) * 100.0;
        insights.push(Insight {
            severity: Severity::Warning,
            message: format!(
                "{fail_pct:.1}% of samples failed to converge. Check dispersion magnitudes \
                 and targeting configuration.",
            ),
        });
    }

    // Dv spread ratio
    let dv = &stats.total_dv_km_s;
    if dv.p05 > 0.0 {
        let ratio = dv.p95 / dv.p05;
        if ratio > insight_thresh::DV_SPREAD_RATIO_ALERT {
            insights.push(Insight {
                severity: Severity::Info,
                message: format!(
                    "\u{0394}v distribution spread is {ratio:.2}\u{00d7} (p05={:.2} m/s, \
                     p95={:.2} m/s). This indicates significant sensitivity to dispersions.",
                    dv.p05 * KM_TO_M,
                    dv.p95 * KM_TO_M,
                ),
            });
        }
    }

    if let Some(insight) = mc_waypoint_growth_insight(stats) {
        insights.push(insight);
    }

    insights
}

/// Render the per-waypoint miss growth insight when the ratio exceeds the alert.
///
/// Uses the shared [`waypoint_miss_growth_ratio`] helper so the verdict
/// and the insight list stay in sync.
fn mc_waypoint_growth_insight(stats: &EnsembleStatistics) -> Option<Insight> {
    let ratio = waypoint_miss_growth_ratio(stats)?;
    if ratio <= insight_thresh::ERROR_GROWTH_RATIO_ALERT {
        return None;
    }
    let miss_medians: Vec<f64> = stats
        .waypoint_miss_km
        .iter()
        .filter_map(|opt| opt.as_ref().map(|s| s.p50))
        .collect();
    let first = miss_medians[0];
    let last = miss_medians[miss_medians.len() - 1];
    Some(Insight {
        severity: Severity::Info,
        message: format!(
            "Waypoint miss distance grows {ratio:.1}\u{00d7} from WP 1 ({:.0}m p50) \
             to WP {} ({:.0}m p50). Later waypoints accumulate more dispersion \
             \u{2014} consider tighter tolerances or additional intermediate waypoints.",
            first * KM_TO_M,
            miss_medians.len(),
            last * KM_TO_M,
        ),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        AvoidanceManeuver, ColaEffectivenessEntry, CorrectionType, DispersionConfig,
        EnsembleStatistics, MonteCarloConfig, MonteCarloMode, MonteCarloReport, OperationalSafety,
        PassiveSafety, PercentileStats, SafetyMetrics, ValidationReport,
    };

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

    fn default_config() -> SafetyConfig {
        SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        }
    }

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

    #[test]
    fn validation_insights_compares_analytical_against_pre_cola_when_available() {
        // Numeric values chosen so the two possible denominators give distinct
        // rounded percentages:
        //   pre-COLA  (correct): (0.200 - 0.120) / 0.120 * 100 = 66.7% -> "67%"
        //   post-COLA (wrong):   (0.200 - 0.080) / 0.080 * 100 = 150.0% -> "150%"
        // The test asserts we see "67" and not "150" in the insight.
        //
        // Note: if a future change surfaces structured numeric fields on
        // `Insight` (e.g. `delta_pct: f64`), re-tighten this test to assert on
        // those fields directly instead of the message substring.
        let report = validation_report_with_pre_cola(
            /* analytical_3d_km = */ 0.200,
            /* pre_cola_3d_km = */ 0.120,
            /* post_cola_3d_km = */ 0.080,
        );
        let config = SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        };

        let insights = validation_insights(&report, &config);
        let overestimate_insight = insights
            .iter()
            .find(|i| i.message.contains("overestimates"))
            .expect("should emit overestimate insight");

        // Primary assertion: the label identifies the pre-COLA comparison baseline.
        assert!(
            overestimate_insight.message.contains("pre-COLA"),
            "expected 'pre-COLA' label in insight, got: {}",
            overestimate_insight.message,
        );

        // Secondary assertion: the rounded delta percentage confirms we divided
        // by the pre-COLA value, not the post-COLA value.
        assert!(
            overestimate_insight.message.contains("67%"),
            "expected 67% delta (pre-COLA denominator), got: {}",
            overestimate_insight.message,
        );
        assert!(
            !overestimate_insight.message.contains("150%"),
            "must not compute 150% (post-COLA denominator), got: {}",
            overestimate_insight.message,
        );
    }

    #[test]
    fn validation_nonconservative_3d() {
        let report = ValidationReport {
            leg_points: vec![],
            max_position_error_km: 0.1,
            mean_position_error_km: 0.05,
            rms_position_error_km: 0.07,
            max_velocity_error_km_s: 0.001,
            analytical_safety: Some(make_safety(0.200, 0.170)),
            numerical_safety: make_safety(0.100, 0.170),
            pre_cola_numerical_safety: None,
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness: vec![],
            leg_summaries: vec![],
        };

        let insights = validation_insights(&report, &default_config());
        let msg = &insights
            .iter()
            .find(|i| i.message.contains("overestimates 3D"))
            .expect("expected non-conservative 3D insight")
            .message;

        // Fallback label must be plain "Nyx" (not "Nyx pre-COLA trajectory")
        assert!(
            msg.contains("relative to Nyx ("),
            "expected plain 'Nyx' label followed by '(', got: {msg}"
        );
        assert!(
            !msg.contains("pre-COLA"),
            "must not mention pre-COLA when not available, got: {msg}"
        );
    }

    #[test]
    fn mc_ei_violation_produces_insight() {
        let stats = EnsembleStatistics {
            total_dv_km_s: PercentileStats::default(),
            min_rc_distance_km: None,
            min_3d_distance_km: None,
            min_ei_separation_km: None,
            waypoint_miss_km: vec![],
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 0.25,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![],
        };
        let report = MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 100,
                dispersions: Default::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.008,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 10.0,
            covariance_cross_check: None,
        };

        let insights = mc_insights(&report, &default_config());
        assert!(
            insights.iter().any(|i| i.message.contains("e/i violation")),
            "expected e/i insight, got: {insights:?}"
        );
    }

    #[test]
    fn mc_clean_run_no_critical_insights() {
        let stats = EnsembleStatistics {
            total_dv_km_s: PercentileStats {
                p05: 0.008,
                p95: 0.009,
                ..Default::default()
            },
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
        let report = MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 100,
                dispersions: Default::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.008,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 10.0,
            covariance_cross_check: None,
        };

        let insights = mc_insights(&report, &default_config());
        assert!(
            insights.is_empty(),
            "expected no insights for clean run, got: {insights:?}"
        );
    }

    fn mc_report_with_tight_rc_plane(
        rc_p05_km: f64,
        min_3d_p05_km: f64,
    ) -> MonteCarloReport {
        let stats = EnsembleStatistics {
            total_dv_km_s: PercentileStats {
                p05: 0.008,
                p95: 0.009,
                ..PercentileStats::default()
            },
            min_rc_distance_km: Some(PercentileStats {
                p05: rc_p05_km,
                ..PercentileStats::default()
            }),
            min_3d_distance_km: Some(PercentileStats {
                p05: min_3d_p05_km,
                ..PercentileStats::default()
            }),
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
                num_samples: 100,
                dispersions: DispersionConfig::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.008,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 10.0,
            covariance_cross_check: None,
        }
    }

    #[test]
    fn mc_insights_does_not_compare_rc_plane_p05_against_3d_keepout() {
        // Build an EnsembleStatistics with a very tight R/C-plane p05 (15 m)
        // but a healthy 3D p05 (131.6 m) — matches the audit's mc.md scenario.
        // No insight should reference 'R/C-plane' or 'breach safety'.
        let report = mc_report_with_tight_rc_plane(
            /* rc_p05_km = */ 0.015,
            /* min_3d_p05_km = */ 0.1316,
        );
        let config = SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        };

        let insights = mc_insights(&report, &config);
        for insight in &insights {
            assert!(
                !insight.message.contains("R/C-plane"),
                "R/C-plane insight must be removed: {}",
                insight.message
            );
            assert!(
                !insight.message.contains("breach safety"),
                "'breach safety' phrasing must be removed: {}",
                insight.message
            );
        }
    }

    #[test]
    fn cola_overestimation_produces_info_insight() {
        let report = ValidationReport {
            leg_points: vec![],
            leg_summaries: vec![],
            max_position_error_km: 0.1,
            mean_position_error_km: 0.05,
            rms_position_error_km: 0.07,
            max_velocity_error_km_s: 0.001,
            analytical_safety: Some(make_safety(0.200, 0.170)),
            numerical_safety: make_safety(0.200, 0.170),
            pre_cola_numerical_safety: None,
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness: vec![ColaEffectivenessEntry {
                leg_index: 2,
                analytical_post_cola_poca_km: Some(0.281),
                nyx_post_cola_min_distance_km: 0.122,
                target_distance_km: Some(0.100),
                threshold_met: Some(true),
            }],
        };

        let insights = validation_insights(&report, &default_config());
        assert!(
            insights.iter().any(|i| matches!(i.severity, Severity::Info)
                && i.message.contains("overestimates Nyx post-COLA")),
            "expected COLA overestimation insight, got: {insights:?}"
        );
    }

    #[test]
    fn cola_no_overestimation_when_within_threshold() {
        let report = ValidationReport {
            leg_points: vec![],
            leg_summaries: vec![],
            max_position_error_km: 0.1,
            mean_position_error_km: 0.05,
            rms_position_error_km: 0.07,
            max_velocity_error_km_s: 0.001,
            analytical_safety: Some(make_safety(0.200, 0.170)),
            numerical_safety: make_safety(0.200, 0.170),
            pre_cola_numerical_safety: None,
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
            cola_effectiveness: vec![ColaEffectivenessEntry {
                leg_index: 2,
                analytical_post_cola_poca_km: Some(0.110),
                nyx_post_cola_min_distance_km: 0.105, // ~5% overestimate
                target_distance_km: Some(0.100),
                threshold_met: Some(true),
            }],
        };

        let insights = validation_insights(&report, &default_config());
        assert!(
            !insights.iter().any(|i| i.message.contains("overestimates Nyx post-COLA")),
            "should not produce overestimation insight when within threshold"
        );
    }

    fn make_avoidance_maneuver(leg_index: usize, post_poca_km: f64) -> AvoidanceManeuver {
        AvoidanceManeuver {
            epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2026, 1, 1),
            dv_ric_km_s: Vector3::new(0.0, 0.0001, 0.0),
            maneuver_location_rad: 0.0,
            post_avoidance_poca_km: post_poca_km,
            fuel_cost_km_s: 0.0001,
            correction_type: CorrectionType::InPlane,
            leg_index,
        }
    }

    #[test]
    fn cola_analytical_miss_fires_on_under_target() {
        // 0.281 km post-COLA POCA, 0.300 km target → warning.
        let maneuvers = vec![make_avoidance_maneuver(2, 0.281)];
        let insights = cola_analytical_miss_insights(&maneuvers, 0.300);
        assert_eq!(insights.len(), 1);
        assert_eq!(insights[0].severity, Severity::Warning);
        assert!(insights[0].message.contains("leg 3"));
        assert!(insights[0].message.contains("281m"));
        assert!(insights[0].message.contains("300m"));
    }

    #[test]
    fn cola_analytical_miss_silent_when_above_target() {
        // 0.305 km post-COLA POCA, 0.300 km target → no insight.
        let maneuvers = vec![make_avoidance_maneuver(0, 0.305)];
        let insights = cola_analytical_miss_insights(&maneuvers, 0.300);
        assert!(insights.is_empty());
    }

    #[test]
    fn cola_analytical_miss_fires_on_multiple_legs() {
        let maneuvers = vec![
            make_avoidance_maneuver(0, 0.305), // above — ignored
            make_avoidance_maneuver(1, 0.200), // below
            make_avoidance_maneuver(3, 0.290), // below
        ];
        let insights = cola_analytical_miss_insights(&maneuvers, 0.300);
        assert_eq!(insights.len(), 2);
        assert!(insights[0].message.contains("leg 2"));
        assert!(insights[1].message.contains("leg 4"));
    }
}

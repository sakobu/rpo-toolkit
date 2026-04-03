//! Cross-tier insight generation.
//!
//! Pure functions that analyze mission data across fidelity tiers
//! and return structured [`Insight`] values with [`Severity`] levels.
//! Used by both terminal and markdown output paths.

use rpo_core::mission::{MonteCarloReport, SafetyConfig, ValidationReport};

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
#[must_use]
pub fn validation_insights(
    report: &ValidationReport,
    config: &SafetyConfig,
) -> Vec<Insight> {
    let mut insights = Vec::new();
    let num_3d = report.numerical_safety.operational.min_distance_3d_km;
    let num_ei = report.numerical_safety.passive.min_ei_separation_km;

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
                        "Analytical overestimates 3D distance by {delta_pct:.0}% relative to Nyx \
                         ({:.0}m \u{2192} {:.0}m). \
                         Numerical 3D margin is {d3d_ratio:.1}\u{00d7} threshold; {ei_part}.",
                        ana_3d * 1000.0,
                        num_3d * 1000.0,
                    ),
                });
                covered_ei = true;
            }
        }
    }

    if let Some(growth_insight) = leg_error_growth_insight(&report.leg_points) {
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
                num_3d * 1000.0,
                config.min_distance_3d_km * 1000.0,
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
                num_ei * 1000.0,
                config.min_ei_separation_km * 1000.0,
            ),
        });
    }

    insights
}

/// Check per-leg position error growth and produce an extrapolation insight.
fn leg_error_growth_insight(
    leg_points: &[Vec<rpo_core::mission::ValidationPoint>],
) -> Option<Insight> {
    if leg_points.len() < 2 {
        return None;
    }

    // Filter post-COLA points: their "error" reflects intentional trajectory change,
    // not model fidelity. Consistent with core's compute_report_statistics.
    let leg_rms: Vec<f64> = leg_points
        .iter()
        .map(|points| {
            let n = points.iter().filter(|p| !p.post_cola).count();
            if n == 0 {
                return 0.0;
            }
            let sum_sq: f64 = points
                .iter()
                .filter(|p| !p.post_cola)
                .map(|p| p.position_error_km.powi(2))
                .sum();
            (sum_sq / f64::from(u32::try_from(n).unwrap_or(u32::MAX))).sqrt()
        })
        .collect();

    let first_rms = leg_rms[0];
    let last_rms = leg_rms[leg_rms.len() - 1];
    if first_rms <= 0.0 || last_rms / first_rms <= insight_thresh::ERROR_GROWTH_RATIO_ALERT {
        return None;
    }

    let growth_ratio = last_rms / first_rms;
    let rms_strs: Vec<String> = leg_rms
        .iter()
        .map(|r| format!("{:.0}m", r * 1000.0))
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
            next_rms * 1000.0,
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
                config.min_distance_3d_km * 1000.0,
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
                    ei.p05 * 1000.0,
                    config.min_ei_separation_km * 1000.0,
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
                    dv.p05 * 1000.0,
                    dv.p95 * 1000.0,
                ),
            });
        }
    }

    // Per-waypoint miss growth
    let miss_medians: Vec<f64> = stats
        .waypoint_miss_km
        .iter()
        .filter_map(|opt| opt.as_ref().map(|s| s.p50))
        .collect();
    if miss_medians.len() >= 2 {
        let first = miss_medians[0];
        let last = miss_medians[miss_medians.len() - 1];
        if first > 0.0 && last / first > insight_thresh::ERROR_GROWTH_RATIO_ALERT {
            insights.push(Insight {
                severity: Severity::Info,
                message: format!(
                    "Waypoint miss distance grows {:.1}\u{00d7} from WP 1 ({:.0}m p50) \
                     to WP {} ({:.0}m p50). Later waypoints accumulate more dispersion \
                     \u{2014} consider tighter tolerances or additional intermediate waypoints.",
                    last / first,
                    first * 1000.0,
                    miss_medians.len(),
                    last * 1000.0,
                ),
            });
        }
    }

    insights
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        EnsembleStatistics, MonteCarloConfig, MonteCarloMode, MonteCarloReport,
        OperationalSafety, PassiveSafety, PercentileStats, SafetyMetrics, ValidationReport,
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
            chief_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            deputy_config: rpo_core::types::SpacecraftConfig::CUBESAT_6U,
            eclipse_validation: None,
        };

        let insights = validation_insights(&report, &default_config());
        assert!(
            insights.iter().any(|i| i.message.contains("overestimates 3D")),
            "expected non-conservative 3D insight, got: {insights:?}"
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
}

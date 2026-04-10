//! Shared output helpers.

use std::borrow::Cow;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use serde::Serialize;

use rpo_core::mission::{
    assess_safety, AvoidanceManeuver, ColaConfig, EiAlignment, EnsembleStatistics, MonteCarloReport,
    SafetyConfig, SafetyRequirements, ValidationReport,
};
use rpo_core::pipeline::{
    resolve_propagator, to_propagation_model, PipelineInput, PipelineOutput, TransferResult,
};
use rpo_core::propagation::{DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;
use rpo_nyx::nyx_bridge::extract_dmf_rates;

use crate::error::CliError;

use super::thresholds::{insight as insight_thresh, mc as mc_thresh, rate as rate_thresh};

/// Whether analytical safety is the governing tier or a baseline for comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyTier {
    /// Analytical is the governing and only fidelity tier (mission command).
    Governing,
    /// Analytical is a baseline; Nyx full-physics validation governs.
    Baseline,
}

/// Default delta-v budget for CLI COLA overlay (km/s).
/// 10 m/s — matches the pipeline auto-COLA default in `execute.rs`.
const DEFAULT_COLA_BUDGET_KM_S: f64 = 0.01;

/// Default formation design enrichment threshold (km).
/// 100 m — a reasonable passive safety margin for close proximity operations.
const DEFAULT_ENRICHMENT_THRESHOLD_KM: f64 = 0.1;

/// CLI overlay flags that mutate a [`PipelineInput`] before execution.
///
/// Groups COLA and formation design enrichment flags to avoid
/// long argument lists in porcelain command `run()` functions.
#[derive(Debug, Clone, Default)]
pub struct OverlayFlags {
    /// Target miss distance for collision avoidance (km).
    pub cola_threshold: Option<f64>,
    /// Maximum delta-v budget for COLA (km/s).
    pub cola_budget: Option<f64>,
    /// Enable formation design enrichment.
    pub auto_enrich: bool,
    /// Custom min separation threshold (km) for formation design.
    pub auto_enrich_threshold: Option<f64>,
}

/// Apply all CLI overlay flags onto a [`PipelineInput`].
///
/// Handles COLA configuration and formation design enrichment in one call.
pub fn apply_overlays(input: &mut PipelineInput, flags: &OverlayFlags) {
    // COLA overlay
    if let Some(threshold) = flags.cola_threshold {
        input.cola = Some(ColaConfig {
            target_distance_km: threshold,
            max_dv_km_s: flags.cola_budget.unwrap_or(DEFAULT_COLA_BUDGET_KM_S),
        });
    }

    // Formation design enrichment overlay
    let separation_km = match (flags.auto_enrich_threshold, flags.auto_enrich) {
        (Some(t), _) => Some(t),
        (None, true) => Some(DEFAULT_ENRICHMENT_THRESHOLD_KM),
        (None, false) => None,
    };
    if let Some(min_separation_km) = separation_km {
        input.safety_requirements = Some(SafetyRequirements {
            min_separation_km,
            alignment: EiAlignment::default(),
        });
    }
}

/// Mission feasibility verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// All safety margins satisfied.
    Feasible,
    /// Operational safety passes; passive safety below threshold but non-governing.
    OperationallyFeasible,
    /// Safety margins violated or insufficient data.
    Caution,
}

impl Verdict {
    /// Whether the verdict indicates the mission is feasible (either fully or operationally).
    #[must_use]
    pub fn is_feasible(self) -> bool {
        matches!(self, Self::Feasible | Self::OperationallyFeasible)
    }
}

impl std::fmt::Display for Verdict {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Feasible => write!(f, "FEASIBLE"),
            Self::OperationallyFeasible => write!(f, "OPERATIONALLY FEASIBLE"),
            Self::Caution => write!(f, "CAUTION"),
        }
    }
}

/// Verdict with contextual reason string.
pub struct VerdictResult {
    /// The verdict category.
    pub verdict: Verdict,
    /// Human-readable reason for the verdict.
    pub reason: Cow<'static, str>,
}

// ── Status / spinner ────────────────────────────────────────────────────

/// Print a status message to the spinner if available, otherwise to stderr.
macro_rules! status {
    ($spinner:expr, $($arg:tt)*) => {
        if let Some(s) = &$spinner {
            s.set_message(format!($($arg)*));
        } else {
            eprintln!($($arg)*);
        }
    };
}
pub(crate) use status;

/// Create an animated spinner for long-running CLI operations.
///
/// The spinner writes to stderr, so it does not interfere with stdout output.
/// Returns `None` if stderr is not a terminal (piped/redirected).
#[must_use]
pub fn create_spinner() -> Option<ProgressBar> {
    if !std::io::IsTerminal::is_terminal(&std::io::stderr()) {
        return None;
    }
    let s = ProgressBar::new_spinner();
    s.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner()),
    );
    s.enable_steady_tick(Duration::from_millis(120));
    Some(s)
}

// ── Drag / propagator resolution ────────────────────────────────────────

/// Resolve propagator with optional auto-drag extraction.
///
/// If `auto_drag` is true, extracts differential drag rates via Nyx DMF
/// and prints them to stderr. Returns the propagation model and optional
/// derived drag config.
///
/// # Errors
///
/// Returns [`CliError`] if drag extraction or propagator resolution fails.
pub fn resolve_drag_and_propagator(
    auto_drag: bool,
    transfer: &TransferResult,
    chief_config: &SpacecraftConfig,
    deputy_config: &SpacecraftConfig,
    almanac: &Arc<Almanac>,
    input: &PipelineInput,
    spinner: Option<&ProgressBar>,
) -> Result<(PropagationModel, Option<DragConfig>), CliError> {
    let auto_drag_config = if auto_drag {
        status!(spinner, "Extracting differential drag rates via Nyx...");
        let drag = extract_dmf_rates(
            &transfer.perch_chief,
            &transfer.perch_deputy,
            chief_config,
            deputy_config,
            almanac,
        )?;
        eprintln!(
            "  da_dot={:.6e}, dex_dot={:.6e}, dey_dot={:.6e}",
            drag.da_dot, drag.dex_dot, drag.dey_dot
        );
        Some(drag)
    } else {
        None
    };
    Ok(resolve_propagator(
        auto_drag_config,
        to_propagation_model(&input.propagator),
    ))
}

// ── Output helpers ─────────────────────────────────────────────────────

/// Create the output sink: a buffered file writer, or locked stdout.
fn open_output(path: Option<&Path>) -> Result<Box<dyn std::io::Write>, CliError> {
    if let Some(p) = path {
        if let Some(parent) = p.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent).map_err(|e| CliError::io(parent, e))?;
            }
        }
        let file = std::fs::File::create(p).map_err(|e| CliError::io(p, e))?;
        Ok(Box::new(std::io::BufWriter::new(file)))
    } else {
        Ok(Box::new(std::io::stdout().lock()))
    }
}

/// Write text content to a file or stdout.
///
/// If `path` is `Some`, writes to that file (creating parent directories)
/// and prints the path to stderr. If `None`, prints to stdout.
///
/// # Errors
///
/// Returns [`CliError::Io`] if directory creation or file write fails.
pub fn output_text(content: &str, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    out.write_all(content.as_bytes())
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Serialize a value as JSON directly to a file or stdout.
///
/// Writes directly to the output stream via [`serde_json::to_writer_pretty`],
/// avoiding an intermediate `String` allocation.
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails, or [`CliError::Io`]
/// if file write fails.
pub fn output_json<T: Serialize>(data: &T, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    serde_json::to_writer_pretty(&mut out, data).map_err(CliError::Serialize)?;
    out.write_all(b"\n")
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Pretty-print a value as JSON to stdout (plumbing commands).
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails.
pub fn print_json<T: Serialize>(value: &T) -> Result<(), CliError> {
    output_json(value, None)
}

// ── Unit formatting ─────────────────────────────────────────────────────

/// Conversion factor from kilometres to metres for display formatting.
pub(crate) const KM_TO_M: f64 = 1000.0;

/// ROE component below which the value is treated as effectively zero for display.
const ROE_DISPLAY_ZERO_THRESHOLD: f64 = 1e-15;

/// RIC frame component labels for display, ordered [Radial, In-track, Cross-track].
const RIC_COMPONENT_LABELS: [&str; 3] = ["R", "I", "C"];

/// Format a value stored in km/s as m/s with given decimal places.
///
/// Example: `fmt_m_s(0.5432, 1)` returns `"543.2 m/s"`.
#[must_use]
pub fn fmt_m_s(value_km_s: f64, decimals: usize) -> String {
    let m_s = value_km_s * KM_TO_M;
    format!("{m_s:.decimals$} m/s")
}

/// Format a value stored in km as meters with given decimal places.
///
/// Example: `fmt_m(0.150, 1)` returns `"150.0 m"`.
#[must_use]
pub fn fmt_m(value_km: f64, decimals: usize) -> String {
    let m = value_km * KM_TO_M;
    format!("{m:.decimals$} m")
}

/// Format a ROE component: scientific notation for non-zero, padded zero for zero.
///
/// Avoids the ugly `+0.000000e0` output for zero values.
#[must_use]
pub fn fmt_roe_component(value: f64) -> String {
    if value.abs() < ROE_DISPLAY_ZERO_THRESHOLD {
        "0".to_string()
    } else {
        format!("{value:+.6e}")
    }
}

// ── Bounded-motion formatting ────────────────────────────────────────────

/// Bounded-motion residual below which the orbit is considered effectively bounded.
/// Residuals below this are display-formatted as "bounded (residual ~ 0)".
const BOUNDED_MOTION_DISPLAY_THRESHOLD: f64 = 1e-10;

/// Format a bounded-motion residual for human display.
#[must_use]
pub fn fmt_bounded_motion_residual(residual: f64) -> String {
    if residual.abs() < BOUNDED_MOTION_DISPLAY_THRESHOLD {
        "bounded (residual ~ 0)".to_string()
    } else {
        format!("drifting (residual = {residual:.3e})")
    }
}

// ── Duration formatting ─────────────────────────────────────────────────

/// Format a duration in seconds as a human-readable string.
///
/// Returns `"Xd YYh"` for durations >= 1 day, `"Xh YYm"` for >= 1 hour,
/// `"Xm YYs"` for >= 1 minute, `"Xs"` for >= 1 second, `"< 1s"` for
/// sub-second, and `"0s"` for zero.
#[must_use]
pub fn fmt_duration(s: f64) -> String {
    // Round to nearest second to avoid fractional artifacts, then format
    // from the integer total. This prevents edge cases like "59m 60s".
    let total_s = s.max(0.0).round();
    if total_s >= 86400.0 {
        let d = (total_s / 86400.0).floor();
        let h = ((total_s - d * 86400.0) / 3600.0).floor();
        format!("{d:.0}d {h:02.0}h")
    } else if total_s >= 3600.0 {
        let h = (total_s / 3600.0).floor();
        let m = ((total_s - h * 3600.0) / 60.0).floor();
        format!("{h:.0}h {m:02.0}m")
    } else if total_s >= 60.0 {
        let m = (total_s / 60.0).floor();
        let sec = total_s - m * 60.0;
        format!("{m:.0}m {sec:02.0}s")
    } else if total_s >= 1.0 {
        format!("{total_s:.0}s")
    } else if s > 0.0 {
        "< 1s".to_string()
    } else {
        "0s".to_string()
    }
}


// ── Velocity target formatting ──────────────────────────────────────────

/// Format a velocity target vector for the waypoint table.
///
/// - Zero vector → "—"
/// - Single dominant component (≥ 70% of magnitude) → signed value + direction tag (e.g. "+1.0 I")
/// - No dominant component → magnitude only (e.g. "1.4")
///
/// All values displayed in m/s.
#[must_use]
pub fn fmt_velocity_target(v: &nalgebra::Vector3<f64>) -> String {
    use super::thresholds::velocity::{DOMINANT_COMPONENT_FRACTION, ZERO_MAGNITUDE_KM_S};

    let mag = v.norm();
    if mag < ZERO_MAGNITUDE_KM_S {
        return "\u{2014}".to_string();
    }

    let abs_components = [v.x.abs(), v.y.abs(), v.z.abs()];
    let values = [v.x, v.y, v.z];

    for (i, &abs_val) in abs_components.iter().enumerate() {
        if abs_val / mag >= DOMINANT_COMPONENT_FRACTION {
            let sign = if values[i] >= 0.0 { "+" } else { "-" };
            let val_m_s = abs_val * KM_TO_M;
            return format!("{sign}{val_m_s:.1} {}", RIC_COMPONENT_LABELS[i]);
        }
    }

    // No single dominant component — show magnitude only
    let mag_m_s = mag * KM_TO_M;
    format!("{mag_m_s:.1}")
}

// ── COLA helpers ────────────────────────────────────────────────────────

/// Compute total COLA Δv and burn count from avoidance maneuvers.
#[must_use]
pub fn cola_dv_summary(cola: Option<&[AvoidanceManeuver]>) -> Option<(f64, usize)> {
    cola.map(|maneuvers| {
        let total: f64 = maneuvers.iter().map(|m| m.fuel_cost_km_s).sum();
        (total, maneuvers.len())
    })
}

// ── MC baseline ─────────────────────────────────────────────────────────

/// Baseline mission parameters passed to MC formatters.
pub struct McBaseline {
    /// Lambert transfer Dv (km/s).
    pub lambert_dv_km_s: f64,
    /// Lambert transfer time of flight (s).
    pub lambert_tof_s: f64,
    /// Waypoint targeting Dv (km/s).
    pub waypoint_dv_km_s: f64,
    /// Waypoint phase duration (s).
    pub waypoint_duration_s: f64,
    /// Number of waypoint legs.
    pub num_legs: usize,
    /// Whether the mission is far-field (Lambert transfer required).
    pub is_far_field: bool,
}

impl McBaseline {
    /// Total Dv: Lambert transfer + waypoint targeting.
    #[must_use]
    pub fn total_dv_km_s(&self) -> f64 {
        self.lambert_dv_km_s + self.waypoint_dv_km_s
    }
}

// ── Verdict determination ───────────────────────────────────────────────

/// Determine the mission verdict from safety assessment.
///
/// Returns a [`VerdictResult`] with verdict category and human-readable reason.
/// If validation data is provided, uses numerical safety; otherwise uses analytical.
///
/// After the primary verdict is constructed, the free-drift analysis is
/// walked to surface abort-case passive safety collapse: any leg whose
/// free-drift e/i minimum is below `min_ei_separation_km *
/// ABORT_EI_COLLAPSE_RATIO` is appended to the verdict reason. This keeps
/// the "non-governing for guided ops" softening honest — if the departure
/// burn is lost on one of those legs, the free-drift e/i would collapse
/// to sub-metre values.
#[must_use]
pub fn determine_verdict(
    output: &PipelineOutput,
    config: &SafetyConfig,
    validation: Option<&ValidationReport>,
) -> VerdictResult {
    let enrichment_active = output.formation_design.is_some();

    let mut verdict = if let Some(report) = validation {
        let assessment = assess_safety(&report.numerical_safety, config);
        if assessment.overall_pass {
            VerdictResult {
                verdict: Verdict::Feasible,
                reason: Cow::Borrowed("safety margins satisfied, Nyx full-physics"),
            }
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult {
                verdict: Verdict::OperationallyFeasible,
                reason: Cow::Borrowed(
                    "3D distance passes (Nyx); e/i below threshold (non-governing for guided ops)",
                ),
            }
        } else {
            VerdictResult {
                verdict: Verdict::Caution,
                reason: Cow::Borrowed("safety margins violated, Nyx full-physics"),
            }
        }
    } else if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, config);
        if assessment.overall_pass {
            VerdictResult {
                verdict: Verdict::Feasible,
                reason: Cow::Borrowed("analytical safety margins satisfied"),
            }
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult {
                verdict: Verdict::OperationallyFeasible,
                reason: Cow::Borrowed(
                    "3D distance passes; e/i below threshold (non-governing for guided ops)",
                ),
            }
        } else {
            VerdictResult {
                verdict: Verdict::Caution,
                reason: Cow::Borrowed("analytical safety margins violated"),
            }
        }
    } else {
        VerdictResult {
            verdict: Verdict::Feasible,
            reason: Cow::Borrowed("no safety configured"),
        }
    };

    let abort_threshold_km = config.min_ei_separation_km * insight_thresh::ABORT_EI_COLLAPSE_RATIO;
    if abort_threshold_km > 0.0 {
        if let Some(ref free_drift) = output.safety.free_drift {
            let collapsed: Vec<usize> = free_drift
                .iter()
                .enumerate()
                .filter(|(_, fd)| fd.safety.passive.min_ei_separation_km < abort_threshold_km)
                .map(|(i, _)| i + 1) // 1-based leg numbers for humans
                .collect();

            if !collapsed.is_empty() {
                let leg_list = if let [single] = collapsed.as_slice() {
                    format!("leg {single}")
                } else {
                    format!(
                        "legs {}",
                        collapsed
                            .iter()
                            .map(ToString::to_string)
                            .collect::<Vec<_>>()
                            .join(", ")
                    )
                };
                // Use a semicolon separator rather than nesting another
                // parenthesised clause. The outer verdict renderer wraps
                // `verdict.reason` in a single pair of parens, so this yields
                // a flat "(a; b; c)" list instead of "(a (b) (c))".
                verdict.reason = Cow::Owned(format!(
                    "{}; abort-case passive safety marginal on {leg_list}",
                    verdict.reason,
                ));
            }
        }
    }

    verdict
}

/// Ratio of last-to-first waypoint p50 miss distance, or `None` if not computable.
///
/// Walks `stats.waypoint_miss_km`, filtering `None` entries, and returns
/// `last.p50 / first.p50` when at least two waypoints have data and the
/// first median is strictly positive. Returns `None` otherwise — callers
/// must treat `None` as "no growth signal", not "zero growth".
///
/// Used by [`determine_mc_verdict`] (to append growth text to the verdict
/// reason) and by `insights::mc_insights` (to emit the existing growth
/// insight). Keeping the walk in one place prevents the two consumers
/// from drifting.
pub(super) fn waypoint_miss_growth_ratio(stats: &EnsembleStatistics) -> Option<f64> {
    let medians: Vec<f64> = stats
        .waypoint_miss_km
        .iter()
        .filter_map(|opt| opt.as_ref().map(|s| s.p50))
        .collect();
    if medians.len() < 2 {
        return None;
    }
    let first = medians[0];
    let last = medians[medians.len() - 1];
    if first > 0.0 {
        Some(last / first)
    } else {
        None
    }
}

/// Determine the MC verdict from ensemble statistics.
///
/// Three-tier logic:
/// - **FEASIBLE (100%):** all converged, no collisions, no keep-out violations.
/// - **FEASIBLE (acceptable):** convergence >= 95%, no collisions.
/// - **CAUTION:** otherwise.
///
/// When per-waypoint position-error growth exceeds
/// [`insight_thresh::ERROR_GROWTH_RATIO_ALERT`], the ratio is appended to
/// the verdict reason string — growth alone never flips the verdict tier.
///
/// Returns a [`VerdictResult`] with verdict category and reason.
#[must_use]
pub fn determine_mc_verdict(report: &MonteCarloReport) -> VerdictResult {
    let stats = &report.statistics;
    let all_converged = (stats.convergence_rate - 1.0).abs() < mc_thresh::CONVERGENCE_EXACT_TOL;
    let no_collisions = stats.collision_probability <= rate_thresh::ZERO_VIOLATIONS;
    let no_keepout = stats.keepout_violation_rate <= rate_thresh::ZERO_VIOLATIONS;

    let ei_degraded = stats.ei_violation_rate > rate_thresh::ZERO_VIOLATIONS;

    let mut result = if all_converged && no_collisions && no_keepout {
        VerdictResult {
            verdict: if ei_degraded { Verdict::OperationallyFeasible } else { Verdict::Feasible },
            reason: Cow::Borrowed(if ei_degraded {
                "operational safety holds; passive abort safety degraded under dispersion"
            } else {
                "all safety margins satisfied"
            }),
        }
    } else if stats.convergence_rate >= mc_thresh::CONVERGENCE_ALERT && no_collisions {
        VerdictResult {
            verdict: if ei_degraded { Verdict::OperationallyFeasible } else { Verdict::Feasible },
            reason: Cow::Borrowed(if ei_degraded {
                "convergence acceptable; passive abort safety degraded"
            } else {
                "convergence acceptable, no collisions"
            }),
        }
    } else {
        VerdictResult {
            verdict: Verdict::Caution,
            reason: Cow::Borrowed("review safety and convergence above"),
        }
    };

    if let Some(ratio) = waypoint_miss_growth_ratio(stats) {
        if ratio > insight_thresh::ERROR_GROWTH_RATIO_ALERT {
            result.reason = Cow::Owned(format!(
                "{} (position error growth across waypoints: {ratio:.1}\u{00d7})",
                result.reason,
            ));
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use rpo_core::mission::{
        FormationDesignReport, FreeDriftAnalysis, MonteCarloConfig, MonteCarloMode,
        MonteCarloReport, OperationalSafety, PassiveSafety, PercentileStats,
        PerchEnrichmentResult, SafetyConfig, SafetyMetrics, WaypointMission,
    };
    use rpo_core::pipeline::SafetyAnalysis;
    use rpo_core::types::QuasiNonsingularROE;

    fn stats_with_waypoint_medians(medians: &[Option<f64>]) -> EnsembleStatistics {
        EnsembleStatistics {
            total_dv_km_s: PercentileStats::default(),
            min_rc_distance_km: None,
            min_3d_distance_km: None,
            min_ei_separation_km: None,
            waypoint_miss_km: medians
                .iter()
                .map(|m| {
                    m.map(|p50| PercentileStats {
                        p50,
                        ..Default::default()
                    })
                })
                .collect(),
            collision_probability: 0.0,
            convergence_rate: 1.0,
            ei_violation_rate: 0.0,
            keepout_violation_rate: 0.0,
            dispersion_envelope: vec![],
        }
    }

    fn clean_report(stats: EnsembleStatistics) -> MonteCarloReport {
        MonteCarloReport {
            config: MonteCarloConfig {
                num_samples: 100,
                dispersions: Default::default(),
                mode: MonteCarloMode::ClosedLoop,
                seed: Some(42),
                trajectory_steps: 50,
            },
            nominal_dv_km_s: 0.0,
            nominal_safety: None,
            statistics: stats,
            samples: vec![],
            num_failures: 0,
            elapsed_wall_s: 0.0,
            covariance_cross_check: None,
        }
    }

    #[test]
    fn waypoint_miss_growth_ratio_none_for_insufficient_data() {
        let stats = stats_with_waypoint_medians(&[]);
        assert!(waypoint_miss_growth_ratio(&stats).is_none());
        let stats = stats_with_waypoint_medians(&[Some(0.001)]);
        assert!(waypoint_miss_growth_ratio(&stats).is_none());
    }

    #[test]
    fn waypoint_miss_growth_ratio_skips_none_entries() {
        // First median 1 m, last median 5 m, intermediate None must not break walking.
        let stats =
            stats_with_waypoint_medians(&[Some(0.001), None, Some(0.003), Some(0.005)]);
        let ratio = waypoint_miss_growth_ratio(&stats).expect("ratio");
        assert!((ratio - 5.0).abs() < 1e-12);
    }

    #[test]
    fn waypoint_miss_growth_ratio_none_when_first_is_zero() {
        let stats = stats_with_waypoint_medians(&[Some(0.0), Some(0.005)]);
        assert!(waypoint_miss_growth_ratio(&stats).is_none());
    }

    #[test]
    fn determine_mc_verdict_appends_growth_when_above_alert() {
        // First 1m, last 5m → 5.0× > 3.0 threshold.
        let stats = stats_with_waypoint_medians(&[Some(0.001), Some(0.005)]);
        let report = clean_report(stats);
        let verdict = determine_mc_verdict(&report);
        assert!(verdict.verdict.is_feasible());
        assert!(
            verdict.reason.contains("position error growth"),
            "reason should include growth text, got: {}",
            verdict.reason
        );
        assert!(
            verdict.reason.contains("5.0"),
            "reason should include 5.0 ratio, got: {}",
            verdict.reason
        );
    }

    #[test]
    fn determine_mc_verdict_omits_growth_when_below_alert() {
        // First 1m, last 2m → 2.0× ≤ 3.0 threshold.
        let stats = stats_with_waypoint_medians(&[Some(0.001), Some(0.002)]);
        let report = clean_report(stats);
        let verdict = determine_mc_verdict(&report);
        assert!(!verdict.reason.contains("position error growth"));
    }

    #[test]
    fn fmt_velocity_target_zero() {
        assert_eq!(fmt_velocity_target(&Vector3::zeros()), "\u{2014}");
    }

    #[test]
    fn fmt_velocity_target_dominant_positive_intrack() {
        // 100% in-track: 0.001 km/s = 1.0 m/s
        let v = Vector3::new(0.0, 0.001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "+1.0 I");
    }

    #[test]
    fn fmt_velocity_target_dominant_negative_intrack() {
        let v = Vector3::new(0.0, -0.001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "-1.0 I");
    }

    #[test]
    fn fmt_velocity_target_no_dominant() {
        // Three roughly equal components — none reaches 70%
        let v = Vector3::new(0.001, 0.001, 0.001);
        let mag_m_s = v.norm() * KM_TO_M;
        assert_eq!(fmt_velocity_target(&v), format!("{mag_m_s:.1}"));
    }

    #[test]
    fn fmt_velocity_target_dominant_crosstrack() {
        // ~98% cross-track
        let v = Vector3::new(0.0002, 0.0, 0.001);
        assert_eq!(fmt_velocity_target(&v), "+1.0 C");
    }

    #[test]
    fn fmt_velocity_target_dominant_radial() {
        let v = Vector3::new(0.002, 0.0001, 0.0);
        assert_eq!(fmt_velocity_target(&v), "+2.0 R");
    }

    // ── Verdict helpers & tests ─────────────────────────────────────────

    /// Build a `SafetyMetrics` with the given 3D distance and e/i separation.
    fn safety_metrics_with(min_distance_3d_km: f64, min_ei_separation_km: f64) -> SafetyMetrics {
        SafetyMetrics {
            operational: OperationalSafety {
                min_rc_separation_km: min_distance_3d_km,
                min_distance_3d_km,
                min_rc_leg_index: 0,
                min_rc_elapsed_s: 0.0,
                min_rc_ric_position_km: Vector3::zeros(),
                min_3d_leg_index: 0,
                min_3d_elapsed_s: 0.0,
                min_3d_ric_position_km: Vector3::zeros(),
            },
            passive: PassiveSafety {
                min_ei_separation_km,
                de_magnitude: 0.0,
                di_magnitude: 0.0,
                ei_phase_angle_rad: 0.0,
            },
        }
    }

    /// Build a `FreeDriftAnalysis` whose passive e/i separation equals
    /// `min_ei_separation_km`. Used only for verdict tests — trajectory is
    /// empty because `determine_verdict` does not walk it.
    fn free_drift_analysis_with_ei(min_ei_separation_km: f64) -> FreeDriftAnalysis {
        FreeDriftAnalysis {
            trajectory: vec![],
            safety: safety_metrics_with(1.0, min_ei_separation_km),
            bounded_motion_residual: 0.0,
        }
    }

    /// Minimal `PipelineOutput` with enrichment active and numerical safety
    /// passing under 3D distance but failing under e/i separation — the
    /// configuration that triggers the `OperationallyFeasible` branch in
    /// `determine_verdict`.
    fn pipeline_output_passing_3d_with_enrichment() -> PipelineOutput {
        use rpo_core::mission::MissionPhase;
        use rpo_core::types::KeplerianElements;

        let placeholder_elements = KeplerianElements {
            a_km: 6786.0,
            e: 0.0001,
            i_rad: 51.6_f64.to_radians(),
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        let placeholder_roe = QuasiNonsingularROE::default();

        let mission = WaypointMission {
            legs: vec![],
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            // 3D distance passes (1 km > 0.050 threshold), e/i fails
            // (0.001 km < 0.100 threshold). Triggers OperationallyFeasible
            // when enrichment is active.
            safety: Some(safety_metrics_with(1.0, 0.001)),
            covariance: None,
            eclipse: None,
        };

        let formation_design = FormationDesignReport {
            perch: PerchEnrichmentResult::Baseline(placeholder_roe),
            waypoints: vec![],
            transit_safety: vec![],
            mission_min_ei_separation_km: None,
            drift_prediction: None,
        };

        PipelineOutput {
            phase: MissionPhase::Proximity {
                roe: placeholder_roe,
                chief_elements: placeholder_elements,
                deputy_elements: placeholder_elements,
                separation_km: 0.0,
                delta_r_over_r: 0.0,
            },
            transfer: None,
            transfer_eclipse: None,
            perch_roe: placeholder_roe,
            mission,
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            auto_drag_config: None,
            covariance: None,
            monte_carlo: None,
            safety: SafetyAnalysis {
                free_drift: None,
                poca: None,
                free_drift_poca: None,
                cola: None,
                secondary_conjunctions: None,
                cola_skipped: None,
            },
            formation_design: Some(formation_design),
        }
    }

    #[test]
    fn determine_verdict_flags_abort_case_passive_safety_collapse() {
        // Build a PipelineOutput whose numerical safety passes, enrichment
        // active, but whose free_drift analysis shows leg 2 with
        // min_ei_separation_km essentially zero (0.0002 km = 0.2 m,
        // threshold = 0.100 km = 100 m, ratio = 0.002).
        let mut output = pipeline_output_passing_3d_with_enrichment();
        output.safety.free_drift = Some(vec![
            free_drift_analysis_with_ei(0.120),  // leg 1: healthy (120 m)
            free_drift_analysis_with_ei(0.0002), // leg 2: collapsed (0.2 m)
            free_drift_analysis_with_ei(0.0001), // leg 3: collapsed (0.1 m)
        ]);
        let config = SafetyConfig {
            min_distance_3d_km: 0.050,
            min_ei_separation_km: 0.100,
        };

        let verdict = determine_verdict(&output, &config, None);

        assert!(
            verdict.reason.to_lowercase().contains("abort"),
            "verdict reason must surface abort-case collapse; got: {}",
            verdict.reason,
        );
        assert!(
            verdict.reason.contains("leg 2") || verdict.reason.contains("legs 2"),
            "verdict reason must identify the offending leg(s); got: {}",
            verdict.reason,
        );
    }
}

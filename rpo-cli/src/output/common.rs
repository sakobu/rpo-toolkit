//! Shared output helpers.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anise::prelude::Almanac;
use indicatif::{ProgressBar, ProgressStyle};
use serde::Serialize;

use rpo_core::mission::{
    assess_safety, AvoidanceManeuver, ColaConfig, EiAlignment, MonteCarloReport, SafetyConfig,
    SafetyRequirements, ValidationReport,
};
use rpo_core::pipeline::{
    resolve_propagator, to_propagation_model, PipelineInput, PipelineOutput, TransferResult,
};
use rpo_core::propagation::{extract_dmf_rates, DragConfig, PropagationModel};
use rpo_core::types::SpacecraftConfig;

use crate::error::CliError;

use super::thresholds::{mc as mc_thresh, rate as rate_thresh};

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
    pub reason: &'static str,
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

/// ROE component below which the value is treated as effectively zero for display.
const ROE_DISPLAY_ZERO_THRESHOLD: f64 = 1e-15;

/// Format a value stored in km/s as m/s with given decimal places.
///
/// Example: `fmt_m_s(0.5432, 1)` returns `"543.2 m/s"`.
#[must_use]
pub fn fmt_m_s(value_km_s: f64, decimals: usize) -> String {
    let m_s = value_km_s * 1000.0;
    format!("{m_s:.decimals$} m/s")
}

/// Format a value stored in km as meters with given decimal places.
///
/// Example: `fmt_m(0.150, 1)` returns `"150.0 m"`.
#[must_use]
pub fn fmt_m(value_km: f64, decimals: usize) -> String {
    let m = value_km * 1000.0;
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


// ── Per-leg error stats ────────────────────────────────────────────────

/// Per-leg position error statistics (km).
// All fields carry the `_km` unit suffix per our naming rules.
// Clippy's `struct_field_names` lint fires on the shared suffix, but
// removing it would violate the project's unit-suffix convention.
#[allow(clippy::struct_field_names)]
pub struct LegErrorStats {
    /// Maximum position error (km).
    pub max_km: f64,
    /// Mean position error (km).
    pub mean_km: f64,
    /// RMS position error (km).
    pub rms_km: f64,
}

/// Compute max/mean/RMS position errors for a single leg in a single pass.
///
/// Returns `None` if `points` is empty.
#[must_use]
pub fn compute_leg_error_stats(
    points: &[rpo_core::mission::ValidationPoint],
) -> Option<LegErrorStats> {
    // Filter post-COLA points: their "error" reflects intentional trajectory change,
    // not model fidelity. Consistent with core's compute_report_statistics.
    let (count, max, sum, sum_sq) = points.iter().filter(|p| !p.post_cola).fold(
        (0u32, 0.0_f64, 0.0_f64, 0.0_f64),
        |(n, mx, s, sq), p| {
            let e = p.position_error_km;
            (n + 1, mx.max(e), s + e, sq + e * e)
        },
    );
    if count == 0 {
        return None;
    }
    let n = f64::from(count);
    Some(LegErrorStats {
        max_km: max,
        mean_km: sum / n,
        rms_km: (sum_sq / n).sqrt(),
    })
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
#[must_use]
pub fn determine_verdict(
    output: &PipelineOutput,
    config: &SafetyConfig,
    validation: Option<&ValidationReport>,
) -> VerdictResult {
    let enrichment_active = output.formation_design.is_some();

    if let Some(report) = validation {
        let assessment = assess_safety(&report.numerical_safety, config);
        if assessment.overall_pass {
            VerdictResult { verdict: Verdict::Feasible, reason: "safety margins satisfied, Nyx full-physics" }
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult {
                verdict: Verdict::OperationallyFeasible,
                reason: "3D distance passes (Nyx); e/i below threshold (non-governing for guided ops)",
            }
        } else {
            VerdictResult { verdict: Verdict::Caution, reason: "safety margins violated, Nyx full-physics" }
        }
    } else if let Some(ref safety) = output.mission.safety {
        let assessment = assess_safety(safety, config);
        if assessment.overall_pass {
            VerdictResult { verdict: Verdict::Feasible, reason: "analytical safety margins satisfied" }
        } else if enrichment_active && assessment.distance_3d_pass {
            VerdictResult {
                verdict: Verdict::OperationallyFeasible,
                reason: "3D distance passes; e/i below threshold (non-governing for guided ops)",
            }
        } else {
            VerdictResult { verdict: Verdict::Caution, reason: "analytical safety margins violated" }
        }
    } else {
        VerdictResult { verdict: Verdict::Feasible, reason: "no safety configured" }
    }
}

/// Determine the MC verdict from ensemble statistics.
///
/// Three-tier logic:
/// - **FEASIBLE (100%):** all converged, no collisions, no keep-out violations.
/// - **FEASIBLE (acceptable):** convergence >= 95%, no collisions.
/// - **CAUTION:** otherwise.
///
/// Returns a [`VerdictResult`] with verdict category and reason.
#[must_use]
pub fn determine_mc_verdict(report: &MonteCarloReport) -> VerdictResult {
    let stats = &report.statistics;
    let all_converged = (stats.convergence_rate - 1.0).abs() < mc_thresh::CONVERGENCE_EXACT_TOL;
    let no_collisions = stats.collision_probability <= rate_thresh::ZERO_VIOLATIONS;
    let no_keepout = stats.keepout_violation_rate <= rate_thresh::ZERO_VIOLATIONS;

    let ei_degraded = stats.ei_violation_rate > rate_thresh::ZERO_VIOLATIONS;

    if all_converged && no_collisions && no_keepout {
        VerdictResult {
            verdict: if ei_degraded { Verdict::OperationallyFeasible } else { Verdict::Feasible },
            reason: if ei_degraded {
                "operational safety holds; passive abort safety degraded under dispersion"
            } else {
                "all safety margins satisfied"
            },
        }
    } else if stats.convergence_rate >= mc_thresh::CONVERGENCE_ALERT && no_collisions {
        VerdictResult {
            verdict: if ei_degraded { Verdict::OperationallyFeasible } else { Verdict::Feasible },
            reason: if ei_degraded {
                "convergence acceptable; passive abort safety degraded"
            } else {
                "convergence acceptable, no collisions"
            },
        }
    } else {
        VerdictResult {
            verdict: Verdict::Caution,
            reason: "review safety and convergence above",
        }
    }
}

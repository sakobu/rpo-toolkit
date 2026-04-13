use std::fmt::Write;

use hifitime::Epoch;
use nalgebra::Vector3;
use rpo_core::pipeline::PipelineOutput;
use crate::output::fmt::{cola_dv_summary, fmt_m_s};
use crate::output::insights;
use crate::output::thresholds::insight::ALERT_BOX_MAX_ITEMS;

/// Which report tier a shared formatter is writing into.
///
/// Used by cross-report formatters to vary guidance text (e.g. "run
/// `validate`" is self-referential inside a validate report and should
/// instead point at a section downstream).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ReportContext {
    /// Writing into a `mission` report.
    Mission,
    /// Writing into a `validate` report.
    Validate,
}

/// Return a pass/fail status string with emoji prefix for summary tables.
pub(super) fn status_emoji(pass: bool) -> &'static str {
    if pass {
        "\u{2705} PASS"
    } else {
        "\u{274c} FAIL"
    }
}

/// Write a one-line COLA dv callout (blockquote) if COLA maneuvers exist.
pub(super) fn write_cola_callout(out: &mut String, output: &PipelineOutput) {
    if let Some((cola_dv, num_burns)) = cola_dv_summary(output.safety.cola.as_deref()) {
        let burn_label = if num_burns == 1 { "burn" } else { "burns" };
        let total_with_cola = output.total_dv_km_s + cola_dv;
        let _ = writeln!(
            out,
            "> COLA: +{} ({num_burns} {burn_label}) \u{2192} {} total (w/ COLA)\n",
            fmt_m_s(cola_dv, 2),
            fmt_m_s(total_with_cola, 1),
        );
    }
}

/// Partition an insight list into `(alert_items, remainder)`.
///
/// Alert items are the first `max_alert` critical/warning insights, in the
/// order they appear. Remainder contains everything else — info items plus
/// any critical/warning overflow. Callers write `alert_items` at the top
/// of the report and `remainder` at the bottom, so no insight is shown
/// twice.
///
/// # Invariants
/// - `alert.len() <= max_alert` — the alert box is hard-capped.
/// - Any critical/warning items beyond `max_alert` land in `remainder`,
///   not dropped.
/// - No insight appears in both lists.
/// - `max_alert = 0` is legal: every insight goes to `remainder`.
/// - Input is consumed by value to avoid the prior double-clone.
fn partition_insights_for_alert(
    insights_in: Vec<insights::Insight>,
    max_alert: usize,
) -> (Vec<insights::Insight>, Vec<insights::Insight>) {
    let mut alert = Vec::with_capacity(max_alert);
    let mut remainder = Vec::with_capacity(insights_in.len());
    for insight in insights_in {
        let is_critical_or_warning = matches!(
            insight.severity,
            insights::Severity::Critical | insights::Severity::Warning
        );
        if alert.len() < max_alert && is_critical_or_warning {
            alert.push(insight);
        } else {
            remainder.push(insight);
        }
    }
    (alert, remainder)
}

/// Write an alert box after the summary table, surfacing critical/warning
/// insights so an operator scanning the first page sees them immediately.
///
/// The caller is responsible for selecting which insights go here (typically
/// via [`partition_insights_for_alert`] or [`emit_alert_box`]). Emits nothing
/// for an empty list. Severity prefixes come from [`insights::Severity::markdown_prefix`]
/// so the label never drifts between the alert box and the bottom list.
///
/// # Arguments
/// - `out` — output buffer; the alert markdown is appended.
/// - `alert_items` — the pre-selected alert insights (critical / warning),
///   typically the first N items returned by `partition_insights_for_alert`.
pub(super) fn write_alert_box(out: &mut String, alert_items: &[insights::Insight]) {
    if alert_items.is_empty() {
        return;
    }
    for insight in alert_items {
        let _ = writeln!(out, "> {} {}\n", insight.severity.markdown_prefix(), insight.message);
    }
}

/// Partition a fully-built insight list, write the above-the-fold alert
/// box, and return the remainder for the caller to render at the bottom
/// of the report.
///
/// Consolidates the build-then-partition-then-write pattern repeated in
/// `mission.rs`, `validation.rs`, and `mc.rs`.
///
/// # Arguments
/// - `out` — output buffer; the alert-box markdown is appended.
/// - `all_insights` — fully-built insight list, consumed by value.
///
/// # Returns
/// The insights NOT surfaced in the alert box. Callers typically pass
/// this to `write_insights` at the bottom of the report.
pub(super) fn emit_alert_box(
    out: &mut String,
    all_insights: Vec<insights::Insight>,
) -> Vec<insights::Insight> {
    let (alert_items, remainder) = partition_insights_for_alert(all_insights, ALERT_BOX_MAX_ITEMS);
    write_alert_box(out, &alert_items);
    remainder
}

/// Write cross-tier insights as blockquote lines with severity prefixes.
pub(super) fn write_insights(out: &mut String, insight_list: &[insights::Insight]) {
    for insight in insight_list {
        let _ = writeln!(out, "> {} {}\n", insight.severity.markdown_prefix(), insight.message);
    }
}

/// Classification of a single burn in the consolidated mission timeline.
///
/// Parallels the local enums that used to live in `schedule.rs` and
/// `eclipse.rs`; consolidated here so the two sections render the same
/// canonical list.
#[derive(Debug)]
pub(super) enum BurnKind {
    /// Lambert transfer departure impulse (ECI frame — no RIC Δv).
    LambertDeparture,
    /// Lambert transfer arrival impulse (ECI frame — no RIC Δv).
    LambertArrival,
    /// Waypoint-leg departure impulse (RIC frame).
    WaypointDeparture { leg: usize },
    /// Waypoint-leg arrival impulse (RIC frame).
    WaypointArrival { leg: usize },
    /// Collision-avoidance maneuver (RIC frame).
    Cola { leg: usize },
}

impl std::fmt::Display for BurnKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LambertDeparture => write!(f, "Lambert dep"),
            Self::LambertArrival => write!(f, "Lambert arr"),
            Self::WaypointDeparture { leg } => write!(f, "WP{} dep", leg + 1),
            Self::WaypointArrival { leg } => write!(f, "WP{} arr", leg + 1),
            Self::Cola { leg } => write!(f, "COLA (L{})", leg + 1),
        }
    }
}

/// A single maneuver in the consolidated mission timeline.
///
/// Carries its kind, epoch, optional RIC Δv (`None` for Lambert ECI burns),
/// and magnitude. Shared by the schedule and eclipse sections; the single
/// source of truth for "what burns does this mission fire, in what order".
#[derive(Debug)]
pub(super) struct Burn {
    pub kind: BurnKind,
    pub epoch: Epoch,
    pub dv_ric_km_s: Option<Vector3<f64>>,
    pub dv_mag_km_s: f64,
}

/// Collect every burn in the mission timeline and return them sorted by
/// epoch. Walks `output.transfer`, `output.mission.legs` (WP dep + arr),
/// and `output.safety.cola`.
///
/// # Invariants
/// - Empty pipeline (no transfer, no legs, no COLA) returns an empty Vec.
/// - Sort key is `epoch`; ties are resolved in the order of insertion:
///   transfer burns first, then per-leg waypoint burns, then COLA burns.
pub(super) fn collect_burns(output: &PipelineOutput) -> Vec<Burn> {
    let cola_count = output.safety.cola.as_ref().map_or(0, Vec::len);
    let capacity = output.mission.legs.len() * 2
        + if output.transfer.is_some() { 2 } else { 0 }
        + cola_count;
    let mut burns: Vec<Burn> = Vec::with_capacity(capacity);

    if let Some(ref transfer) = output.transfer {
        burns.push(Burn {
            kind: BurnKind::LambertDeparture,
            epoch: transfer.departure_state.epoch,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.departure_dv_eci_km_s.norm(),
        });
        burns.push(Burn {
            kind: BurnKind::LambertArrival,
            epoch: transfer.arrival_state.epoch,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.arrival_dv_eci_km_s.norm(),
        });
    }

    for (i, leg) in output.mission.legs.iter().enumerate() {
        burns.push(Burn {
            kind: BurnKind::WaypointDeparture { leg: i },
            epoch: leg.departure_maneuver.epoch,
            dv_ric_km_s: Some(leg.departure_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.departure_maneuver.dv_ric_km_s.norm(),
        });
        burns.push(Burn {
            kind: BurnKind::WaypointArrival { leg: i },
            epoch: leg.arrival_maneuver.epoch,
            dv_ric_km_s: Some(leg.arrival_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.arrival_maneuver.dv_ric_km_s.norm(),
        });
    }

    if let Some(ref cola) = output.safety.cola {
        for m in cola {
            burns.push(Burn {
                kind: BurnKind::Cola { leg: m.leg_index },
                epoch: m.epoch,
                dv_ric_km_s: Some(m.dv_ric_km_s),
                dv_mag_km_s: m.dv_ric_km_s.norm(),
            });
        }
    }

    burns.sort_by_key(|b| b.epoch);
    burns
}

use std::fmt::Write;

use rpo_core::pipeline::PipelineOutput;
use rpo_core::propagation::{DragConfig, PropagationModel};

use crate::output::common::{cola_dv_summary, fmt_m_s};
use crate::output::insights;

/// Which report tier a shared formatter is writing into.
///
/// Used by cross-report formatters to vary guidance text (e.g. "run
/// `validate`" is self-referential inside a validate report and should
/// instead point at a section downstream).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ReportContext {
    /// Writing into a `mission` report.
    Mission,
    /// Writing into a `validate` report.
    Validate,
}

pub(super) fn propagator_label(propagator: &PropagationModel, auto_drag: bool) -> &'static str {
    match propagator {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => "J2+Drag STM (auto-derived)",
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    }
}

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

pub(super) fn write_drag_table(out: &mut String, drag: &DragConfig) {
    let _ = writeln!(out, "### Auto-Derived Drag\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");
    let _ = writeln!(out, "| da_dot | {:+.6e} |", drag.da_dot);
    let _ = writeln!(out, "| dex_dot | {:+.6e} |", drag.dex_dot);
    let _ = writeln!(out, "| dey_dot | {:+.6e} |", drag.dey_dot);
    let _ = writeln!(out);
}

pub(super) fn write_insights(out: &mut String, insight_list: &[insights::Insight]) {
    for insight in insight_list {
        let prefix = match insight.severity {
            insights::Severity::Critical => "**CRITICAL:**",
            insights::Severity::Warning => "**Warning:**",
            insights::Severity::Info => "**Insight:**",
        };
        let _ = writeln!(out, "> {prefix} {}\n", insight.message);
    }
}

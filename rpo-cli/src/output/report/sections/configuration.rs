//! Configuration section writer.
//!
//! Emits an upfront assumptions block so an operator knows *what* they are
//! looking at before reading *what it says*.

use std::fmt::Write;

use rpo_core::pipeline::{PipelineInput, PipelineOutput};

use rpo_core::propagation::PropagationModel;

use crate::output::fmt::{fmt_epoch_rounded, fmt_m};

/// Write the Configuration section summarising the gravity model, drag model,
/// epoch, thresholds, and propagator used for a given report.
///
/// # Arguments
/// - `out` — output buffer; the section markdown is appended.
/// - `input` — pipeline input (source of epoch and safety thresholds).
/// - `output` — pipeline output (source of `auto_drag_config`).
/// - `propagator_display` — pre-built propagator label (typically from
///   [`propagator_label`]).
pub(crate) fn write_configuration_section(
    out: &mut String,
    input: &PipelineInput,
    output: &PipelineOutput,
    propagator_display: &str,
) {
    let _ = writeln!(out, "## Configuration\n");
    let _ = writeln!(out, "| Parameter | Value |");
    let _ = writeln!(out, "| --- | --- |");

    let _ = writeln!(out, "| Propagator | {propagator_display} |");

    // Drag model
    if let Some(ref drag) = output.auto_drag_config {
        let _ = writeln!(
            out,
            "| Drag | da_dot={:+.3e}, dex_dot={:+.3e}, dey_dot={:+.3e} |",
            drag.da_dot, drag.dex_dot, drag.dey_dot,
        );
    } else {
        let _ = writeln!(out, "| Drag | None |");
    }

    // Epoch (from chief state)
    let _ = writeln!(
        out,
        "| Epoch | {} |",
        fmt_epoch_rounded(input.base.chief.epoch),
    );

    // Central body
    let _ = writeln!(
        out,
        "| Central body | Earth (\u{03bc} = {:.4} km\u{00b3}/s\u{00b2}) |",
        rpo_core::constants::MU_EARTH,
    );

    // Safety thresholds
    let sc = input.base.config.safety.unwrap_or_default();
    let _ = writeln!(
        out,
        "| Min 3D distance threshold | {} |",
        fmt_m(sc.min_distance_3d_km, 0),
    );
    let _ = writeln!(
        out,
        "| Min e/i separation threshold | {} |",
        fmt_m(sc.min_ei_separation_km, 0),
    );

    let _ = writeln!(out);
}

/// Canonical label for the effective propagator when drag was auto-derived
/// by the pipeline (or when the user explicitly passed `--auto-drag`).
/// Named so both `raw_propagator_label` and `propagator_label` reference
/// exactly one source string — preventing silent divergence if the wording
/// ever changes.
const AUTO_DERIVED_LABEL: &str = "J2+Drag STM (auto-derived)";

/// Map a propagation model to its human-readable label (does not
/// pick up auto-drag overrides).
fn raw_propagator_label(propagator: &PropagationModel, auto_drag: bool) -> &'static str {
    match propagator {
        PropagationModel::J2Stm => "J2 STM",
        PropagationModel::J2DragStm { .. } if auto_drag => AUTO_DERIVED_LABEL,
        PropagationModel::J2DragStm { .. } => "J2+Drag STM (user-specified)",
    }
}

/// Build the propagator display label for the Configuration section.
///
/// Prefers the *effective* propagator over the base: when drag has been
/// auto-derived (`output.auto_drag_config.is_some()`), the samples actually
/// ran under J2+Drag even if `input.base.propagator` is plain J2. Reading
/// only the base would label mc.md as "J2 STM" while the MC header block
/// says "J2+Drag STM (analytical)" — the two disagree.
///
/// # Arguments
/// - `input` — pipeline input (source of the base propagator model).
/// - `output` — pipeline output (source of `auto_drag_config` override).
/// - `auto_drag` — whether the caller was invoked with `--auto-drag`.
pub(crate) fn propagator_label(
    input: &PipelineInput,
    output: &PipelineOutput,
    auto_drag: bool,
) -> &'static str {
    // If drag was derived, the effective propagator is J2+Drag regardless of base.
    if output.auto_drag_config.is_some() {
        return AUTO_DERIVED_LABEL;
    }
    let propagator = rpo_core::pipeline::to_propagation_model(&input.base.propagator);
    raw_propagator_label(&propagator, auto_drag)
}

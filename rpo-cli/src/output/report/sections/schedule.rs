//! Maneuver schedule section writer.

use std::fmt::Write;

use rpo_core::pipeline::PipelineOutput;

use crate::output::fmt::{fmt_epoch_rounded, KM_TO_M};

/// Which report the maneuver schedule is being rendered into.
///
/// Controls both the section heading and whether COLA rows carry a
/// reference-marker suffix. Encoded as an enum so the mission / MC
/// dispatch is explicit and cannot be accidentally mixed (e.g. mission
/// heading with MC marker, or vice versa).
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum ScheduleVariant {
    /// Standard mission.md schedule. No COLA-row marker.
    Mission,
    /// mc.md schedule. COLA rows carry a `†` suffix because Monte Carlo
    /// samples propagate the pre-COLA baseline — the burn is shown for
    /// reference only, not executed in the ensemble.
    MonteCarlo,
}

impl ScheduleVariant {
    /// Full section heading line, including the leading `##`.
    fn section_title(self) -> &'static str {
        match self {
            Self::Mission => "## Maneuver Schedule",
            Self::MonteCarlo => "## Nominal Maneuver Schedule (reference)",
        }
    }

    /// Optional suffix appended to COLA rows' Type cell. `None` means
    /// no marker; callers match the presence of the marker to the
    /// footnote they emit beneath the table.
    fn cola_marker(self) -> Option<&'static str> {
        match self {
            Self::Mission => None,
            Self::MonteCarlo => Some("\u{2020}"),
        }
    }
}

/// Write a consolidated chronological maneuver schedule table in the
/// default mission-report variant. Thin wrapper around
/// [`write_maneuver_schedule_with`].
pub(crate) fn write_maneuver_schedule(out: &mut String, output: &PipelineOutput) {
    write_maneuver_schedule_with(out, output, ScheduleVariant::Mission);
}

/// Write a consolidated chronological maneuver schedule table.
///
/// Collects Lambert, waypoint, and COLA burns from the pipeline output
/// (via [`super::super::helpers::collect_burns`]), then renders them in
/// chronological order.
///
/// # Arguments
/// - `out` — output buffer; the rendered table is appended.
/// - `output` — pipeline output carrying `transfer`, `mission.legs`,
///   and `safety.cola` maneuvers.
/// - `variant` — mission vs MC dispatch (see [`ScheduleVariant`]).
pub(crate) fn write_maneuver_schedule_with(
    out: &mut String,
    output: &PipelineOutput,
    variant: ScheduleVariant,
) {
    let burns = super::super::helpers::collect_burns(output);
    if burns.is_empty() {
        return;
    }

    let _ = writeln!(out, "{}\n", variant.section_title());
    let _ = writeln!(
        out,
        "| # | Epoch (UTC) | Type | \u{0394}vR (m/s) | \u{0394}vI (m/s) | \u{0394}vC (m/s) | \\|\u{0394}v\\| (m/s) |",
    );
    let _ = writeln!(
        out,
        "|---|-------------|------|-----------|-----------|-----------|------------|",
    );

    let cola_marker = variant.cola_marker();
    for (i, burn) in burns.iter().enumerate() {
        // Mark COLA rows with the configured suffix (e.g. `"†"`) so mc.md can
        // visually flag the row as "not executed in Monte Carlo samples".
        let type_cell = match (&burn.kind, cola_marker) {
            (super::super::helpers::BurnKind::Cola { .. }, Some(mark)) => {
                format!("{kind} {mark}", kind = burn.kind)
            }
            _ => format!("{}", burn.kind),
        };
        let epoch_str = fmt_epoch_rounded(burn.epoch);
        match burn.dv_ric_km_s {
            Some(v) => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | {:.2} | {:.2} | {:.2} | {:.1} |",
                    i + 1,
                    epoch_str,
                    v.x * KM_TO_M,
                    v.y * KM_TO_M,
                    v.z * KM_TO_M,
                    burn.dv_mag_km_s * KM_TO_M,
                );
            }
            None => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | \u{2014} | \u{2014} | \u{2014} | {:.1} |",
                    i + 1,
                    epoch_str,
                    burn.dv_mag_km_s * KM_TO_M,
                );
            }
        }
    }
    let _ = writeln!(out);

    write_same_epoch_footnote(out, &burns);
}

/// Emit an explanatory footnote when a Lambert arrival row and a WP1 departure
/// row share the same epoch. Without context an operator asks "is this a
/// combined burn? two impulses? a coast?"
fn write_same_epoch_footnote(out: &mut String, burns: &[super::super::helpers::Burn]) {
    use super::super::helpers::BurnKind;
    let lambert_arr = burns
        .iter()
        .find(|b| matches!(b.kind, BurnKind::LambertArrival))
        .map(|b| b.epoch);
    let wp1_dep = burns
        .iter()
        .find(|b| matches!(b.kind, BurnKind::WaypointDeparture { leg: 0 }))
        .map(|b| b.epoch);

    if let (Some(arr), Some(dep)) = (lambert_arr, wp1_dep) {
        if arr == dep {
            let _ = writeln!(
                out,
                "> Lambert arrival and WP1 departure share the same epoch. \
                 The Lambert arrival impulse transitions the deputy onto the proximity orbit; \
                 the WP1 departure impulse begins the first waypoint-targeting leg. Both are \
                 applied instantaneously at the handoff point.\n",
            );
        }
    }
}

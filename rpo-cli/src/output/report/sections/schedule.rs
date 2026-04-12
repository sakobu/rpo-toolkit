//! Maneuver schedule section writer.

use std::fmt::Write;

use rpo_core::pipeline::PipelineOutput;

use crate::output::fmt::KM_TO_M;

/// Maneuver type for the consolidated schedule table.
enum ScheduleEntryKind {
    /// Lambert transfer departure impulse (ECI frame).
    LambertDeparture,
    /// Lambert transfer arrival impulse (ECI frame).
    LambertArrival,
    /// Waypoint-leg departure impulse (RIC frame).
    WaypointDeparture { leg: usize },
    /// Waypoint-leg arrival impulse (RIC frame).
    WaypointArrival { leg: usize },
    /// Collision avoidance maneuver (RIC frame).
    Cola { leg: usize },
}

impl std::fmt::Display for ScheduleEntryKind {
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

/// A single row in the consolidated maneuver schedule table.
struct ScheduleEntry {
    /// Burn epoch (UTC).
    epoch: hifitime::Epoch,
    /// Maneuver classification.
    kind: ScheduleEntryKind,
    /// RIC Dv components (`None` for Lambert burns which are ECI-only).
    dv_ric_km_s: Option<nalgebra::Vector3<f64>>,
    /// Total Dv magnitude (km/s).
    dv_mag_km_s: f64,
}

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
/// Collects Lambert, waypoint, and COLA burns from the pipeline output,
/// sorts by epoch, and renders a single table.
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
    let cola_count = output.safety.cola.as_ref().map_or(0, Vec::len);
    let capacity = output.mission.legs.len() * 2
        + if output.transfer.is_some() { 2 } else { 0 }
        + cola_count;
    let mut entries: Vec<ScheduleEntry> = Vec::with_capacity(capacity);

    // Lambert burns (ECI Dv — no RIC decomposition)
    if let Some(ref transfer) = output.transfer {
        entries.push(ScheduleEntry {
            epoch: transfer.departure_state.epoch,
            kind: ScheduleEntryKind::LambertDeparture,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.departure_dv_eci_km_s.norm(),
        });
        entries.push(ScheduleEntry {
            epoch: transfer.arrival_state.epoch,
            kind: ScheduleEntryKind::LambertArrival,
            dv_ric_km_s: None,
            dv_mag_km_s: transfer.arrival_dv_eci_km_s.norm(),
        });
    }

    // Waypoint departure / arrival burns (RIC Dv)
    for (i, leg) in output.mission.legs.iter().enumerate() {
        entries.push(ScheduleEntry {
            epoch: leg.departure_maneuver.epoch,
            kind: ScheduleEntryKind::WaypointDeparture { leg: i },
            dv_ric_km_s: Some(leg.departure_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.departure_maneuver.dv_ric_km_s.norm(),
        });
        entries.push(ScheduleEntry {
            epoch: leg.arrival_maneuver.epoch,
            kind: ScheduleEntryKind::WaypointArrival { leg: i },
            dv_ric_km_s: Some(leg.arrival_maneuver.dv_ric_km_s),
            dv_mag_km_s: leg.arrival_maneuver.dv_ric_km_s.norm(),
        });
    }

    // COLA burns (RIC Dv)
    if let Some(ref cola) = output.safety.cola {
        for m in cola {
            entries.push(ScheduleEntry {
                epoch: m.epoch,
                kind: ScheduleEntryKind::Cola { leg: m.leg_index },
                dv_ric_km_s: Some(m.dv_ric_km_s),
                dv_mag_km_s: m.dv_ric_km_s.norm(),
            });
        }
    }

    if entries.is_empty() {
        return;
    }

    entries.sort_by_key(|e| e.epoch);

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
    for (i, entry) in entries.iter().enumerate() {
        // Mark COLA rows with the configured suffix (e.g. `"†"`) so mc.md can
        // visually flag the row as "not executed in Monte Carlo samples".
        let type_cell = match (&entry.kind, cola_marker) {
            (ScheduleEntryKind::Cola { .. }, Some(mark)) => {
                format!("{kind} {mark}", kind = entry.kind)
            }
            _ => format!("{}", entry.kind),
        };
        match entry.dv_ric_km_s {
            Some(v) => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | {:.2} | {:.2} | {:.2} | {:.1} |",
                    i + 1,
                    entry.epoch,
                    v.x * KM_TO_M,
                    v.y * KM_TO_M,
                    v.z * KM_TO_M,
                    entry.dv_mag_km_s * KM_TO_M,
                );
            }
            None => {
                let _ = writeln!(
                    out,
                    "| {} | {} | {type_cell} | \u{2014} | \u{2014} | \u{2014} | {:.1} |",
                    i + 1,
                    entry.epoch,
                    entry.dv_mag_km_s * KM_TO_M,
                );
            }
        }
    }
    let _ = writeln!(out);
}

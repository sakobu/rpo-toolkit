//! Unit and display formatting helpers.

use rpo_core::mission::AvoidanceManeuver;

use super::thresholds::velocity::{DOMINANT_COMPONENT_FRACTION, ZERO_MAGNITUDE_KM_S};

/// Conversion factor from kilometres to metres for display formatting.
pub(crate) const KM_TO_M: f64 = 1000.0;

/// Seconds per day (display formatting).
const SECONDS_PER_DAY: f64 = 86_400.0;
/// Seconds per hour (display formatting).
const SECONDS_PER_HOUR: f64 = 3_600.0;
/// Seconds per minute (display formatting).
const SECONDS_PER_MINUTE: f64 = 60.0;

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
    if total_s >= SECONDS_PER_DAY {
        let d = (total_s / SECONDS_PER_DAY).floor();
        let h = ((total_s - d * SECONDS_PER_DAY) / SECONDS_PER_HOUR).floor();
        format!("{d:.0}d {h:02.0}h")
    } else if total_s >= SECONDS_PER_HOUR {
        let h = (total_s / SECONDS_PER_HOUR).floor();
        let m = ((total_s - h * SECONDS_PER_HOUR) / SECONDS_PER_MINUTE).floor();
        format!("{h:.0}h {m:02.0}m")
    } else if total_s >= SECONDS_PER_MINUTE {
        let m = (total_s / SECONDS_PER_MINUTE).floor();
        let sec = total_s - m * SECONDS_PER_MINUTE;
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

/// Join a list of pre-formatted items with Oxford-comma style:
///
/// - `[]` → `""`
/// - `[a]` → `"a"`
/// - `[a, b]` → `"a and b"`
/// - `[a, b, c, ...]` → `"a, b, ..., and last"`
///
/// Shared by the verdict-reason leg enumeration (`render_leg_sep_list`
/// in `verdict.rs`) and the COLA callout leg enumeration (`format_leg_list` in
/// `report/mission.rs`) so every renderer in the crate speaks the
/// same English.
#[must_use]
pub(crate) fn oxford_join(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [single] => single.clone(),
        [a, b] => format!("{a} and {b}"),
        multi => {
            let head = multi[..multi.len() - 1].join(", ");
            let last = &multi[multi.len() - 1];
            format!("{head}, and {last}")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

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

    #[test]
    fn oxford_join_zero_one_two_three_items() {
        assert_eq!(oxford_join(&[]), "");
        assert_eq!(oxford_join(&["a".to_string()]), "a");
        assert_eq!(oxford_join(&["a".to_string(), "b".to_string()]), "a and b");
        assert_eq!(
            oxford_join(&["a".to_string(), "b".to_string(), "c".to_string()]),
            "a, b, and c",
        );
        assert_eq!(
            oxford_join(&[
                "a".to_string(),
                "b".to_string(),
                "c".to_string(),
                "d".to_string(),
            ]),
            "a, b, c, and d",
        );
    }
}

//! Unit and display formatting helpers.
//!
//! # Display precision conventions
//!
//! | Quantity               | Precision | Example       |
//! |------------------------|-----------|---------------|
//! | Lambert Δv             | 1 dp      | `283.9 m/s`   |
//! | Waypoint per-burn Δv   | 2 dp      | `0.56 m/s`    |
//! | Schedule \|Δv\| mag    | 1 dp      | `0.6 m/s`     |
//! | Schedule RIC components| 2 dp      | `-0.21 m/s`   |
//! | COLA Δv components     | 2 dp      | `[-0.71, …]`  |
//! | COLA fuel cost         | 2 dp      | `0.86 m/s`    |
//! | Distances (safety)     | 0–1 dp    | `185 m`       |
//! | RIC positions (< 1 km) | 1 dp      | `[-69.9, …] m`|
//! | RIC positions (≥ 1 km) | 4 dp      | `[1.58, …] km`|

use rpo_core::mission::AvoidanceManeuver;

use super::thresholds::velocity::{DOMINANT_COMPONENT_FRACTION, ZERO_MAGNITUDE_KM_S};

/// Conversion factor from kilometers to metres for display formatting.
pub(crate) const KM_TO_M: f64 = 1000.0;

/// Millimeters per kilometer. Used by the covariance section when
/// velocity sigmas in km/s are rendered as mm/s for readability
/// (the values are conventionally quoted in mm/s in navigation specs).
pub(crate) const MM_PER_KM: f64 = 1_000_000.0;

/// Threshold (km) at which `fmt_distance_auto` switches between
/// metres (below) and kilometers (at or above).
///
/// Named so the adaptive cutover is discoverable instead of a loose
/// `1.0` embedded inside one function body; the covariance and safety
/// tables both rely on the cutover being exactly 1 km.
pub(crate) const DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM: f64 = 1.0;

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

/// Format a distance stored in km with adaptive units.
///
/// - `|value_km| < DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM` → metres at 0 dp
/// - `|value_km| >= DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM` → km at 2 dp
///
/// Used by the covariance section, which spans 0.3 km (cross-track sigma)
/// to 26 km (in-track sigma) in a single table. Use `fmt_m` when the
/// context is consistently sub-km (safety distances).
///
/// # Invariants
/// - `abs()` gates the branch, but the signed value reaches `format!`, so
///   `-0.5` renders as `"-500 m"`.
/// - The boundary is strict (`<`): exactly `1.0` km routes to the km
///   branch, not the metres branch.
#[must_use]
pub fn fmt_distance_auto(value_km: f64) -> String {
    if value_km.abs() < DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM {
        format!("{:.0} m", value_km * KM_TO_M)
    } else {
        format!("{value_km:.2} km")
    }
}

/// Format a float with "smart" precision: drop the trailing zero fractional
/// part when the value rounds to an integer at the chosen precision.
///
/// - `fmt_smart_f64(100.0, 1)` → `"100"`
/// - `fmt_smart_f64(12.5, 1)`  → `"12.5"`
/// - `fmt_smart_f64(0.05, 2)`  → `"0.05"`
/// - `fmt_smart_f64(99.96, 1)` → `"100"` (rounds to 100.0 at 1 dp, zero stripped)
/// - `fmt_smart_f64(99.96, 2)` → `"99.96"` (frac preserved at 2 dp)
///
/// Avoids noise like `"100.0 / 100.0 / 100.0 mm/s"` in navigation-sigma rows
/// where the magnitudes are whole numbers at the displayed precision.
///
/// # Invariants
/// - `decimals = 0` emits an integer string (no `.`), so the zero-strip
///   branch is never entered — behavior is `format!("{value:.0}")`.
/// - Negative numbers survive the `find('.')` split because the sign is
///   before the dot; `fmt_smart_f64(-12.0, 1)` returns `"-12"`.
/// - `NaN` and `±Inf` pass through unchanged — Rust's `{:.N}` formats
///   them as `"NaN"` / `"inf"` / `"-inf"` with no decimal point.
/// - Finite inputs never produce scientific notation from `{:.N}`, so
///   the `find('.')` search is sufficient.
#[must_use]
pub fn fmt_smart_f64(value: f64, decimals: usize) -> String {
    let formatted = format!("{value:.decimals$}");
    if let Some(dot_idx) = formatted.find('.') {
        let frac = &formatted[dot_idx + 1..];
        if frac.chars().all(|c| c == '0') {
            return formatted[..dot_idx].to_string();
        }
    }
    formatted
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

// ── RIC position formatting ─────────────────────────────────────────────

/// Format a RIC position vector, switching to metres when all components are
/// sub-kilometer.
///
/// - All components < 1 km → `"[-69.9, 129.0, 113.1] m"` (1 dp, metres)
/// - Otherwise            → `"[1.5765, 1.0209, 0.1459] km"` (4 dp, km)
///
/// # Invariants
/// - Input is a `Vector3<f64>` tagged by the caller as RIC — the
///   formatter itself is frame-agnostic, but the docs name the frame
///   because every call site is RIC-bound.
/// - The metres branch fires only when ALL three components are
///   sub-kilometer by absolute value; a mixed vector like
///   `[1.0, 0.5, 0.3]` stays in km.
#[must_use]
pub fn fmt_ric_position(position_ric_km: &nalgebra::Vector3<f64>) -> String {
    let x_km = position_ric_km.x;
    let y_km = position_ric_km.y;
    let z_km = position_ric_km.z;
    if x_km.abs() < DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM
        && y_km.abs() < DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM
        && z_km.abs() < DISTANCE_ADAPTIVE_UNIT_THRESHOLD_KM
    {
        format!(
            "[{:.1}, {:.1}, {:.1}] m",
            x_km * KM_TO_M,
            y_km * KM_TO_M,
            z_km * KM_TO_M,
        )
    } else {
        format!("[{x_km:.4}, {y_km:.4}, {z_km:.4}] km")
    }
}

// ── Epoch formatting ───────────────────────────────────────────────────

/// Format an epoch rounded to the nearest second, suppressing nanosecond noise.
///
/// Hifitime's default `Display` emits sub-nanosecond precision
/// (e.g. `2024-01-01T03:31:49.414183680 UTC`). For report display
/// the nearest second is sufficient.
///
/// # Invariants
/// - Uses `hifitime::Duration::round`, which rounds half-away-from-zero
///   per hifitime's documented semantics. The `fmt_epoch_rounded_strips_nanoseconds`
///   test in this module pins the expected behavior for a 0.414-s fractional
///   input (rounds down to the nearest second).
#[must_use]
pub fn fmt_epoch_rounded(epoch: hifitime::Epoch) -> String {
    let rounded = epoch.round(hifitime::Duration::from_seconds(1.0));
    format!("{rounded}")
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
    fn fmt_ric_position_sub_km_uses_metres() {
        let v = nalgebra::Vector3::new(-0.0699, 0.1290, 0.1131);
        let s = fmt_ric_position(&v);
        assert!(s.ends_with("] m"), "expected metres, got: {s}");
        assert!(s.contains("-69.9"), "expected -69.9, got: {s}");
    }

    #[test]
    fn fmt_ric_position_over_km_uses_km() {
        let v = nalgebra::Vector3::new(1.5765, 1.0209, 0.1459);
        let s = fmt_ric_position(&v);
        assert!(s.ends_with("] km"), "expected km, got: {s}");
        assert!(s.contains("1.5765"), "expected 4dp, got: {s}");
    }

    #[test]
    fn fmt_ric_position_mixed_stays_km() {
        // One component >= 1 km → km format
        let v = nalgebra::Vector3::new(1.0, 0.5, 0.3);
        let s = fmt_ric_position(&v);
        assert!(s.ends_with("] km"), "mixed should use km, got: {s}");
    }

    #[test]
    fn fmt_distance_auto_uses_metres_below_1_km() {
        assert_eq!(fmt_distance_auto(0.307), "307 m");
        assert_eq!(fmt_distance_auto(0.001), "1 m");
        assert_eq!(fmt_distance_auto(0.999), "999 m");
    }

    #[test]
    fn fmt_distance_auto_uses_km_at_or_above_1_km() {
        assert_eq!(fmt_distance_auto(1.0), "1.00 km");
        assert_eq!(fmt_distance_auto(26.078), "26.08 km");
        assert_eq!(fmt_distance_auto(100.0), "100.00 km");
    }

    #[test]
    fn fmt_smart_f64_strips_trailing_zero_for_integers() {
        assert_eq!(fmt_smart_f64(100.0, 1), "100");
        assert_eq!(fmt_smart_f64(12.0, 1), "12");
        assert_eq!(fmt_smart_f64(1.0, 2), "1");
    }

    #[test]
    fn fmt_smart_f64_preserves_precision_for_non_integers() {
        assert_eq!(fmt_smart_f64(12.5, 1), "12.5");
        assert_eq!(fmt_smart_f64(0.05, 2), "0.05");
        // 99.96 at 2dp preserves the fractional part
        assert_eq!(fmt_smart_f64(99.96, 2), "99.96");
    }

    #[test]
    fn fmt_smart_f64_strips_when_rounding_produces_integer() {
        // 99.96 rounds to 100.0 at 1dp, zero frac stripped
        assert_eq!(fmt_smart_f64(99.96, 1), "100");
        // 0.03 rounds to 0.0 at 1dp, zero frac stripped
        assert_eq!(fmt_smart_f64(0.03, 1), "0");
    }

    #[test]
    fn fmt_distance_auto_handles_negative_values() {
        // abs() threshold so -0.5 km still renders in metres
        assert_eq!(fmt_distance_auto(-0.5), "-500 m");
        assert_eq!(fmt_distance_auto(-5.0), "-5.00 km");
    }

    #[test]
    fn fmt_epoch_rounded_strips_nanoseconds() {
        use hifitime::Epoch;
        let e = Epoch::from_gregorian_utc(2024, 1, 1, 3, 31, 49, 414_183_680);
        let s = fmt_epoch_rounded(e);
        // Should round to 03:31:49 (< 0.5s fraction)
        assert!(
            !s.contains('.'),
            "rounded epoch should not have sub-second precision, got: {s}",
        );
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

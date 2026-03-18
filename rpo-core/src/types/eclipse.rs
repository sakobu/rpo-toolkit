//! Eclipse and celestial body types for mission timeline and 3D visualization.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::state::epoch_serde;

/// Shadow state classification for a spacecraft.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum EclipseState {
    /// Spacecraft is fully illuminated by the Sun.
    Sunlit,
    /// Spacecraft is in Earth's penumbral shadow (partial illumination).
    /// `shadow_fraction` is the fraction of the Sun's disk obscured by Earth,
    /// ranging from 0.0 (just entered penumbra, almost fully illuminated)
    /// to 1.0 (approaching full umbra, Sun nearly fully blocked).
    Penumbra {
        /// Fraction of the Sun's disk obscured by Earth (0.0–1.0).
        shadow_fraction: f64,
    },
    /// Spacecraft is in Earth's umbral shadow (no direct sunlight).
    Umbra,
}

/// Sun and Moon directions at a single trajectory point.
///
/// Directions are unit vectors from the chief spacecraft toward each body,
/// expressed in ECI J2000. The frontend applies the ECI-to-RIC DCM for 3D rendering.
///
/// Each snapshot carries its own epoch, making slices self-contained and
/// eliminating the need to pass parallel epoch arrays alongside snapshot data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CelestialSnapshot {
    /// Epoch of this snapshot.
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
    /// Unit vector from spacecraft toward the Sun in ECI J2000.
    pub sun_direction_eci: Vector3<f64>,
    /// Unit vector from spacecraft toward the Moon in ECI J2000.
    pub moon_direction_eci: Vector3<f64>,
    /// Distance from spacecraft to Sun (km).
    pub sun_distance_km: f64,
    /// Distance from spacecraft to Moon (km).
    pub moon_distance_km: f64,
    /// Eclipse state at this point.
    pub eclipse_state: EclipseState,
}

/// A contiguous time interval during which the spacecraft is in shadow.
///
/// An interval begins when the spacecraft transitions from Sunlit to any
/// shadow state (Penumbra or Umbra) and ends when it returns to Sunlit.
/// The `state` field records the **worst-case** (deepest shadow) state
/// observed during the interval: `Umbra` if any sample was in umbra,
/// otherwise `Penumbra` with the maximum shadow fraction observed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EclipseInterval {
    /// Start epoch of the shadow interval.
    #[serde(with = "epoch_serde")]
    pub start: Epoch,
    /// End epoch of the shadow interval.
    #[serde(with = "epoch_serde")]
    pub end: Epoch,
    /// Duration of the shadow interval (seconds).
    pub duration_s: f64,
    /// Worst-case shadow state during this interval.
    pub state: EclipseState,
}

/// Summary of eclipse conditions across a mission or leg.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EclipseSummary {
    /// All shadow intervals detected in the trajectory.
    pub intervals: Vec<EclipseInterval>,
    /// Total time spent in shadow (seconds).
    pub total_shadow_duration_s: f64,
    /// Fraction of total mission time spent in shadow (0.0–1.0).
    pub time_in_shadow_fraction: f64,
    /// Duration of the longest single shadow interval (seconds).
    pub max_shadow_duration_s: f64,
}

/// Eclipse data for the Lambert transfer phase.
///
/// Contains per-point celestial snapshots for the deputy (traveling along
/// the Lambert arc), chief eclipse states (at the home orbit), and a
/// deputy-based summary. During far-field transfers the two spacecraft
/// can be hundreds to thousands of km apart with genuinely different
/// eclipse states — unlike the waypoint proximity phase where the
/// ~km-scale separation makes chief/deputy shadow states nearly identical.
///
/// The summary is deputy-based because the deputy is the one maneuvering
/// and mission planners care about the deputy's shadow conditions during
/// transfer (comm/power constraints).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransferEclipseData {
    /// Eclipse summary for the deputy's transfer arc.
    pub summary: EclipseSummary,
    /// Celestial snapshots along the deputy's Lambert arc.
    pub deputy_celestial: Vec<CelestialSnapshot>,
    /// Eclipse states along the chief's home orbit during the transfer,
    /// indexed parallel to `deputy_celestial`.
    pub chief_eclipse: Vec<EclipseState>,
}

/// Per-leg eclipse data: chief celestial snapshots and deputy shadow states.
///
/// Each field has the same length — one entry per trajectory point in the
/// corresponding `ManeuverLeg.trajectory`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegEclipseData {
    /// Chief celestial snapshots (Sun/Moon direction, eclipse state) at each
    /// trajectory point.
    pub chief_celestial: Vec<CelestialSnapshot>,
    /// Deputy eclipse states at each trajectory point.
    ///
    /// Computed from the deputy's ECI position (chief + RIC offset). At LEO
    /// with km-scale separations, these nearly always match the chief's
    /// [`CelestialSnapshot::eclipse_state`].
    ///
    /// # Validity
    ///
    /// Assumes the formation separation is small relative to the shadow cone
    /// geometry. For LEO (~6800 km orbit), separations up to ~100 km produce
    /// negligible eclipse timing differences (<1 s). At GEO (~42,164 km),
    /// the shadow cone is narrower but the approximation still holds for
    /// separations up to ~50 km.
    pub deputy_eclipse: Vec<EclipseState>,
}

/// Eclipse data for a complete waypoint mission.
///
/// Contains both the aggregate summary (intervals, durations) and per-leg
/// eclipse data (Sun/Moon directions, eclipse state at each trajectory point).
///
/// Follows the same pattern as `MissionCovarianceReport`, which carries
/// per-leg `LegCovarianceReport` alongside aggregate metrics — overlay data
/// lives in its own report structure, not on `ManeuverLeg`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionEclipseData {
    /// Aggregate eclipse summary across all legs (merged intervals, chief-based).
    pub summary: EclipseSummary,
    /// Per-leg eclipse data, indexed parallel to `WaypointMission.legs`.
    pub legs: Vec<LegEclipseData>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use hifitime::Epoch;
    use nalgebra::Vector3;

    #[test]
    fn serde_roundtrip_eclipse_state() {
        let states = vec![
            EclipseState::Sunlit,
            EclipseState::Penumbra {
                shadow_fraction: 0.42,
            },
            EclipseState::Umbra,
        ];
        for state in &states {
            let json = serde_json::to_string(state).expect("serialize");
            let deserialized: EclipseState =
                serde_json::from_str(&json).expect("deserialize");
            assert_eq!(*state, deserialized);
        }
    }

    #[test]
    fn serde_roundtrip_celestial_snapshot() {
        let snapshot = CelestialSnapshot {
            epoch: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 0, 0),
            sun_direction_eci: Vector3::new(1.0, 0.0, 0.0),
            moon_direction_eci: Vector3::new(0.0, 1.0, 0.0),
            sun_distance_km: 149_597_870.7,
            moon_distance_km: 384_400.0,
            eclipse_state: EclipseState::Sunlit,
        };
        let json = serde_json::to_string(&snapshot).expect("serialize");
        let deserialized: CelestialSnapshot =
            serde_json::from_str(&json).expect("deserialize");
        assert_eq!(deserialized.sun_distance_km, snapshot.sun_distance_km);
        assert_eq!(deserialized.eclipse_state, snapshot.eclipse_state);
    }

    #[test]
    fn serde_roundtrip_eclipse_interval() {
        let interval = EclipseInterval {
            start: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 0, 0),
            end: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 36, 0),
            duration_s: 2160.0,
            state: EclipseState::Umbra,
        };
        let json = serde_json::to_string(&interval).expect("serialize");
        let deserialized: EclipseInterval =
            serde_json::from_str(&json).expect("deserialize");
        assert!((deserialized.duration_s - 2160.0).abs() < f64::EPSILON);
    }

    #[test]
    fn serde_roundtrip_transfer_eclipse_data() {
        let data = TransferEclipseData {
            summary: EclipseSummary {
                intervals: vec![],
                total_shadow_duration_s: 0.0,
                time_in_shadow_fraction: 0.0,
                max_shadow_duration_s: 0.0,
            },
            deputy_celestial: vec![CelestialSnapshot {
                epoch: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 0, 0),
                sun_direction_eci: Vector3::new(1.0, 0.0, 0.0),
                moon_direction_eci: Vector3::new(0.0, 1.0, 0.0),
                sun_distance_km: 149_597_870.7,
                moon_distance_km: 384_400.0,
                eclipse_state: EclipseState::Sunlit,
            }],
            chief_eclipse: vec![EclipseState::Sunlit],
        };
        let json = serde_json::to_string(&data).expect("serialize");
        let deser: TransferEclipseData =
            serde_json::from_str(&json).expect("deserialize");
        assert_eq!(deser.deputy_celestial.len(), 1);
        assert_eq!(deser.chief_eclipse.len(), 1);
        assert_eq!(deser.chief_eclipse[0], EclipseState::Sunlit);
    }

    #[test]
    fn serde_roundtrip_mission_eclipse_data() {
        let data = MissionEclipseData {
            summary: EclipseSummary {
                intervals: vec![EclipseInterval {
                    start: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 0, 0),
                    end: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 36, 0),
                    duration_s: 2160.0,
                    state: EclipseState::Umbra,
                }],
                total_shadow_duration_s: 2160.0,
                time_in_shadow_fraction: 0.342,
                max_shadow_duration_s: 2160.0,
            },
            legs: vec![LegEclipseData {
                chief_celestial: vec![CelestialSnapshot {
                    epoch: Epoch::from_gregorian_utc_hms(2024, 3, 20, 12, 0, 0),
                    sun_direction_eci: Vector3::new(1.0, 0.0, 0.0),
                    moon_direction_eci: Vector3::new(0.0, 1.0, 0.0),
                    sun_distance_km: 149_597_870.7,
                    moon_distance_km: 384_400.0,
                    eclipse_state: EclipseState::Sunlit,
                }],
                deputy_eclipse: vec![EclipseState::Sunlit],
            }],
        };
        let json = serde_json::to_string(&data).expect("serialize");
        let deserialized: MissionEclipseData =
            serde_json::from_str(&json).expect("deserialize");
        assert_eq!(deserialized.summary.intervals.len(), 1);
        assert_eq!(deserialized.legs.len(), 1);
        assert_eq!(deserialized.legs[0].chief_celestial.len(), 1);
        assert_eq!(deserialized.legs[0].deputy_eclipse.len(), 1);
        assert_eq!(deserialized.legs[0].deputy_eclipse[0], EclipseState::Sunlit);
    }
}

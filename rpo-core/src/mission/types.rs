//! Mission-level domain types: phases, waypoints, maneuvers, legs, and missions.

use hifitime::Epoch;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::propagation::lambert::LambertTransfer;
use crate::propagation::propagator::PropagatedState;
use crate::propagation::covariance::types::MissionCovarianceReport;
use crate::types::{KeplerianElements, MissionEclipseData, QuasiNonsingularROE, RICState, SpacecraftConfig};

/// Serde helper: skip serialization when a `bool` is `false`.
///
/// Serde requires `&T` signature for `skip_serializing_if` predicates.
#[allow(clippy::trivially_copy_pass_by_ref)]
fn is_false(v: &bool) -> bool {
    !*v
}

/// Result of analyzing the separation between two spacecraft.
///
/// Determines whether the spacecraft are close enough for linearized
/// ROE operations or require a far-field transfer.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionPhase {
    /// Separation within ROE-valid range — proceed with analytical ROE ops.
    Proximity {
        /// Computed quasi-nonsingular ROE state
        roe: QuasiNonsingularROE,
        /// Chief Keplerian elements
        chief_elements: KeplerianElements,
        /// Deputy Keplerian elements
        deputy_elements: KeplerianElements,
        /// ECI separation distance (km)
        separation_km: f64,
        /// Dimensionless separation metric δr/r
        delta_r_over_r: f64,
    },
    /// Separation exceeds ROE threshold — far-field transfer required.
    FarField {
        /// Chief Keplerian elements
        chief_elements: KeplerianElements,
        /// Deputy Keplerian elements
        deputy_elements: KeplerianElements,
        /// ECI separation distance (km)
        separation_km: f64,
        /// Dimensionless separation metric δr/r
        delta_r_over_r: f64,
    },
}

/// Predefined perch orbit geometries relative to the chief.
///
/// A perch orbit is a safe holding geometry at the boundary of the
/// ROE-valid region, used as the handoff point between far-field
/// Lambert transfers and near-field ROE proximity operations.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PerchGeometry {
    /// V-bar hold: deputy ahead/behind chief along the velocity vector.
    VBar {
        /// Along-track offset (km). Positive = ahead of chief.
        along_track_km: f64,
    },
    /// R-bar hold: deputy above/below chief along the radial vector.
    RBar {
        /// Radial offset (km). Positive = above chief.
        radial_km: f64,
    },
    /// Custom ROE state for advanced perch geometries.
    Custom(QuasiNonsingularROE),
}

/// A complete mission plan from two arbitrary ECI states.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionPlan {
    /// Mission phase classification
    pub phase: MissionPhase,
    /// Lambert transfer (only present if `FarField`)
    pub transfer: Option<LambertTransfer>,
    /// Perch orbit ROE state (transfer target / proximity start)
    pub perch_roe: QuasiNonsingularROE,
    /// Chief Keplerian elements at the waypoint mission start epoch.
    ///
    /// For far-field missions, mean anomaly is advanced to the Lambert
    /// arrival epoch (two-body). For proximity missions, same as departure elements.
    pub chief_at_arrival: KeplerianElements,
}

/// A target waypoint in RIC space for maneuver targeting.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Waypoint {
    /// Target position in RIC frame (km): [radial, in-track, cross-track]
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub position_ric_km: Vector3<f64>,
    /// Target velocity in RIC frame (km/s): \[radial, in-track, cross-track\].
    ///
    /// - `None`: minimum-norm arrival velocity derived analytically.
    ///   The solver finds the cheapest velocity consistent with the target
    ///   position (D'Amico Eq. 2.17 `T_vel` applied to `T_pos` pseudoinverse).
    ///   For transit waypoints this is typically much cheaper than
    ///   station-keeping. Use `Some([0, 0, 0])` to explicitly request
    ///   station-keeping.
    /// - `Some(v)`: concrete velocity target; solver targets `v` exactly.
    ///
    /// **Formation enrichment** (`safety_envelope::enrich_waypoint`):
    /// `None` means 3-DOF null-space freedom — velocity is selected to
    /// maximize e/i safety separation (D'Amico Eq. 2.17 pseudoinverse).
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number] | null"))]
    pub velocity_ric_km_s: Option<Vector3<f64>>,
    /// Time of flight to this waypoint (seconds). If `None`, TOF will be optimized.
    pub tof_s: Option<f64>,
}

/// A single impulsive maneuver (Δv) in the RIC frame.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Maneuver {
    /// Δv in RIC frame (km/s): [radial, in-track, cross-track]
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub dv_ric_km_s: Vector3<f64>,
    /// Epoch at which the maneuver is applied
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub epoch: Epoch,
}

/// A single leg of a waypoint transfer (two burns + coast).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManeuverLeg {
    /// Departure burn (applied at departure epoch)
    pub departure_maneuver: Maneuver,
    /// Arrival burn (applied at arrival epoch)
    pub arrival_maneuver: Maneuver,
    /// Time of flight for this leg (seconds)
    pub tof_s: f64,
    /// Total Δv for this leg (km/s)
    pub total_dv_km_s: f64,
    /// ROE state before the departure burn (pre-maneuver).
    /// Used for free-drift / abort-case analysis.
    pub pre_departure_roe: QuasiNonsingularROE,
    /// ROE state after departure burn
    pub post_departure_roe: QuasiNonsingularROE,
    /// Chief mean Keplerian elements at departure epoch
    pub departure_chief_mean: KeplerianElements,
    /// ROE state at arrival (before arrival burn)
    pub pre_arrival_roe: QuasiNonsingularROE,
    /// ROE state after arrival burn
    pub post_arrival_roe: QuasiNonsingularROE,
    /// Chief mean Keplerian elements at arrival epoch
    pub arrival_chief_mean: KeplerianElements,
    /// Coast trajectory between burns
    pub trajectory: Vec<PropagatedState>,
    /// Departure RIC position (km)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub from_position_ric_km: Vector3<f64>,
    /// Target RIC position (km)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub to_position_ric_km: Vector3<f64>,
    /// Target RIC velocity (km/s)
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub target_velocity_ric_km_s: Vector3<f64>,
    /// Number of Newton-Raphson iterations used
    pub iterations: u32,
    /// Final position error after convergence (km)
    pub position_error_km: f64,
}

/// A complete waypoint-based mission plan.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaypointMission {
    /// Ordered sequence of maneuver legs
    pub legs: Vec<ManeuverLeg>,
    /// Total Δv across all legs (km/s)
    pub total_dv_km_s: f64,
    /// Total mission duration (seconds)
    pub total_duration_s: f64,
    /// Safety metrics for the full mission (if computed)
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub safety: Option<SafetyMetrics>,
    /// Covariance propagation results (if computed via `propagate_mission_covariance`)
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub covariance: Option<MissionCovarianceReport>,
    /// Eclipse data across all legs. `None` if a degenerate orbit prevents
    /// ECI reconstruction, or if deserialized from JSON without eclipse data.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub eclipse: Option<MissionEclipseData>,
}

/// Operational safety metrics — instantaneous geometric distance measures.
///
/// Meaningful for all mission types including actively guided approaches.
/// These metrics reflect the actual physical separation between vehicles
/// at each trajectory point.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct OperationalSafety {
    /// Minimum instantaneous R/C distance (km): min sqrt(R² + C²) along trajectory.
    pub min_rc_separation_km: f64,
    /// Minimum 3D distance between vehicles (km): min ‖RIC position‖ along trajectory.
    pub min_distance_3d_km: f64,

    /// Leg index (0-based) where minimum R/C separation occurs.
    pub min_rc_leg_index: usize,
    /// Mission elapsed time (s) at minimum R/C separation.
    pub min_rc_elapsed_s: f64,
    /// RIC position (km) at minimum R/C separation.
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub min_rc_ric_position_km: Vector3<f64>,

    /// Leg index (0-based) where minimum 3D distance occurs.
    pub min_3d_leg_index: usize,
    /// Mission elapsed time (s) at minimum 3D distance.
    pub min_3d_elapsed_s: f64,
    /// RIC position (km) at minimum 3D distance.
    #[cfg_attr(feature = "wasm", tsify(type = "[number, number, number]"))]
    pub min_3d_ric_position_km: Vector3<f64>,
}

/// Passive / abort safety metrics — orbit-averaged formation geometry bounds
/// derived from D'Amico Eq. 2.22.
///
/// Represents the minimum R/C distance a deputy would achieve in free drift
/// (no maneuvers) based on the relative eccentricity and inclination vectors.
/// Meaningful for:
/// - Abort/contingency analysis ("if guidance fails mid-leg")
/// - Long-coast formation flying
/// - Drift-safe orbit design
///
/// **Not enforced during active waypoint targeting.** High violation rates
/// in guided approach missions are expected and do not indicate operational
/// hazard — they reflect that intermediate ROE geometries during short
/// maneuver legs are not optimized for passive safety.
///
/// # Validity
///
/// - Assumes linearized ROE dynamics with near-circular chief orbit.
/// - Orbit-averaged: designed for mean elements. When computed from
///   osculating elements (e.g. nyx full-physics), short-period oscillations
///   add noise to the metric.
/// - Singular when both δe and δi magnitudes are below `ROE_MAG_EPSILON`
///   (returns 0.0).
///
/// See [`crate::mission::safety::analyze_safety`] for the full computation.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct PassiveSafety {
    /// Minimum e/i vector separation (km) from D'Amico Eq. 2.22 (analytic orbit-averaged bound).
    pub min_ei_separation_km: f64,
    /// Magnitude of relative eccentricity vector (dimensionless).
    pub de_magnitude: f64,
    /// Magnitude of relative inclination vector (dimensionless).
    pub di_magnitude: f64,
    /// Phase angle between e/i vectors (rad). Perpendicular (±π/2) = maximum passive safety.
    pub ei_phase_angle_rad: f64,
}

/// Complete safety analysis combining operational and passive/abort metrics.
///
/// # Safety categories
///
/// - **Operational** ([`OperationalSafety`]): instantaneous 3D distance and R/C
///   separation — meaningful for all mission types.
/// - **Passive / abort** ([`PassiveSafety`]): orbit-averaged e/i vector separation
///   (D'Amico Eq. 2.22) — meaningful for free-drift contingency analysis.
///   Not a planning constraint in the current targeting solver.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SafetyMetrics {
    /// Operational safety: instantaneous geometric distance measures.
    pub operational: OperationalSafety,
    /// Passive / abort safety: orbit-averaged formation geometry bounds.
    pub passive: PassiveSafety,
}

/// Per-timestep comparison between analytical and numerical propagation.
///
/// Defined in rpo-core for WASM serialization. Values are populated by
/// the full-physics nyx validation pipeline.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationPoint {
    /// Time since mission start (seconds)
    pub elapsed_s: f64,
    /// Analytical (Phase 4) RIC state
    pub analytical_ric: RICState,
    /// Numerical (nyx) RIC state
    pub numerical_ric: RICState,
    /// Position difference magnitude (km)
    pub position_error_km: f64,
    /// Velocity difference magnitude (km/s)
    pub velocity_error_km_s: f64,
    /// Whether this sample follows a COLA burn in nyx propagation.
    /// Excluded from fidelity statistics (conflates model error with COLA perturbation).
    #[serde(skip_serializing_if = "is_false", default)]
    pub post_cola: bool,
}

/// Aggregate validation results for a mission.
///
/// Defined in rpo-core for WASM serialization. Values are populated by
/// the full-physics nyx validation pipeline.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReport {
    /// Per-leg comparison points
    pub leg_points: Vec<Vec<ValidationPoint>>,
    /// Maximum position error across all points (km)
    pub max_position_error_km: f64,
    /// Mean position error across all points (km)
    pub mean_position_error_km: f64,
    /// RMS position error across all points (km)
    pub rms_position_error_km: f64,
    /// Maximum velocity error across all points (km/s)
    pub max_velocity_error_km_s: f64,
    /// Safety metrics from analytical trajectory (from `WaypointMission.safety`)
    pub analytical_safety: Option<SafetyMetrics>,
    /// Safety metrics recomputed from nyx trajectory
    pub numerical_safety: SafetyMetrics,
    /// Chief spacecraft configuration used
    pub chief_config: SpacecraftConfig,
    /// Deputy spacecraft configuration used
    pub deputy_config: SpacecraftConfig,
    /// Eclipse validation (analytical Meeus vs ANISE ephemeris).
    /// Present when the mission includes eclipse data.
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub eclipse_validation: Option<EclipseValidation>,
}

impl ValidationReport {
    /// Whether any COLA burns were injected during nyx validation.
    ///
    /// Derived from the presence of `post_cola` samples — no stored field needed.
    #[must_use]
    pub fn cola_validated(&self) -> bool {
        self.leg_points.iter().flatten().any(|p| p.post_cola)
    }
}

/// Per-sample eclipse comparison (analytical vs ANISE).
///
/// Defined in rpo-core for WASM serialization. Values are populated by
/// the full-physics nyx validation pipeline (eclipse comparison module).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EclipseValidationPoint {
    /// Time since mission start (seconds).
    pub elapsed_s: f64,
    /// Analytical eclipse percentage (0.0 = sunlit, 100.0 = umbra).
    pub analytical_eclipse_pct: f64,
    /// ANISE eclipse percentage (0.0 = sunlit, 100.0 = umbra).
    pub numerical_eclipse_pct: f64,
    /// Absolute difference in eclipse percentage.
    pub eclipse_pct_error: f64,
    /// Angular error between analytical (Meeus) and ANISE Sun directions (rad).
    pub sun_direction_error_rad: f64,
}

/// Comparison of a single eclipse interval (analytical vs ANISE).
///
/// Defined in rpo-core for WASM serialization. Values are populated by
/// the full-physics nyx validation pipeline (eclipse comparison module).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EclipseIntervalComparison {
    /// Analytical interval start epoch.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub analytical_start: Epoch,
    /// ANISE interval start epoch.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub numerical_start: Epoch,
    /// Entry timing error (seconds). Positive = analytical enters shadow later.
    pub entry_error_s: f64,
    /// Analytical interval end epoch.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub analytical_end: Epoch,
    /// ANISE interval end epoch.
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
    pub numerical_end: Epoch,
    /// Exit timing error (seconds). Positive = analytical exits shadow later.
    pub exit_error_s: f64,
    /// Duration error (seconds).
    pub duration_error_s: f64,
}

/// Aggregate eclipse validation results.
///
/// Defined in rpo-core for WASM serialization. Values are populated by
/// the full-physics nyx validation pipeline when the mission includes
/// eclipse data (`WaypointMission.eclipse`).
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EclipseValidation {
    /// Per-sample eclipse comparison points.
    pub points: Vec<EclipseValidationPoint>,
    /// Per-interval timing comparison (matched analytical vs ANISE intervals).
    pub interval_comparisons: Vec<EclipseIntervalComparison>,
    /// Maximum Sun direction angular error across all points (rad).
    pub max_sun_direction_error_rad: f64,
    /// Mean Sun direction angular error (rad).
    pub mean_sun_direction_error_rad: f64,
    /// Maximum eclipse entry/exit timing error (seconds).
    pub max_timing_error_s: f64,
    /// Mean absolute eclipse entry/exit timing error (seconds).
    pub mean_timing_error_s: f64,
    /// Number of analytical intervals.
    pub analytical_interval_count: usize,
    /// Number of ANISE intervals.
    pub numerical_interval_count: usize,
    /// Number of matched interval pairs.
    pub matched_interval_count: usize,
    /// Number of unmatched intervals (present in one but not the other).
    pub unmatched_interval_count: usize,
}

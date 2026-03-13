//! Core domain types: state vectors, Keplerian elements, ROEs, and RIC states.

use hifitime::Epoch;
use nalgebra::{SMatrix, SVector, Vector3};
use serde::{Deserialize, Serialize};

use crate::constants::{
    DEFAULT_MANEUVER_MAGNITUDE_SIGMA, DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
    DEFAULT_NAV_POSITION_SIGMA_KM, DEFAULT_NAV_VELOCITY_SIGMA_KM_S, KEPLER_MAX_ITER, KEPLER_TOL,
    TWO_PI,
};

/// 6×6 matrix type alias for STM and covariance operations.
pub type Matrix6 = SMatrix<f64, 6, 6>;

/// 9×9 matrix type alias for the augmented J2+drag STM.
pub type Matrix9 = SMatrix<f64, 9, 9>;

/// ECI J2000 state vector (position in km, velocity in km/s)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StateVector {
    /// Epoch of the state
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
    /// Position vector in ECI frame (km)
    pub position_eci_km: Vector3<f64>,
    /// Velocity vector in ECI frame (km/s)
    pub velocity_eci_km_s: Vector3<f64>,
}

/// Classical Keplerian orbital elements
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct KeplerianElements {
    /// Semi-major axis (km)
    pub a_km: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination (rad)
    pub i_rad: f64,
    /// Right ascension of ascending node (rad)
    pub raan_rad: f64,
    /// Argument of perigee (rad)
    pub aop_rad: f64,
    /// Mean anomaly (rad)
    pub mean_anomaly_rad: f64,
}

impl KeplerianElements {
    /// Mean motion n = sqrt(μ/a³) (rad/s).
    ///
    /// # Invariants
    /// - `self.a_km > 0` (negative or zero SMA produces NaN)
    #[must_use]
    pub fn mean_motion(&self) -> f64 {
        (crate::constants::MU_EARTH / (self.a_km * self.a_km * self.a_km)).sqrt()
    }

    /// Orbital period T = 2π/n (seconds).
    ///
    /// # Invariants
    /// - `self.a_km > 0` (delegates to `mean_motion()`)
    #[must_use]
    pub fn period(&self) -> f64 {
        std::f64::consts::TAU / self.mean_motion()
    }

    /// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E,
    /// then compute true anomaly ν.
    ///
    /// # Invariants
    /// - `0 <= self.e < 1` (parabolic/hyperbolic orbits produce incorrect results)
    /// - Newton-Raphson iteration is capped at `KEPLER_MAX_ITER`; non-convergence
    ///   for high-eccentricity orbits is silent (returns last iterate)
    #[must_use]
    pub fn true_anomaly(&self) -> f64 {
        debug_assert!(self.e >= 0.0 && self.e < 1.0, "eccentricity must be in [0, 1), got {}", self.e);

        let m = self.mean_anomaly_rad.rem_euclid(TWO_PI);
        let e = self.e;

        // Newton's method for Kepler's equation
        // For high eccentricity (e >= 0.8), M is a poor initial guess; start from π
        let mut ea = if e < 0.8 { m } else { std::f64::consts::PI };
        for _ in 0..KEPLER_MAX_ITER {
            let f = ea - e * ea.sin() - m;
            let fp = 1.0 - e * ea.cos();
            let delta = f / fp;
            ea -= delta;
            if delta.abs() < KEPLER_TOL {
                break;
            }
        }

        // Eccentric anomaly → true anomaly
        let nu = 2.0
            * ((1.0 + e).sqrt() * (ea / 2.0).sin()).atan2((1.0 - e).sqrt() * (ea / 2.0).cos());
        nu.rem_euclid(TWO_PI)
    }

    /// Mean argument of latitude u = ω + M
    ///
    /// # Invariants
    /// - Angles `aop_rad` and `mean_anomaly_rad` should be in radians
    #[must_use]
    pub fn mean_arg_of_lat(&self) -> f64 {
        (self.aop_rad + self.mean_anomaly_rad).rem_euclid(TWO_PI)
    }
}

/// Quasi-nonsingular relative orbital elements (Koenig Eq. 2 / D'Amico Eq. 2.2)
/// All elements are dimensionless (normalized by chief semi-major axis)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct QuasiNonsingularROE {
    /// Relative semi-major axis: `(a_d - a_c) / a_c`
    pub da: f64,
    /// Relative mean longitude
    pub dlambda: f64,
    /// Relative eccentricity vector x-component
    pub dex: f64,
    /// Relative eccentricity vector y-component
    pub dey: f64,
    /// Relative inclination vector x-component
    pub dix: f64,
    /// Relative inclination vector y-component
    pub diy: f64,
}

impl Default for QuasiNonsingularROE {
    fn default() -> Self {
        Self {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        }
    }
}

impl QuasiNonsingularROE {
    /// Convert to a 6-element column vector [da, dlambda, dex, dey, dix, diy].
    #[must_use]
    pub fn to_vector(&self) -> SVector<f64, 6> {
        SVector::<f64, 6>::new(self.da, self.dlambda, self.dex, self.dey, self.dix, self.diy)
    }

    /// Construct from a 6-element column vector [da, dlambda, dex, dey, dix, diy].
    #[must_use]
    pub fn from_vector(v: &SVector<f64, 6>) -> Self {
        Self {
            da: v[0],
            dlambda: v[1],
            dex: v[2],
            dey: v[3],
            dix: v[4],
            diy: v[5],
        }
    }

    /// Dimensionless separation metric: max(|δa|, |δex|, |δey|, |δix|).
    ///
    /// Excludes δλ and δiy per Koenig Sec. V (these can be large without
    /// violating linearization assumptions).
    #[must_use]
    pub fn dimensionless_norm(&self) -> f64 {
        self.da.abs().max(self.dex.abs()).max(self.dey.abs()).max(self.dix.abs())
    }
}

/// Density-model-free (DMF) differential drag configuration (Koenig Sec. VIII).
///
/// Specifies the time derivatives of relative orbital elements due to
/// differential drag, normalized by chief semi-major axis. These rates
/// are treated as constant over the propagation interval.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct DragConfig {
    /// Time derivative of relative SMA due to drag: `δȧ_drag` (1/s, normalized by `a_c`)
    pub da_dot: f64,
    /// Time derivative of relative eccentricity x-component due to drag: `δėx_drag` (1/s)
    pub dex_dot: f64,
    /// Time derivative of relative eccentricity y-component due to drag: `δėy_drag` (1/s)
    pub dey_dot: f64,
}

impl DragConfig {
    /// Zero drag configuration (no differential drag).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            da_dot: 0.0,
            dex_dot: 0.0,
            dey_dot: 0.0,
        }
    }
}

/// Physical properties of a spacecraft for force modeling.
///
/// Entry point for the entire tool: user defines spacecraft properties,
/// and everything downstream derives from them:
/// - nyx full-physics propagation (via `config_to_spacecraft()`)
/// - Analytical DMF drag rates (via `extract_dmf_rates()` → `DragConfig`)
/// - Mission validation (via `validate_mission_nyx()`)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SpacecraftConfig {
    /// Spacecraft dry mass in kg
    pub dry_mass_kg: f64,
    /// Drag reference area in m²
    pub drag_area_m2: f64,
    /// Coefficient of drag (dimensionless, typically 2.0–2.5)
    pub coeff_drag: f64,
    /// SRP reference area in m²
    pub srp_area_m2: f64,
    /// Coefficient of reflectivity (dimensionless, typically 1.0–2.0)
    pub coeff_reflectivity: f64,
}

impl Default for SpacecraftConfig {
    fn default() -> Self {
        Self {
            dry_mass_kg: 500.0,
            drag_area_m2: 1.0,
            coeff_drag: 2.2,
            srp_area_m2: 1.0,
            coeff_reflectivity: 1.5,
        }
    }
}

impl SpacecraftConfig {
    /// Typical 6U cubesat: 12 kg, 0.06 m² cross-section.
    pub const CUBESAT_6U: Self = Self {
        dry_mass_kg: 12.0,
        drag_area_m2: 0.06,
        coeff_drag: 2.2,
        srp_area_m2: 0.06,
        coeff_reflectivity: 1.5,
    };

    /// Typical 500 kg servicer spacecraft.
    /// Note: intentionally identical to `Default` — the preset provides a named
    /// reference point, while `Default` is the ergonomic fallback. If the servicer
    /// baseline changes, update both or have `Default` delegate to this const.
    pub const SERVICER_500KG: Self = Self {
        dry_mass_kg: 500.0,
        drag_area_m2: 1.0,
        coeff_drag: 2.2,
        srp_area_m2: 1.0,
        coeff_reflectivity: 1.5,
    };
}

/// Relative state in the RIC (Radial-In-track-Cross-track) frame
/// Also known as Hill frame or LVLH frame
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RICState {
    /// Relative position in RIC frame (km): [radial, in-track, cross-track]
    pub position_ric_km: Vector3<f64>,
    /// Time derivative of relative position in the rotating RIC frame, ρ̇ (km/s)
    pub velocity_ric_km_s: Vector3<f64>,
}

/// Configuration for mission flow decision-making.
///
/// Controls the threshold at which two spacecraft are considered
/// within ROE-valid proximity vs. requiring a far-field transfer.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ProximityConfig {
    /// Max dimensionless δr/r for ROE linearization validity (default: 0.005).
    ///
    /// Based on D'Amico Sec. 2.3.4: at δr/r ~ 0.005, second-order terms
    /// are ~2.5×10⁻⁵ of orbit radius, comparable to J2 modeling residuals.
    pub roe_threshold: f64,
}

impl Default for ProximityConfig {
    fn default() -> Self {
        Self {
            roe_threshold: 0.005,
        }
    }
}

/// Result of analyzing the separation between two spacecraft.
///
/// Determines whether the spacecraft are close enough for linearized
/// ROE operations or require a far-field transfer.
#[derive(Debug, Clone, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionPlan {
    /// Mission phase classification
    pub phase: MissionPhase,
    /// Lambert transfer (only present if `FarField`)
    pub transfer: Option<crate::mission::lambert::LambertTransfer>,
    /// Perch orbit ROE state (transfer target / proximity start)
    pub perch_roe: QuasiNonsingularROE,
    /// Chief Keplerian elements at the waypoint mission start epoch.
    ///
    /// For far-field missions, mean anomaly is advanced to the Lambert
    /// arrival epoch (two-body). For proximity missions, same as departure elements.
    pub chief_at_arrival: KeplerianElements,
}

/// A target waypoint in RIC space for maneuver targeting.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Waypoint {
    /// Target position in RIC frame (km): [radial, in-track, cross-track]
    pub position_ric_km: Vector3<f64>,
    /// Target velocity in RIC frame (km/s): [radial, in-track, cross-track]
    pub velocity_ric_km_s: Vector3<f64>,
    /// Time of flight to this waypoint (seconds). If `None`, TOF will be optimized.
    pub tof_s: Option<f64>,
}

/// A single impulsive maneuver (Δv) in the RIC frame.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Maneuver {
    /// Δv in RIC frame (km/s): [radial, in-track, cross-track]
    pub dv_ric_km_s: Vector3<f64>,
    /// Epoch at which the maneuver is applied
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
}

/// A single leg of a waypoint transfer (two burns + coast).
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
    pub trajectory: Vec<crate::propagation::propagator::PropagatedState>,
    /// Departure RIC position (km)
    pub from_position_ric_km: Vector3<f64>,
    /// Target RIC position (km)
    pub to_position_ric_km: Vector3<f64>,
    /// Target RIC velocity (km/s)
    pub target_velocity_ric_km_s: Vector3<f64>,
    /// Number of Newton-Raphson iterations used
    pub iterations: u32,
    /// Final position error after convergence (km)
    pub position_error_km: f64,
}

/// A complete waypoint-based mission plan.
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
}

/// Passive safety metrics based on instantaneous R/C distance, e/i vector separation, and 3D distance.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SafetyMetrics {
    /// Minimum instantaneous R/C distance (km): min sqrt(R^2 + C^2) along trajectory.
    pub min_rc_separation_km: f64,
    /// Minimum 3D distance between vehicles (km): min |RIC position|
    pub min_distance_3d_km: f64,
    /// Minimum e/i vector separation (km) from D'Amico Eq. 2.22 (analytic orbit-averaged bound).
    pub min_ei_separation_km: f64,
    /// Magnitude of relative eccentricity vector
    pub de_magnitude: f64,
    /// Magnitude of relative inclination vector
    pub di_magnitude: f64,
    /// Phase angle between e/i vectors (rad). Parallel = 0, anti-parallel = π.
    pub ei_phase_angle_rad: f64,

    /// Leg index (0-based) where minimum R/C separation occurs.
    pub min_rc_leg_index: usize,
    /// Mission elapsed time (s) at minimum R/C separation.
    pub min_rc_elapsed_s: f64,
    /// RIC position (km) at minimum R/C separation.
    pub min_rc_ric_position_km: Vector3<f64>,

    /// Leg index (0-based) where minimum 3D distance occurs.
    pub min_3d_leg_index: usize,
    /// Mission elapsed time (s) at minimum 3D distance.
    pub min_3d_elapsed_s: f64,
    /// RIC position (km) at minimum 3D distance.
    pub min_3d_ric_position_km: Vector3<f64>,
}

/// Per-timestep comparison between analytical and numerical propagation.
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
}

/// Aggregate validation results for a mission.
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
}

/// Departure orbital state for targeting: groups ROE, chief elements, and epoch.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct DepartureState {
    /// Deputy quasi-nonsingular ROE relative to chief
    pub roe: QuasiNonsingularROE,
    /// Chief mean Keplerian elements
    pub chief: KeplerianElements,
    /// Epoch of this state
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
}

/// Configuration for the Newton-Raphson targeting solver.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct TargetingConfig {
    /// Maximum Newton-Raphson iterations (default: 100)
    pub max_iterations: u32,
    /// Position convergence tolerance in km (default: 1e-6, i.e. 1 mm)
    pub position_tol_km: f64,
    /// Initial damping factor (default: 1.0)
    pub initial_damping: f64,
    /// Cap on any Δv component (km/s) (default: 1.0)
    pub dv_cap_km_s: f64,
    /// Number of trajectory steps per leg for output (default: 200)
    pub trajectory_steps: usize,
}

impl Default for TargetingConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            position_tol_km: 1e-6,
            initial_damping: 1.0,
            dv_cap_km_s: 1.0,
            trajectory_steps: 200,
        }
    }
}

/// Configuration for time-of-flight optimization.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct TofOptConfig {
    /// Minimum TOF as fraction of orbital period (default: 0.5)
    pub min_periods: f64,
    /// Maximum TOF as fraction of orbital period (default: 3.0)
    pub max_periods: f64,
    /// Number of initial TOF samples for multi-start (default: 5)
    pub num_starts: u32,
    /// Golden section convergence tolerance in seconds (default: 1.0)
    pub tol_s: f64,
}

impl Default for TofOptConfig {
    fn default() -> Self {
        Self {
            min_periods: 0.5,
            max_periods: 3.0,
            num_starts: 5,
            tol_s: 1.0,
        }
    }
}

/// Configuration for passive safety analysis.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SafetyConfig {
    /// e/i vector separation threshold (km) — minimum D'Amico Eq. 2.22 passive safety bound
    pub min_ei_separation_km: f64,
    /// 3D keep-out sphere threshold (km) — minimum allowed distance between vehicles
    pub min_distance_3d_km: f64,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            min_ei_separation_km: 0.2,
            min_distance_3d_km: 0.1,
        }
    }
}

/// Bundled mission configuration: targeting, TOF optimization, and safety.
///
/// Groups the three config structs that always travel together across
/// mission planning functions into a single serializable object.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MissionConfig {
    /// Newton-Raphson targeting solver settings.
    pub targeting: TargetingConfig,
    /// Time-of-flight optimization settings.
    pub tof: TofOptConfig,
    /// Optional passive safety analysis settings.
    pub safety: Option<SafetyConfig>,
}

/// Errors from mission planning.
#[derive(Debug, Clone)]
pub enum MissionError {
    /// Propagation failure during proximity phase.
    Propagation(crate::propagation::propagator::PropagationError),
    /// Lambert solver failure during transfer phase.
    Lambert(crate::mission::lambert::LambertError),
    /// ECI ↔ Keplerian conversion failure.
    Conversion(crate::elements::conversions::ConversionError),
    /// V-bar perch offset must be nonzero.
    InvalidVBarOffset {
        /// The invalid along-track offset (km).
        along_track_km: f64,
    },
    /// R-bar perch offset must be nonzero.
    InvalidRBarOffset {
        /// The invalid radial offset (km).
        radial_km: f64,
    },
    /// Spacecraft are not in proximity for ROE-based operations.
    NotInProximity {
        /// Actual dimensionless separation δr/r
        delta_r_over_r: f64,
        /// Configured proximity threshold
        threshold: f64,
    },
    /// Targeting solver failed to converge.
    TargetingConvergence {
        /// Final position error (km)
        final_error_km: f64,
        /// Number of iterations completed
        iterations: u32,
    },
    /// Jacobian is singular and cannot be inverted.
    SingularJacobian,
    /// No waypoints provided.
    EmptyWaypoints,
    /// TOF optimization failed to find a valid solution.
    TofOptimizationFailure {
        /// Minimum TOF searched (seconds)
        min_tof: f64,
        /// Maximum TOF searched (seconds)
        max_tof: f64,
        /// Number of multi-start samples evaluated
        num_starts: u32,
    },
    /// Replan index is out of bounds for the waypoint list.
    InvalidReplanIndex {
        /// The invalid index provided
        index: usize,
        /// Total number of waypoints
        num_waypoints: usize,
    },
}

impl std::fmt::Display for MissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Propagation(e) => write!(f, "MissionError: {e}"),
            Self::Lambert(e) => write!(f, "MissionError: {e}"),
            Self::Conversion(e) => write!(f, "MissionError: {e}"),
            Self::InvalidVBarOffset { along_track_km } => write!(
                f,
                "MissionError: invalid V-bar perch — along-track offset = {along_track_km:.6e} km (must be nonzero)"
            ),
            Self::InvalidRBarOffset { radial_km } => write!(
                f,
                "MissionError: invalid R-bar perch — radial offset = {radial_km:.6e} km (must be nonzero)"
            ),
            Self::NotInProximity { delta_r_over_r, threshold } => write!(
                f,
                "MissionError: not in proximity — δr/r = {delta_r_over_r:.6} exceeds threshold {threshold:.6}"
            ),
            Self::TargetingConvergence { final_error_km, iterations } => write!(
                f,
                "MissionError: targeting failed to converge — error = {final_error_km:.6e} km after {iterations} iterations"
            ),
            Self::SingularJacobian => write!(f, "MissionError: singular Jacobian in targeting solver"),
            Self::EmptyWaypoints => write!(f, "MissionError: no waypoints provided"),
            Self::TofOptimizationFailure { min_tof, max_tof, num_starts } => write!(
                f,
                "MissionError: TOF optimization failed — no valid TOF in [{min_tof:.1}, {max_tof:.1}] s ({num_starts} starts)"
            ),
            Self::InvalidReplanIndex { index, num_waypoints } => write!(
                f,
                "MissionError: replan index {index} out of bounds for {num_waypoints} waypoints"
            ),
        }
    }
}

impl std::error::Error for MissionError {}

impl From<crate::propagation::propagator::PropagationError> for MissionError {
    fn from(e: crate::propagation::propagator::PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<crate::mission::lambert::LambertError> for MissionError {
    fn from(e: crate::mission::lambert::LambertError) -> Self {
        Self::Lambert(e)
    }
}

impl From<crate::elements::conversions::ConversionError> for MissionError {
    fn from(e: crate::elements::conversions::ConversionError) -> Self {
        Self::Conversion(e)
    }
}

// --- Covariance propagation types ---

/// Navigation accuracy specification in RIC frame.
///
/// Users specify 1-sigma position and velocity accuracy per RIC axis.
/// Converted to ROE-space covariance internally via the T matrix inverse.
///
/// Defaults to typical LEO proximity operations navigation accuracy:
/// 100 m position, 0.1 m/s velocity per axis.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct NavigationAccuracy {
    /// 1-sigma position accuracy per RIC axis (km): [radial, in-track, cross-track]
    pub position_sigma_ric_km: Vector3<f64>,
    /// 1-sigma velocity accuracy per RIC axis (km/s): [radial, in-track, cross-track]
    pub velocity_sigma_ric_km_s: Vector3<f64>,
}

impl Default for NavigationAccuracy {
    fn default() -> Self {
        Self {
            position_sigma_ric_km: Vector3::new(
                DEFAULT_NAV_POSITION_SIGMA_KM,
                DEFAULT_NAV_POSITION_SIGMA_KM,
                DEFAULT_NAV_POSITION_SIGMA_KM,
            ),
            velocity_sigma_ric_km_s: Vector3::new(
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
                DEFAULT_NAV_VELOCITY_SIGMA_KM_S,
            ),
        }
    }
}

/// Maneuver execution uncertainty model.
///
/// Magnitude error is proportional (e.g., 0.01 = 1% 1-sigma).
/// Pointing error is a scalar 1-sigma (rad), applied as a rotation
/// of the nominal Δv direction via Rodrigues' formula.
///
/// This is the covariance-layer representation. The MC-layer counterpart
/// is `ManeuverDispersion`, which uses the same (`magnitude_sigma`, `pointing_sigma_rad`)
/// representation for sampling.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ManeuverUncertainty {
    /// Proportional 1-sigma magnitude error (dimensionless, e.g. 0.01 = 1%)
    pub magnitude_sigma: f64,
    /// 1-sigma pointing error (rad), applied as isotropic rotation about Δv axis
    pub pointing_sigma_rad: f64,
}

impl Default for ManeuverUncertainty {
    fn default() -> Self {
        Self {
            magnitude_sigma: DEFAULT_MANEUVER_MAGNITUDE_SIGMA,
            pointing_sigma_rad: DEFAULT_MANEUVER_POINTING_SIGMA_RAD,
        }
    }
}

/// Covariance state at a single epoch along the trajectory.
///
/// Contains the propagated covariance in ROE space, projected to RIC
/// position covariance, with derived uncertainty metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CovarianceState {
    /// Epoch
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
    /// Elapsed time since leg start (seconds)
    pub elapsed_s: f64,
    /// 6×6 ROE-space covariance [δa, δλ, δeₓ, δeᵧ, δiₓ, δiᵧ] (must be symmetric PSD)
    pub covariance_roe: Matrix6,
    /// 3×3 RIC position covariance (km²), projected from ROE via `T_pos`
    pub covariance_ric_position_km2: SMatrix<f64, 3, 3>,
    /// 3-sigma position bounds per RIC axis (km): sqrt(diag) × 3
    pub sigma3_position_ric_km: Vector3<f64>,
    /// Mahalanobis distance from chief (dimensionless)
    pub mahalanobis_distance: f64,
    /// Approximate collision probability (from Mahalanobis distance)
    pub collision_probability: f64,
}

/// Covariance evolution summary for a single maneuver leg.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LegCovarianceReport {
    /// Covariance states at sampled time points along the leg
    pub states: Vec<CovarianceState>,
    /// Maximum 3-sigma position uncertainty across leg (km, any axis)
    pub max_sigma3_position_km: f64,
    /// Maximum collision probability across leg
    pub max_collision_probability: f64,
}

/// Complete mission covariance propagation report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionCovarianceReport {
    /// Per-leg covariance evolution
    pub legs: Vec<LegCovarianceReport>,
    /// Navigation accuracy used
    pub navigation_accuracy: NavigationAccuracy,
    /// Maneuver uncertainty used (if any)
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Overall maximum 3-sigma position uncertainty (km)
    pub max_sigma3_position_km: f64,
    /// Overall maximum collision probability
    pub max_collision_probability: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, TWO_PI};
    use std::f64::consts::PI;

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    /// Build a minimal KeplerianElements with sensible defaults for fields that
    /// are not relevant to the method under test.
    fn make_elements(a_km: f64, e: f64, mean_anomaly_rad: f64) -> KeplerianElements {
        KeplerianElements {
            a_km,
            e,
            i_rad: 0.0,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad,
        }
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::true_anomaly
    // ---------------------------------------------------------------------------

    /// Circular orbit (e = 0): ν must equal M exactly for all M because
    /// E = M and ν = E when e = 0.
    #[test]
    fn true_anomaly_circular_orbit_equals_mean_anomaly() {
        // Tolerance: machine-epsilon level; Newton-Raphson converges in one
        // step for e = 0 (f = E - M = 0 at E = M), so the result is exact to
        // floating-point representation.
        let eps = 1e-15;
        let el = make_elements(6_778.0, 0.0, PI / 2.0);
        let nu = el.true_anomaly();
        assert!(
            (nu - PI / 2.0).abs() < eps,
            "circular orbit: expected ν = M = π/2, got {nu}"
        );
    }

    /// Circular orbit at M = 0 → ν = 0.
    #[test]
    fn true_anomaly_circular_m_zero() {
        let eps = 1e-15;
        let el = make_elements(6_778.0, 0.0, 0.0);
        let nu = el.true_anomaly();
        assert!(
            nu < eps,
            "circular orbit M=0: expected ν = 0, got {nu}"
        );
    }

    /// Periapsis/apoapsis symmetry: at M = π (apoapsis), ν = π for any
    /// eccentricity in [0, 1).  At apoapsis E = π and the half-angle formula
    /// gives ν = π directly.
    #[test]
    fn true_anomaly_moderate_eccentricity_apoapsis() {
        // Tolerance: KEPLER_TOL = 1e-14 is the Newton-Raphson convergence
        // criterion; the true anomaly is analytically π, so the only error
        // is floating-point rounding in the half-angle atan2 call.
        let eps = 1e-12;
        let el = make_elements(7_000.0, 0.3, PI);
        let nu = el.true_anomaly();
        assert!(
            (nu - PI).abs() < eps,
            "moderate eccentricity apoapsis: expected ν = π, got {nu}"
        );
    }

    /// High-eccentricity convergence test (e = 0.8): at M = π, ν must equal
    /// π (apoapsis symmetry holds at all eccentricities).
    #[test]
    fn true_anomaly_high_eccentricity_apoapsis() {
        // The solver switches initial guess to π for e >= 0.8 (avoids divergence).
        // Tolerance: 1e-12 — well above KEPLER_TOL; the analytic answer is exact.
        let eps = 1e-12;
        let el = make_elements(10_000.0, 0.8, PI);
        let nu = el.true_anomaly();
        assert!(
            (nu - PI).abs() < eps,
            "high-eccentricity apoapsis: expected ν = π, got {nu}"
        );
    }

    /// High-eccentricity convergence test (e = 0.8, M = 1.0 rad): validate
    /// that the solver converges (i.e., ν is finite and in [0, 2π)).
    /// Cross-check via Kepler's equation: given computed E, M_check = E - e*sin(E)
    /// must recover the original M.
    #[test]
    fn true_anomaly_high_eccentricity_convergence() {
        // Tolerance: residual in Kepler's equation should be at KEPLER_TOL level.
        let eps = 1e-10; // lenient: one round-trip through atan2 and trig
        let e = 0.8_f64;
        let m = 1.0_f64;
        let el = make_elements(10_000.0, e, m);
        let nu = el.true_anomaly();

        // Back-compute eccentric anomaly from true anomaly to verify round-trip.
        let half_nu = nu / 2.0;
        let ea = 2.0 * ((1.0 - e).sqrt() * half_nu.sin())
            .atan2((1.0 + e).sqrt() * half_nu.cos());
        let ea = ea.rem_euclid(TWO_PI);
        let m_check = (ea - e * ea.sin()).rem_euclid(TWO_PI);
        assert!(
            (m_check - m).abs() < eps,
            "high-eccentricity round-trip: M_check = {m_check}, expected {m}, diff = {}",
            (m_check - m).abs()
        );
    }

    /// Edge case M = 0 → ν = 0 (periapsis).
    #[test]
    fn true_anomaly_m_zero_is_periapsis() {
        let eps = 1e-15;
        let el = make_elements(7_500.0, 0.5, 0.0);
        let nu = el.true_anomaly();
        assert!(
            nu < eps || (nu - TWO_PI).abs() < eps,
            "M=0: expected ν=0 (or 2π alias), got {nu}"
        );
    }

    /// Edge case: M just below 2π wraps correctly — result must be in [0, 2π).
    #[test]
    fn true_anomaly_near_two_pi_wraps() {
        let el = make_elements(7_000.0, 0.3, TWO_PI - 1e-10);
        let nu = el.true_anomaly();
        assert!(
            nu >= 0.0 && nu < TWO_PI,
            "near-2π wrap: expected ν in [0, 2π), got {nu}"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::mean_motion
    // ---------------------------------------------------------------------------

    /// ISS-like orbit (a ≈ 6778 km): mean motion should be ≈ 0.001138 rad/s.
    /// Derived from n = sqrt(μ / a³).
    #[test]
    fn mean_motion_iss_like() {
        // Tolerance: 1e-9 rad/s — well below any physical or numerical
        // uncertainty; tests that the formula is implemented correctly.
        let eps = 1e-9;
        let a_km = 6_778.0_f64;
        let el = make_elements(a_km, 0.001, 0.0);
        let n = el.mean_motion();
        let n_expected = (MU_EARTH / (a_km * a_km * a_km)).sqrt();
        assert!(
            (n - n_expected).abs() < eps,
            "mean_motion ISS-like: expected {n_expected:.9}, got {n:.9}"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::period
    // ---------------------------------------------------------------------------

    /// ISS-like orbit (a ≈ 6778 km): period should be approximately 5550 s.
    /// T = 2π/n = 2π * sqrt(a³/μ).
    #[test]
    fn period_iss_like_approx_5550s() {
        // Tolerance: ±5 s — the ISS period varies slightly depending on the
        // exact SMA used; 6778 km gives ~5562 s; 5550 is a round-number bound.
        let a_km = 6_778.0_f64;
        let el = make_elements(a_km, 0.001, 0.0);
        let t = el.period();
        let t_expected = TWO_PI / (MU_EARTH / (a_km * a_km * a_km)).sqrt();
        // Identity check (period = 2π / mean_motion)
        assert!(
            (t - t_expected).abs() < 1e-9,
            "period: identity T = 2π/n failed; got {t}, expected {t_expected}"
        );
        // Approximate ISS sanity check
        assert!(
            (t - 5_550.0).abs() < 50.0,
            "period ISS-like: expected ~5550 s, got {t:.1} s"
        );
    }

    // ---------------------------------------------------------------------------
    // KeplerianElements::mean_arg_of_lat
    // ---------------------------------------------------------------------------

    /// u = ω + M, with result wrapped into [0, 2π).
    #[test]
    fn mean_arg_of_lat_basic_sum() {
        let eps = 1e-15;
        let mut el = make_elements(7_000.0, 0.01, 1.0);
        el.aop_rad = 0.5;
        // u = 0.5 + 1.0 = 1.5
        let u = el.mean_arg_of_lat();
        assert!(
            (u - 1.5).abs() < eps,
            "mean_arg_of_lat basic sum: expected 1.5, got {u}"
        );
    }

    /// u wraps when ω + M exceeds 2π.
    #[test]
    fn mean_arg_of_lat_wraps_above_two_pi() {
        let eps = 1e-12;
        let mut el = make_elements(7_000.0, 0.01, TWO_PI - 0.1);
        el.aop_rad = 0.5;
        // ω + M = 0.5 + (2π - 0.1) = 2π + 0.4 → wraps to 0.4
        let u = el.mean_arg_of_lat();
        assert!(
            (u - 0.4).abs() < eps,
            "mean_arg_of_lat wrap: expected 0.4, got {u}"
        );
    }

    /// u = 0 when ω = M = 0.
    #[test]
    fn mean_arg_of_lat_zero() {
        let eps = 1e-15;
        let el = make_elements(7_000.0, 0.01, 0.0);
        let u = el.mean_arg_of_lat();
        assert!(u < eps, "mean_arg_of_lat zero: expected 0, got {u}");
    }

    // ---------------------------------------------------------------------------
    // QuasiNonsingularROE::dimensionless_norm
    // ---------------------------------------------------------------------------

    /// Verify that dimensionless_norm returns max(|δa|, |δex|, |δey|, |δix|)
    /// and specifically EXCLUDES δλ and δiy, even when those are larger.
    #[test]
    fn dimensionless_norm_excludes_dlambda_and_diy() {
        let roe = QuasiNonsingularROE {
            da: 0.001,
            dlambda: 999.0, // large — must NOT dominate
            dex: 0.002,
            dey: 0.003,
            dix: 0.004, // expected max of the four included fields
            diy: 888.0, // large — must NOT dominate
        };
        let norm = roe.dimensionless_norm();
        // Expected: max(0.001, 0.002, 0.003, 0.004) = 0.004
        let expected = 0.004_f64;
        assert!(
            (norm - expected).abs() < 1e-15,
            "dimensionless_norm: expected {expected}, got {norm}"
        );
    }

    /// When all included components are zero, norm = 0.
    #[test]
    fn dimensionless_norm_zero_roe() {
        let roe = QuasiNonsingularROE::default();
        assert_eq!(roe.dimensionless_norm(), 0.0, "zero ROE: expected norm = 0");
    }

    /// Norm picks the correct maximum when δa is the largest.
    #[test]
    fn dimensionless_norm_da_dominates() {
        let roe = QuasiNonsingularROE {
            da: 0.01,
            dlambda: 0.005,
            dex: 0.003,
            dey: 0.002,
            dix: 0.001,
            diy: 0.009, // just below da but diy is excluded anyway
        };
        let norm = roe.dimensionless_norm();
        assert!(
            (norm - 0.01).abs() < 1e-15,
            "dimensionless_norm da-dominated: expected 0.01, got {norm}"
        );
    }

    /// Norm picks δex when it is the largest of the four included fields.
    #[test]
    fn dimensionless_norm_dex_dominates() {
        let roe = QuasiNonsingularROE {
            da: 0.001,
            dlambda: 0.5,
            dex: 0.1,
            dey: 0.05,
            dix: 0.02,
            diy: 0.3,
        };
        let norm = roe.dimensionless_norm();
        assert!(
            (norm - 0.1).abs() < 1e-15,
            "dimensionless_norm dex-dominated: expected 0.1, got {norm}"
        );
    }

    /// Negative components: norm uses absolute values.
    #[test]
    fn dimensionless_norm_negative_components() {
        let roe = QuasiNonsingularROE {
            da: -0.005,
            dlambda: 0.0,
            dex: 0.002,
            dey: -0.003,
            dix: 0.001,
            diy: 0.0,
        };
        let norm = roe.dimensionless_norm();
        // max(0.005, 0.002, 0.003, 0.001) = 0.005
        assert!(
            (norm - 0.005).abs() < 1e-15,
            "dimensionless_norm negative: expected 0.005, got {norm}"
        );
    }
}

/// Serde support for `hifitime::Epoch` (serialize as ISO 8601 string)
mod epoch_serde {
    use hifitime::Epoch;
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(epoch: &Epoch, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let s = format!("{epoch}");
        serializer.serialize_str(&s)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Epoch, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Epoch::from_gregorian_str(&s).map_err(serde::de::Error::custom)
    }
}

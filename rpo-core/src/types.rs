//! Core domain types: state vectors, Keplerian elements, ROEs, and RIC states.

use hifitime::Epoch;
use nalgebra::{SVector, Vector3};
use serde::{Deserialize, Serialize};

use crate::constants::{KEPLER_MAX_ITER, KEPLER_TOL, TWO_PI};

/// ECI J2000 state vector (position in km, velocity in km/s)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StateVector {
    /// Epoch of the state
    #[serde(with = "epoch_serde")]
    pub epoch: Epoch,
    /// Position vector in ECI frame (km)
    pub position: Vector3<f64>,
    /// Velocity vector in ECI frame (km/s)
    pub velocity: Vector3<f64>,
}

/// Classical Keplerian orbital elements
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct KeplerianElements {
    /// Semi-major axis (km)
    pub a: f64,
    /// Eccentricity
    pub e: f64,
    /// Inclination (rad)
    pub i: f64,
    /// Right ascension of ascending node (rad)
    pub raan: f64,
    /// Argument of perigee (rad)
    pub aop: f64,
    /// Mean anomaly (rad)
    pub mean_anomaly: f64,
}

impl KeplerianElements {
    /// Mean motion n = sqrt(μ/a³) (rad/s).
    #[must_use]
    pub fn mean_motion(&self) -> f64 {
        (crate::constants::MU_EARTH / (self.a * self.a * self.a)).sqrt()
    }

    /// Orbital period T = 2π/n (seconds).
    #[must_use]
    pub fn period(&self) -> f64 {
        std::f64::consts::TAU / self.mean_motion()
    }

    /// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E,
    /// then compute true anomaly ν.
    #[must_use]
    pub fn true_anomaly(&self) -> f64 {
        debug_assert!(self.e >= 0.0 && self.e < 1.0, "eccentricity must be in [0, 1), got {}", self.e);

        let m = self.mean_anomaly.rem_euclid(TWO_PI);
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
    #[must_use]
    pub fn mean_arg_of_lat(&self) -> f64 {
        (self.aop + self.mean_anomaly).rem_euclid(TWO_PI)
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

/// Relative state in the RIC (Radial-In-track-Cross-track) frame
/// Also known as Hill frame or LVLH frame
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RICState {
    /// Relative position in RIC frame (km): [radial, in-track, cross-track]
    pub position: Vector3<f64>,
    /// Time derivative of relative position in the rotating RIC frame, ρ̇ (km/s)
    pub velocity: Vector3<f64>,
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
}

/// A target waypoint in RIC space for maneuver targeting.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Waypoint {
    /// Target position in RIC frame (km): [radial, in-track, cross-track]
    pub position: Vector3<f64>,
    /// Target velocity in RIC frame (km/s): [radial, in-track, cross-track]
    pub velocity: Vector3<f64>,
    /// Time of flight to this waypoint (seconds). If `None`, TOF will be optimized.
    pub tof_s: Option<f64>,
}

/// A single impulsive maneuver (Δv) in the RIC frame.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Maneuver {
    /// Δv in RIC frame (km/s): [radial, in-track, cross-track]
    pub dv: Vector3<f64>,
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
    pub total_dv: f64,
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
    pub from_position: Vector3<f64>,
    /// Target RIC position (km)
    pub to_position: Vector3<f64>,
    /// Target RIC velocity (km/s)
    pub target_velocity: Vector3<f64>,
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
    pub total_dv: f64,
    /// Total mission duration (seconds)
    pub total_duration_s: f64,
    /// Safety metrics for the full mission (if computed)
    pub safety: Option<SafetyMetrics>,
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
    pub min_rc_ric_position: Vector3<f64>,

    /// Leg index (0-based) where minimum 3D distance occurs.
    pub min_3d_leg_index: usize,
    /// Mission elapsed time (s) at minimum 3D distance.
    pub min_3d_elapsed_s: f64,
    /// RIC position (km) at minimum 3D distance.
    pub min_3d_ric_position: Vector3<f64>,
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
    /// Invalid perch geometry configuration.
    InvalidPerch(String),
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
            Self::InvalidPerch(msg) => write!(f, "MissionError: invalid perch — {msg}"),
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

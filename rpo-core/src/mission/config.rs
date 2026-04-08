//! Mission configuration types: targeting, TOF optimization, and safety.

use serde::{Deserialize, Serialize};

/// Configuration for mission flow decision-making.
///
/// Controls the threshold at which two spacecraft are considered
/// within ROE-valid proximity vs. requiring a far-field transfer.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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

/// Configuration for the Newton-Raphson targeting solver.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
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

/// Configuration for safety analysis thresholds.
///
/// Contains thresholds for both operational and passive safety checks:
/// - `min_distance_3d_km`: operational keep-out sphere (3D distance)
/// - `min_ei_separation_km`: passive/abort e/i separation (D'Amico Eq. 2.22)
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SafetyConfig {
    /// e/i vector separation threshold (km) — passive/abort safety bound (D'Amico Eq. 2.22).
    /// Violations indicate the formation geometry would be unsafe in free drift,
    /// not necessarily during active guidance.
    pub min_ei_separation_km: f64,
    /// 3D keep-out sphere threshold (km) — operational safety bound.
    /// Violations indicate actual physical proximity below the threshold.
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
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MissionConfig {
    /// Newton-Raphson targeting solver settings.
    #[serde(default)]
    pub targeting: TargetingConfig,
    /// Time-of-flight optimization settings.
    #[serde(default)]
    pub tof: TofOptConfig,
    /// Optional passive safety analysis settings.
    #[serde(default)]
    pub safety: Option<SafetyConfig>,
}

//! Mission-level orchestration: classification, Lambert transfers, targeting, safety, and validation.

pub mod config;
pub mod covariance;
pub mod errors;
pub mod monte_carlo;
pub mod planning;
pub mod safety;
pub mod targeting;
pub mod types;
pub mod validation;
pub mod waypoints;

#[cfg(test)]
mod regression_tests;

pub use config::{MissionConfig, ProximityConfig, SafetyConfig, TargetingConfig, TofOptConfig};
pub use errors::{EclipseComputeError, MissionError};
pub use crate::propagation::lambert::{
    solve_lambert, solve_lambert_izzo, solve_lambert_with_config, LambertConfig, LambertError,
    LambertTransfer, TransferDirection,
};
pub use planning::{
    classify_separation, compute_transfer_eclipse, dimensionless_separation, eci_separation_km,
    plan_mission,
};
pub use safety::{
    analyze_safety, analyze_trajectory_safety, assess_safety, RcContext, SafetyAssessment,
    SafetyError,
};
pub use targeting::{optimize_tof, solve_leg};
pub use types::{
    EclipseIntervalComparison, EclipseValidation, EclipseValidationPoint,
    Maneuver, ManeuverLeg, MissionPhase, MissionPlan, OperationalSafety, PassiveSafety,
    PerchGeometry, SafetyMetrics, ValidationPoint, ValidationReport, Waypoint, WaypointMission,
};
pub use crate::propagation::nyx_bridge::{
    extract_dmf_rates, load_default_almanac, load_full_almanac, NyxBridgeError,
};
pub use validation::{validate_mission_nyx, ValidationError};
pub use covariance::propagate_mission_covariance;
pub use monte_carlo::{
    run_monte_carlo, MonteCarloError,
    CovarianceCrossCheck, DispersionConfig, DispersionEnvelope, Distribution,
    EnsembleStatistics, ManeuverDispersion, MonteCarloConfig, MonteCarloInput, MonteCarloMode,
    MonteCarloReport, PercentileStats, SampleResult, SpacecraftDispersion, StateDispersion,
};
pub use waypoints::{
    get_mission_state_at_time, plan_waypoint_mission, replan_from_waypoint,
    resample_leg_trajectory,
};

//! Mission-level orchestration: classification, Lambert transfers, targeting, safety, and validation.

pub mod cola_assessment;
pub mod avoidance;
pub mod closest_approach;
pub mod config;
pub mod covariance;
pub mod errors;
pub mod formation;
pub mod free_drift;
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
pub use closest_approach::{find_closest_approaches, ClosestApproach, PocaError};
pub use formation::{
    DriftPrediction, EiAlignment, EiSample, EnrichedWaypoint,
    EnrichmentMode, FormationDesignError, FormationDesignReport, PerchEnrichmentResult,
    PerchFallbackReason, SafePerch, SafetyRequirements, TransitSafetyReport,
};
pub use free_drift::{bounded_motion_residual, compute_free_drift, FreeDriftAnalysis, FreeDriftError};
pub use planning::{
    classify_separation, compute_transfer_eclipse, dimensionless_separation, eci_separation_km,
    plan_mission,
};
pub use safety::{
    analyze_safety, analyze_trajectory_safety, assess_safety, compute_ei_separation,
    EiSeparation, RcContext, SafetyAssessment, SafetyError,
};
pub use targeting::{optimize_tof, solve_leg, ArrivalVelocityStrategy};
pub use types::{
    EclipseIntervalComparison, EclipseValidation, EclipseValidationPoint,
    Maneuver, ManeuverLeg, MissionPhase, MissionPlan, OperationalSafety, PassiveSafety,
    PerchGeometry, SafetyMetrics, ValidationPoint, ValidationReport, Waypoint, WaypointMission,
};
pub use validation::{
    validate_leg_nyx, validate_mission_nyx, ColaBurn, LegValidationOutput, ValidationConfig,
    ValidationError, convert_cola_to_burns,
};
pub use covariance::propagate_mission_covariance;
pub use monte_carlo::{
    run_monte_carlo, MonteCarloControl, MonteCarloError,
    CovarianceCrossCheck, DispersionConfig, DispersionEnvelope, Distribution,
    EnsembleStatistics, ManeuverDispersion, MonteCarloConfig, MonteCarloInput, MonteCarloMode,
    MonteCarloReport, PercentileStats, SampleResult, SpacecraftDispersion, StateDispersion,
};
pub use waypoints::{
    get_mission_state_at_time, plan_waypoint_mission, replan_from_waypoint,
    resample_leg_trajectory,
};
pub use cola_assessment::{
    assess_cola, ColaAssessment, SecondaryViolation, SkippedLeg,
};
pub use avoidance::{
    compute_avoidance, AvoidanceError, AvoidanceManeuver, ColaConfig, CorrectionType,
};

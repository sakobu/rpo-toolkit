//! Core astrodynamics library for RPO mission planning.
//!
//! Provides Keplerian element conversions, quasi-nonsingular ROE computation
//! (D'Amico / Koenig formulation), J2-perturbed propagation via the Koenig STM,
//! J2+drag propagation via the DMF STM (Koenig Sec. VIII),
//! linearized ROE-to-RIC frame mapping, Lambert transfers, and mission planning.
//!
//! ## Module groups
//! - `elements` — coordinate conversions, ROE computation, frame mapping
//! - `propagation` — J2 params, STMs, propagation model, covariance
//! - `mission` — classification, Lambert transfers, validation

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

pub mod constants;
pub mod elements;
pub mod mission;
pub mod propagation;
pub mod types;

pub use elements::{
    apply_maneuver, compute_b_matrix, compute_roe, compute_t_matrix, compute_t_position,
    compute_t_velocity, eci_to_ric_dcm, eci_to_ric_dv, eci_to_ric_relative, keplerian_to_state,
    ric_position_to_roe, ric_to_eci_dv, ric_to_eci_position, ric_to_eci_state, roe_to_ric,
    state_to_keplerian, wrap_angle, ConversionError, DcmError, RicError,
};
pub use mission::{
    analyze_safety, analyze_trajectory_safety, classify_separation,
    dimensionless_separation, eci_separation_km, extract_dmf_rates,
    get_mission_state_at_time, load_default_almanac, load_full_almanac, optimize_tof,
    plan_mission, plan_waypoint_mission, propagate_mission_covariance, replan_from_waypoint,
    resample_leg_trajectory, run_monte_carlo, solve_lambert, solve_lambert_izzo,
    solve_lambert_with_config, solve_leg, validate_mission_nyx, LambertConfig, LambertError,
    LambertTransfer, MonteCarloError, SafetyError, TransferDirection, ValidationError,
    MissionConfig, MissionError, NyxBridgeError, ProximityConfig, SafetyConfig,
    TargetingConfig, TofOptConfig,
    Maneuver, ManeuverLeg, MissionPhase, MissionPlan, OperationalSafety, PassiveSafety,
    PerchGeometry, SafetyMetrics, ValidationPoint, ValidationReport, Waypoint, WaypointMission,
    CovarianceValidation, DispersionConfig, DispersionEnvelope, Distribution,
    EnsembleStatistics, ManeuverDispersion, MonteCarloConfig, MonteCarloInput, MonteCarloMode,
    MonteCarloReport, PercentileStats, SampleResult, SpacecraftDispersion, StateDispersion,
};
pub use propagation::{
    compute_j2_drag_stm, compute_j2_params, compute_stm, compute_stm_with_params,
    propagate_keplerian, ric_accuracy_to_roe_covariance,
    propagate_roe_j2_drag, propagate_roe_stm,
    CovarianceError, CovarianceState, LegCovarianceReport, ManeuverUncertainty,
    MissionCovarianceReport, NavigationAccuracy,
    DragConfig, J2Params, PropagatedState, PropagationError, PropagationModel,
};
pub use types::{
    DepartureState, KeplerError, KeplerianElements, Matrix6, Matrix9,
    QuasiNonsingularROE, RICState, SpacecraftConfig, StateVector,
};

#[cfg(test)]
mod test_helpers;

#[cfg(test)]
mod copy_trait_tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    fn assert_copy<T: Copy>() {}

    #[test]
    fn test_vector3_is_copy() {
        assert_copy::<Vector3<f64>>();
    }

    #[test]
    fn test_epoch_is_copy() {
        assert_copy::<Epoch>();
    }
}

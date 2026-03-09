//! Core astrodynamics library for RPO mission planning.
//!
//! Provides Keplerian element conversions, quasi-nonsingular ROE computation
//! (D'Amico / Koenig formulation), J2-perturbed propagation via the Koenig STM,
//! J2+drag propagation via the DMF STM (Koenig Sec. VIII),
//! linearized ROE-to-RIC frame mapping, Lambert transfers, and mission planning.
//!
//! ## Module groups
//! - `elements` — coordinate conversions, ROE computation, frame mapping
//! - `propagation` — J2 params, STMs, propagator trait/impls
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
    compute_t_velocity, keplerian_to_state, ric_position_to_roe, roe_to_ric, state_to_keplerian,
};
pub use mission::{
    analyze_safety, analyze_trajectory_safety, classify_separation,
    dimensionless_separation, eci_separation_km, get_mission_state_at_time, optimize_tof,
    plan_mission, plan_proximity_mission, plan_waypoint_mission, replan_from_waypoint,
    solve_lambert, solve_lambert_izzo, solve_lambert_with_config, solve_leg, LambertConfig,
    LambertError, LambertTransfer, TransferDirection,
};
pub use propagation::{
    compute_j2_drag_stm, compute_j2_params, compute_stm, compute_stm_with_params,
    propagate_roe_j2_drag, propagate_roe_stm, J2DragStmPropagator, J2Params, J2StmPropagator,
    PropagatedState, PropagationError, RelativePropagator,
};
pub use types::{
    DepartureState, DragConfig, KeplerianElements, Maneuver, ManeuverLeg, MissionConfig,
    MissionError, MissionPhase, MissionPlan, MissionPlanConfig, PerchGeometry, ProximityConfig,
    QuasiNonsingularROE, RICState, SafetyConfig, SafetyMetrics, StateVector, TargetingConfig,
    TofOptConfig, Waypoint, WaypointMission,
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

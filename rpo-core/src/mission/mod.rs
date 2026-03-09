//! Mission-level orchestration: classification, Lambert transfers, targeting, safety, and validation.

pub mod lambert;
pub mod planning;
pub mod safety;
pub mod targeting;
pub mod validation;
pub mod waypoints;

pub use lambert::{
    solve_lambert, solve_lambert_izzo, solve_lambert_with_config, LambertConfig, LambertError,
    LambertTransfer, TransferDirection,
};
pub use planning::{
    classify_separation, dimensionless_separation, eci_separation_km, plan_mission,
    plan_proximity_mission,
};
pub use safety::{analyze_safety, analyze_trajectory_safety, check_ei_separation};
pub use targeting::{optimize_tof, solve_leg};
pub use waypoints::{get_mission_state_at_time, plan_waypoint_mission};

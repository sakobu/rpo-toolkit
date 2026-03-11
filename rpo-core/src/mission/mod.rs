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
};
pub use safety::{analyze_safety, analyze_trajectory_safety, SafetyError};
pub use targeting::{optimize_tof, solve_leg};
pub use validation::{
    extract_dmf_rates, load_default_almanac, load_full_almanac, validate_mission_nyx,
    ValidationError,
};
pub use waypoints::{
    get_mission_state_at_time, plan_waypoint_mission, replan_from_waypoint,
    resample_leg_trajectory,
};

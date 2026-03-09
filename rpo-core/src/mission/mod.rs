//! Mission-level orchestration: classification, Lambert transfers, and validation.

pub mod lambert;
pub mod planning;
pub mod validation;

pub use lambert::{
    solve_lambert, solve_lambert_izzo, solve_lambert_with_config, LambertConfig, LambertError,
    LambertTransfer, TransferDirection,
};
pub use planning::{
    classify_separation, dimensionless_separation, eci_separation_km, plan_mission,
    plan_proximity_mission,
};

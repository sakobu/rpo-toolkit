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

pub use elements::{compute_roe, keplerian_to_state, roe_to_ric, state_to_keplerian};
pub use mission::{
    classify_separation, dimensionless_separation, eci_separation_km, plan_mission,
    plan_proximity_mission, solve_lambert, solve_lambert_izzo, solve_lambert_with_config,
    LambertConfig, LambertError, LambertTransfer, TransferDirection,
};
pub use propagation::{
    compute_j2_drag_stm, compute_j2_params, compute_stm, compute_stm_with_params,
    propagate_roe_j2_drag, propagate_roe_stm, J2DragStmPropagator, J2Params, J2StmPropagator,
    PropagatedState, PropagationError, RelativePropagator,
};
pub use types::{
    DragConfig, KeplerianElements, MissionError, MissionPhase, MissionPlan, MissionPlanConfig,
    PerchGeometry, ProximityConfig, QuasiNonsingularROE, RICState, StateVector,
};

#[cfg(test)]
mod test_helpers;

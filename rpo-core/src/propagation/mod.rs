//! J2 and J2+drag propagation via closed-form STMs, and nyx-space integration bridge.

pub mod covariance;
pub mod drag_stm;
pub mod j2_params;
pub mod keplerian;
pub mod lambert;
#[cfg(feature = "server")]
pub mod nyx_bridge;
pub mod propagator;
pub mod stm;

pub use covariance::{
    ric_accuracy_to_roe_covariance, CovarianceError,
    CovarianceState, LegCovarianceReport, ManeuverUncertainty, MissionCovarianceReport,
    NavigationAccuracy,
};
pub use drag_stm::{compute_j2_drag_stm, propagate_roe_j2_drag};
pub use j2_params::{compute_j2_params, J2Params};
pub use keplerian::propagate_keplerian;
pub use lambert::{LambertConfig, LambertError, LambertTransfer, TransferDirection};
#[cfg(feature = "server")]
pub use lambert::{solve_lambert, solve_lambert_izzo, solve_lambert_with_config};
#[cfg(feature = "server")]
pub use nyx_bridge::{
    extract_dmf_rates, load_default_almanac, load_full_almanac, ChiefDeputySnapshot,
    NyxBridgeError,
};
pub use propagator::{DragConfig, PropagatedState, PropagationError, PropagationModel};
pub use stm::{compute_stm, compute_stm_with_params, propagate_roe_stm};

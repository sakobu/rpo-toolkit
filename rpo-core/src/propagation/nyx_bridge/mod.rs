//! nyx-space adapter: almanac management, force model dynamics, coordinate conversions,
//! and full-physics propagation wrappers.

use crate::constants::MU_EARTH;

/// Earth J2000 frame for anise orbit construction.
///
/// Scoped to `nyx_bridge` because this constant depends on the `anise` crate,
/// which is only needed by the numerical (nyx) engine — not the analytical
/// STM-based engine.
pub(crate) const EARTH_J2000: anise::prelude::Frame = anise::prelude::Frame {
    ephemeris_id: 399,
    orientation_id: 1,
    mu_km3_s2: Some(MU_EARTH),
    shape: None,
};

mod errors;
mod almanac;
mod conversions;
mod dynamics;
mod propagate;

pub use errors::NyxBridgeError;
pub use almanac::{load_default_almanac, load_full_almanac};
pub use dynamics::extract_dmf_rates;
pub use propagate::ChiefDeputySnapshot;

// pub(crate) re-exports for internal consumers
pub(crate) use conversions::state_to_orbit;
pub(crate) use propagate::{
    TimedState, apply_impulse, build_nyx_safety_states, nyx_propagate_segment,
    query_anise_eclipse,
};
pub(crate) use dynamics::build_full_physics_dynamics;

//! Conversion from wire protocol types to rpo-core domain types.
//!
//! Most types are shared between wire and core — no translation needed.
//! This module exists primarily for `PropagatorChoice` → `PropagationModel`
//! and for assembling composite types (e.g., `DepartureState`).

use nalgebra::Vector3;
use serde::Deserialize;

use rpo_core::mission::Waypoint;
use rpo_core::propagation::{DragConfig, PropagationModel};

use crate::protocol::PropagatorChoice;

/// Waypoint input from the client (array-based position/velocity).
#[derive(Debug, Deserialize)]
pub struct WaypointInput {
    /// RIC position target [R, I, C] in km.
    pub position: [f64; 3],
    /// RIC velocity target [R, I, C] in km/s (defaults to zero).
    #[serde(default)]
    pub velocity: Option<[f64; 3]>,
    /// Time-of-flight hint (seconds). If None, solver optimizes TOF.
    #[serde(default)]
    pub tof_s: Option<f64>,
}

/// Convert a `PropagatorChoice` (wire) to `PropagationModel` (core).
pub fn to_propagation_model(choice: &PropagatorChoice) -> PropagationModel {
    match choice {
        PropagatorChoice::J2 => PropagationModel::J2Stm,
        PropagatorChoice::J2Drag { drag } => PropagationModel::J2DragStm { drag: *drag },
    }
}

/// Convert wire `WaypointInput`s to core `Waypoint`s.
pub fn to_waypoints(inputs: &[WaypointInput]) -> Vec<Waypoint> {
    inputs
        .iter()
        .map(|wp| {
            let vel = wp.velocity.unwrap_or([0.0, 0.0, 0.0]);
            Waypoint {
                position_ric_km: Vector3::new(wp.position[0], wp.position[1], wp.position[2]),
                velocity_ric_km_s: Vector3::new(vel[0], vel[1], vel[2]),
                tof_s: wp.tof_s,
            }
        })
        .collect()
}

/// Resolve the propagation model, optionally extracting differential drag rates.
///
/// When `auto_drag` is true, calls `extract_dmf_rates` and returns a
/// `J2DragStm` propagator with the derived drag config.
pub fn resolve_propagator(
    auto_drag: bool,
    perch_chief: &rpo_core::types::StateVector,
    perch_deputy: &rpo_core::types::StateVector,
    chief_config: &rpo_core::types::SpacecraftConfig,
    deputy_config: &rpo_core::types::SpacecraftConfig,
    almanac: &std::sync::Arc<anise::prelude::Almanac>,
    default_prop: PropagationModel,
) -> Result<(PropagationModel, Option<DragConfig>), crate::error::ApiError> {
    if !auto_drag {
        return Ok((default_prop, None));
    }
    let drag = rpo_core::propagation::extract_dmf_rates(
        perch_chief,
        perch_deputy,
        chief_config,
        deputy_config,
        almanac,
    )?;
    Ok((PropagationModel::J2DragStm { drag }, Some(drag)))
}

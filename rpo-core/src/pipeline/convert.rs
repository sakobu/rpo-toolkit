//! Type conversions between pipeline wire types and rpo-core domain types.
//!
//! Converts `PropagatorChoice` → `PropagationModel` and
//! `WaypointInput` → `Waypoint` (array-based → nalgebra-based).

use nalgebra::Vector3;

use crate::mission::types::Waypoint;
use crate::propagation::propagator::{DragConfig, PropagationModel};
use super::types::PropagatorChoice;

/// Convert a [`PropagatorChoice`] (user-facing) to [`PropagationModel`] (core).
#[must_use]
pub fn to_propagation_model(choice: &PropagatorChoice) -> PropagationModel {
    match choice {
        PropagatorChoice::J2 => PropagationModel::J2Stm,
        PropagatorChoice::J2Drag { drag } => PropagationModel::J2DragStm { drag: *drag },
    }
}

/// Convert a slice of [`WaypointInput`](super::types::WaypointInput)s
/// to core [`Waypoint`]s (array-based → nalgebra `Vector3`).
#[must_use]
pub fn to_waypoints(inputs: &[super::types::WaypointInput]) -> Vec<Waypoint> {
    inputs
        .iter()
        .map(|wp| {
            let vel = wp.velocity_ric_km_s.unwrap_or([0.0, 0.0, 0.0]);
            Waypoint {
                position_ric_km: Vector3::new(
                    wp.position_ric_km[0],
                    wp.position_ric_km[1],
                    wp.position_ric_km[2],
                ),
                velocity_ric_km_s: Vector3::new(vel[0], vel[1], vel[2]),
                tof_s: wp.tof_s,
            }
        })
        .collect()
}

/// Resolve the propagation model, optionally using auto-derived drag.
///
/// When `auto_drag` is `Some`, returns a `J2DragStm` propagator with
/// the derived drag config, ignoring the user's propagator choice.
/// Otherwise returns the propagator from the user's choice.
#[must_use]
pub fn resolve_propagator(
    auto_drag: Option<DragConfig>,
    default: PropagationModel,
) -> (PropagationModel, Option<DragConfig>) {
    match auto_drag {
        Some(drag) => (PropagationModel::J2DragStm { drag }, Some(drag)),
        None => (default, None),
    }
}

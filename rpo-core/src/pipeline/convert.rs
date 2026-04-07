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
        .map(|wp| Waypoint {
            position_ric_km: Vector3::from(wp.position_ric_km),
            velocity_ric_km_s: wp.velocity_ric_km_s.map(Vector3::from),
            tof_s: wp.tof_s,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::types::WaypointInput;

    #[test]
    fn to_waypoints_preserves_none_velocity() {
        let inputs = vec![
            WaypointInput {
                position_ric_km: [1.0, 0.0, 0.0],
                velocity_ric_km_s: None,
                tof_s: Some(3600.0),
                label: None,
            },
            WaypointInput {
                position_ric_km: [0.0, 1.0, 0.0],
                velocity_ric_km_s: Some([0.1, 0.0, 0.0]),
                tof_s: Some(3600.0),
                label: None,
            },
        ];
        let waypoints = to_waypoints(&inputs);
        assert!(waypoints[0].velocity_ric_km_s.is_none());
        assert!(waypoints[1].velocity_ric_km_s.is_some());
        assert_eq!(
            waypoints[1].velocity_ric_km_s.unwrap(),
            Vector3::new(0.1, 0.0, 0.0),
        );
    }

    /// Copy fidelity: array→Vector3 copies f64 values without arithmetic.
    /// Tolerance is machine epsilon — the copy should be bitwise exact.
    const COPY_FIDELITY_TOL: f64 = f64::EPSILON;

    #[test]
    fn to_waypoints_conversion() {
        let inputs = vec![
            WaypointInput {
                position_ric_km: [1.0, 2.0, 3.0],
                velocity_ric_km_s: Some([0.1, 0.2, 0.3]),
                tof_s: Some(100.0),
                label: None,
            },
            WaypointInput {
                position_ric_km: [4.0, 5.0, 6.0],
                velocity_ric_km_s: None,
                tof_s: None,
                label: None,
            },
        ];

        let waypoints = to_waypoints(&inputs);
        assert_eq!(waypoints.len(), 2);
        assert!(
            (waypoints[0].position_ric_km.x - 1.0).abs() < COPY_FIDELITY_TOL,
            "position_ric_km.x copy mismatch"
        );
        assert!(
            (waypoints[0].velocity_ric_km_s.unwrap().y - 0.2).abs() < COPY_FIDELITY_TOL,
            "velocity_ric_km_s.y copy mismatch"
        );
        assert!(
            (waypoints[0].tof_s.unwrap() - 100.0).abs() < COPY_FIDELITY_TOL,
            "tof_s copy mismatch"
        );
        assert!(
            waypoints[1].velocity_ric_km_s.is_none(),
            "None velocity should be preserved as position-only"
        );
        assert!(waypoints[1].tof_s.is_none());
    }
}

//! COLA burn types and analytical-to-nyx conversion.

use nalgebra::Vector3;
use serde::Deserialize;

use rpo_core::mission::avoidance::AvoidanceManeuver;
use rpo_core::mission::types::WaypointMission;

use super::super::errors::ValidationError;

/// A mid-coast COLA impulse for nyx validation.
///
/// # Invariants
/// - `elapsed_s` must be in `(0, leg.tof_s)` -- burn cannot be at leg boundaries
/// - `dv_ric_km_s` must be finite
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct ColaBurn {
    /// Index of the mission leg this burn applies to.
    pub leg_index: usize,
    /// Time from leg departure to COLA burn (seconds).
    pub elapsed_s: f64,
    /// Delta-v in RIC frame (km/s).
    pub dv_ric_km_s: Vector3<f64>,
}

/// All COLA-related inputs for nyx validation: burns to inject plus analytical
/// context for effectiveness comparison.
///
/// Bundles the nyx-format impulse burns with the analytical avoidance maneuvers
/// and target threshold so callers pass a single COLA struct instead of
/// separate `&[ColaBurn]` + context parameters.
///
/// Unit and frame assumptions are carried by field-level types
/// (`ColaBurn::dv_ric_km_s`, `AvoidanceManeuver::post_avoidance_poca_km`, etc.)
/// per crate boundary convention.
#[derive(Debug, Clone, Default)]
pub struct ColaValidationInput {
    /// Nyx-format impulse burns to inject during propagation.
    pub burns: Vec<ColaBurn>,
    /// Analytical avoidance maneuvers from `assess_cola()` for effectiveness comparison.
    pub analytical_maneuvers: Vec<AvoidanceManeuver>,
    /// Target COLA separation threshold (km) from `ColaConfig::target_distance_km`.
    pub target_distance_km: Option<f64>,
}

/// Convert COLA avoidance maneuvers to validation burn commands.
///
/// Uses each maneuver's epoch minus the leg departure epoch to compute `elapsed_s`.
///
/// Uses strict rejection (`elapsed_s <= 0 || >= tof_s`) because this function
/// consumes externally-specified burn times that must be exactly within the leg.
/// This differs from the avoidance module's lenient clamping
/// (`BURN_TIME_CLAMP_FRACTION`), which generates burns and can safely nudge
/// near-boundary solutions inward.
///
/// # Errors
/// Returns [`ValidationError::ColaEpochOutOfBounds`] if a maneuver's epoch
/// falls outside `(0, leg.tof_s)` for the corresponding leg.
pub(crate) fn convert_cola_to_burns(
    maneuvers: Option<&[AvoidanceManeuver]>,
    mission: &WaypointMission,
) -> Result<Vec<ColaBurn>, ValidationError> {
    let Some(maneuvers) = maneuvers else {
        return Ok(Vec::new());
    };

    let mut burns = Vec::with_capacity(maneuvers.len());
    for m in maneuvers {
        let leg = &mission.legs[m.leg_index];
        let leg_departure_epoch = leg.departure_maneuver.epoch;
        let elapsed_s = (m.epoch - leg_departure_epoch).to_seconds();

        if elapsed_s <= 0.0 || elapsed_s >= leg.tof_s {
            return Err(ValidationError::ColaEpochOutOfBounds {
                elapsed_s,
                tof_s: leg.tof_s,
                leg_index: m.leg_index,
            });
        }

        burns.push(ColaBurn {
            leg_index: m.leg_index,
            elapsed_s,
            dv_ric_km_s: m.dv_ric_km_s,
        });
    }
    Ok(burns)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use rpo_core::test_helpers::test_epoch;

    // =========================================================================
    // COLA Burn Conversion Tests (pure logic, no nyx)
    // =========================================================================

    /// Build a minimal `AvoidanceManeuver` at the given epoch and leg index.
    fn make_avoidance(epoch: hifitime::Epoch, leg_index: usize) -> rpo_core::mission::avoidance::AvoidanceManeuver {
        rpo_core::mission::avoidance::AvoidanceManeuver {
            epoch,
            dv_ric_km_s: Vector3::new(0.0, 0.001, 0.0),
            maneuver_location_rad: 0.0,
            post_avoidance_poca_km: 0.5,
            fuel_cost_km_s: 0.001,
            correction_type: rpo_core::mission::avoidance::CorrectionType::InPlane,
            leg_index,
        }
    }

    /// Build a minimal `WaypointMission` with one leg of given TOF starting at `dep_epoch`.
    fn make_one_leg_mission(dep_epoch: hifitime::Epoch, tof_s: f64) -> rpo_core::mission::types::WaypointMission {
        use rpo_core::mission::types::{Maneuver, ManeuverLeg};
        use rpo_core::types::QuasiNonsingularROE;
        let zero_dv = Vector3::zeros();
        let arr_epoch = dep_epoch + hifitime::Duration::from_seconds(tof_s);

        rpo_core::mission::types::WaypointMission {
            legs: vec![ManeuverLeg {
                departure_maneuver: Maneuver { dv_ric_km_s: zero_dv, epoch: dep_epoch },
                arrival_maneuver: Maneuver { dv_ric_km_s: zero_dv, epoch: arr_epoch },
                tof_s,
                total_dv_km_s: 0.0,
                pre_departure_roe: QuasiNonsingularROE::default(),
                post_departure_roe: QuasiNonsingularROE::default(),
                departure_chief_mean: rpo_core::test_helpers::iss_like_elements(),
                pre_arrival_roe: QuasiNonsingularROE::default(),
                post_arrival_roe: QuasiNonsingularROE::default(),
                arrival_chief_mean: rpo_core::test_helpers::iss_like_elements(),
                trajectory: vec![],
                from_position_ric_km: Vector3::zeros(),
                to_position_ric_km: Vector3::zeros(),
                target_velocity_ric_km_s: Vector3::zeros(),
                iterations: 0,
                position_error_km: 0.0,
            }],
            total_dv_km_s: 0.0,
            total_duration_s: tof_s,
            safety: None,
            covariance: None,
            eclipse: None,
        }
    }

    /// Verify `convert_cola_to_burns` computes correct `elapsed_s` from epoch delta.
    #[test]
    fn test_convert_cola_to_burns_epoch_to_elapsed() {
        let dep_epoch = test_epoch();
        let tof_s = 4200.0;
        let mission = make_one_leg_mission(dep_epoch, tof_s);

        // COLA maneuver at 2000s into the leg
        let cola_epoch = dep_epoch + hifitime::Duration::from_seconds(2000.0);
        let maneuvers = vec![make_avoidance(cola_epoch, 0)];

        let burns = super::convert_cola_to_burns(Some(&maneuvers), &mission)
            .expect("conversion should succeed");

        assert_eq!(burns.len(), 1);
        assert!(
            (burns[0].elapsed_s - 2000.0).abs() < rpo_core::constants::ELAPSED_TIME_TOL_S,
            "elapsed_s should be ~2000",
        );
        assert_eq!(burns[0].leg_index, 0);
    }

    /// Verify `convert_cola_to_burns` returns error for out-of-bounds epochs.
    #[test]
    fn test_convert_cola_to_burns_out_of_bounds() {
        let dep_epoch = test_epoch();
        let tof_s = 4200.0;
        let mission = make_one_leg_mission(dep_epoch, tof_s);

        // Epoch before leg start -> elapsed_s <= 0
        let before = dep_epoch - hifitime::Duration::from_seconds(100.0);
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(before, 0)]), &mission);
        assert!(result.is_err(), "epoch before leg start should fail");

        // Epoch after leg end -> elapsed_s >= tof_s
        let after = dep_epoch + hifitime::Duration::from_seconds(tof_s + 100.0);
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(after, 0)]), &mission);
        assert!(result.is_err(), "epoch after leg end should fail");

        // Epoch exactly at departure -> elapsed_s = 0 (boundary, should fail)
        let result = super::convert_cola_to_burns(Some(&[make_avoidance(dep_epoch, 0)]), &mission);
        assert!(result.is_err(), "epoch at exact departure should fail");
    }

    /// Verify `convert_cola_to_burns` returns empty vec for None input.
    #[test]
    fn test_convert_cola_to_burns_none_returns_empty() {
        let mission = make_one_leg_mission(test_epoch(), 4200.0);
        let burns = super::convert_cola_to_burns(None, &mission)
            .expect("None input should succeed");
        assert!(burns.is_empty());
    }
}

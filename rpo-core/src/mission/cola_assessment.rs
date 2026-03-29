//! COLA assessment: detect → suggest → verify across all legs.
//!
//! Evaluates a planned mission for POCA violations, computes avoidance
//! maneuvers for violations, and verifies that corrections do not create
//! secondary conjunctions on downstream legs. Returns a structured
//! assessment — the caller decides whether to accept the suggestions.
//!
//! # Assessment model
//!
//! Three outcomes with deterministic transitions:
//!
//! ```text
//! Nominal ──(violation detected)──▶ Avoidance ──(verify downstream)──▶ Recovery
//!                                                                        │
//!                                                              clean ◀───┤
//!                                                                        │
//!                                                  SecondaryConjunction ◀─┘
//! ```
//!
//! - **Nominal**: all POCAs satisfy the alert threshold.
//! - **Avoidance**: avoidance maneuvers computed for violating legs via
//!   [`compute_avoidance`](crate::mission::avoidance::compute_avoidance).
//! - **Recovery**: post-maneuver state propagated through ALL remaining legs,
//!   checking for secondary conjunctions created by the avoidance burns.
//!
//! # Design
//!
//! `assess_cola` is a pure function — the caller owns any loop. This
//! enables use from CLI (batch), WebSocket (request/response), or as
//! pre-computed contingency products for flight software interfaces.
//!
//! The multi-leg verification correctly cascades the COLA perturbation through
//! downstream maneuver boundaries: each leg applies the PLANNED Δv (from the
//! targeter) to the PERTURBED ROE (from the COLA cascade). The chief orbit is
//! unaffected by the deputy's avoidance maneuver, so all chief elements remain
//! as planned. `apply_maneuver` is linear (`new_roe = old_roe + B·Δv`), so the
//! perturbation cascades correctly through each boundary.
//!
//! # References
//! - D'Amico, "Formation Flying" §4.3.4 (AFC mode logic — assessment pattern)
//! - D'Amico Eqs. 2.38-2.56 (inverse GVE, delegated to `avoidance.rs`)
//! - D'Amico Eq. 2.22 (e/i separation)
//! - D'Amico Eq. 2.23 (minimum distance bound)

use hifitime::Duration;
use serde::{Deserialize, Serialize};

use crate::constants::TWO_PI;
use crate::elements::gve;
use crate::mission::avoidance::{compute_avoidance, AvoidanceManeuver, ColaConfig};
use crate::mission::closest_approach::{find_closest_approaches, ClosestApproach};
use crate::mission::types::{ManeuverLeg, WaypointMission};
use crate::propagation::propagator::PropagationModel;

// ---------------------------------------------------------------------------
// Named tolerances
// ---------------------------------------------------------------------------

/// Trajectory samples for downstream POCA verification.
/// Matches `COLA_VERIFICATION_STEPS` in `avoidance.rs`.
const COLA_ASSESSMENT_VERIFICATION_STEPS: usize = 200;

use crate::mission::avoidance::BURN_TIME_CLAMP_FRACTION;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A downstream POCA violation created by an avoidance maneuver.
///
/// Reports that a COLA maneuver on `original_leg_index` caused a new
/// conjunction risk on `violated_leg_index` after the perturbation cascaded
/// through the intervening maneuver boundaries.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondaryViolation {
    /// Leg index where the avoidance maneuver was applied.
    pub original_leg_index: usize,
    /// Downstream leg index where the new violation was detected.
    pub violated_leg_index: usize,
    /// The closest approach on the violated downstream leg.
    pub poca: ClosestApproach,
}

/// A leg where COLA was attempted but failed.
///
/// Uses a string error message rather than the full `AvoidanceError` enum,
/// since the error chain (`PocaError`, `PropagationError`, `ConversionError`)
/// does not derive `Serialize`/`Deserialize`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkippedLeg {
    /// Index of the leg with the unaddressed POCA violation.
    pub leg_index: usize,
    /// Human-readable description of why COLA failed for this leg.
    pub error_message: String,
}

/// Result of COLA assessment.
///
/// Downstream code pattern-matches on this to decide how to present results.
/// The `skipped` field in `Avoidance` and `SecondaryConjunction` surfaces legs
/// where COLA failed (budget exceeded, degenerate geometry, etc.) so the
/// operator can see why a violation was not addressed.
#[derive(Debug, Clone)]
pub enum ColaAssessment {
    /// All POCAs satisfy the alert threshold — no action needed.
    Nominal,
    /// Avoidance maneuvers computed; all downstream legs verified clean.
    Avoidance {
        /// Computed avoidance maneuvers (one per violating leg that succeeded).
        maneuvers: Vec<AvoidanceManeuver>,
        /// Legs where COLA was attempted but failed.
        skipped: Vec<SkippedLeg>,
    },
    /// Avoidance maneuvers computed, but at least one created a new
    /// conjunction risk on a downstream leg.
    SecondaryConjunction {
        /// Computed avoidance maneuvers.
        maneuvers: Vec<AvoidanceManeuver>,
        /// Downstream violations created by the avoidance burns.
        secondary_violations: Vec<SecondaryViolation>,
        /// Legs where COLA was attempted but failed.
        skipped: Vec<SkippedLeg>,
    },
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Assess a planned mission for COLA violations and compute avoidance suggestions.
///
/// Pure function — stateless, deterministic, caller owns any loop. Checks
/// all legs for POCA violations, computes avoidance maneuvers for those that
/// violate the threshold, then cascades each maneuver's perturbation through
/// all remaining legs to detect secondary conjunctions.
///
/// # Arguments
/// * `mission` — Planned waypoint mission with legs and trajectories
/// * `poca` — Per-leg closest-approach data (outer = legs, inner = POCAs)
/// * `propagator` — Resolved propagation model (J2 or J2+Drag)
/// * `config` — COLA target distance and delta-v budget
///
/// # Returns
/// An [`ColaAssessment`] indicating whether the mission is nominal, has
/// avoidance maneuvers, or has secondary conjunction warnings.
#[must_use]
pub fn assess_cola(
    mission: &WaypointMission,
    poca: &[Vec<ClosestApproach>],
    propagator: &PropagationModel,
    config: &ColaConfig,
) -> ColaAssessment {
    // 1. Find legs with POCA violations
    let violations: Vec<(usize, &ClosestApproach)> = poca
        .iter()
        .enumerate()
        .filter_map(|(i, leg_pocas)| {
            ClosestApproach::nearest(leg_pocas)
                .filter(|p| p.distance_km < config.target_distance_km)
                .map(|p| (i, p))
        })
        .collect();

    if violations.is_empty() {
        return ColaAssessment::Nominal;
    }

    // 2. Compute avoidance for each violation
    let mut maneuvers = Vec::new();
    let mut skipped = Vec::new();

    for &(i, closest) in &violations {
        let leg = &mission.legs[i];
        match compute_avoidance(
            closest,
            &leg.post_departure_roe,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            leg.tof_s,
            propagator,
            config,
        ) {
            Ok(mut m) => {
                m.leg_index = i;
                maneuvers.push(m);
            }
            Err(e) => {
                skipped.push(SkippedLeg {
                    leg_index: i,
                    error_message: format!("{e}"),
                });
            }
        }
    }

    if maneuvers.is_empty() {
        return ColaAssessment::Avoidance {
            maneuvers: Vec::new(),
            skipped,
        };
    }

    // 3. Multi-leg post-avoidance verification
    let mut secondary_violations = Vec::new();
    for m in &maneuvers {
        let secondaries =
            verify_downstream(m, &mission.legs, m.leg_index, propagator, config);
        secondary_violations.extend(secondaries);
    }

    if secondary_violations.is_empty() {
        ColaAssessment::Avoidance { maneuvers, skipped }
    } else {
        ColaAssessment::SecondaryConjunction {
            maneuvers,
            secondary_violations,
            skipped,
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Cascade a COLA maneuver's perturbation through downstream legs and check
/// for secondary conjunctions.
///
/// For each downstream leg M after the COLA leg N:
/// 1. Apply leg M's departure Δv to the perturbed ROE at M's boundary
/// 2. Propagate through M's coast arc
/// 3. Run POCA refinement on the perturbed trajectory
/// 4. Apply leg M's arrival Δv to cascade the perturbation to M+1
///
/// The chief orbit is unaffected (COLA only changes the deputy), so all
/// chief elements use the planned values from the mission.
fn verify_downstream(
    maneuver: &AvoidanceManeuver,
    legs: &[ManeuverLeg],
    leg_index: usize,
    propagator: &PropagationModel,
    config: &ColaConfig,
) -> Vec<SecondaryViolation> {
    if leg_index + 1 >= legs.len() {
        return Vec::new();
    }

    // Propagate COLA perturbation to the end of the current leg
    let Some(perturbed_post_arrival) =
        propagate_cola_to_leg_end(maneuver, &legs[leg_index], propagator)
    else {
        return Vec::new();
    };

    // Cascade through downstream legs
    let mut current_roe = perturbed_post_arrival;
    let mut violations = Vec::new();

    for (j, downstream_leg) in legs[leg_index + 1..].iter().enumerate() {
        let downstream_index = leg_index + 1 + j;

        // Apply departure maneuver of downstream leg
        let Ok(post_departure) = gve::apply_maneuver(
            &current_roe,
            &downstream_leg.departure_maneuver.dv_ric_km_s,
            &downstream_leg.departure_chief_mean,
        ) else {
            break;
        };

        // Propagate through downstream leg's coast
        let Ok(traj) = propagator.propagate_with_steps(
            &post_departure,
            &downstream_leg.departure_chief_mean,
            downstream_leg.departure_maneuver.epoch,
            downstream_leg.tof_s,
            COLA_ASSESSMENT_VERIFICATION_STEPS,
        ) else {
            break;
        };

        // Check POCA on the perturbed trajectory
        if let Ok(pocas) = find_closest_approaches(
            &traj,
            &downstream_leg.departure_chief_mean,
            downstream_leg.departure_maneuver.epoch,
            propagator,
            &post_departure,
            downstream_index,
        ) {
            if let Some(closest) = ClosestApproach::nearest(&pocas) {
                if closest.distance_km < config.target_distance_km {
                    violations.push(SecondaryViolation {
                        original_leg_index: leg_index,
                        violated_leg_index: downstream_index,
                        poca: closest.clone(),
                    });
                }
            }
        }

        let Some(state_at_end) = traj.last() else {
            break;
        };

        // Apply arrival maneuver to cascade perturbation to next leg
        let Ok(next_roe) = gve::apply_maneuver(
            &state_at_end.roe,
            &downstream_leg.arrival_maneuver.dv_ric_km_s,
            &downstream_leg.arrival_chief_mean,
        ) else {
            break;
        };
        current_roe = next_roe;
    }

    violations
}

/// Propagate the COLA maneuver's effect to the end of the current leg.
///
/// Steps: propagate to burn → apply COLA → propagate to leg end → apply
/// arrival maneuver. Returns the perturbed post-arrival ROE, which equals
/// the perturbed pre-departure ROE for the next leg.
fn propagate_cola_to_leg_end(
    maneuver: &AvoidanceManeuver,
    leg: &ManeuverLeg,
    propagator: &PropagationModel,
) -> Option<crate::types::QuasiNonsingularROE> {
    let n = leg.departure_chief_mean.mean_motion().ok()?;
    let u_0 = leg.departure_chief_mean.mean_arg_of_lat();

    // Time to COLA burn point
    let mut delta_t = ((maneuver.maneuver_location_rad - u_0).rem_euclid(TWO_PI)) / n;
    if delta_t >= leg.tof_s {
        delta_t = leg.tof_s * BURN_TIME_CLAMP_FRACTION;
    }

    // Propagate ROE to burn time
    let state_at_burn = propagator
        .propagate(
            &leg.post_departure_roe,
            &leg.departure_chief_mean,
            leg.departure_maneuver.epoch,
            delta_t,
        )
        .ok()?;

    // Advance chief mean anomaly to burn time
    let mut chief_at_burn = leg.departure_chief_mean;
    chief_at_burn.mean_anomaly_rad =
        (leg.departure_chief_mean.mean_anomaly_rad + n * delta_t).rem_euclid(TWO_PI);

    // Apply COLA maneuver
    let post_cola_roe =
        gve::apply_maneuver(&state_at_burn.roe, &maneuver.dv_ric_km_s, &chief_at_burn).ok()?;

    // Propagate to end of current leg
    let remaining_tof = leg.tof_s - delta_t;
    let burn_epoch = leg.departure_maneuver.epoch + Duration::from_seconds(delta_t);

    if remaining_tof <= 0.0 {
        // Burn at leg end — apply arrival maneuver directly to post-COLA state
        return gve::apply_maneuver(
            &post_cola_roe,
            &leg.arrival_maneuver.dv_ric_km_s,
            &leg.arrival_chief_mean,
        )
        .ok();
    }

    let state_at_leg_end = propagator
        .propagate(&post_cola_roe, &chief_at_burn, burn_epoch, remaining_tof)
        .ok()?;

    // Apply current leg's arrival maneuver
    gve::apply_maneuver(
        &state_at_leg_end.roe,
        &leg.arrival_maneuver.dv_ric_km_s,
        &leg.arrival_chief_mean,
    )
    .ok()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::closest_approach::ClosestApproach;
    use crate::mission::types::{Maneuver, ManeuverLeg, WaypointMission};
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::{damico_table21_case1_roe, damico_table21_chief};
    use crate::types::{KeplerianElements, QuasiNonsingularROE};
    use hifitime::Epoch;
    use nalgebra::Vector3;

    /// Number of trajectory steps for test legs.
    const TEST_STEPS: usize = 200;

    /// Build a free-coast `ManeuverLeg` (zero departure/arrival Δv).
    ///
    /// The trajectory is a pure propagation from the given ROE. The POCA
    /// during the coast equals the minimum distance of the bounded relative
    /// motion, giving predictable geometry for testing.
    fn build_free_coast_leg(
        roe: &QuasiNonsingularROE,
        chief: &KeplerianElements,
        epoch: Epoch,
        tof_periods: f64,
        model: &PropagationModel,
    ) -> ManeuverLeg {
        let period_s = chief.period().unwrap();
        let tof_s = tof_periods * period_s;

        let trajectory = model
            .propagate_with_steps(roe, chief, epoch, tof_s, TEST_STEPS)
            .unwrap();

        // Propagate to get arrival state
        let arrival_state = model.propagate(roe, chief, epoch, tof_s).unwrap();
        let n = chief.mean_motion().unwrap();
        let mut arrival_chief = *chief;
        arrival_chief.mean_anomaly_rad =
            (chief.mean_anomaly_rad + n * tof_s).rem_euclid(crate::constants::TWO_PI);
        let arrival_epoch = epoch + hifitime::Duration::from_seconds(tof_s);

        let from_pos = trajectory[0].ric.position_ric_km;

        ManeuverLeg {
            departure_maneuver: Maneuver {
                dv_ric_km_s: Vector3::zeros(),
                epoch,
            },
            arrival_maneuver: Maneuver {
                dv_ric_km_s: Vector3::zeros(),
                epoch: arrival_epoch,
            },
            tof_s,
            total_dv_km_s: 0.0,
            pre_departure_roe: *roe,
            post_departure_roe: *roe,
            departure_chief_mean: *chief,
            pre_arrival_roe: arrival_state.roe,
            post_arrival_roe: arrival_state.roe,
            arrival_chief_mean: arrival_chief,
            trajectory,
            from_position_ric_km: from_pos,
            to_position_ric_km: arrival_state.ric.position_ric_km,
            target_velocity_ric_km_s: Vector3::zeros(),
            iterations: 0,
            position_error_km: 0.0,
        }
    }

    /// Build a single-leg free-coast mission.
    fn build_free_coast_mission(
        roe: &QuasiNonsingularROE,
        chief: &KeplerianElements,
        tof_periods: f64,
        model: &PropagationModel,
    ) -> WaypointMission {
        let epoch = Epoch::from_gregorian_utc(2024, 1, 1, 0, 0, 0, 0);
        let leg = build_free_coast_leg(roe, chief, epoch, tof_periods, model);
        WaypointMission {
            total_dv_km_s: 0.0,
            total_duration_s: leg.tof_s,
            legs: vec![leg],
            safety: None,
            covariance: None,
            eclipse: None,
        }
    }

    /// Build POCA data for a mission by running `find_closest_approaches` on each leg.
    fn build_poca(
        mission: &WaypointMission,
        propagator: &PropagationModel,
    ) -> Vec<Vec<ClosestApproach>> {
        mission
            .legs
            .iter()
            .enumerate()
            .map(|(i, leg)| {
                find_closest_approaches(
                    &leg.trajectory,
                    &leg.departure_chief_mean,
                    leg.departure_maneuver.epoch,
                    propagator,
                    &leg.post_departure_roe,
                    i,
                )
                .unwrap_or_default()
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Test 1: Nominal passthrough — safe geometry, no violations
    // -----------------------------------------------------------------------

    #[test]
    fn test_nominal_passthrough() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let model = PropagationModel::J2Stm;

        // D'Amico Case 1: a·δey = 400 m → POCA ≈ 0.4 km (bounded motion)
        // Threshold well below POCA → Nominal
        let mission = build_free_coast_mission(&roe, &chief, 1.0, &model);
        let poca = build_poca(&mission, &model);

        let config = ColaConfig {
            target_distance_km: 0.1, // 100 m, well below ~400 m POCA
            max_dv_km_s: 0.01,
        };

        let decision = assess_cola(&mission, &poca, &model, &config);
        assert!(
            matches!(decision, ColaAssessment::Nominal),
            "expected Nominal for safe geometry, got {decision:?}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: Avoidance triggered — tight ROE, POCA below threshold
    // -----------------------------------------------------------------------

    #[test]
    fn test_avoidance_triggered() {
        let chief = damico_table21_chief();
        let a_km = chief.a_km;
        // Scale ROE down: a·δey = 40 m → POCA ≈ 0.04 km
        let tight_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.040 / a_km,
            dix: 0.0,
            diy: 0.020 / a_km,
        };
        let model = PropagationModel::J2Stm;

        let mission = build_free_coast_mission(&tight_roe, &chief, 1.0, &model);
        let poca = build_poca(&mission, &model);

        let config = ColaConfig {
            target_distance_km: 0.1, // 100 m — above the ~40 m POCA
            max_dv_km_s: 0.01,       // 10 m/s budget
        };

        let decision = assess_cola(&mission, &poca, &model, &config);
        match &decision {
            ColaAssessment::Avoidance { maneuvers, skipped } => {
                assert!(!maneuvers.is_empty(), "expected at least one maneuver");
                assert!(skipped.is_empty(), "expected no skipped legs");
                for m in maneuvers {
                    assert!(
                        m.post_avoidance_poca_km >= config.target_distance_km * 0.5,
                        "post-avoidance POCA {:.4} km too small (threshold {:.4} km)",
                        m.post_avoidance_poca_km,
                        config.target_distance_km,
                    );
                }
            }
            other => panic!("expected Avoidance, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Test 3: Budget exhaustion surfaces in skipped
    // -----------------------------------------------------------------------

    #[test]
    fn test_budget_exhaustion_surfaces_in_skipped() {
        let chief = damico_table21_chief();
        let a_km = chief.a_km;
        let tight_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.040 / a_km,
            dix: 0.0,
            diy: 0.020 / a_km,
        };
        let model = PropagationModel::J2Stm;

        let mission = build_free_coast_mission(&tight_roe, &chief, 1.0, &model);
        let poca = build_poca(&mission, &model);

        let config = ColaConfig {
            target_distance_km: 0.1,
            max_dv_km_s: 1e-10, // Impossibly tiny budget
        };

        let decision = assess_cola(&mission, &poca, &model, &config);
        match &decision {
            ColaAssessment::Avoidance { maneuvers, skipped } => {
                assert!(maneuvers.is_empty(), "expected no maneuvers with tiny budget");
                assert!(!skipped.is_empty(), "expected skipped legs");
                assert!(
                    skipped[0].error_message.contains("budget"),
                    "expected budget error, got: {}",
                    skipped[0].error_message,
                );
            }
            other => panic!("expected Avoidance with skipped, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Test 4: Single leg — no downstream verification needed
    // -----------------------------------------------------------------------

    #[test]
    fn test_single_leg_no_downstream() {
        let chief = damico_table21_chief();
        let a_km = chief.a_km;
        let tight_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.040 / a_km,
            dix: 0.0,
            diy: 0.020 / a_km,
        };
        let model = PropagationModel::J2Stm;

        let mission = build_free_coast_mission(&tight_roe, &chief, 1.0, &model);
        let poca = build_poca(&mission, &model);

        let config = ColaConfig {
            target_distance_km: 0.1,
            max_dv_km_s: 0.01,
        };

        let decision = assess_cola(&mission, &poca, &model, &config);
        // Single leg → Avoidance (never SecondaryConjunction — no downstream)
        assert!(
            matches!(decision, ColaAssessment::Avoidance { .. }),
            "single-leg mission should produce Avoidance, not SecondaryConjunction"
        );
    }

    // -----------------------------------------------------------------------
    // Test 5: Multi-leg downstream verification exercises the cascade path
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_leg_downstream_verification() {
        let chief = damico_table21_chief();
        let a_km = chief.a_km;
        let model = PropagationModel::J2Stm;
        let epoch = Epoch::from_gregorian_utc(2024, 1, 1, 0, 0, 0, 0);

        // Leg 0: tight ROE → POCA violation, triggers COLA
        let tight_roe = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.040 / a_km,
            dix: 0.0,
            diy: 0.020 / a_km,
        };
        let leg0 = build_free_coast_leg(&tight_roe, &chief, epoch, 1.0, &model);

        // Leg 1: departs from leg 0's arrival, also free coast
        let leg1_roe = leg0.post_arrival_roe;
        let leg1_chief = leg0.arrival_chief_mean;
        let leg1_epoch = leg0.arrival_maneuver.epoch;
        let leg1 = build_free_coast_leg(&leg1_roe, &leg1_chief, leg1_epoch, 1.0, &model);

        let mission = WaypointMission {
            total_dv_km_s: 0.0,
            total_duration_s: leg0.tof_s + leg1.tof_s,
            legs: vec![leg0, leg1],
            safety: None,
            covariance: None,
            eclipse: None,
        };

        let poca = build_poca(&mission, &model);

        let config = ColaConfig {
            target_distance_km: 0.1, // Triggers on leg 0
            max_dv_km_s: 0.01,
        };

        let decision = assess_cola(&mission, &poca, &model, &config);

        // Must not be Nominal — leg 0 has a violation
        match &decision {
            ColaAssessment::Nominal => {
                panic!("expected non-Nominal for tight leg 0")
            }
            ColaAssessment::Avoidance { maneuvers, .. } => {
                assert!(
                    !maneuvers.is_empty(),
                    "expected at least one maneuver for tight leg 0"
                );
                // Multi-leg verification ran and found no secondary — valid.
            }
            ColaAssessment::SecondaryConjunction {
                maneuvers,
                secondary_violations,
                ..
            } => {
                assert!(!maneuvers.is_empty());
                assert!(!secondary_violations.is_empty());
                for sv in secondary_violations {
                    assert_eq!(sv.original_leg_index, 0);
                    assert_eq!(sv.violated_leg_index, 1);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Test 6: Empty POCA data → Nominal
    // -----------------------------------------------------------------------

    #[test]
    fn test_empty_poca_nominal() {
        let chief = damico_table21_chief();
        let roe = damico_table21_case1_roe();
        let model = PropagationModel::J2Stm;

        let mission = build_free_coast_mission(&roe, &chief, 1.0, &model);

        // No POCAs at all → Nominal
        let empty_poca: Vec<Vec<ClosestApproach>> = vec![vec![]];
        let config = ColaConfig {
            target_distance_km: 10.0,
            max_dv_km_s: 0.01,
        };

        let decision = assess_cola(&mission, &empty_poca, &model, &config);
        assert!(matches!(decision, ColaAssessment::Nominal));
    }
}

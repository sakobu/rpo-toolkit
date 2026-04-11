//! Shared scaffolding for full-physics validation tests.
//!
//! Encapsulates the chief / deputy / epoch / almanac bootstrap and the
//! plan → nyx-validate pipeline that the `validate_*_full_physics*` tests
//! in [`super::trajectory`] and [`super::eclipse`] would otherwise duplicate
//! line-for-line. Kept internal to the `validation` module via `pub(super)`.
//!
//! # Paper traceability
//!
//! `iss_formation_roe` constructs a [`QuasiNonsingularROE`] per
//! **D'Amico Eq. 2.2** (QNS ROE definition). The km-scale input convention
//! is motivated by **D'Amico Eq. 2.17** (ROE→RIC mapping `T * ROE = r_ric`,
//! which scales the dimensionless ROE components by `a`): callers can
//! specify the *target* RIC-frame separation in km and the function
//! divides by `a` to yield dimensionless ROE. This matches how the rest
//! of the codebase reports formations for humans (meters / kilometers)
//! while keeping the stored ROE dimensionless.
//!
//! The QNS ROE components (`dex`/`dey`/`dix`/`diy`, 3-character paper
//! identifiers) trigger `clippy::similar_names`; a scoped allow preserves
//! the paper traceability.
#![allow(clippy::similar_names)]

use std::sync::Arc;

use anise::almanac::Almanac;
use hifitime::Epoch;

use rpo_core::elements::keplerian_to_state;
use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::types::{ValidationReport, Waypoint, WaypointMission};
use rpo_core::mission::waypoints::plan_waypoint_mission;
use rpo_core::propagation::PropagationModel;
use rpo_core::test_helpers::{deputy_from_roe, iss_like_elements, test_epoch};
use rpo_core::types::{
    DepartureState, KeplerianElements, QuasiNonsingularROE, SpacecraftConfig, StateVector,
};

use crate::nyx_bridge;

use super::trajectory::{ColaValidationInput, ValidationConfig};

/// Bootstrap state common to every full-physics validation test: the chief
/// Keplerian elements, derived deputy Keplerian elements, ECI state vectors
/// at a shared epoch, and the loaded (possibly network-backed) almanac.
///
/// Built once per test via [`ValidationContext::iss_with_formation`] or
/// [`ValidationContext::iss_colocated`], then threaded into
/// [`plan_and_validate`] (possibly multiple times for comparison tests).
pub(super) struct ValidationContext {
    pub(super) epoch: Epoch,
    pub(super) chief_ke: KeplerianElements,
    pub(super) chief_sv: StateVector,
    pub(super) deputy_sv: StateVector,
    pub(super) almanac: Arc<Almanac>,
    /// QNS ROE from which `deputy_sv` was derived. Stored here so that
    /// `plan_mission` has a single source of truth and cannot silently
    /// construct a `DepartureState.roe` that disagrees with `deputy_sv`.
    /// Populated by every constructor; never mutated.
    pub(super) formation_roe: QuasiNonsingularROE,
}

impl ValidationContext {
    /// ISS-like chief with a deputy derived from the given formation ROE
    /// (via [`deputy_from_roe`]). Loads the full almanac eagerly, so the
    /// first use in a test session hits the network.
    pub(super) fn iss_with_formation(formation_roe: &QuasiNonsingularROE) -> Self {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let deputy_ke = deputy_from_roe(&chief_ke, formation_roe);
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = keplerian_to_state(&deputy_ke, epoch).unwrap();
        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");

        // Hardening invariant: `deputy_sv` was derived from `formation_roe`
        // via `deputy_from_roe` + `keplerian_to_state`. Re-run that
        // derivation and confirm the result agrees with the stored
        // `deputy_sv` within f64 position noise. Today this passes by
        // construction, but the assert locks in the invariant so a future
        // refactor that decouples `deputy_sv` from `formation_roe` cannot
        // merge without tripping this check.
        debug_assert!({
            let roundtrip_ke = deputy_from_roe(&chief_ke, formation_roe);
            let roundtrip_sv = keplerian_to_state(&roundtrip_ke, epoch).unwrap();
            (roundtrip_sv.position_eci_km - deputy_sv.position_eci_km).norm()
                < rpo_core::constants::TEST_F64_POSITION_NOISE_KM
        });

        Self {
            epoch,
            chief_ke,
            chief_sv,
            deputy_sv,
            almanac,
            formation_roe: *formation_roe,
        }
    }

    /// ISS-like chief with a deputy colocated at the chief state vector.
    /// Used by differential-drag tests where chief and deputy start in the
    /// same orbit but have distinct ballistic coefficients.
    pub(super) fn iss_colocated() -> Self {
        let epoch = test_epoch();
        let chief_ke = iss_like_elements();
        let chief_sv = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_sv = chief_sv.clone();
        let almanac = nyx_bridge::load_full_almanac().expect("full almanac should load");
        Self {
            epoch,
            chief_ke,
            chief_sv,
            deputy_sv,
            almanac,
            formation_roe: QuasiNonsingularROE::zeros(),
        }
    }
}

/// Parameter bundle for [`plan_and_validate`], keeping the argument count
/// manageable while still allowing every knob to vary per call site.
///
/// Note: the formation ROE is no longer a field here — it lives on
/// `ValidationContext` as the single source of truth (see C-4), so
/// `plan_mission` reads `ctx.formation_roe` directly and cannot drift
/// away from the ROE that produced `ctx.deputy_sv`.
pub(super) struct PlanAndValidateInput<'a> {
    /// Target waypoints for the mission.
    pub(super) waypoints: &'a [Waypoint],
    /// Mission configuration (safety, etc.).
    pub(super) config: &'a MissionConfig,
    /// Propagation model to plan the mission with.
    pub(super) propagator: &'a PropagationModel,
    /// Chief spacecraft ballistic / mass properties.
    pub(super) chief_config: SpacecraftConfig,
    /// Deputy spacecraft ballistic / mass properties.
    pub(super) deputy_config: SpacecraftConfig,
    /// nyx sample count per mission leg.
    pub(super) samples_per_leg: u32,
    /// COLA validation input (use [`ColaValidationInput::default`] when the
    /// test does not exercise the collision avoidance path).
    pub(super) cola_input: &'a ColaValidationInput,
}

/// Plan a waypoint mission from the given context + input, without running
/// nyx validation. Useful when the caller needs to do mission-level analysis
/// (POCA computation, COLA assessment) before validating.
pub(super) fn plan_mission(
    ctx: &ValidationContext,
    input: &PlanAndValidateInput<'_>,
) -> WaypointMission {
    let departure = DepartureState {
        roe: ctx.formation_roe,
        chief: ctx.chief_ke,
        epoch: ctx.epoch,
    };
    plan_waypoint_mission(
        &departure,
        input.waypoints,
        input.config,
        input.propagator,
    )
    .expect("mission planning should succeed")
}

/// Run `validate_mission_nyx` against an already-planned mission using the
/// chief/deputy/almanac bundled in `ctx`. Kept separate from [`plan_mission`]
/// so callers that need to instrument the mission between planning and
/// validation (e.g. COLA assessment) don't pay for a re-plan.
pub(super) fn validate_planned(
    ctx: &ValidationContext,
    mission: &WaypointMission,
    input: &PlanAndValidateInput<'_>,
) -> ValidationReport {
    let val_config = ValidationConfig {
        samples_per_leg: input.samples_per_leg,
        chief_config: input.chief_config,
        deputy_config: input.deputy_config,
    };
    super::validate_mission_nyx(
        mission,
        &ctx.chief_sv,
        &ctx.deputy_sv,
        &val_config,
        input.cola_input,
        &ctx.almanac,
    )
    .expect("validation should succeed")
}

/// Standard plan → nyx-validate pipeline — a thin convenience wrapper over
/// [`plan_mission`] + [`validate_planned`] for tests that don't need to
/// intervene between the two steps.
pub(super) fn plan_and_validate(
    ctx: &ValidationContext,
    input: &PlanAndValidateInput<'_>,
) -> (WaypointMission, ValidationReport) {
    let mission = plan_mission(ctx, input);
    let report = validate_planned(ctx, &mission, input);
    (mission, report)
}

/// Build a formation ROE keyed to the ISS-like chief's semi-major axis.
///
/// The four arguments are the *target* RIC-frame separation scale in km
/// — the function divides each by the chief's `a` to produce the
/// dimensionless QNS ROE components (`dex`, `dey`, `dix`, `diy`) per
/// D'Amico Eq. 2.2. For example, passing `0.3` for `de_radial_km` means
/// "target 300 m of radial eccentricity separation", which becomes
/// `dex = 0.3 / a` in the returned ROE.
///
/// `da` and `dlambda` are always zero — drift tests that need non-trivial
/// `da`/`dlambda` should build the struct directly instead.
pub(super) fn iss_formation_roe(
    de_radial_km: f64,
    de_tangential_km: f64,
    di_normal_km: f64,
    di_phase_km: f64,
) -> QuasiNonsingularROE {
    let a = iss_like_elements().a_km;
    QuasiNonsingularROE {
        da: 0.0,
        dlambda: 0.0,
        dex: de_radial_km / a,
        dey: de_tangential_km / a,
        dix: di_normal_km / a,
        diy: di_phase_km / a,
    }
}

//! Stateful session for the WebSocket API.
//!
//! [`Session`] holds the mutable state for a single WebSocket connection.
//! Fields are organized into tiers: base states (tier 0), spacecraft
//! configuration (tier 1), transfer design (tier 2), mission plan (tier 3),
//! and analysis overlays (tier 4). Setter methods enforce invalidation:
//! changing an upstream tier clears downstream computed results.

use serde::Serialize;

use rpo_core::mission::config::MissionConfig;
use rpo_core::mission::monte_carlo::MonteCarloConfig;
use rpo_core::mission::types::{PerchGeometry, WaypointMission};
use rpo_core::mission::ProximityConfig;
use rpo_core::pipeline::convert::to_propagation_model;
use rpo_core::pipeline::types::DEFAULT_LAMBERT_TOF_S;
use rpo_core::pipeline::{PipelineInput, PropagatorChoice, SpacecraftChoice, TransferResult, WaypointInput};
use rpo_core::propagation::covariance::{ManeuverUncertainty, NavigationAccuracy};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::propagation::propagator::{DragConfig, PropagationModel};
use rpo_core::types::StateVector;

use crate::error::{ApiError, InvalidInputError};

/// Default V-bar perch distance (km), matching `rpo_core::pipeline::types::default_perch()`.
const DEFAULT_PERCH_ALONG_TRACK_KM: f64 = 5.0;

/// Mutable per-connection session state.
///
/// Organizes mission planning data into five tiers. Setters enforce
/// invalidation: changing an upstream tier clears downstream computed
/// results so that stale data is never served.
///
/// # Tiers
///
/// | Tier | Contents | Cleared by |
/// |------|----------|------------|
/// | 0 | chief, deputy ECI states | `set_states` |
/// | 1 | spacecraft configs | `set_spacecraft` |
/// | 2 | transfer design params + result | `set_transfer_params`, `store_transfer` |
/// | 3 | waypoints, config, propagator, mission result | `set_waypoints`, `set_config`, `set_propagator` |
/// | 4 | navigation, maneuver uncertainty, MC config | (overlays, no invalidation) |
#[derive(Debug)]
pub struct Session {
    // ---- Tier 0: Base states ----
    /// Chief spacecraft ECI state.
    pub chief: Option<StateVector>,
    /// Deputy spacecraft ECI state.
    pub deputy: Option<StateVector>,

    // ---- Tier 1: Spacecraft configuration ----
    /// Chief spacecraft preset or custom config (default: `Cubesat6U`).
    pub chief_config: SpacecraftChoice,
    /// Deputy spacecraft preset or custom config (default: `Cubesat6U`).
    pub deputy_config: SpacecraftChoice,

    // ---- Tier 2: Transfer design ----
    /// Computed transfer result (classification + Lambert + perch states).
    pub transfer: Option<TransferResult>,
    /// Perch geometry for Lambert-to-proximity handoff.
    pub perch: PerchGeometry,
    /// Lambert transfer time-of-flight (seconds).
    pub lambert_tof_s: f64,
    /// Lambert solver configuration (direction, revolutions).
    pub lambert_config: LambertConfig,
    /// Classification threshold configuration.
    pub proximity: ProximityConfig,
    /// Differential drag configuration (auto-derived or manual).
    pub drag_config: Option<DragConfig>,

    // ---- Tier 3: Mission plan ----
    /// Waypoint targets in RIC frame.
    pub waypoints: Vec<WaypointInput>,
    /// Solver configuration (targeting, TOF optimization, safety).
    pub config: MissionConfig,
    /// Propagator selection: J2 or J2+Drag.
    pub propagator: PropagatorChoice,
    /// Computed waypoint mission result.
    pub mission: Option<WaypointMission>,

    // ---- Tier 4: Analysis overlays ----
    /// Navigation accuracy for covariance propagation.
    pub navigation_accuracy: Option<NavigationAccuracy>,
    /// Maneuver execution uncertainty for covariance propagation.
    pub maneuver_uncertainty: Option<ManeuverUncertainty>,
    /// Monte Carlo ensemble configuration.
    pub monte_carlo_config: Option<MonteCarloConfig>,
}

impl Default for Session {
    fn default() -> Self {
        Self {
            // Tier 0
            chief: None,
            deputy: None,
            // Tier 1 — default to Cubesat6U per session spec
            chief_config: SpacecraftChoice::Cubesat6U,
            deputy_config: SpacecraftChoice::Cubesat6U,
            // Tier 2
            transfer: None,
            perch: PerchGeometry::VBar {
                along_track_km: DEFAULT_PERCH_ALONG_TRACK_KM,
            },
            lambert_tof_s: DEFAULT_LAMBERT_TOF_S,
            lambert_config: LambertConfig::default(),
            proximity: ProximityConfig::default(),
            drag_config: None,
            // Tier 3
            waypoints: Vec::new(),
            config: MissionConfig::default(),
            propagator: PropagatorChoice::default(),
            mission: None,
            // Tier 4
            navigation_accuracy: None,
            maneuver_uncertainty: None,
            monte_carlo_config: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Setters with invalidation
// ---------------------------------------------------------------------------

impl Session {
    /// Set chief and deputy ECI states, clearing downstream transfer and mission.
    pub fn set_states(&mut self, chief: StateVector, deputy: StateVector) {
        self.chief = Some(chief);
        self.deputy = Some(deputy);
        // Invalidate: transfer, drag_config, mission
        self.transfer = None;
        self.drag_config = None;
        self.mission = None;
    }

    /// Set spacecraft configurations, clearing drag config.
    pub fn set_spacecraft(&mut self, chief_config: SpacecraftChoice, deputy_config: SpacecraftChoice) {
        self.chief_config = chief_config;
        self.deputy_config = deputy_config;
        // Invalidate: drag_config only
        self.drag_config = None;
    }

    /// Optionally update chief and/or deputy spacecraft config.
    ///
    /// Returns the list of field names that were updated.
    /// Clears `drag_config` if any spacecraft config was changed.
    pub fn update_spacecraft(
        &mut self,
        chief_config: Option<SpacecraftChoice>,
        deputy_config: Option<SpacecraftChoice>,
    ) -> Vec<&'static str> {
        let mut updated = Vec::new();
        if let Some(cc) = chief_config {
            self.chief_config = cc;
            updated.push("chief_config");
        }
        if let Some(dc) = deputy_config {
            self.deputy_config = dc;
            updated.push("deputy_config");
        }
        if !updated.is_empty() {
            self.drag_config = None;
        }
        updated
    }

    /// Set transfer design parameters (perch, TOF, Lambert config), clearing transfer and mission.
    pub fn set_transfer_params(
        &mut self,
        perch: PerchGeometry,
        lambert_tof_s: f64,
        lambert_config: LambertConfig,
    ) {
        self.perch = perch;
        self.lambert_tof_s = lambert_tof_s;
        self.lambert_config = lambert_config;
        // Invalidate: transfer, mission
        self.transfer = None;
        self.mission = None;
    }

    /// Store a computed transfer result, clearing the mission.
    pub fn store_transfer(&mut self, result: TransferResult) {
        self.transfer = Some(result);
        // Invalidate: mission
        self.mission = None;
    }

    /// Store a drag configuration. Does not invalidate anything.
    pub fn store_drag(&mut self, drag: DragConfig) {
        self.drag_config = Some(drag);
    }

    /// Store a computed waypoint mission. Does not invalidate anything.
    pub fn store_mission(&mut self, mission: WaypointMission) {
        self.mission = Some(mission);
    }

    /// Set waypoint targets, clearing the mission.
    pub fn set_waypoints(&mut self, waypoints: Vec<WaypointInput>) {
        self.waypoints = waypoints;
        // Invalidate: mission
        self.mission = None;
    }

    /// Set solver configuration, clearing the mission.
    pub fn set_config(&mut self, config: MissionConfig) {
        self.config = config;
        // Invalidate: mission
        self.mission = None;
    }

    /// Set the propagator choice, clearing the mission.
    pub fn set_propagator(&mut self, choice: PropagatorChoice) {
        self.propagator = choice;
        // Invalidate: mission
        self.mission = None;
    }

    /// Set navigation accuracy for covariance propagation. No invalidation.
    pub fn set_navigation_accuracy(&mut self, accuracy: NavigationAccuracy) {
        self.navigation_accuracy = Some(accuracy);
    }

    /// Set maneuver execution uncertainty. No invalidation.
    pub fn set_maneuver_uncertainty(&mut self, uncertainty: ManeuverUncertainty) {
        self.maneuver_uncertainty = Some(uncertainty);
    }

    /// Set Monte Carlo ensemble configuration. No invalidation.
    pub fn set_monte_carlo_config(&mut self, config: MonteCarloConfig) {
        self.monte_carlo_config = Some(config);
    }

    /// Reset all session state to defaults.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

// ---------------------------------------------------------------------------
// Require accessors
// ---------------------------------------------------------------------------

impl Session {
    /// Require the chief ECI state.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when the chief state has not been set.
    pub fn require_chief(&self) -> Result<&StateVector, ApiError> {
        self.chief.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "chief",
                context: "set chief/deputy states via set_states before this operation",
            },
        ))
    }

    /// Require the deputy ECI state.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when the deputy state has not been set.
    pub fn require_deputy(&self) -> Result<&StateVector, ApiError> {
        self.deputy.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "deputy",
                context: "set chief/deputy states via set_states before this operation",
            },
        ))
    }

    /// Require a computed transfer result.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when no transfer has been computed.
    pub fn require_transfer(&self) -> Result<&TransferResult, ApiError> {
        self.transfer.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "transfer",
                context: "call compute_transfer before this operation",
            },
        ))
    }

    /// Require a computed waypoint mission.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when no mission has been planned.
    pub fn require_mission(&self) -> Result<&WaypointMission, ApiError> {
        self.mission.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "mission",
                context: "call plan_mission before this operation",
            },
        ))
    }

    /// Get the chief spacecraft configuration (infallible, always has a default).
    #[must_use]
    pub fn chief_config(&self) -> &SpacecraftChoice {
        &self.chief_config
    }

    /// Get the deputy spacecraft configuration (infallible, always has a default).
    #[must_use]
    pub fn deputy_config(&self) -> &SpacecraftChoice {
        &self.deputy_config
    }

    /// Require navigation accuracy configuration.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when navigation accuracy has not been set.
    pub fn require_navigation_accuracy(&self) -> Result<&NavigationAccuracy, ApiError> {
        self.navigation_accuracy.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "navigation_accuracy",
                context: "set navigation accuracy before covariance analysis",
            },
        ))
    }

    /// Require Monte Carlo configuration.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidInput`] with [`InvalidInputError::MissingSessionState`]
    /// when Monte Carlo config has not been set.
    pub fn require_monte_carlo_config(&self) -> Result<&MonteCarloConfig, ApiError> {
        self.monte_carlo_config.as_ref().ok_or(ApiError::InvalidInput(
            InvalidInputError::MissingSessionState {
                missing: "monte_carlo_config",
                context: "set Monte Carlo configuration before running MC analysis",
            },
        ))
    }
}

// ---------------------------------------------------------------------------
// Pipeline assembly
// ---------------------------------------------------------------------------

impl Session {
    /// Reconstruct a [`PipelineInput`] from the current session state.
    ///
    /// Requires chief and deputy states. All other fields use the current
    /// session values (which have sensible defaults).
    ///
    /// # Errors
    /// Returns [`ApiError`] if chief or deputy states are missing.
    pub fn assemble_pipeline_input(&self) -> Result<PipelineInput, ApiError> {
        let chief = self.require_chief()?.clone();
        let deputy = self.require_deputy()?.clone();

        Ok(PipelineInput {
            chief,
            deputy,
            perch: self.perch.clone(),
            lambert_tof_s: self.lambert_tof_s,
            lambert_config: self.lambert_config.clone(),
            waypoints: self.waypoints.clone(),
            proximity: self.proximity,
            config: self.config.clone(),
            propagator: self.propagator,
            chief_config: Some(self.chief_config),
            deputy_config: Some(self.deputy_config),
            navigation_accuracy: self.navigation_accuracy,
            maneuver_uncertainty: self.maneuver_uncertainty,
            monte_carlo: self.monte_carlo_config.clone(),
            cola: None,
            safety_requirements: None,
        })
    }

    /// Resolve the current propagator choice to a [`PropagationModel`].
    #[must_use]
    pub fn resolve_propagation_model(&self) -> PropagationModel {
        to_propagation_model(&self.propagator)
    }
}

// ---------------------------------------------------------------------------
// Summary
// ---------------------------------------------------------------------------

/// Snapshot of session state for diagnostics and status responses.
#[derive(Debug, Serialize)]
#[allow(clippy::struct_excessive_bools)]
pub struct SessionSummary {
    /// Whether a chief ECI state is set.
    pub has_chief: bool,
    /// Whether a deputy ECI state is set.
    pub has_deputy: bool,
    /// Whether a transfer result has been computed.
    pub has_transfer: bool,
    /// Whether a waypoint mission has been planned.
    pub has_mission: bool,
    /// Whether a drag configuration is available.
    pub has_drag_config: bool,
    /// Whether navigation accuracy is configured.
    pub has_navigation_accuracy: bool,
    /// Whether maneuver uncertainty is configured.
    pub has_maneuver_uncertainty: bool,
    /// Whether Monte Carlo config is set.
    pub has_monte_carlo_config: bool,
    /// Number of waypoints defined.
    pub waypoint_count: usize,
    /// Current chief spacecraft choice.
    pub chief_config: SpacecraftChoice,
    /// Current deputy spacecraft choice.
    pub deputy_config: SpacecraftChoice,
    /// Current propagator choice.
    pub propagator: PropagatorChoice,
    /// Current Lambert time-of-flight (seconds).
    pub lambert_tof_s: f64,
}

impl Session {
    /// Produce a diagnostic summary of the current session state.
    #[must_use]
    pub fn to_summary(&self) -> SessionSummary {
        SessionSummary {
            has_chief: self.chief.is_some(),
            has_deputy: self.deputy.is_some(),
            has_transfer: self.transfer.is_some(),
            has_mission: self.mission.is_some(),
            has_drag_config: self.drag_config.is_some(),
            has_navigation_accuracy: self.navigation_accuracy.is_some(),
            has_maneuver_uncertainty: self.maneuver_uncertainty.is_some(),
            has_monte_carlo_config: self.monte_carlo_config.is_some(),
            waypoint_count: self.waypoints.len(),
            chief_config: self.chief_config,
            deputy_config: self.deputy_config,
            propagator: self.propagator,
            lambert_tof_s: self.lambert_tof_s,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    use hifitime::Epoch;
    use nalgebra::Vector3;
    use rpo_core::mission::monte_carlo::{DispersionConfig, MonteCarloMode};
    use rpo_core::mission::types::{MissionPhase, MissionPlan};
    use rpo_core::types::{KeplerianElements, QuasiNonsingularROE};

    /// Construct a test `MonteCarloConfig` (the type has no `Default` impl).
    fn test_mc_config() -> MonteCarloConfig {
        MonteCarloConfig {
            num_samples: 10,
            dispersions: DispersionConfig {
                state: None,
                maneuver: None,
                spacecraft: None,
            },
            mode: MonteCarloMode::default(),
            seed: Some(42),
            trajectory_steps: 50,
        }
    }

    /// Construct a test state vector.
    fn test_state(x: f64) -> StateVector {
        StateVector {
            epoch: Epoch::from_gregorian_utc(2024, 6, 15, 12, 0, 0, 0),
            position_eci_km: Vector3::new(x, 0.0, 0.0),
            velocity_eci_km_s: Vector3::new(0.0, 7.61, 0.0),
        }
    }

    /// Construct a dummy transfer result for invalidation tests.
    fn dummy_transfer() -> TransferResult {
        let elements = KeplerianElements {
            a_km: 6878.0,
            e: 0.001,
            i_rad: 0.9,
            raan_rad: 0.0,
            aop_rad: 0.0,
            mean_anomaly_rad: 0.0,
        };
        TransferResult {
            plan: MissionPlan {
                phase: MissionPhase::Proximity {
                    roe: QuasiNonsingularROE::default(),
                    chief_elements: elements,
                    deputy_elements: elements,
                    separation_km: 5.0,
                    delta_r_over_r: 0.001,
                },
                transfer: None,
                perch_roe: QuasiNonsingularROE::default(),
                chief_at_arrival: elements,
            },
            perch_chief: test_state(6878.0),
            perch_deputy: test_state(6883.0),
            arrival_epoch: Epoch::from_gregorian_utc(2024, 6, 15, 13, 0, 0, 0),
            lambert_dv_km_s: 0.0,
        }
    }

    /// Construct a dummy waypoint mission for invalidation tests.
    fn dummy_mission() -> WaypointMission {
        WaypointMission {
            legs: vec![],
            total_dv_km_s: 0.0,
            total_duration_s: 0.0,
            safety: None,
            covariance: None,
            eclipse: None,
        }
    }

    // 1. set_states clears transfer, drag_config, mission
    #[test]
    fn test_set_states_invalidation() {
        let mut session = Session::default();
        session.chief = Some(test_state(6878.0));
        session.deputy = Some(test_state(6883.0));
        session.transfer = Some(dummy_transfer());
        session.drag_config = Some(DragConfig::zero());
        session.mission = Some(dummy_mission());

        session.set_states(test_state(7000.0), test_state(7005.0));

        assert!(session.chief.is_some());
        assert!(session.deputy.is_some());
        assert!(session.transfer.is_none(), "transfer should be cleared");
        assert!(session.drag_config.is_none(), "drag_config should be cleared");
        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 2. set_spacecraft clears drag_config, preserves transfer
    #[test]
    fn test_set_spacecraft_invalidation() {
        let mut session = Session::default();
        session.transfer = Some(dummy_transfer());
        session.drag_config = Some(DragConfig::zero());

        session.set_spacecraft(SpacecraftChoice::Servicer500Kg, SpacecraftChoice::Servicer500Kg);

        assert!(session.drag_config.is_none(), "drag_config should be cleared");
        assert!(session.transfer.is_some(), "transfer should be preserved");
    }

    // 3. set_transfer_params clears transfer and mission
    #[test]
    fn test_set_transfer_params_invalidation() {
        let mut session = Session::default();
        session.transfer = Some(dummy_transfer());
        session.mission = Some(dummy_mission());

        session.set_transfer_params(
            PerchGeometry::RBar { radial_km: 2.0 },
            1800.0,
            LambertConfig::default(),
        );

        assert!(session.transfer.is_none(), "transfer should be cleared");
        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 4. store_transfer clears mission
    #[test]
    fn test_store_transfer_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.store_transfer(dummy_transfer());

        assert!(session.transfer.is_some());
        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 5. store_drag does not clear mission
    #[test]
    fn test_store_drag_no_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.store_drag(DragConfig::zero());

        assert!(session.drag_config.is_some());
        assert!(session.mission.is_some(), "mission should be preserved");
    }

    // 6. set_waypoints clears mission
    #[test]
    fn test_set_waypoints_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.set_waypoints(vec![WaypointInput {
            position_ric_km: [0.0, 1.0, 0.0],
            velocity_ric_km_s: None,
            tof_s: None,
            label: None,
        }]);

        assert_eq!(session.waypoints.len(), 1);
        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 6b. set_config clears mission
    #[test]
    fn test_set_config_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.set_config(MissionConfig::default());

        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 6c. set_propagator clears mission
    #[test]
    fn test_set_propagator_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.set_propagator(PropagatorChoice::J2);

        assert!(session.mission.is_none(), "mission should be cleared");
    }

    // 7. overlay setters do not clear mission
    #[test]
    fn test_overlay_no_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        session.set_navigation_accuracy(NavigationAccuracy::default());
        session.set_maneuver_uncertainty(ManeuverUncertainty::default());

        assert!(session.mission.is_some(), "mission should be preserved");
        assert!(session.navigation_accuracy.is_some());
        assert!(session.maneuver_uncertainty.is_some());
    }

    // 7b. set_monte_carlo_config stores config and does not clear mission
    #[test]
    fn test_mc_config_stored_and_no_invalidation() {
        let mut session = Session::default();
        session.mission = Some(dummy_mission());

        let mc_config = test_mc_config();
        session.set_monte_carlo_config(mc_config.clone());

        assert!(session.mission.is_some(), "mission should be preserved");
        assert!(session.monte_carlo_config.is_some());
        // Verify we can retrieve it
        let retrieved = session.require_monte_carlo_config().expect("should be Some");
        assert_eq!(retrieved.num_samples, mc_config.num_samples);
    }

    // 8. require_chief returns error on empty session
    #[test]
    fn test_require_chief_none() {
        let session = Session::default();
        let err = session.require_chief().unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("chief"),
            "error message should mention 'chief': {msg}"
        );
    }

    // 9. require_transfer returns error on empty session
    #[test]
    fn test_require_transfer_none() {
        let session = Session::default();
        let err = session.require_transfer().unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("transfer"),
            "error message should mention 'transfer': {msg}"
        );
    }

    // 10. assemble_pipeline_input with fully populated session
    #[test]
    fn test_assemble_pipeline_input() {
        let mut session = Session::default();
        session.set_states(test_state(6878.0), test_state(6883.0));
        session.set_waypoints(vec![WaypointInput {
            position_ric_km: [0.0, 1.0, 0.0],
            velocity_ric_km_s: None,
            tof_s: Some(3600.0),
            label: Some("wp1".to_string()),
        }]);
        session.set_navigation_accuracy(NavigationAccuracy::default());

        let input = session.assemble_pipeline_input().expect("should succeed");

        // Verify chief/deputy
        assert!((input.chief.position_eci_km[0] - 6878.0).abs() < 1e-10);
        assert!((input.deputy.position_eci_km[0] - 6883.0).abs() < 1e-10);

        // Verify waypoints
        assert_eq!(input.waypoints.len(), 1);
        assert!((input.waypoints[0].position_ric_km[1] - 1.0).abs() < 1e-10);

        // Verify spacecraft configs are Some (Cubesat6U)
        assert!(input.chief_config.is_some());
        assert!(input.deputy_config.is_some());

        // Verify nav accuracy
        assert!(input.navigation_accuracy.is_some());

        // Verify default lambert_tof_s
        assert!((input.lambert_tof_s - DEFAULT_LAMBERT_TOF_S).abs() < 1e-10);
    }

    // 10b. assemble_pipeline_input fails without states
    #[test]
    fn test_assemble_pipeline_input_no_states() {
        let session = Session::default();
        let err = session.assemble_pipeline_input().unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("chief"), "should fail on missing chief: {msg}");
    }

    // 11. reset clears everything
    #[test]
    fn test_reset() {
        let mut session = Session::default();
        session.set_states(test_state(6878.0), test_state(6883.0));
        session.transfer = Some(dummy_transfer());
        session.mission = Some(dummy_mission());
        session.drag_config = Some(DragConfig::zero());
        session.set_navigation_accuracy(NavigationAccuracy::default());
        session.set_maneuver_uncertainty(ManeuverUncertainty::default());
        session.set_monte_carlo_config(test_mc_config());
        session.set_waypoints(vec![WaypointInput {
            position_ric_km: [0.0, 1.0, 0.0],
            velocity_ric_km_s: None,
            tof_s: None,
            label: None,
        }]);

        session.reset();

        assert!(session.chief.is_none());
        assert!(session.deputy.is_none());
        assert!(session.transfer.is_none());
        assert!(session.mission.is_none());
        assert!(session.drag_config.is_none());
        assert!(session.navigation_accuracy.is_none());
        assert!(session.maneuver_uncertainty.is_none());
        assert!(session.monte_carlo_config.is_none());
        assert!(session.waypoints.is_empty());
    }

    // 12. default spacecraft configs are Cubesat6U
    #[test]
    fn test_default_spacecraft_configs() {
        let session = Session::default();
        assert_eq!(*session.chief_config(), SpacecraftChoice::Cubesat6U);
        assert_eq!(*session.deputy_config(), SpacecraftChoice::Cubesat6U);
    }

    // Additional: resolve_propagation_model returns J2Stm by default
    #[test]
    fn test_resolve_propagation_model_default() {
        let session = Session::default();
        match session.resolve_propagation_model() {
            PropagationModel::J2Stm => {} // expected
            other => panic!("expected J2Stm, got {other:?}"),
        }
    }

    // Additional: to_summary reflects session state
    #[test]
    fn test_to_summary() {
        let mut session = Session::default();
        let summary = session.to_summary();
        assert!(!summary.has_chief);
        assert!(!summary.has_transfer);
        assert_eq!(summary.waypoint_count, 0);

        session.set_states(test_state(6878.0), test_state(6883.0));
        session.set_waypoints(vec![
            WaypointInput {
                position_ric_km: [0.0, 1.0, 0.0],
                velocity_ric_km_s: None,
                tof_s: None,
                label: None,
            },
            WaypointInput {
                position_ric_km: [0.0, 2.0, 0.0],
                velocity_ric_km_s: None,
                tof_s: None,
                label: None,
            },
        ]);

        let summary = session.to_summary();
        assert!(summary.has_chief);
        assert!(summary.has_deputy);
        assert!(!summary.has_transfer);
        assert_eq!(summary.waypoint_count, 2);
        assert_eq!(summary.chief_config, SpacecraftChoice::Cubesat6U);
    }
}

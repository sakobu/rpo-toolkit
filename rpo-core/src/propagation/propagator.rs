//! Propagation model enum and analytical J2 STM implementations.

use hifitime::{Duration, Epoch};
use serde::{Deserialize, Serialize};

use crate::propagation::drag_stm::propagate_roe_j2_drag;
use crate::elements::ric::roe_to_ric;
use crate::propagation::stm::propagate_roe_stm;
use crate::types::{DragConfig, KeplerianElements, QuasiNonsingularROE, RICState};

/// Result of a single propagation step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropagatedState {
    /// Absolute epoch of this state
    pub epoch: Epoch,
    /// Propagated quasi-nonsingular ROE
    pub roe: QuasiNonsingularROE,
    /// Propagated chief mean Keplerian elements
    pub chief_mean: KeplerianElements,
    /// Relative state in RIC frame
    pub ric: RICState,
    /// Elapsed time from epoch (seconds)
    pub elapsed_s: f64,
}

/// Error type for propagation failures.
#[derive(Debug, Clone)]
pub enum PropagationError {
    /// Number of steps must be greater than zero.
    ZeroSteps,
    /// Step count exceeds representable range.
    StepCountOverflow {
        /// The requested number of steps.
        n_steps: usize,
    },
}

impl std::fmt::Display for PropagationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ZeroSteps => write!(f, "PropagationError: n_steps must be > 0"),
            Self::StepCountOverflow { n_steps } => {
                write!(f, "PropagationError: n_steps = {n_steps} exceeds u32::MAX")
            }
        }
    }
}

impl std::error::Error for PropagationError {}

/// Analytical relative-orbit propagator, selected at construction time.
///
/// Dispatches to the appropriate closed-form STM at each call.
/// Use [`PropagationModel::J2Stm`] for J2-only and
/// [`PropagationModel::J2DragStm`] to include density-model-free
/// differential drag (Koenig Sec. VIII).
#[derive(Debug, Clone)]
pub enum PropagationModel {
    /// J2-perturbed STM propagator (Koenig formulation, Eq. A6).
    J2Stm,
    /// J2 + differential drag STM propagator (Koenig Sec. VIII / Appendix D).
    J2DragStm {
        /// DMF differential drag configuration.
        drag: DragConfig,
    },
}

impl PropagationModel {
    /// Propagate ROE forward by `dt_seconds` from `epoch_0`.
    ///
    /// # Invariants
    /// - `chief_mean_0.a_km > 0` and `0 <= chief_mean_0.e < 1`
    /// - `chief_mean_0` must be **mean** Keplerian elements, not osculating
    /// - `dt_seconds` must be finite
    #[must_use]
    pub fn propagate(
        &self,
        roe_0: &QuasiNonsingularROE,
        chief_mean_0: &KeplerianElements,
        epoch_0: Epoch,
        dt_seconds: f64,
    ) -> PropagatedState {
        match self {
            Self::J2Stm => {
                let (roe, chief_mean) = propagate_roe_stm(roe_0, chief_mean_0, dt_seconds);
                make_propagated_state(roe, chief_mean, epoch_0, dt_seconds)
            }
            Self::J2DragStm { drag } => {
                let (roe, chief_mean) =
                    propagate_roe_j2_drag(roe_0, chief_mean_0, drag, dt_seconds);
                make_propagated_state(roe, chief_mean, epoch_0, dt_seconds)
            }
        }
    }

    /// Propagate with intermediate steps, returning a trajectory.
    ///
    /// # Invariants
    /// - `chief_mean_0.a_km > 0` and `0 <= chief_mean_0.e < 1`
    /// - `chief_mean_0` must be **mean** Keplerian elements, not osculating
    /// - `n_steps > 0`
    /// - `dt_seconds` must be finite
    ///
    /// # Errors
    /// Returns `PropagationError` if `n_steps` is zero.
    pub fn propagate_with_steps(
        &self,
        roe_0: &QuasiNonsingularROE,
        chief_mean_0: &KeplerianElements,
        epoch_0: Epoch,
        dt_seconds: f64,
        n_steps: usize,
    ) -> Result<Vec<PropagatedState>, PropagationError> {
        if n_steps == 0 {
            return Err(PropagationError::ZeroSteps);
        }

        let n_steps_u32 = u32::try_from(n_steps).map_err(|_| {
            PropagationError::StepCountOverflow { n_steps }
        })?;
        let step = dt_seconds / f64::from(n_steps_u32);
        let mut states = Vec::with_capacity(n_steps + 1);

        // Initial state at t=0
        let initial = self.propagate(roe_0, chief_mean_0, epoch_0, 0.0);
        states.push(initial);

        // Propagate each step from the original epoch
        let mut t = 0.0;
        for _ in 1..=n_steps {
            t += step;
            let state = self.propagate(roe_0, chief_mean_0, epoch_0, t);
            states.push(state);
        }

        Ok(states)
    }
}

/// Build a [`PropagatedState`] from propagated ROE and chief elements.
fn make_propagated_state(
    roe: QuasiNonsingularROE,
    chief_mean: KeplerianElements,
    epoch_0: Epoch,
    dt_seconds: f64,
) -> PropagatedState {
    let ric = roe_to_ric(&roe, &chief_mean);
    PropagatedState {
        epoch: epoch_0 + Duration::from_seconds(dt_seconds),
        roe,
        chief_mean,
        ric,
        elapsed_s: dt_seconds,
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{iss_like_elements, test_epoch};

    #[test]
    fn propagator_initial_state() {
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let state = prop.propagate(&roe, &chief, epoch, 0.0);
        assert!((state.elapsed_s).abs() < 1e-15);
        assert!((state.roe.da - roe.da).abs() < 1e-10);
        assert_eq!(state.epoch, epoch);
    }

    #[test]
    fn propagator_multi_step() {
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let period = std::f64::consts::TAU / chief.mean_motion();
        let states = prop
            .propagate_with_steps(&roe, &chief, epoch, period, 10)
            .expect("propagation should succeed");

        assert_eq!(states.len(), 11); // 10 steps + initial
        assert!((states[0].elapsed_s).abs() < 1e-15);
        assert!((states[10].elapsed_s - period).abs() < 1e-6);
    }

    #[test]
    fn propagator_zero_steps_error() {
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE::default();

        let result = prop.propagate_with_steps(&roe, &chief, epoch, 3600.0, 0);
        assert!(result.is_err());
    }

    #[test]
    fn along_track_drift_grows() {
        // With nonzero δa, along-track distance should grow over time
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.0,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };

        let s1 = prop.propagate(&roe, &chief, epoch, 1000.0);
        let s2 = prop.propagate(&roe, &chief, epoch, 2000.0);

        // Along-track (y) should be growing in magnitude
        assert!(
            s2.ric.position_ric_km.y.abs() > s1.ric.position_ric_km.y.abs(),
            "Along-track drift should grow: {} vs {}",
            s1.ric.position_ric_km.y.abs(),
            s2.ric.position_ric_km.y.abs()
        );
    }

    #[test]
    fn j2_drag_zero_matches_j2_only() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE {
            da: 1.0 / chief.a_km,
            dlambda: 0.001,
            dex: 0.0001,
            dey: 0.0001,
            dix: 0.0005,
            diy: 0.0003,
        };

        let j2_prop = PropagationModel::J2Stm;
        let drag_prop = PropagationModel::J2DragStm {
            drag: DragConfig::zero(),
        };

        let tau = 3600.0;
        let s_j2 = j2_prop.propagate(&roe, &chief, epoch, tau);
        let s_drag = drag_prop.propagate(&roe, &chief, epoch, tau);

        assert!(
            (s_j2.roe.da - s_drag.roe.da).abs() < 1e-14,
            "da mismatch"
        );
        assert!(
            (s_j2.roe.dlambda - s_drag.roe.dlambda).abs() < 1e-14,
            "dlambda mismatch"
        );
        assert!(
            (s_j2.roe.dex - s_drag.roe.dex).abs() < 1e-14,
            "dex mismatch"
        );
        assert!(
            (s_j2.roe.dey - s_drag.roe.dey).abs() < 1e-14,
            "dey mismatch"
        );
        assert!(
            (s_j2.roe.dix - s_drag.roe.dix).abs() < 1e-14,
            "dix mismatch"
        );
        assert!(
            (s_j2.roe.diy - s_drag.roe.diy).abs() < 1e-14,
            "diy mismatch"
        );
        assert!(
            (s_j2.ric.position_ric_km - s_drag.ric.position_ric_km).norm() < 1e-10,
            "RIC position mismatch"
        );
    }

    #[test]
    fn j2_drag_multi_step() {
        use crate::test_helpers::test_drag_config;

        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE::default();

        let drag_prop = PropagationModel::J2DragStm {
            drag: test_drag_config(),
        };

        let period = std::f64::consts::TAU / chief.mean_motion();
        let states = drag_prop
            .propagate_with_steps(&roe, &chief, epoch, 3.0 * period, 30)
            .expect("propagation should succeed");

        assert_eq!(states.len(), 31);

        // Along-track drift should grow monotonically (drag causes secular drift)
        let mut prev_abs = 0.0_f64;
        for state in &states[1..] {
            let curr_abs = state.ric.position_ric_km.y.abs();
            assert!(
                curr_abs >= prev_abs * 0.9, // allow small oscillations
                "Along-track drift should generally grow: {curr_abs} vs {prev_abs}"
            );
            prev_abs = prev_abs.max(curr_abs);
        }
    }
}

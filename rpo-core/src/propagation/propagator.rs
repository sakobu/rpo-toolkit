//! Propagation model enum and analytical J2 STM implementations.

use hifitime::{Duration, Epoch};
use serde::{Deserialize, Serialize};

use crate::elements::keplerian_conversions::ConversionError;
use crate::propagation::drag_stm::propagate_roe_j2_drag;
use crate::elements::roe_to_ric::roe_to_ric;
use crate::propagation::stm::propagate_roe_stm;
use crate::types::{KeplerError, KeplerianElements, QuasiNonsingularROE, RICState};

/// Density-model-free (DMF) differential drag configuration (Koenig Sec. VIII).
///
/// Specifies the time derivatives of relative orbital elements due to
/// differential drag, normalized by chief semi-major axis. These rates
/// are treated as constant over the propagation interval.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct DragConfig {
    /// Time derivative of relative SMA due to drag: `δȧ_drag` (1/s, normalized by `a_c`)
    pub da_dot: f64,
    /// Time derivative of relative eccentricity x-component due to drag: `δėx_drag` (1/s)
    pub dex_dot: f64,
    /// Time derivative of relative eccentricity y-component due to drag: `δėy_drag` (1/s)
    pub dey_dot: f64,
}

impl DragConfig {
    /// Zero drag configuration (no differential drag).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            da_dot: 0.0,
            dex_dot: 0.0,
            dey_dot: 0.0,
        }
    }
}

/// Result of a single propagation step.
#[cfg_attr(feature = "wasm", derive(tsify_next::Tsify))]
#[cfg_attr(feature = "wasm", tsify(into_wasm_abi, from_wasm_abi))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropagatedState {
    /// Absolute epoch of this state
    #[serde(with = "crate::types::state::epoch_serde")]
    #[cfg_attr(feature = "wasm", tsify(type = "string"))]
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
    /// ROE→RIC conversion failed (theoretically unreachable after STM validation).
    RicConversion(ConversionError),
    /// Kepler equation or derived-quantity failure.
    KeplerFailure(KeplerError),
}

impl std::fmt::Display for PropagationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ZeroSteps => write!(f, "PropagationError: n_steps must be > 0"),
            Self::StepCountOverflow { n_steps } => {
                write!(f, "PropagationError: n_steps = {n_steps} exceeds u32::MAX")
            }
            Self::RicConversion(e) => {
                write!(f, "PropagationError: RIC conversion failed — {e}")
            }
            Self::KeplerFailure(e) => {
                write!(f, "PropagationError: {e}")
            }
        }
    }
}

impl From<ConversionError> for PropagationError {
    fn from(e: ConversionError) -> Self {
        Self::RicConversion(e)
    }
}

impl From<KeplerError> for PropagationError {
    fn from(e: KeplerError) -> Self {
        Self::KeplerFailure(e)
    }
}

impl std::error::Error for PropagationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::RicConversion(e) => Some(e),
            Self::KeplerFailure(e) => Some(e),
            _ => None,
        }
    }
}

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
    /// Propagate ROE forward by `dt_s` from `epoch_0`.
    ///
    /// # Invariants
    /// - `chief_mean_0.a_km > 0` and `0 <= chief_mean_0.e < 1`
    /// - `chief_mean_0` must be **mean** Keplerian elements, not osculating
    /// - `dt_s` must be finite (seconds)
    ///
    /// # Errors
    /// Returns `PropagationError` if eccentricity or SMA are out of range.
    pub fn propagate(
        &self,
        roe_0: &QuasiNonsingularROE,
        chief_mean_0: &KeplerianElements,
        epoch_0: Epoch,
        dt_s: f64,
    ) -> Result<PropagatedState, PropagationError> {
        match self {
            Self::J2Stm => {
                let (roe, chief_mean) = propagate_roe_stm(roe_0, chief_mean_0, dt_s)?;
                Ok(make_propagated_state(roe, chief_mean, epoch_0, dt_s)?)
            }
            Self::J2DragStm { drag } => {
                let (roe, chief_mean) =
                    propagate_roe_j2_drag(roe_0, chief_mean_0, drag, dt_s)?;
                Ok(make_propagated_state(roe, chief_mean, epoch_0, dt_s)?)
            }
        }
    }

    /// Propagate with intermediate steps, returning a trajectory.
    ///
    /// # Invariants
    /// - `chief_mean_0.a_km > 0` and `0 <= chief_mean_0.e < 1`
    /// - `chief_mean_0` must be **mean** Keplerian elements, not osculating
    /// - `n_steps > 0`
    /// - `dt_s` must be finite (seconds)
    ///
    /// # Errors
    /// Returns `PropagationError` if `n_steps` is zero.
    pub fn propagate_with_steps(
        &self,
        roe_0: &QuasiNonsingularROE,
        chief_mean_0: &KeplerianElements,
        epoch_0: Epoch,
        dt_s: f64,
        n_steps: usize,
    ) -> Result<Vec<PropagatedState>, PropagationError> {
        if n_steps == 0 {
            return Err(PropagationError::ZeroSteps);
        }

        let n_steps_u32 = u32::try_from(n_steps).map_err(|_| {
            PropagationError::StepCountOverflow { n_steps }
        })?;
        let step = dt_s / f64::from(n_steps_u32);
        let mut states = Vec::with_capacity(n_steps + 1);

        // Initial state at t=0
        let initial = self.propagate(roe_0, chief_mean_0, epoch_0, 0.0)?;
        states.push(initial);

        // Propagate each step from the original epoch
        let mut t = 0.0;
        for _ in 1..=n_steps {
            t += step;
            let state = self.propagate(roe_0, chief_mean_0, epoch_0, t)?;
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
    dt_s: f64,
) -> Result<PropagatedState, PropagationError> {
    let ric = roe_to_ric(&roe, &chief_mean)?;
    Ok(PropagatedState {
        epoch: epoch_0 + Duration::from_seconds(dt_s),
        roe,
        chief_mean,
        ric,
        elapsed_s: dt_s,
    })
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::{iss_like_elements, test_epoch};


    /// Elapsed time at t=0 should be exactly zero. 1e-15 accounts for
    /// floating-point representation of 0.0 through the propagator chain.
    const ELAPSED_ZERO_TOL: f64 = 1e-15;

    /// ROE at t=0 should match initial values. STM at tau=0 is identity
    /// within machine precision; 1e-10 is conservative.
    const ROE_INITIAL_TOL: f64 = 1e-10;

    /// Elapsed time at end of one orbital period. Accumulated step error
    /// from floating-point addition of step sizes.
    const PERIOD_ELAPSED_TOL: f64 = 1e-6;

    /// Zero-drag J2+drag must match J2-only to machine precision.
    /// The 6×6 J2 block is structurally identical when drag rates are zero.
    const J2_DRAG_MATCH_TOL: f64 = 1e-14;

    /// RIC position agreement between J2 and zero-drag J2+drag propagators.
    /// The ROE→RIC mapping introduces additional O(1e-10) error.
    const RIC_POSITION_MATCH_TOL: f64 = 1e-10;

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

        let state = prop.propagate(&roe, &chief, epoch, 0.0).unwrap();
        assert!((state.elapsed_s).abs() < ELAPSED_ZERO_TOL);
        assert!((state.roe.da - roe.da).abs() < ROE_INITIAL_TOL);
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

        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let states = prop
            .propagate_with_steps(&roe, &chief, epoch, period, 10)
            .expect("propagation should succeed");

        assert_eq!(states.len(), 11); // 10 steps + initial
        assert!((states[0].elapsed_s).abs() < ELAPSED_ZERO_TOL);
        assert!((states[10].elapsed_s - period).abs() < PERIOD_ELAPSED_TOL);
    }

    #[test]
    fn propagator_zero_steps_error() {
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE::default();

        let result = prop.propagate_with_steps(&roe, &chief, epoch, 3600.0, 0);
        assert!(
            matches!(result, Err(PropagationError::ZeroSteps)),
            "n_steps=0 should return ZeroSteps, got {result:?}"
        );
    }

    /// StepCountOverflow: requesting more than u32::MAX steps.
    #[test]
    fn propagator_step_count_overflow() {
        let prop = PropagationModel::J2Stm;
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE::default();

        // usize::MAX > u32::MAX on 64-bit platforms
        let huge_steps = u32::MAX as usize + 1;
        let result = prop.propagate_with_steps(&roe, &chief, epoch, 3600.0, huge_steps);
        assert!(
            matches!(result, Err(PropagationError::StepCountOverflow { .. })),
            "Huge step count should return StepCountOverflow, got {result:?}"
        );
    }

    /// Invalid eccentricity in chief elements should return PropagationError.
    #[test]
    fn propagator_invalid_eccentricity() {
        let prop = PropagationModel::J2Stm;
        let mut chief = iss_like_elements();
        chief.e = 1.0; // parabolic
        let epoch = test_epoch();
        let roe = QuasiNonsingularROE::default();

        let result = prop.propagate(&roe, &chief, epoch, 3600.0);
        assert!(
            matches!(result, Err(PropagationError::KeplerFailure(KeplerError::InvalidEccentricity { .. }))),
            "e=1.0 should return KeplerFailure(InvalidEccentricity), got {result:?}"
        );
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

        let s1 = prop.propagate(&roe, &chief, epoch, 1000.0).unwrap();
        let s2 = prop.propagate(&roe, &chief, epoch, 2000.0).unwrap();

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
        let s_j2 = j2_prop.propagate(&roe, &chief, epoch, tau).unwrap();
        let s_drag = drag_prop.propagate(&roe, &chief, epoch, tau).unwrap();

        assert!(
            (s_j2.roe.da - s_drag.roe.da).abs() < J2_DRAG_MATCH_TOL,
            "da mismatch"
        );
        assert!(
            (s_j2.roe.dlambda - s_drag.roe.dlambda).abs() < J2_DRAG_MATCH_TOL,
            "dlambda mismatch"
        );
        assert!(
            (s_j2.roe.dex - s_drag.roe.dex).abs() < J2_DRAG_MATCH_TOL,
            "dex mismatch"
        );
        assert!(
            (s_j2.roe.dey - s_drag.roe.dey).abs() < J2_DRAG_MATCH_TOL,
            "dey mismatch"
        );
        assert!(
            (s_j2.roe.dix - s_drag.roe.dix).abs() < J2_DRAG_MATCH_TOL,
            "dix mismatch"
        );
        assert!(
            (s_j2.roe.diy - s_drag.roe.diy).abs() < J2_DRAG_MATCH_TOL,
            "diy mismatch"
        );
        assert!(
            (s_j2.ric.position_ric_km - s_drag.ric.position_ric_km).norm() < RIC_POSITION_MATCH_TOL,
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

        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
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

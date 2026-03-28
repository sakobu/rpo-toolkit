//! Two-burn waypoint targeting via Newton-Raphson shooting in ROE space.
//!
//! Solves for departure and arrival Δv that transfer between ROE states
//! while hitting a target RIC position/velocity, using the
//! [`PropagationModel`] for the forward model.

use nalgebra::{SMatrix, Vector3};

use crate::elements::gve::apply_maneuver;
use crate::propagation::propagator::{PropagatedState, PropagationModel};
use crate::types::{DepartureState, KeplerianElements, QuasiNonsingularROE};

use super::config::{TargetingConfig, TofOptConfig};
use super::errors::MissionError;
use super::types::{Maneuver, ManeuverLeg};

/// Dimensionless `n*t` threshold below which the CW short-transfer (linear) approximation is used.
const CW_SHORT_TRANSFER_NT: f64 = 0.1;

/// SVD pseudo-inverse regularization epsilon for near-singular Jacobians.
const JACOBIAN_PINV_TOL: f64 = 1e-10;

/// Golden-section bracket half-width in orbital periods around the best multi-start TOF.
const TOF_REFINE_HALF_WINDOW: f64 = 0.5;

/// Golden ratio for golden-section TOF refinement: (√5 − 1) / 2.
const GOLDEN_RATIO: f64 = 0.618_033_988_749_894_9;

/// Clamp each component of a vector to `[-cap, cap]`.
fn clamp_dv(dv: &Vector3<f64>, cap: f64) -> Vector3<f64> {
    Vector3::new(
        dv.x.clamp(-cap, cap),
        dv.y.clamp(-cap, cap),
        dv.z.clamp(-cap, cap),
    )
}

/// CW (Clohessy-Wiltshire) initial guess for departure Δv.
///
/// For short transfers (`n*t < CW_SHORT_TRANSFER_NT`) uses linear approximation;
/// for longer transfers uses full CW STM inversion.
fn cw_initial_guess(
    chief: &KeplerianElements,
    target_pos: &Vector3<f64>,
    target_vel: &Vector3<f64>,
    tof_s: f64,
    cap: f64,
) -> Result<Vector3<f64>, MissionError> {
    let n = chief.mean_motion()?;
    let nt = n * tof_s;

    let dv = if nt.abs() < CW_SHORT_TRANSFER_NT {
        // Linear approximation for very short transfers
        Vector3::new(
            target_pos.x / tof_s,
            target_pos.y / tof_s,
            target_pos.z / tof_s,
        )
    } else {
        // Full CW STM inversion: Φ_rv maps initial velocity to final position
        let (sin_nt, cos_nt) = nt.sin_cos();
        let phi_rv = SMatrix::<f64, 3, 3>::new(
            sin_nt / n,
            2.0 * (1.0 - cos_nt) / n,
            0.0,
            -2.0 * (1.0 - cos_nt) / n,
            (4.0 * sin_nt - 3.0 * nt) / n,
            0.0,
            0.0,
            0.0,
            sin_nt / n,
        );

        // Solve: target_pos = Φ_rv · dv₀  →  dv₀ = Φ_rv⁻¹ · target_pos
        match phi_rv.try_inverse() {
            Some(inv) => inv * target_pos,
            None => *target_vel, // fallback
        }
    };

    Ok(clamp_dv(&dv, cap))
}

/// Forward model: apply departure Δv, propagate, compute arrival state.
///
/// Returns the propagated arrival state and the post-departure ROE.
fn forward_model(
    departure: &DepartureState,
    dv1: &Vector3<f64>,
    tof_s: f64,
    propagator: &PropagationModel,
) -> Result<(PropagatedState, QuasiNonsingularROE), MissionError> {
    let post_dv1_roe = apply_maneuver(&departure.roe, dv1, &departure.chief)?;
    let state = propagator.propagate(&post_dv1_roe, &departure.chief, departure.epoch, tof_s)?;
    Ok((state, post_dv1_roe))
}

/// Analytical 3×3 position Jacobian: `J = T_pos(chief_arr) · Φ(τ) · B(chief_dep)`.
///
/// Constant w.r.t. Δv because the forward model is affine in Δv.
/// Uses the J2 STM regardless of propagator type (exact for J2,
/// excellent approximation for J2+drag over typical 1-3 period legs).
fn compute_analytical_jacobian(
    chief_dep: &KeplerianElements,
    tof_s: f64,
) -> Result<SMatrix<f64, 3, 3>, MissionError> {
    let j2p = crate::propagation::j2_params::compute_j2_params(chief_dep)?;
    let phi = crate::propagation::stm::compute_stm_with_params(&j2p, chief_dep, tof_s);
    let chief_arr = crate::propagation::stm::propagate_chief_mean(chief_dep, &j2p, tof_s);
    let b = crate::elements::gve::compute_b_matrix(chief_dep)?;
    let t_pos = crate::elements::roe_to_ric::compute_t_position(&chief_arr)?;
    Ok(t_pos * phi * b)
}

/// Output of a converged Newton-Raphson iteration.
struct ConvergedSolution {
    /// Departure delta-v found by the solver.
    dv1: Vector3<f64>,
    /// ROE state immediately after the departure burn.
    post_dep_roe: QuasiNonsingularROE,
    /// Propagated state at arrival.
    arrival: PropagatedState,
    /// Number of Newton-Raphson iterations used (1-indexed).
    iterations: u32,
    /// Final position error at convergence (km).
    position_error_km: f64,
    /// Target RIC position that was solved for.
    target_position: Vector3<f64>,
    /// Target RIC velocity that was solved for.
    target_velocity: Vector3<f64>,
}

/// Solve a single two-burn transfer leg using Newton-Raphson shooting.
///
/// Finds departure Δv₁ (to hit target position) and arrival Δv₂ (to match
/// target velocity). The departure burn is optimized via Newton-Raphson on
/// the position residual; the arrival burn corrects the remaining velocity error.
///
/// # Invariants
/// - `tof_s > 0` (non-positive TOF produces undefined propagation)
/// - `departure.chief.a_km > 0` and `0 <= departure.chief.e < 1`
/// - `departure.chief` must be **mean** Keplerian elements, not osculating
/// - `config.max_iterations > 0`
/// - CW STM is singular at integer orbital periods for radial/cross-track
///   targets; use non-integer period TOFs to avoid singular Jacobian
///
/// # Arguments
/// * `departure` — Departure ROE state, chief elements, and epoch
/// * `target_position` — Target RIC position (km)
/// * `target_velocity` — Target RIC velocity (km/s)
/// * `tof_s` — Time of flight (seconds)
/// * `config` — Targeting solver configuration (tolerances, max iterations)
/// * `propagator` — Propagation model (J2 or J2+Drag)
///
/// # Errors
/// Returns `MissionError::TargetingConvergence` if the solver fails to converge,
/// or `MissionError::SingularJacobian` if the Jacobian cannot be inverted.
pub fn solve_leg(
    departure: &DepartureState,
    target_position: &Vector3<f64>,
    target_velocity: &Vector3<f64>,
    tof_s: f64,
    config: &TargetingConfig,
    propagator: &PropagationModel,
) -> Result<ManeuverLeg, MissionError> {
    let mut dv1 = cw_initial_guess(
        &departure.chief,
        target_position,
        target_velocity,
        tof_s,
        config.dv_cap_km_s,
    )?;

    // Analytical Jacobian: constant w.r.t. Δv (affine forward model).
    // Computed once; pre-invert for direct Newton steps.
    let jac = compute_analytical_jacobian(&departure.chief, tof_s)?;
    let jac_inv = if let Some(inv) = jac.try_inverse() {
        inv
    } else {
        let svd = jac.svd(true, true);
        svd.pseudo_inverse(JACOBIAN_PINV_TOL)
            .map_err(|_| MissionError::SingularJacobian)?
    };

    let mut last_error = f64::INFINITY;
    let max_iter = f64::from(config.max_iterations.max(1));

    for iter in 0..config.max_iterations {
        let (arrival, post_dep_roe) = forward_model(departure, &dv1, tof_s, propagator)?;

        // Position residual only — velocity corrected by arrival burn
        let pos_err = target_position - arrival.ric.position_ric_km;
        let pos_error_km = pos_err.norm();
        last_error = pos_error_km;

        if pos_error_km < config.position_tol_km {
            let solution = ConvergedSolution {
                dv1,
                post_dep_roe,
                arrival,
                iterations: iter + 1,
                position_error_km: pos_error_km,
                target_position: *target_position,
                target_velocity: *target_velocity,
            };
            return build_leg(departure, &solution, tof_s, config, propagator);
        }

        let delta_dv = jac_inv * pos_err;

        // Damping: ramp from initial_damping to 1.0
        let progress = f64::from(iter) / max_iter;
        let damping = config.initial_damping + (1.0 - config.initial_damping) * progress;

        dv1 += damping * delta_dv;
        dv1 = clamp_dv(&dv1, config.dv_cap_km_s);
    }

    Err(MissionError::TargetingConvergence {
        final_error_km: last_error,
        iterations: config.max_iterations,
    })
}

/// Cost-only variant: returns only the total Δv (km/s) without building
/// a full `ManeuverLeg`. Skips `build_leg` entirely — no trajectory,
/// epoch arithmetic, or ROE post-processing.
///
/// Used internally by [`optimize_tof`] where only `total_dv` is needed.
fn solve_leg_cost_only(
    departure: &DepartureState,
    target_position: &Vector3<f64>,
    target_velocity: &Vector3<f64>,
    tof_s: f64,
    config: &TargetingConfig,
    propagator: &PropagationModel,
) -> Result<f64, MissionError> {
    let mut dv1 = cw_initial_guess(
        &departure.chief,
        target_position,
        target_velocity,
        tof_s,
        config.dv_cap_km_s,
    )?;

    let jac = compute_analytical_jacobian(&departure.chief, tof_s)?;
    let jac_inv = if let Some(inv) = jac.try_inverse() {
        inv
    } else {
        let svd = jac.svd(true, true);
        svd.pseudo_inverse(JACOBIAN_PINV_TOL)
            .map_err(|_| MissionError::SingularJacobian)?
    };

    let mut last_error = f64::INFINITY;
    let max_iter = f64::from(config.max_iterations.max(1));

    for iter in 0..config.max_iterations {
        let (arrival, _) = forward_model(departure, &dv1, tof_s, propagator)?;

        let pos_err = target_position - arrival.ric.position_ric_km;
        let pos_error_km = pos_err.norm();
        last_error = pos_error_km;

        if pos_error_km < config.position_tol_km {
            let dv2 = target_velocity - arrival.ric.velocity_ric_km_s;
            return Ok(dv1.norm() + dv2.norm());
        }

        let delta_dv = jac_inv * pos_err;
        let progress = f64::from(iter) / max_iter;
        let damping = config.initial_damping + (1.0 - config.initial_damping) * progress;

        dv1 += damping * delta_dv;
        dv1 = clamp_dv(&dv1, config.dv_cap_km_s);
    }

    Err(MissionError::TargetingConvergence {
        final_error_km: last_error,
        iterations: config.max_iterations,
    })
}

/// Build a [`ManeuverLeg`] from converged solver results.
fn build_leg(
    departure: &DepartureState,
    solution: &ConvergedSolution,
    tof_s: f64,
    config: &TargetingConfig,
    propagator: &PropagationModel,
) -> Result<ManeuverLeg, MissionError> {
    let arrival_epoch = departure.epoch + hifitime::Duration::from_seconds(tof_s);

    // Arrival Δv corrects velocity mismatch
    let dv2 = solution.target_velocity - solution.arrival.ric.velocity_ric_km_s;
    let post_arr_roe = apply_maneuver(&solution.arrival.roe, &dv2, &solution.arrival.chief_mean)?;

    let total_dv_km_s = solution.dv1.norm() + dv2.norm();

    let trajectory = propagator.propagate_with_steps(
        &solution.post_dep_roe,
        &departure.chief,
        departure.epoch,
        tof_s,
        config.trajectory_steps,
    )?;

    let from_position_ric_km = crate::elements::roe_to_ric::roe_to_ric(&departure.roe, &departure.chief)?.position_ric_km;

    Ok(ManeuverLeg {
        departure_maneuver: Maneuver {
            dv_ric_km_s: solution.dv1,
            epoch: departure.epoch,
        },
        arrival_maneuver: Maneuver {
            dv_ric_km_s: dv2,
            epoch: arrival_epoch,
        },
        tof_s,
        total_dv_km_s,
        pre_departure_roe: departure.roe,
        post_departure_roe: solution.post_dep_roe,
        departure_chief_mean: departure.chief,
        pre_arrival_roe: solution.arrival.roe,
        post_arrival_roe: post_arr_roe,
        arrival_chief_mean: solution.arrival.chief_mean,
        trajectory,
        from_position_ric_km,
        to_position_ric_km: solution.target_position,
        target_velocity_ric_km_s: solution.target_velocity,
        iterations: solution.iterations,
        position_error_km: solution.position_error_km,
    })
}

/// Optimize the time of flight for a transfer leg using golden section search.
///
/// Searches over `[min_periods, max_periods]` orbital periods with multi-start.
///
/// # Invariants
/// - `departure.chief.a_km > 0` and `0 <= departure.chief.e < 1`
/// - `departure.chief` must be **mean** Keplerian elements, not osculating
/// - `tof_config.min_periods > 0` and `tof_config.max_periods > tof_config.min_periods`
/// - `tof_config.num_starts > 0`
///
/// # Arguments
/// * `departure` — Departure ROE state, chief elements, and epoch
/// * `target_position` — Target RIC position (km)
/// * `target_velocity` — Target RIC velocity (km/s)
/// * `targeting_config` — Targeting solver configuration
/// * `tof_config` — TOF optimization bounds and search configuration
/// * `propagator` — Propagation model (J2 or J2+Drag)
///
/// # Errors
/// Returns `MissionError::TofOptimizationFailure` if no valid TOF is found.
pub fn optimize_tof(
    departure: &DepartureState,
    target_position: &Vector3<f64>,
    target_velocity: &Vector3<f64>,
    targeting_config: &TargetingConfig,
    tof_config: &TofOptConfig,
    propagator: &PropagationModel,
) -> Result<(f64, ManeuverLeg), MissionError> {
    let period = departure.chief.period()?;
    let tof_min = tof_config.min_periods * period;
    let tof_max = tof_config.max_periods * period;
    let num_starts = f64::from(tof_config.num_starts);

    // Multi-start: evaluate at equally spaced TOFs
    let mut best_tof = None;
    let mut best_dv = f64::INFINITY;

    for k in 0..tof_config.num_starts {
        let frac = (f64::from(k) + 0.5) / num_starts;
        let tof = tof_min + frac * (tof_max - tof_min);

        if let Ok(dv) = solve_leg_cost_only(
            departure,
            target_position,
            target_velocity,
            tof,
            targeting_config,
            propagator,
        ) && dv < best_dv
        {
            best_dv = dv;
            best_tof = Some(tof);
        }
    }

    let center_tof = best_tof.ok_or(MissionError::TofOptimizationFailure {
        min_tof: tof_min,
        max_tof: tof_max,
        num_starts: tof_config.num_starts,
    })?;

    // Golden section search around best TOF
    let golden = GOLDEN_RATIO;
    let mut lo = (center_tof - TOF_REFINE_HALF_WINDOW * period).max(tof_min);
    let mut hi = (center_tof + TOF_REFINE_HALF_WINDOW * period).min(tof_max);

    let eval = |tof: f64| -> f64 {
        solve_leg_cost_only(
            departure,
            target_position,
            target_velocity,
            tof,
            targeting_config,
            propagator,
        )
        .unwrap_or(f64::INFINITY)
    };

    let mut c = hi - golden * (hi - lo);
    let mut d = lo + golden * (hi - lo);
    let mut f_c = eval(c);
    let mut f_d = eval(d);

    while (hi - lo) > tof_config.tol_s {
        if f_c < f_d {
            hi = d;
            d = c;
            f_d = f_c;
            c = hi - golden * (hi - lo);
            f_c = eval(c);
        } else {
            lo = c;
            c = d;
            f_c = f_d;
            d = lo + golden * (hi - lo);
            f_d = eval(d);
        }
    }

    let optimal_tof = lo.midpoint(hi);
    let leg = solve_leg(
        departure,
        target_position,
        target_velocity,
        optimal_tof,
        targeting_config,
        propagator,
    )?;

    Ok((optimal_tof, leg))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::propagation::propagator::PropagationModel;
    use crate::test_helpers::iss_like_elements;
    use crate::types::QuasiNonsingularROE;
    use hifitime::Epoch;

    /// Tolerance for deterministic Δv reproducibility: same inputs through the
    /// same code path must produce bitwise-identical results.
    /// Limited only by f64 representation noise — no independent evaluation.
    const DETERMINISTIC_DV_TOL: f64 = 1e-14;

    /// Relative tolerance for analytical-vs-finite-difference Jacobian comparison.
    /// The analytical Jacobian (T·Φ·B) and central-difference FD Jacobian agree
    /// to ~1e-6 due to FD truncation error; 1e-5 gives comfortable margin.
    const JACOBIAN_FD_REL_TOL: f64 = 1e-5;

    /// Finite-difference perturbation step size (km/s) for Jacobian verification.
    /// Small enough for accurate central differences, large enough to avoid
    /// catastrophic cancellation in f64 subtraction.
    const JACOBIAN_FD_STEP_KM_S: f64 = 1e-7;

    /// Near-zero threshold for switching from relative to absolute error in
    /// Jacobian element comparison. When the FD reference value is below this
    /// magnitude, relative error is meaningless — use absolute error instead.
    const JACOBIAN_NEAR_ZERO_THRESHOLD: f64 = 1e-10;

    /// Extremely tight position tolerance (km) used to force targeting
    /// non-convergence in a single iteration. No real mission would use this;
    /// it exists solely to exercise the TargetingConvergence error path.
    const FORCED_NONCONVERGENCE_TOL_KM: f64 = 1e-12;

    fn test_epoch() -> Epoch {
        crate::test_helpers::test_epoch()
    }

    fn zero_roe() -> QuasiNonsingularROE {
        QuasiNonsingularROE::default()
    }

    /// V-bar to V-bar transfer (along-track only).
    #[test]
    fn vbar_to_vbar_transfer() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let leg = solve_leg(
            &departure, &target_pos, &target_vel, period, &config, &propagator,
        )
        .expect("V-bar to V-bar should converge");

        assert!(leg.total_dv_km_s > 0.0, "Transfer should require nonzero Δv");
        assert!(leg.total_dv_km_s < 0.1, "Δv should be small for 5 km transfer");
    }

    /// V-bar to R-bar transfer (half-period TOF avoids CW singularity).
    #[test]
    fn vbar_to_rbar_transfer() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(2.0, 0.0, 0.0);
        let target_vel = Vector3::zeros();

        let leg = solve_leg(
            &departure, &target_pos, &target_vel, period * 0.5, &config, &propagator,
        )
        .expect("V-bar to R-bar should converge");

        assert!(leg.total_dv_km_s > 0.0);
    }

    /// 3D target (all RIC components nonzero, 0.75-period TOF).
    #[test]
    fn three_d_target() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();

        let roe0 = QuasiNonsingularROE {
            da: 0.0,
            dlambda: 0.001,
            dex: 0.0,
            dey: 0.0,
            dix: 0.0,
            diy: 0.0,
        };
        let departure = DepartureState { roe: roe0, chief, epoch };
        let target_pos = Vector3::new(1.0, 3.0, 0.5);
        let target_vel = Vector3::zeros();

        let leg = solve_leg(
            &departure, &target_pos, &target_vel, period * 0.75, &config, &propagator,
        )
        .expect("3D target should converge");

        assert!(leg.total_dv_km_s > 0.0);
    }

    /// Verify convergence: arrival position matches target within tolerance.
    #[test]
    fn convergence_check() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let leg = solve_leg(
            &departure, &target_pos, &target_vel, period, &config, &propagator,
        )
        .expect("should converge");

        // Verify by propagating with the computed Δv
        let post_dep = apply_maneuver(&departure.roe, &leg.departure_maneuver.dv_ric_km_s, &chief).unwrap();
        let state = propagator.propagate(&post_dep, &chief, epoch, period).unwrap();
        let pos_err = (state.ric.position_ric_km - target_pos).norm();

        assert!(
            pos_err < config.position_tol_km * 10.0,
            "Arrival position error should be small, got {pos_err} km"
        );
    }

    /// CW short transfer (`nt < 0.1`) uses linear approximation.
    #[test]
    fn cw_short_transfer() {
        let chief = iss_like_elements();
        let target = Vector3::new(0.0, 1.0, 0.0);
        let vel = Vector3::zeros();
        let tof = 0.01 / chief.mean_motion().unwrap();

        let guess = cw_initial_guess(&chief, &target, &vel, tof, 1.0).unwrap();
        assert!(guess.norm() > 0.0, "Short transfer should produce nonzero guess");
    }

    /// CW long transfer uses full STM inversion.
    #[test]
    fn cw_long_transfer() {
        let chief = iss_like_elements();
        let target = Vector3::new(0.0, 5.0, 0.0);
        let vel = Vector3::zeros();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();

        let guess = cw_initial_guess(&chief, &target, &vel, period, 1.0).unwrap();
        assert!(guess.norm() > 0.0, "Long transfer should produce nonzero guess");
    }

    /// Deterministic: same inputs produce same outputs.
    #[test]
    fn deterministic() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let leg1 = solve_leg(
            &departure, &target_pos, &target_vel, period, &config, &propagator,
        )
        .expect("should converge");
        let leg2 = solve_leg(
            &departure, &target_pos, &target_vel, period, &config, &propagator,
        )
        .expect("should converge");

        assert!(
            (leg1.total_dv_km_s - leg2.total_dv_km_s).abs() < DETERMINISTIC_DV_TOL,
            "Same inputs should give same Δv"
        );
    }

    /// TOF optimization: optimal TOF should have lower Δv than endpoints.
    #[test]
    fn tof_opt_lower_than_endpoints() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let tof_config = TofOptConfig::default();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let (opt_tof, opt_leg) = optimize_tof(
            &departure, &target_pos, &target_vel, &config, &tof_config, &propagator,
        )
        .expect("TOF optimization should succeed");

        // Check endpoints
        if let Ok(ref leg_min) = solve_leg(
            &departure, &target_pos, &target_vel,
            tof_config.min_periods * period, &config, &propagator,
        ) {
            assert!(
                opt_leg.total_dv_km_s <= leg_min.total_dv_km_s * 1.01,
                "Optimal Δv ({}) should be ≤ min-period Δv ({})",
                opt_leg.total_dv_km_s,
                leg_min.total_dv_km_s,
            );
        }
        if let Ok(ref leg_max) = solve_leg(
            &departure, &target_pos, &target_vel,
            tof_config.max_periods * period, &config, &propagator,
        ) {
            assert!(
                opt_leg.total_dv_km_s <= leg_max.total_dv_km_s * 1.01,
                "Optimal Δv ({}) should be ≤ max-period Δv ({})",
                opt_leg.total_dv_km_s,
                leg_max.total_dv_km_s,
            );
        }

        assert!(opt_tof > 0.0, "Optimal TOF should be positive");
    }

    /// Simple geometry: optimal TOF near 1 period for coplanar V-bar transfer.
    #[test]
    fn tof_opt_near_one_period() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig::default();
        let tof_config = TofOptConfig::default();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let (opt_tof, _) = optimize_tof(
            &departure, &target_pos, &target_vel, &config, &tof_config, &propagator,
        )
        .expect("TOF optimization should succeed");

        assert!(
            opt_tof > 0.3 * period && opt_tof < 3.5 * period,
            "Optimal TOF ({opt_tof:.0} s) should be in search range"
        );
    }

    /// Analytical Jacobian matches central-difference FD Jacobian.
    #[test]
    fn analytical_jacobian_matches_fd() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let tof_s = period * 0.75;
        let dv1 = Vector3::new(0.001, 0.002, 0.0005);
        let fd_step = JACOBIAN_FD_STEP_KM_S;

        // Analytical Jacobian
        let jac_analytical = compute_analytical_jacobian(&chief, tof_s).unwrap();

        // Central-difference FD Jacobian
        let mut jac_fd = SMatrix::<f64, 3, 3>::zeros();
        for j in 0..3 {
            let mut dv_plus = dv1;
            let mut dv_minus = dv1;
            dv_plus[j] += fd_step;
            dv_minus[j] -= fd_step;

            let (state_p, _) = forward_model(&departure, &dv_plus, tof_s, &propagator).unwrap();
            let (state_m, _) = forward_model(&departure, &dv_minus, tof_s, &propagator).unwrap();

            let inv_2h = 1.0 / (2.0 * fd_step);
            for i in 0..3 {
                jac_fd[(i, j)] = (state_p.ric.position_ric_km[i] - state_m.ric.position_ric_km[i]) * inv_2h;
            }
        }

        // Compare element-by-element
        for i in 0..3 {
            for j in 0..3 {
                let a = jac_analytical[(i, j)];
                let f = jac_fd[(i, j)];
                let rel_err = if f.abs() > JACOBIAN_NEAR_ZERO_THRESHOLD {
                    (a - f).abs() / f.abs()
                } else {
                    (a - f).abs()
                };
                assert!(
                    rel_err < JACOBIAN_FD_REL_TOL,
                    "Jacobian mismatch at ({i},{j}): analytical={a:.6e}, fd={f:.6e}, rel_err={rel_err:.2e}"
                );
            }
        }
    }

    /// TargetingConvergence: max_iterations=1 with a large target should not converge.
    #[test]
    fn targeting_convergence_failure() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig {
            max_iterations: 1,
            position_tol_km: FORCED_NONCONVERGENCE_TOL_KM,
            ..TargetingConfig::default()
        };
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        // Large target far from CW initial guess
        let target_pos = Vector3::new(50.0, 100.0, 20.0);
        let target_vel = Vector3::zeros();

        let result = solve_leg(
            &departure, &target_pos, &target_vel, period * 0.75, &config, &propagator,
        );
        assert!(
            matches!(result, Err(MissionError::TargetingConvergence { .. })),
            "Should fail to converge with max_iterations=1 and tight tolerance, got {result:?}"
        );
    }

    /// J2 propagator converges in exactly 1 Newton correction (linear system).
    ///
    /// `max_iterations: 2` because iteration 0 computes the correction and
    /// iteration 1 evaluates the corrected guess (verifying convergence).
    #[test]
    fn j2_converges_in_one_iteration() {
        let chief = iss_like_elements();
        let epoch = test_epoch();
        let period = std::f64::consts::TAU / chief.mean_motion().unwrap();
        let propagator = PropagationModel::J2Stm;
        let config = TargetingConfig {
            max_iterations: 2,
            ..TargetingConfig::default()
        };
        let departure = DepartureState { roe: zero_roe(), chief, epoch };

        let target_pos = Vector3::new(0.0, 5.0, 0.0);
        let target_vel = Vector3::zeros();

        let result = solve_leg(
            &departure, &target_pos, &target_vel, period * 0.75, &config, &propagator,
        );
        assert!(
            result.is_ok(),
            "J2 targeting should converge in 1 iteration, got: {result:?}"
        );
    }
}

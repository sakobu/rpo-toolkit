//! Free-drift (abort-case) trajectory handler.
//!
//! Computes free-drift trajectories on demand from the stored mission.

use rpo_core::pipeline::{
    compute_free_drift_analysis, propagated_to_point, resample_propagated, LegTrajectory,
};

use crate::error::ApiError;
use crate::protocol::FreeDriftSummary;
use crate::session::Session;

/// Response from the free-drift handler.
pub struct FreeDriftResponse {
    /// Per-leg trajectory points for visualization.
    pub legs: Vec<LegTrajectory>,
    /// Per-leg safety summaries.
    pub analyses: Vec<FreeDriftSummary>,
}

/// Compute free-drift trajectories for selected legs.
///
/// Delegates per-leg propagation to [`compute_free_drift_analysis`], then
/// filters to the requested legs and downsamples trajectories.
///
/// # Errors
///
/// Returns [`ApiError`] if no mission is planned, or if propagation fails.
pub fn handle_get_free_drift(
    session: &Session,
    legs: Option<&[usize]>,
    max_points: Option<u32>,
) -> Result<FreeDriftResponse, ApiError> {
    let mission = session.require_mission()?;
    let propagator = session.resolve_propagation_model();

    let all_analyses = compute_free_drift_analysis(mission, &propagator)
        .ok_or(ApiError::InvalidInput(
            crate::error::InvalidInputError::EmptyTrajectory,
        ))?;

    let leg_indices: Vec<usize> = match legs {
        Some(indices) => indices
            .iter()
            .copied()
            .filter(|&i| i < all_analyses.len())
            .collect(),
        None => (0..all_analyses.len()).collect(),
    };

    let mut result_legs = Vec::with_capacity(leg_indices.len());
    let mut result_analyses = Vec::with_capacity(leg_indices.len());

    for &idx in &leg_indices {
        let analysis = &all_analyses[idx];

        let points = match max_points {
            Some(max) => resample_propagated(&analysis.trajectory, max),
            None => analysis.trajectory.iter().map(propagated_to_point).collect(),
        };
        result_legs.push(LegTrajectory {
            leg_index: idx,
            points,
        });
        result_analyses.push(FreeDriftSummary::from_analysis(analysis, idx));
    }

    Ok(FreeDriftResponse {
        legs: result_legs,
        analyses: result_analyses,
    })
}

//! POCA (closest-approach) handler.
//!
//! Computes refined closest-approach points on demand from the stored mission.

use rpo_core::pipeline::compute_poca_analysis;

use crate::error::ApiError;
use crate::protocol::PocaPoint;
use crate::session::Session;

/// Compute refined closest-approach data for selected legs.
///
/// Delegates to [`compute_poca_analysis`] and projects results
/// to lean [`PocaPoint`] values for the wire.
///
/// # Errors
///
/// Returns [`ApiError`] if no mission is planned, or if POCA computation fails.
pub fn handle_get_poca(
    session: &Session,
    legs: Option<&[usize]>,
) -> Result<Vec<PocaPoint>, ApiError> {
    let mission = session.require_mission()?;
    let propagator = session.resolve_propagation_model();

    let all_poca = compute_poca_analysis(mission, &propagator).ok_or(
        ApiError::InvalidInput(crate::error::InvalidInputError::EmptyTrajectory),
    )?;

    let leg_indices: Vec<usize> = match legs {
        Some(indices) => indices
            .iter()
            .copied()
            .filter(|&i| i < all_poca.len())
            .collect(),
        None => (0..all_poca.len()).collect(),
    };

    let mut points = Vec::new();
    for &idx in &leg_indices {
        for ca in &all_poca[idx] {
            points.push(PocaPoint::from_closest_approach(ca));
        }
    }
    Ok(points)
}

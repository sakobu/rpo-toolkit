//! Lambert transfer handler — synchronous, ~100ms.

use crate::error::ServerError;
use rpo_core::mission::config::ProximityConfig;
use rpo_core::mission::formation::SafetyRequirements;
use rpo_core::mission::types::PerchGeometry;
use rpo_core::pipeline::types::{EnrichmentSuggestion, TransferComputationInput, TransferResult};
use rpo_core::pipeline::{apply_perch_enrichment, suggest_enrichment_from_parts};
use rpo_core::propagation::lambert::LambertConfig;
use rpo_core::types::state::StateVector;
use rpo_nyx::pipeline::compute_transfer;

/// Handle a `ComputeTransfer` message.
///
/// Constructs a `TransferComputationInput` from the self-contained message
/// fields and delegates to `rpo_nyx::pipeline::compute_transfer()`. When
/// `safety_requirements` is present, runs perch enrichment on the result
/// before returning; the returned `EnrichmentSuggestion` lets the client
/// distinguish `Enriched` from `Fallback` outcomes.
///
/// # Errors
///
/// - [`ServerError::Lambert`] if the Lambert solver fails (convergence,
///   degenerate geometry, non-positive TOF).
/// - [`ServerError::PipelineFailure`] if classification fails (mission
///   planning error).
pub fn handle_compute_transfer(
    chief_eci: StateVector,
    deputy_eci: StateVector,
    perch: PerchGeometry,
    proximity: ProximityConfig,
    lambert_tof_s: f64,
    lambert_config: LambertConfig,
    safety_requirements: Option<SafetyRequirements>,
) -> Result<(TransferResult, Option<EnrichmentSuggestion>), ServerError> {
    let input = TransferComputationInput {
        chief: chief_eci,
        deputy: deputy_eci,
        perch,
        proximity,
        lambert_tof_s,
        lambert_config,
    };
    let mut transfer = compute_transfer(&input)?;
    let enrichment = safety_requirements.as_ref().map(|reqs| {
        suggest_enrichment_from_parts(
            &input.perch,
            &transfer.plan.chief_at_arrival,
            transfer.plan.perch_roe,
            reqs,
        )
    });
    if let Some(ref s) = enrichment {
        apply_perch_enrichment(&mut transfer, s);
    }
    Ok((transfer, enrichment))
}

#[cfg(test)]
mod tests {
    use super::*;
    use hifitime::Epoch;
    use rpo_core::elements::keplerian_conversions::keplerian_to_state;
    use rpo_core::mission::formation::{EiAlignment, PerchEnrichmentResult};
    use rpo_core::pipeline::default_perch;
    use rpo_core::propagation::lambert::TransferDirection;
    use rpo_core::test_helpers::{far_field_chief, far_field_deputy, iss_like_elements};

    /// Single-rev LEO time of flight (1 hour). Used for default proximity
    /// and direction-toggle tests where a 1 hr arc is long enough to
    /// resolve a meaningful Δv but short enough to stay comfortably below
    /// one orbital period.
    const SINGLE_REV_TOF_S: f64 = 3600.0;

    /// Multi-rev LEO time of flight (4 hours ≈ 2.6 LEO periods). Long
    /// enough to accommodate a 1-rev Izzo solution for the canonical
    /// far-field geometry without running near the minimum-TOF bound.
    const MULTI_REV_TOF_S: f64 = 4.0 * SINGLE_REV_TOF_S;

    /// Canonical V-bar perch offset (km). Matches the frontend default
    /// and exercises the perch-to-ROE mapping without touching the
    /// linearization bound.
    const PERCH_VBAR_ALONG_TRACK_KM: f64 = 1.0;

    // Proximity pair: small δa and phase offsets keep δr/r below the default
    // 5e-3 threshold, exercising the enrichment path without a Lambert solve.
    fn proximity_pair() -> (StateVector, StateVector) {
        let epoch = Epoch::from_gregorian_str("2024-01-01T00:00:00 UTC").unwrap();
        let chief_ke = iss_like_elements();
        let mut deputy_ke = chief_ke;
        deputy_ke.a_km += 0.5;
        deputy_ke.mean_anomaly_rad += 1e-4;
        let chief_eci = keplerian_to_state(&chief_ke, epoch).unwrap();
        let deputy_eci = keplerian_to_state(&deputy_ke, epoch).unwrap();
        (chief_eci, deputy_eci)
    }

    fn call_far_field(direction: TransferDirection) -> TransferResult {
        call_far_field_config(LambertConfig { direction, revolutions: 0 }, SINGLE_REV_TOF_S)
    }

    fn call_far_field_config(lambert_config: LambertConfig, tof_s: f64) -> TransferResult {
        let (transfer, _) = handle_compute_transfer(
            far_field_chief(),
            far_field_deputy(),
            PerchGeometry::VBar { along_track_km: PERCH_VBAR_ALONG_TRACK_KM },
            ProximityConfig::default(),
            tof_s,
            lambert_config,
            None,
        )
        .expect("far-field Lambert should succeed");
        transfer
    }

    fn call(
        safety: Option<SafetyRequirements>,
    ) -> (TransferResult, Option<EnrichmentSuggestion>) {
        let (chief_eci, deputy_eci) = proximity_pair();
        handle_compute_transfer(
            chief_eci,
            deputy_eci,
            default_perch(),
            ProximityConfig::default(),
            SINGLE_REV_TOF_S,
            LambertConfig::default(),
            safety,
        )
        .expect("handler should succeed")
    }

    #[test]
    fn handle_compute_transfer_baseline_when_safety_absent() {
        let (_transfer, enrichment) = call(None);
        assert!(enrichment.is_none(), "no safety_requirements → no enrichment");
    }

    #[test]
    fn handle_compute_transfer_enriches_when_safety_set() {
        let (transfer, enrichment) = call(Some(SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        }));

        let suggestion = enrichment.expect("enrichment should be present when safety is set");
        assert!(
            matches!(suggestion.perch, PerchEnrichmentResult::Enriched(_)),
            "expected Enriched variant, got {:?}",
            suggestion.perch,
        );
        // Enrichment populates e/i components to achieve passive safety.
        let roe = &transfer.plan.perch_roe;
        assert!(
            roe.dey.abs() > 1e-8 || roe.diy.abs() > 1e-8,
            "enrichment should populate at least one e/i component (dey={}, diy={})",
            roe.dey,
            roe.diy,
        );
    }

    #[test]
    fn handle_compute_transfer_preserves_classification() {
        // Regression: enrichment must not alter the phase field or perch states;
        // it only mutates plan.perch_roe.
        let (baseline, _) = call(None);
        let (enriched, _) = call(Some(SafetyRequirements {
            min_separation_km: 0.15,
            alignment: EiAlignment::Parallel,
        }));
        assert_eq!(
            std::mem::discriminant(&baseline.plan.phase),
            std::mem::discriminant(&enriched.plan.phase),
            "classification must not change under enrichment",
        );
        assert_eq!(
            baseline.arrival_epoch, enriched.arrival_epoch,
            "arrival epoch must not change under enrichment",
        );
    }

    /// End-to-end: when the requested `min_separation_km` exceeds the V-bar
    /// linearization bound for the chief's orbit (ISS-like, a ≈ 6878 km →
    /// cap ≈ 48.6 km), the handler must return a `Fallback` enrichment and
    /// leave `transfer.plan.perch_roe` at the unenriched geometric baseline.
    /// Covers the Phase 2c gate in `enrich_simple_perch` via the full API
    /// stack (proximity handler → core pipeline → formation engine).
    #[test]
    fn handle_compute_transfer_falls_back_when_safety_unachievable() {
        use rpo_core::mission::formation::PerchFallbackReason;
        use rpo_core::types::QuasiNonsingularROE;

        // ROE equality tolerance (dimensionless). Both values come from the
        // same `perch_to_roe(V-bar, chief)` call; only f64 arithmetic noise
        // from the `offset / a_km` normalization contributes error.
        const ROE_EQUALITY_TOL: f64 = 1e-15;
        fn assert_roe_eq(actual: &QuasiNonsingularROE, expected: &QuasiNonsingularROE, ctx: &str) {
            for (lhs, rhs, name) in [
                (actual.da, expected.da, "da"),
                (actual.dlambda, expected.dlambda, "dlambda"),
                (actual.dex, expected.dex, "dex"),
                (actual.dey, expected.dey, "dey"),
                (actual.dix, expected.dix, "dix"),
                (actual.diy, expected.diy, "diy"),
            ] {
                assert!(
                    (lhs - rhs).abs() < ROE_EQUALITY_TOL,
                    "{ctx}: {name} = {lhs}, expected {rhs}"
                );
            }
        }

        let (baseline, _) = call(None);
        let (transfer, enrichment) = call(Some(SafetyRequirements {
            min_separation_km: 100.0,
            alignment: EiAlignment::Parallel,
        }));

        let suggestion = enrichment.expect("enrichment is produced when safety is set");
        match suggestion.perch {
            PerchEnrichmentResult::Fallback { reason, unenriched_roe } => {
                assert!(
                    matches!(
                        reason,
                        PerchFallbackReason::SeparationUnachievable { .. }
                    ),
                    "expected SeparationUnachievable reason, got {reason:?}"
                );
                // The Fallback's `unenriched_roe` must agree with the
                // geometric baseline (no-safety call) — both come from the
                // same `perch_to_roe(V-bar, chief)` call.
                assert_roe_eq(
                    &unenriched_roe,
                    &baseline.plan.perch_roe,
                    "Fallback unenriched_roe vs. geometric baseline",
                );
            }
            other => panic!("expected Fallback, got {other:?}"),
        }
        // `apply_perch_enrichment` must no-op on Fallback: the delivered
        // perch_roe equals the no-safety baseline.
        assert_roe_eq(
            &transfer.plan.perch_roe,
            &baseline.plan.perch_roe,
            "transfer.plan.perch_roe vs. geometric baseline",
        );
    }

    /// For the canonical far-field fixture (`far_field_chief` /
    /// `far_field_deputy` in `rpo_core::test_helpers`) the transfer sweep
    /// angle is well below π, so a correct `Auto` dispatch should pick
    /// short-way and produce the same ΔV as an explicit `ShortWay`.
    /// nyx-space 2.3.1 also happens to return short-way here — but via the
    /// buggy `r_init[1].atan2(r_final[1])` expression in
    /// `TransferKind::Auto`, not because the computation is correct. A
    /// tight tolerance (1 mm/s) is used instead of `to_bits()` equality so
    /// that once nyx fixes the typo and `Auto` takes a different code
    /// path, this test still passes as long as the direction pick remains
    /// correct. If the assertion regresses, re-run with both directions
    /// and check whether `Auto` now flips to `LongWay` for this geometry
    /// before widening the tolerance.
    #[test]
    fn direction_auto_matches_short_way_for_far_field() {
        const AUTO_SHORT_TOLERANCE_KM_S: f64 = 1.0e-6;

        let auto = call_far_field(TransferDirection::Auto);
        let short = call_far_field(TransferDirection::ShortWay);
        let delta_km_s = (auto.lambert_dv_km_s - short.lambert_dv_km_s).abs();
        assert!(
            delta_km_s < AUTO_SHORT_TOLERANCE_KM_S,
            "Auto ΔV ({:.9} km/s) should match ShortWay ({:.9} km/s) within {:.0e} km/s for this geometry; got Δ = {:.9} km/s",
            auto.lambert_dv_km_s,
            short.lambert_dv_km_s,
            AUTO_SHORT_TOLERANCE_KM_S,
            delta_km_s,
        );
    }

    /// For the canonical far-field fixture, flipping to `LongWay` must
    /// produce a measurably different ΔV. A 10 m/s floor is conservative
    /// for a 1-hour single-rev transfer; if this regresses, the Direction
    /// toggle has silently become a no-op across the wire or in nyx. This
    /// is the regression guard for the Izzo `|| retrograde`
    /// geometry-coupling bug that made all three directions collapse.
    #[test]
    fn direction_long_way_differs_from_short_way() {
        const MIN_DV_DIFFERENCE_KM_S: f64 = 0.010;

        let short = call_far_field(TransferDirection::ShortWay);
        let long = call_far_field(TransferDirection::LongWay);
        let delta_km_s = (long.lambert_dv_km_s - short.lambert_dv_km_s).abs();
        assert!(
            delta_km_s > MIN_DV_DIFFERENCE_KM_S,
            "LongWay ΔV ({:.6} km/s) should differ from ShortWay ({:.6} km/s) by > 10 m/s; got Δ = {:.6} km/s",
            long.lambert_dv_km_s,
            short.lambert_dv_km_s,
            delta_km_s,
        );
    }

    /// Multi-rev (`revolutions > 0`) routes through Izzo with
    /// `TransferKind::NRevs`, which has no `long_way` variant. Direction is
    /// dropped silently. This test pins that behavior so the UI team knows to
    /// grey out the Direction control whenever revolutions > 0.
    #[test]
    fn multi_rev_ignores_direction() {
        let short = call_far_field_config(
            LambertConfig { direction: TransferDirection::ShortWay, revolutions: 1 },
            MULTI_REV_TOF_S,
        );
        let long = call_far_field_config(
            LambertConfig { direction: TransferDirection::LongWay, revolutions: 1 },
            MULTI_REV_TOF_S,
        );
        assert_eq!(
            short.lambert_dv_km_s.to_bits(),
            long.lambert_dv_km_s.to_bits(),
            "multi-rev must ignore direction (short={}, long={})",
            short.lambert_dv_km_s,
            long.lambert_dv_km_s,
        );
    }
}

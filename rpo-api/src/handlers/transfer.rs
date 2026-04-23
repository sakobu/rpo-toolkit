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
    use rpo_core::test_helpers::iss_like_elements;

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

    fn call(
        safety: Option<SafetyRequirements>,
    ) -> (TransferResult, Option<EnrichmentSuggestion>) {
        let (chief_eci, deputy_eci) = proximity_pair();
        handle_compute_transfer(
            chief_eci,
            deputy_eci,
            default_perch(),
            ProximityConfig::default(),
            3600.0,
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
}

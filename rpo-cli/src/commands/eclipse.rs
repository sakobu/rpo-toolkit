//! Plumbing: eclipse computation for a planned mission (JSON only).

use std::path::Path;

use rpo_core::pipeline::PipelineInput;
use rpo_core::types::EclipseState;
use rpo_nyx::pipeline::execute_mission;

use crate::error::CliError;
use crate::input::load_json_with_hint;
use crate::output::common::print_json;

/// Compute eclipse data for a planned mission and print summary JSON.
///
/// Emits only the eclipse summary (intervals, durations, fractions) and
/// per-leg shadow counts — not the full per-sample celestial snapshots.
pub fn run(input_path: &Path) -> Result<(), CliError> {
    let input: PipelineInput = load_json_with_hint(
        input_path,
        "eclipse expects full PipelineInput (same as mission); see examples/mission.json",
    )?;
    let output = execute_mission(&input)?;

    let per_leg: Vec<serde_json::Value> = output
        .mission
        .eclipse
        .as_ref()
        .map(|e| {
            e.legs
                .iter()
                .enumerate()
                .map(|(i, leg)| {
                    let shadow_count = leg
                        .deputy_eclipse
                        .iter()
                        .filter(|s| !matches!(s, EclipseState::Sunlit))
                        .count();
                    serde_json::json!({
                        "leg": i + 1,
                        "samples": leg.deputy_eclipse.len(),
                        "shadow_samples": shadow_count,
                    })
                })
                .collect()
        })
        .unwrap_or_default();

    let eclipse_data = serde_json::json!({
        "transfer_eclipse": output.transfer_eclipse.as_ref().map(|te| {
            serde_json::json!({
                "summary": te.summary,
                "sample_count": te.deputy_celestial.len(),
            })
        }),
        "mission_eclipse": output.mission.eclipse.as_ref().map(|me| {
            serde_json::json!({
                "summary": me.summary,
                "legs": per_leg,
            })
        }),
    });

    print_json(&eclipse_data)
}

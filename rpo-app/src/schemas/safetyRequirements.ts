import * as S from '@railway-ts/pipelines/schema';

export const ALIGNMENT_VALUES = ['parallel', 'anti_parallel', 'auto'] as const;
export type AlignmentValue = (typeof ALIGNMENT_VALUES)[number];

export const ALIGNMENT_LABELS: Record<AlignmentValue, string> = {
  parallel: 'Parallel',
  anti_parallel: 'Anti-parallel',
  auto: 'Auto',
};

/// Lower bound on requested R/C separation (1 m). Below this, slider
/// quantization (`MIN_SEPARATION_STEP_KM`) makes the value unreachable and
/// the engine's e/i null-space projection loses meaning — `d_min` would be
/// at the same scale as numerical noise from the quadratic solver in
/// `compute_safety_projection`.
export const MIN_SEPARATION_LOWER_KM = 0.001;

/// Slider quantization for `min_separation_km` (100 m). UX precision floor:
/// finer steps don't change targeting outcomes meaningfully and make the
/// slider feel sluggish.
export const MIN_SEPARATION_STEP_KM = 0.1;

/// Initial value when the user first opens the safety-requirements form
/// (100 m). Conservative for proximity ops; users adjust up to the
/// per-orbit `useAchievableCap` cap or down to `MIN_SEPARATION_LOWER_KM`.
export const DEFAULT_SAFETY_REQUIREMENTS = {
  min_separation_km: 0.1,
  alignment: 'parallel',
} as const satisfies { min_separation_km: number; alignment: AlignmentValue };

// Upper bound on `min_separation_km` is enforced by the slider's `max` prop
// (per-orbit, derived from `useAchievableCap`) rather than the schema, which
// has no access to chief state at construction time. The schema validates the
// lower bound only.
export const safetyRequirementsSchema = S.object({
  min_separation_km: S.required(
    S.chain(
      S.parseNumber(),
      S.min(
        MIN_SEPARATION_LOWER_KM,
        `Min separation must be at least ${MIN_SEPARATION_LOWER_KM} km`,
      ),
    ),
  ),
  alignment: S.required(
    S.stringEnum<AlignmentValue>([...ALIGNMENT_VALUES], 'Pick an alignment strategy'),
  ),
});

export type SafetyRequirementsFormValues = S.InferSchemaType<typeof safetyRequirementsSchema>;

import * as S from '@railway-ts/pipelines/schema';

export const ALIGNMENT_VALUES = ['parallel', 'anti_parallel', 'auto'] as const;
export type AlignmentValue = (typeof ALIGNMENT_VALUES)[number];

export const ALIGNMENT_LABELS: Record<AlignmentValue, string> = {
  parallel: 'Parallel',
  anti_parallel: 'Anti-parallel',
  auto: 'Auto',
};

export const MIN_SEPARATION_LOWER_KM = 0.001;
export const MIN_SEPARATION_UPPER_KM = 100;

export const DEFAULT_SAFETY_REQUIREMENTS = {
  min_separation_km: 0.1,
  alignment: 'parallel',
} as const satisfies { min_separation_km: number; alignment: AlignmentValue };

export const safetyRequirementsSchema = S.object({
  min_separation_km: S.required(
    S.chain(
      S.parseNumber(),
      S.between(
        MIN_SEPARATION_LOWER_KM,
        MIN_SEPARATION_UPPER_KM,
        `Min separation must be ${MIN_SEPARATION_LOWER_KM}–${MIN_SEPARATION_UPPER_KM} km`,
      ),
    ),
  ),
  alignment: S.required(
    S.stringEnum<AlignmentValue>([...ALIGNMENT_VALUES], 'Pick an alignment strategy'),
  ),
});

export type SafetyRequirementsFormValues = S.InferSchemaType<typeof safetyRequirementsSchema>;

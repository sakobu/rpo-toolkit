import * as S from '@railway-ts/pipelines/schema';

export const ROE_THRESHOLD_MIN = 0.0001;
export const ROE_THRESHOLD_MAX = 0.01;
/// D'Amico §2.3.4: above this, 2nd-order ROE terms exceed J2 modeling residuals.
/// Inside the hard range but above this is the soft-advisory zone.
export const ROE_THRESHOLD_SOFT_MAX = 0.005;
export const ROE_THRESHOLD_DEFAULT = ROE_THRESHOLD_SOFT_MAX;

export const proximityConfigSchema = S.object({
  roe_threshold: S.required(
    S.chain(
      S.parseNumber('δr/r threshold must be a number'),
      S.finite('δr/r threshold must be finite'),
      S.between(
        ROE_THRESHOLD_MIN,
        ROE_THRESHOLD_MAX,
        `δr/r threshold must be between ${ROE_THRESHOLD_MIN} and ${ROE_THRESHOLD_MAX} — outside this range breaks ROE linearization (D'Amico §2.3.4)`,
      ),
    ),
  ),
});

export type ProximityConfigInput = S.InferSchemaType<typeof proximityConfigSchema>;

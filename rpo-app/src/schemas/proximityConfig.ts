import * as S from '@railway-ts/pipelines/schema';

import { getEngineConstants } from '@/wasm/constants';

export const ROE_THRESHOLD_MIN = 0.0001;

export function proximityConfigSchema() {
  const max = getEngineConstants().linearization_perturbation_bound;
  return S.object({
    roe_threshold: S.required(
      S.chain(
        S.parseNumber('δr/r threshold must be a number'),
        S.finite('δr/r threshold must be finite'),
        S.between(
          ROE_THRESHOLD_MIN,
          max,
          `δr/r threshold must be between ${ROE_THRESHOLD_MIN} and ${max} — outside this range breaks ROE linearization (D'Amico §2.3.4)`,
        ),
      ),
    ),
  });
}

export type ProximityConfigInput = S.InferSchemaType<ReturnType<typeof proximityConfigSchema>>;

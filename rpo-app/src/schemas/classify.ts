import * as S from '@railway-ts/pipelines/schema';

import { proximityConfigSchema } from './proximityConfig';
import { stateVectorSchema } from './stateVector';

export function classifyInputSchema() {
  return S.object({
    chief: S.required(stateVectorSchema),
    deputy: S.required(stateVectorSchema),
    proximity: S.required(proximityConfigSchema()),
  });
}

export type ClassifyInput = S.InferSchemaType<ReturnType<typeof classifyInputSchema>>;

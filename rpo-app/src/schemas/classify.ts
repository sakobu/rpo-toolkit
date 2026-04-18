import * as S from '@railway-ts/pipelines/schema';

import { proximityConfigSchema } from './proximityConfig';
import { stateVectorSchema } from './stateVector';

export const classifyInputSchema = S.object({
  chief: S.required(stateVectorSchema),
  deputy: S.required(stateVectorSchema),
  proximity: S.required(proximityConfigSchema),
});

export type ClassifyInput = S.InferSchemaType<typeof classifyInputSchema>;

import { pipe, pipeAsync } from '@railway-ts/pipelines/composition';
import {
  flatMapWith,
  fromPromise,
  fromTry,
  mapErrWith,
  type Result,
} from '@railway-ts/pipelines/result';
import { validate, type ValidationError } from '@railway-ts/pipelines/schema';

import { type StateVectorInput, stateVectorSchema } from '@/schemas/stateVector';

export function loadStateVectorFromFile(file: File): Promise<Result<StateVectorInput, string>> {
  return pipeAsync(
    fromPromise(file.text()),
    mapErrWith((msg) => `could not read file: ${msg}`),
    flatMapWith(parseJson),
    flatMapWith(validateStateVector),
  );
}

function parseJson(text: string): Result<unknown, string> {
  return pipe(
    fromTry(() => JSON.parse(text)),
    mapErrWith((msg) => `invalid JSON: ${msg}`),
  );
}

function validateStateVector(value: unknown): Result<StateVectorInput, string> {
  return pipe(validate(value, stateVectorSchema), mapErrWith(formatValidationErrors));
}

function formatValidationErrors(errors: ValidationError[]): string {
  const first = errors[0];
  if (!first) return 'invalid state vector';
  const path = first.path.length > 0 ? `${first.path.join('.')}: ` : '';
  return `${path}${first.message}`;
}

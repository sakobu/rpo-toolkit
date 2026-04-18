import { pipe } from '@railway-ts/pipelines/composition';
import { fromTryWithError, mapErrWith, type Result } from '@railway-ts/pipelines/result';

import type { WasmError } from 'rpo-wasm';

export function callWasm<T>(fn: () => T): Result<T, WasmError> {
  return pipe(fromTryWithError(fn), mapErrWith(toWasmError));
}

export function toWasmError(e: unknown): WasmError {
  if (isWasmError(e)) return e;
  return {
    code: 'mission',
    message: e instanceof Error ? e.message : String(e),
  };
}

export function isWasmError(e: unknown): e is WasmError {
  return (
    typeof e === 'object' &&
    e !== null &&
    'code' in e &&
    'message' in e &&
    typeof (e as { message: unknown }).message === 'string'
  );
}

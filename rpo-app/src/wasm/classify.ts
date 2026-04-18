import { classify_separation } from 'rpo-wasm';
import type { MissionPhase, ProximityConfig, StateVector, WasmError } from 'rpo-wasm';
import { fromTryWithError, mapErrWith, type Result } from '@railway-ts/pipelines/result';
import { pipe } from '@railway-ts/pipelines/composition';

export function classify(
  chief: StateVector,
  deputy: StateVector,
  config: ProximityConfig,
): Result<MissionPhase, WasmError> {
  return pipe(
    fromTryWithError(() => classify_separation(chief, deputy, config)),
    mapErrWith((e: Error) => toWasmError(e)),
  );
}

function toWasmError(e: unknown): WasmError {
  if (isWasmError(e)) return e;
  return {
    code: 'mission',
    message: e instanceof Error ? e.message : String(e),
  };
}

function isWasmError(e: unknown): e is WasmError {
  return (
    typeof e === 'object' &&
    e !== null &&
    'code' in e &&
    'message' in e &&
    typeof (e as { message: unknown }).message === 'string'
  );
}

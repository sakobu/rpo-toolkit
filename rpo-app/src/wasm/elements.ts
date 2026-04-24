import type { Result } from '@railway-ts/pipelines/result';

import type { KeplerianElements, StateVector, WasmError } from 'rpo-wasm';
import { state_to_keplerian } from 'rpo-wasm';

import { callWasm } from './error';

export function stateToKeplerian(state: StateVector): Result<KeplerianElements, WasmError> {
  return callWasm(() => state_to_keplerian(state));
}

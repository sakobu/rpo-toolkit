import type { Result } from '@railway-ts/pipelines/result';

import type { MissionPhase, ProximityConfig, StateVector, WasmError } from 'rpo-wasm';
import { classify_separation } from 'rpo-wasm';

import { callWasm } from './error';

export function classify(
  chief: StateVector,
  deputy: StateVector,
  config: ProximityConfig,
): Result<MissionPhase, WasmError> {
  return callWasm(() => classify_separation(chief, deputy, config));
}

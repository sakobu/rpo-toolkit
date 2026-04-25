import type { Result } from '@railway-ts/pipelines/result';

import type { KeplerianElements, StateVector, Vec3, WasmError } from 'rpo-wasm';
import { sample_orbit_eci, state_to_keplerian } from 'rpo-wasm';

import { callWasm } from './error';

export function stateToKeplerian(state: StateVector): Result<KeplerianElements, WasmError> {
  return callWasm(() => state_to_keplerian(state));
}

export function sampleOrbitEci(
  elements: KeplerianElements,
  nPoints: number,
): Result<Vec3[], WasmError> {
  return callWasm(() => sample_orbit_eci(elements, nPoints));
}

import type { Result } from '@railway-ts/pipelines/result';

import type { Vec3, WasmError } from 'rpo-wasm';
import { moon_position_eci_km, sun_position_eci_km } from 'rpo-wasm';

import { callWasm } from './error';

export function sunPositionEciKm(epoch: string): Result<Vec3, WasmError> {
  return callWasm(() => sun_position_eci_km(epoch));
}

export function moonPositionEciKm(epoch: string): Result<Vec3, WasmError> {
  return callWasm(() => moon_position_eci_km(epoch));
}

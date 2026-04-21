import type { Result } from '@railway-ts/pipelines/result';

import type { Matrix3Rows, RICState, StateVector, WasmError } from 'rpo-wasm';
import { eci_to_ric_dcm, eci_to_ric_relative_state } from 'rpo-wasm';

import { callWasm } from './error';

export function eciToRicDcm(chief: StateVector): Result<Matrix3Rows, WasmError> {
  return callWasm(() => eci_to_ric_dcm(chief));
}

export function eciToRicRelativeState(
  chief: StateVector,
  deputy: StateVector,
): Result<RICState, WasmError> {
  return callWasm(() => eci_to_ric_relative_state(chief, deputy));
}

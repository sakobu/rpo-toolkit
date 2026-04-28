import { err, match, ok, type Result } from '@railway-ts/pipelines/result';
import * as S from '@railway-ts/pipelines/schema';

import type { WasmError, WasmErrorCode } from 'rpo-wasm';

// Mirrors the auto-generated `WasmErrorCode` union from rpo-wasm. The
// `WasmErrorCode[]` annotation guarantees every entry is a valid code; codes
// added to the Rust enum must be appended here.
const WASM_ERROR_CODES: WasmErrorCode[] = [
  'mission',
  'lambert',
  'propagation',
  'covariance',
  'avoidance',
  'missing_field',
  'empty_trajectory',
  'eclipse',
  'formation',
  'deserialization',
  'frame',
];

const wasmErrorSchema = S.object({
  code: S.required(S.stringEnum(WASM_ERROR_CODES)),
  message: S.required(S.string()),
  details: S.optional(S.string()),
});

export function callWasm<T>(fn: () => T): Result<T, WasmError> {
  try {
    return ok(fn());
  } catch (e) {
    return err(
      match(S.validate(e, wasmErrorSchema), {
        ok: (v) => v,
        err: () => ({
          code: 'mission' as const,
          message: e instanceof Error ? e.message : String(e),
        }),
      }),
    );
  }
}

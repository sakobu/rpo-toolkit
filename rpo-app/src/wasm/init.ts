import { fromPromiseWithError, type Result } from '@railway-ts/pipelines/result';

import init from 'rpo-wasm';

export type WasmInitError = { kind: 'wasm-init-failed'; message: string };

export const wasmReady: Promise<Result<void, WasmInitError>> = fromPromiseWithError(
  init().then(() => undefined),
  (e) => ({ kind: 'wasm-init-failed', message: String(e) }),
);

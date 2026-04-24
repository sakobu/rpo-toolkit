import type { EngineConstants } from 'rpo-wasm';
import { engine_constants } from 'rpo-wasm';

// Cached snapshot of `rpo-core::engine_constants` — these never change at
// runtime, so a single WASM call is enough. Caller must ensure WASM is
// initialized first (the app's `await wasmReady` in `main.tsx` covers this
// for any code that runs inside the React render tree).
let cached: EngineConstants | null = null;

export function getEngineConstants(): EngineConstants {
  if (cached === null) cached = engine_constants();
  return cached;
}

import { useEffect, useState } from 'react';

import { match } from '@railway-ts/pipelines/result';

import { type WasmInitError, wasmReady } from '@/wasm/init';

export type WasmState =
  | { kind: 'loading' }
  | { kind: 'ready' }
  | { kind: 'error'; error: WasmInitError };

export function useWasm(): WasmState {
  const [state, setState] = useState<WasmState>({ kind: 'loading' });

  useEffect(() => {
    let cancelled = false;
    void wasmReady.then((result) => {
      if (cancelled) return;
      setState(
        match(result, {
          ok: () => ({ kind: 'ready' as const }),
          err: (error) => ({ kind: 'error' as const, error }),
        }),
      );
    });
    return () => {
      cancelled = true;
    };
  }, []);

  return state;
}

import { type ReactNode } from 'react';

import { useWasm } from '@/hooks/useWasm';

export function WasmGate({ children }: { children: ReactNode }) {
  const wasm = useWasm();

  if (wasm.kind === 'ready') return <>{children}</>;

  return (
    <main className="flex min-h-screen items-center justify-center bg-bg p-8">
      {wasm.kind === 'loading' ? (
        <div className="flex items-center gap-2 text-sm text-text-muted">
          <span className="inline-block size-1.5 animate-pulse rounded-full bg-signal-info" />
          loading wasm…
        </div>
      ) : (
        <pre className="max-w-2xl overflow-auto rounded-md border border-signal-abort/30 bg-signal-abort-dim p-4 font-mono text-xs text-signal-abort">
          wasm init failed: {wasm.error.message}
        </pre>
      )}
    </main>
  );
}

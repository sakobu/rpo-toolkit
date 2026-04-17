import { useEffect, useState } from 'react';
import init, {
  classify_separation,
  type StateVector,
  type ProximityConfig,
  type MissionPhase,
} from 'rpo-wasm';

const chief: StateVector = {
  epoch: '2024-01-01T00:00:00 UTC',
  position_eci_km: [5876.261, 3392.661, 0.0],
  velocity_eci_km_s: [-2.380512, 4.123167, 6.006917],
};

const deputy: StateVector = {
  epoch: '2024-01-01T00:00:00 UTC',
  position_eci_km: [5199.839421, 4281.648523, 1398.070066],
  velocity_eci_km_s: [-3.993103, 2.970313, 5.76454],
};

const config: ProximityConfig = { roe_threshold: 0.005 };

type Status =
  | { kind: 'loading' }
  | { kind: 'ok'; phase: MissionPhase }
  | { kind: 'err'; message: string };

export default function App() {
  const [status, setStatus] = useState<Status>({ kind: 'loading' });

  useEffect(() => {
    init()
      .then(() => {
        try {
          const phase = classify_separation(chief, deputy, config);
          setStatus({ kind: 'ok', phase });
        } catch (e) {
          setStatus({ kind: 'err', message: String(e) });
        }
      })
      .catch((e) => setStatus({ kind: 'err', message: String(e) }));
  }, []);

  return (
    <main className="mx-auto min-h-screen max-w-4xl p-8">
      <header className="mb-6 flex items-baseline justify-between border-b border-border pb-3">
        <h1 className="text-xl font-semibold tracking-tight text-text">rpo-app</h1>
        <span className="font-mono text-xs tracking-wider text-text-dim uppercase">
          wasm smoke test
        </span>
      </header>

      <p className="mb-6 text-text-muted">
        <code className="font-mono text-accent">classify_separation</code> returns the mission phase
        for a chief/deputy pair.
      </p>

      {status.kind === 'loading' && (
        <div className="flex items-center gap-2 text-sm text-text-muted">
          <span className="inline-block size-1.5 animate-pulse rounded-full bg-signal-info" />
          loading wasm…
        </div>
      )}

      {status.kind === 'err' && (
        <pre className="overflow-auto rounded-md border border-signal-abort/30 bg-signal-abort-dim p-4 font-mono text-xs text-signal-abort">
          error: {status.message}
        </pre>
      )}

      {status.kind === 'ok' && (
        <pre className="overflow-auto rounded-md border border-border bg-surface-1 p-4 font-mono text-xs leading-tight text-text">
          {JSON.stringify(status.phase, null, 2)}
        </pre>
      )}
    </main>
  );
}

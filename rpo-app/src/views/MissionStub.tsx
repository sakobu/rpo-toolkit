import { Link } from 'react-router';

import { Callout } from '@/components/primitives/Callout';
import { Panel } from '@/components/primitives/Panel';
import { useMission } from '@/stores/mission';

export function MissionStub() {
  const classification = useMission((s) => s.classification);

  if (classification.status !== 'ok') {
    return (
      <Callout tone="abort">
        setup incomplete —{' '}
        <Link to="/" className="underline hover:text-signal-abort">
          return to setup
        </Link>
      </Callout>
    );
  }

  return (
    <Panel title="Main Loop" subtitle="step 4 · proximity">
      <div className="flex flex-col gap-3 font-mono text-xs">
        <p className="text-text-muted">Waypoint planning with RIC view — coming soon.</p>
        <Link
          to="/"
          className="self-start tracking-wider text-text-dim uppercase hover:text-text-muted"
        >
          ← back to setup
        </Link>
      </div>
    </Panel>
  );
}

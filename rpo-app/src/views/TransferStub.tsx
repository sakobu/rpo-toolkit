import { Link } from 'react-router';

import { Callout } from '../components/primitives/Callout';
import { Panel } from '../components/primitives/Panel';
import { useMission } from '../stores/mission';

export function TransferStub() {
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
    <Panel title="Transfer Design" subtitle="step 3f · far-field">
      <div className="flex flex-col gap-3 font-mono text-xs">
        <p className="text-text-muted">
          Lambert transfer design with ECI globe view — coming soon.
        </p>
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

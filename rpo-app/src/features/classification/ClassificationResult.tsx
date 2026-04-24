import { usePlanner } from '@/stores/planner';
import { Callout } from '@/ui/Callout';
import { RegimePill } from '@/ui/RegimePill';

export function ClassificationResult() {
  const classification = usePlanner((s) => s.classification);

  if (classification.status === 'idle') {
    return (
      <div className="rounded-sm border border-border bg-surface-1 px-4 py-2 font-mono text-xs text-text-dim">
        upload both states to classify
      </div>
    );
  }

  if (classification.status === 'err') {
    return (
      <Callout tone="abort">
        classify failed ({classification.error.code}): {classification.error.message}
      </Callout>
    );
  }

  const { phase } = classification;
  const isProximity = 'proximity' in phase;
  const details = isProximity ? phase.proximity : phase.far_field;
  const regime = isProximity ? 'PROXIMITY' : 'FAR-FIELD';

  return (
    <div className="flex flex-wrap items-center gap-4 rounded-sm border border-border bg-surface-1 px-4 py-2">
      <RegimePill regime={regime} />
      <span className="font-mono text-xs text-text-muted">
        separation{' '}
        <span className="text-text tabular-nums">{details.separation_km.toFixed(3)}</span> km
      </span>
      <span className="font-mono text-xs text-text-muted">
        δr/r{' '}
        <span className="text-text tabular-nums">{details.delta_r_over_r.toExponential(3)}</span>
      </span>
    </div>
  );
}

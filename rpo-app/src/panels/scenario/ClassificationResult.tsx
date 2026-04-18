import { Callout } from '@/components/primitives/Callout';
import { useMission } from '@/stores/mission';

export function ClassificationResult() {
  const classification = useMission((s) => s.classification);

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
  const label = isProximity ? 'PROXIMITY' : 'FAR-FIELD';
  const badgeClass = isProximity
    ? 'border-signal-go/40 bg-signal-go-dim text-signal-go'
    : 'border-signal-hold/40 bg-signal-hold-dim text-signal-hold';

  return (
    <div className="flex flex-wrap items-center gap-4 rounded-sm border border-border bg-surface-1 px-4 py-2">
      <span
        className={`rounded-xs border px-2 py-0.5 font-mono text-xs font-semibold tracking-wider ${badgeClass}`}
      >
        {label}
      </span>
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

import type { QuasiNonsingularROE } from 'rpo-wasm';

import { QNS_COMPONENTS, QNS_LABELS } from '@/schemas/lambertRequest';
import { formatRoeComponent } from '@/utils/format';

export function PerchRoeDetails({
  roe,
  enriched,
}: {
  roe: QuasiNonsingularROE;
  enriched: boolean;
}) {
  const label = enriched ? 'enriched perch roe' : 'perch roe';
  return (
    <details className="group rounded-xs border border-border bg-surface-2 px-2 py-1">
      <summary className="flex cursor-pointer items-center justify-between font-mono text-[10px] tracking-wider text-text-dim uppercase hover:text-text">
        {label}
        <span className="text-[9px] transition-transform group-open:rotate-90">›</span>
      </summary>
      <div className="mt-1 grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5">
        {QNS_COMPONENTS.map((key) => (
          <div key={key} className="contents">
            <span className="font-mono text-[10px] text-text-dim">{QNS_LABELS[key]}</span>
            <span className="text-right font-mono text-[10px] text-text tabular-nums">
              {formatRoeComponent(roe[key])}
            </span>
          </div>
        ))}
      </div>
    </details>
  );
}

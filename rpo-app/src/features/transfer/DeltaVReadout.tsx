import type { EnrichmentSuggestion } from 'rpo-wasm';

import { selectTransferSubSurface, type TransferSlot, usePlanner } from '@/stores/planner';
import { Callout } from '@/ui/Callout';
import { Chip } from '@/ui/Chip';
import { KV } from '@/ui/KV';
import { vectorMag } from '@/utils/math';

import { PerchRoeDetails } from './PerchRoeDetails';

function EnrichmentBadge({ enrichment }: { enrichment: EnrichmentSuggestion }) {
  switch (enrichment.status) {
    case 'enriched':
      return <Chip tone="go">enriched</Chip>;
    case 'baseline':
      return <Chip tone="muted">baseline</Chip>;
  }
}

export function DeltaVReadout({ slot }: { slot: TransferSlot | null }) {
  const subSurface = usePlanner(selectTransferSubSurface);

  if (slot === null) {
    return (
      <div className="flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <KV k="Δv lambert" v="—" />
      </div>
    );
  }

  const { transfer, enrichment } = slot;
  const lambert = transfer.plan.transfer ?? null;
  const depDv = lambert ? vectorMag(lambert.departure_dv_eci_km_s) * 1000 : null;
  const arrDv = lambert ? vectorMag(lambert.arrival_dv_eci_km_s) * 1000 : null;
  const enriched = enrichment.status === 'enriched';
  const perchRoe = enriched ? enrichment.safe_perch.roe : transfer.plan.perch_roe;

  return (
    <div className="flex flex-col gap-1.5">
      <div className="flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <KV k="Δv total" v={`${(transfer.lambert_dv_km_s * 1000).toFixed(2)} m/s`} />
        {depDv !== null && arrDv !== null ? (
          <>
            <KV k="Δv departure" v={`${depDv.toFixed(2)} m/s`} />
            <KV k="Δv arrival" v={`${arrDv.toFixed(2)} m/s`} />
          </>
        ) : null}
        <div className="flex justify-end">
          <EnrichmentBadge enrichment={enrichment} />
        </div>
      </div>
      <PerchRoeDetails roe={perchRoe} enriched={enriched} />
      {subSurface ? (
        <Callout tone="abort">
          <div className="flex flex-col gap-1">
            <span className="tracking-wider uppercase">transfer below 200 km altitude floor</span>
            <span className="text-[10px] lowercase">
              periapsis {subSurface.periapsis_altitude_km.toFixed(0)} km — atmospheric drag
              invalidates the two-body arc below this floor. adjust tof, revolutions, or direction
              to lift the transfer ellipse above 200 km.
            </span>
          </div>
        </Callout>
      ) : null}
    </div>
  );
}

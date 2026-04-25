import type { EnrichmentSuggestion, PerchFallbackReason, TransferResult } from 'rpo-wasm';

import { selectTransferSubSurface, usePlanner } from '@/stores/planner';
import { Callout } from '@/ui/Callout';
import { Chip } from '@/ui/Chip';
import { KV } from '@/ui/KV';
import { vectorMag } from '@/utils/math';

import { PerchRoeDetails } from './PerchRoeDetails';

function formatFallbackReason(reason: PerchFallbackReason): string {
  switch (reason.type) {
    case 'separation_unachievable':
      return `requested ${(reason.requested_km * 1000).toFixed(0)} m separation; achievable ${(reason.achievable_km * 1000).toFixed(0)} m`;
    case 'singular_geometry':
      return `singular geometry at mean argument of latitude ${reason.mean_arg_lat_rad.toFixed(3)} rad`;
    case 'invalid_chief_elements':
    case 'safety_analysis':
    case 'propagation':
    case 'kepler_failure':
      return reason.detail;
    case 'insufficient_sampling':
      return `insufficient sampling: ${reason.total_samples} of ${reason.required_per_orbit}/orbit required`;
  }
}

function EnrichmentBadge({ enrichment }: { enrichment: EnrichmentSuggestion | null }) {
  if (enrichment === null) return null;
  switch (enrichment.perch.status) {
    case 'enriched':
      return <Chip tone="go">enriched</Chip>;
    case 'baseline':
      return <Chip tone="muted">baseline</Chip>;
    case 'fallback':
      return <Chip tone="hold">fallback</Chip>;
  }
}

export function DeltaVReadout({
  transfer,
  enrichment,
  submitting,
}: {
  transfer: TransferResult | null;
  enrichment: EnrichmentSuggestion | null;
  submitting: boolean;
}) {
  // Sub-surface (non-physical) conics still render their Δv numbers (so the
  // user can see what their inputs produced) plus a periapsis callout; the
  // parent gates the ACCEPT TRANSFER button on the same selector. Hooks must
  // run unconditionally — keep this above the early returns.
  const subSurface = usePlanner(selectTransferSubSurface);
  if (submitting && transfer === null) {
    return (
      <div className="flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5 font-mono text-[10px] text-text-dim">
        solving…
      </div>
    );
  }
  if (transfer === null) {
    return (
      <div className="flex flex-col gap-1 rounded-xs border border-border bg-surface-2 px-2 py-1.5">
        <KV k="Δv lambert" v="—" />
      </div>
    );
  }
  const fallback = enrichment?.perch.status === 'fallback' ? enrichment.perch : null;
  const lambert = transfer.plan.transfer ?? null;
  const depDv = lambert ? vectorMag(lambert.departure_dv_eci_km_s) * 1000 : null;
  const arrDv = lambert ? vectorMag(lambert.arrival_dv_eci_km_s) * 1000 : null;
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
        {enrichment !== null ? (
          <div className="flex justify-end">
            <EnrichmentBadge enrichment={enrichment} />
          </div>
        ) : null}
      </div>
      <PerchRoeDetails
        roe={transfer.plan.perch_roe}
        enriched={enrichment?.perch.status === 'enriched'}
      />
      {subSurface ? (
        <Callout tone="abort">
          <div className="flex flex-col gap-1">
            <span className="tracking-wider uppercase">transfer passes through earth</span>
            <span className="text-[10px] normal-case">
              periapsis {subSurface.periapsis_altitude_km.toFixed(0)} km — adjust tof, revolutions,
              or direction to lift the transfer ellipse above the surface.
            </span>
          </div>
        </Callout>
      ) : null}
      {fallback ? (
        <Callout tone="hold">
          <div className="flex flex-col gap-1">
            <span className="tracking-wider uppercase">formation safety not applied</span>
            <span className="text-[10px] normal-case">{formatFallbackReason(fallback.reason)}</span>
            {fallback.reason.type === 'separation_unachievable' ? (
              <>
                <span className="text-[10px] text-text-muted normal-case">
                  requested separation exceeds the ROE linearization bound for this orbit. reduce to
                  ≤ {fallback.reason.achievable_km.toFixed(1)} km to apply formation safety, or
                  continue with the unenriched baseline.
                </span>
                <details className="text-[10px]">
                  <summary className="cursor-pointer text-text-dim hover:text-text">why?</summary>
                  <p className="mt-1 text-text-dim normal-case">
                    The planner uses a linear perturbation model (D'Amico §2.3.4) that starts
                    producing meter-scale errors beyond ~1% of the chief semi-major axis. Requests
                    past that bound can't be honored without leaving the regime where the
                    formation-safety analysis is trustworthy.
                  </p>
                </details>
              </>
            ) : null}
          </div>
        </Callout>
      ) : null}
    </div>
  );
}

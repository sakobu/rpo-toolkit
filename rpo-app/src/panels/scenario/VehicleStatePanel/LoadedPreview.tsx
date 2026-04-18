import type { VehicleStateSlot } from '@/stores/scenario';

type LoadedSlot = Extract<VehicleStateSlot, { status: 'loaded' }>;

type LoadedPreviewProps = {
  slot: LoadedSlot;
  onClear: () => void;
};

export function LoadedPreview({ slot, onClear }: LoadedPreviewProps) {
  const { fileName, vector } = slot;
  return (
    <div className="flex flex-col gap-3">
      <div className="flex items-baseline justify-between gap-2">
        <span className="truncate font-mono text-xs text-text-muted">{fileName}</span>
        <button
          type="button"
          onClick={onClear}
          className="shrink-0 cursor-pointer font-mono text-xs tracking-wider text-text-dim uppercase hover:text-text-muted"
        >
          clear
        </button>
      </div>
      <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 font-mono text-xs">
        <dt className="text-text-dim uppercase">epoch</dt>
        <dd className="text-text">{vector.epoch}</dd>
        <dt className="text-text-dim uppercase">r [km]</dt>
        <dd className="text-text tabular-nums">{formatVec(vector.position_eci_km)}</dd>
        <dt className="text-text-dim uppercase">v [km/s]</dt>
        <dd className="text-text tabular-nums">{formatVec(vector.velocity_eci_km_s)}</dd>
      </dl>
    </div>
  );
}

function formatVec(v: readonly [number, number, number]): string {
  return v.map((x) => x.toFixed(3).padStart(12)).join('  ');
}

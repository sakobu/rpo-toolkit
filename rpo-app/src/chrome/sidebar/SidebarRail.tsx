import { useShallow } from 'zustand/react/shallow';

import { SIDEBAR_RAIL_WIDTH_CLASS } from '@/chrome/constants';
import { EditConfigButton } from '@/chrome/EditConfigButton';
import { PHASE_DEFINITIONS, PHASE_KEYS, type PhaseState, usePlanner } from '@/stores/planner';
import { useUI } from '@/stores/ui';
import { withBlur } from '@/utils/blur';

const DOT_VARIANT: Record<PhaseState, string> = {
  done: 'bg-signal-go',
  active: 'bg-accent',
  idle: 'border border-border-strong',
  locked: 'border border-dashed border-border-strong',
  skipped: 'border border-dashed border-border-strong',
};

export function SidebarRail() {
  const phases = usePlanner(useShallow((s) => s.phases));
  const expand = useUI((s) => s.expand);

  return (
    <aside
      className={`pointer-events-auto fixed top-0 left-0 z-30 flex h-screen ${SIDEBAR_RAIL_WIDTH_CLASS} flex-col items-center gap-2 border-r border-border bg-surface-1 py-2.5`}
    >
      <button
        type="button"
        onClick={withBlur(() => expand('sidebar'))}
        aria-label="expand sidebar"
        className="cursor-pointer rounded-xs border border-border px-1 py-0.5 font-mono text-[9px] text-text-muted hover:text-text"
      >
        ›
      </button>
      <div className="h-1.5" />
      {PHASE_KEYS.map((key) => (
        <RailPhase key={key} label={PHASE_DEFINITIONS[key].shortLabel} state={phases[key].state} />
      ))}
      <div className="flex-1" />
      <EditConfigButton variant="rail" />
      <span className="font-mono text-[8px] tracking-wider text-text-dim">S</span>
    </aside>
  );
}

function RailPhase({ label, state }: { label: string; state: PhaseState }) {
  const isActive = state === 'active';
  const isDone = state === 'done';

  const textColor = isDone ? 'text-signal-go' : isActive ? 'text-accent' : 'text-text-dim';
  const containerClass = isActive
    ? 'border border-accent bg-accent-dim'
    : 'border border-border bg-transparent';

  return (
    <div
      className={`flex h-5 w-6 flex-col items-center justify-center gap-0.5 rounded-xs ${containerClass}`}
      title={label}
    >
      <span className={`font-mono text-[8px] font-semibold tracking-wide uppercase ${textColor}`}>
        {label}
      </span>
      <span className={`h-1 w-1 rounded-full ${DOT_VARIANT[state]}`} aria-hidden />
    </div>
  );
}

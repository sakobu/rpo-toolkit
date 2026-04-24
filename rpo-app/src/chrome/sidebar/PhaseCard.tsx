import type { ReactNode } from 'react';

import { PHASE_DEFINITIONS, type PhaseKey, type PhaseState } from '@/stores/planner';
import { Chip } from '@/ui/Chip';

type PhaseCardProps = {
  phaseKey: PhaseKey;
  state: PhaseState;
  dimmedBy?: PhaseKey[];
  resets?: string;
  onEdit?: () => void;
  summary?: ReactNode;
  children?: ReactNode;
};

const TITLE_COLOR: Record<PhaseState, string> = {
  active: 'text-text',
  done: 'text-text-muted',
  skipped: 'text-text-dim',
  idle: 'text-text-muted',
  locked: 'text-text-dim',
};

export function PhaseCard({
  phaseKey,
  state,
  dimmedBy = [],
  resets,
  onEdit,
  summary,
  children,
}: PhaseCardProps) {
  const editable = state === 'done' && Boolean(onEdit);
  const isActive = state === 'active';
  const title = PHASE_DEFINITIONS[phaseKey].title;

  return (
    <div
      data-phase={phaseKey}
      data-editable={editable || undefined}
      data-dimmed-by={dimmedBy.length > 0 ? dimmedBy.join(' ') : undefined}
      className={`duration-fast relative border-b border-border px-4 py-3 transition-opacity ease-out ${
        isActive
          ? 'border-l-2 border-l-accent bg-accent/5 pl-3.5'
          : 'border-l-2 border-l-transparent'
      } ${state === 'skipped' || state === 'locked' ? 'opacity-55' : ''}`}
    >
      <div className="flex items-center gap-2.5">
        <StateDot state={state} />
        <span
          className={`flex-1 font-mono text-[10px] font-semibold tracking-wider uppercase ${TITLE_COLOR[state]}`}
        >
          {title}
        </span>
        <StateBadge state={state} />
        {editable && onEdit && (
          <button
            type="button"
            onClick={onEdit}
            data-reveal="edit"
            className="inline-flex cursor-pointer items-center gap-1 rounded-xs border border-border-strong bg-surface-2 px-1.5 py-0.5 font-mono text-[9px] tracking-wider text-signal-hold uppercase hover:bg-surface-3"
            aria-label={`edit ${title} · will reset downstream`}
          >
            <span aria-hidden>⚠</span>
            edit
          </button>
        )}
      </div>
      {summary && <div className="mt-1.5 ml-5">{summary}</div>}
      {children && <div className="mt-2.5 ml-5">{children}</div>}
      {editable && resets && (
        <div
          data-reveal="cascade"
          className="mt-2 ml-5 rounded-xs border border-signal-hold/40 bg-signal-hold-dim px-2 py-1.5 font-mono text-[10px] text-signal-hold"
          role="status"
        >
          Editing will reset: <span className="text-text">{resets}</span>
        </div>
      )}
    </div>
  );
}

function StateBadge({ state }: { state: PhaseState }) {
  if (state === 'active') return <Chip tone="accent">active</Chip>;
  if (state === 'done') return <Chip tone="go">done</Chip>;
  if (state === 'skipped') return <Chip tone="muted">skipped</Chip>;
  return null;
}

const DOT_VARIANT: Record<PhaseState, string> = {
  done: 'bg-signal-go shadow-[0_0_0_3px_var(--color-signal-go-dim)]',
  active: 'bg-accent shadow-[0_0_0_3px_var(--color-accent-dim)]',
  idle: 'border border-border-strong',
  locked: 'border border-dashed border-border-strong',
  skipped: 'border border-dashed border-border-strong',
};

export function StateDot({ state }: { state: PhaseState }) {
  return (
    <span
      className={`flex h-2.5 w-2.5 shrink-0 items-center justify-center rounded-full ${DOT_VARIANT[state]}`}
      aria-hidden
    >
      {state === 'done' && (
        <svg viewBox="0 0 8 8" className="h-1.5 w-1.5 text-bg">
          <path
            d="M1 4L3 6L7 2"
            stroke="currentColor"
            strokeWidth="1.8"
            fill="none"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      )}
    </span>
  );
}

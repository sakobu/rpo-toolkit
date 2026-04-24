import type { ReactNode } from 'react';

export type Tone = 'muted' | 'go' | 'hold' | 'info' | 'abort' | 'accent';

const TONE_CLASSES: Record<Tone, string> = {
  muted: 'border-border text-text-muted',
  go: 'border-signal-go/40 bg-signal-go-dim text-signal-go',
  hold: 'border-signal-hold/40 bg-signal-hold-dim text-signal-hold',
  info: 'border-signal-info/40 bg-signal-info-dim text-signal-info',
  abort: 'border-signal-abort/40 bg-signal-abort-dim text-signal-abort',
  accent: 'border-accent/40 bg-accent-dim text-accent',
};

type ChipProps = {
  tone?: Tone;
  children: ReactNode;
};

export function Chip({ tone = 'muted', children }: ChipProps) {
  return (
    <span
      className={`inline-flex items-center rounded-xs border px-1.5 py-0.5 font-mono text-[9px] tracking-wider uppercase ${TONE_CLASSES[tone]}`}
    >
      {children}
    </span>
  );
}

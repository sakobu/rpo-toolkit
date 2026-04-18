import { type ReactNode } from 'react';

type CalloutTone = 'abort' | 'hold' | 'info' | 'go';

type CalloutProps = {
  tone: CalloutTone;
  children: ReactNode;
  className?: string;
};

const TONE_CHROME: Record<CalloutTone, string> = {
  abort: 'border-signal-abort/40 bg-signal-abort-dim text-signal-abort',
  hold: 'border-signal-hold/40 bg-signal-hold-dim text-signal-hold',
  info: 'border-signal-info/40 bg-signal-info-dim text-signal-info',
  go: 'border-signal-go/40 bg-signal-go-dim text-signal-go',
};

const ALERT_TONES: ReadonlySet<CalloutTone> = new Set(['abort', 'hold']);

export function Callout({ tone, children, className }: CalloutProps) {
  const role = ALERT_TONES.has(tone) ? 'alert' : 'status';
  return (
    <div
      role={role}
      className={`rounded-sm border px-4 py-2 font-mono text-xs ${TONE_CHROME[tone]} ${className ?? ''}`}
    >
      {children}
    </div>
  );
}

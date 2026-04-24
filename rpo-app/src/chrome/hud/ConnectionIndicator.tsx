import { type ConnectionStatus, useConnection } from '@/stores/connection';
import { Chip, type Tone } from '@/ui/Chip';

function toneFor(status: ConnectionStatus, heartbeatWarning: boolean): Tone {
  if (heartbeatWarning && status.status === 'BUSY') return 'hold';
  switch (status.status) {
    case 'IDLE':
      return 'go';
    case 'BUSY':
      return 'info';
    case 'CONNECTING':
    case 'RECONNECTING':
      return 'hold';
    case 'DISCONNECTED':
      return 'abort';
  }
}

function labelFor(status: ConnectionStatus, heartbeatWarning: boolean): string {
  if (heartbeatWarning && status.status === 'BUSY') return 'slow';
  switch (status.status) {
    case 'IDLE':
      return 'live';
    case 'BUSY':
      return 'busy';
    case 'CONNECTING':
      return 'conn…';
    case 'RECONNECTING':
      return `retry ${status.attempt}`;
    case 'DISCONNECTED':
      return 'offline';
  }
}

export function ConnectionIndicator() {
  const status = useConnection((s) => s.connection);
  const warning = useConnection((s) => s.heartbeatWarning);
  return (
    <span data-connection-slot>
      <Chip tone={toneFor(status, warning)}>{labelFor(status, warning)}</Chip>
    </span>
  );
}

const DOT_CLASS: Record<Tone, string> = {
  muted: 'bg-text-dim',
  go: 'bg-signal-go',
  hold: 'bg-signal-hold',
  info: 'bg-signal-info',
  abort: 'bg-signal-abort',
  accent: 'bg-accent',
};

export function ConnectionDot() {
  const status = useConnection((s) => s.connection);
  const warning = useConnection((s) => s.heartbeatWarning);
  const tone = toneFor(status, warning);
  return (
    <span
      data-connection-slot
      aria-label={`connection ${labelFor(status, warning)}`}
      className={`inline-block size-1.5 rounded-full ${DOT_CLASS[tone]}`}
    />
  );
}

import type { ReactNode } from 'react';

import { withBlur } from '@/utils/blur';

type Tone = 'default' | 'abort';

type InlineActionButtonProps = {
  icon: ReactNode;
  label: string;
  onClick: () => void;
  tone?: Tone;
  'aria-label'?: string;
};

const TONE_HOVER: Record<Tone, string> = {
  default: 'hover:text-text',
  abort: 'hover:text-signal-abort',
};

export function InlineActionButton({
  icon,
  label,
  onClick,
  tone = 'default',
  'aria-label': ariaLabel,
}: InlineActionButtonProps) {
  return (
    <button
      type="button"
      onClick={withBlur(onClick)}
      aria-label={ariaLabel}
      className={`flex cursor-pointer items-center gap-1 rounded-xs border border-border-strong bg-surface-1 px-1.5 py-px font-mono text-[9px] tracking-wider text-text-muted uppercase ${TONE_HOVER[tone]}`}
    >
      {icon}
      {label}
    </button>
  );
}

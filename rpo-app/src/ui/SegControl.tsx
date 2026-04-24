import { withBlur } from '@/utils/blur';

export function SegControl({
  active,
  onClick,
  disabled = false,
  children,
}: {
  active: boolean;
  onClick: () => void;
  disabled?: boolean;
  children: React.ReactNode;
}) {
  // Preserve the active highlight under disabled so the user still sees which
  // option was selected; the muted accent palette signals "your choice, but
  // currently ignored".
  const activeStyle = disabled
    ? 'border-accent/40 bg-accent-dim/50 text-accent/60'
    : 'border-accent bg-accent-dim text-accent';
  const idleStyle = disabled
    ? 'border-border bg-surface-2 text-text-dim'
    : 'border-border bg-transparent text-text-muted hover:text-text';
  return (
    <button
      type="button"
      disabled={disabled}
      onClick={withBlur(onClick)}
      className={`duration-fast rounded-xs border px-2 py-0.5 font-mono tracking-wider uppercase transition-colors ease-out ${
        disabled ? 'cursor-not-allowed' : 'cursor-pointer'
      } ${active ? activeStyle : idleStyle}`}
    >
      {children}
    </button>
  );
}

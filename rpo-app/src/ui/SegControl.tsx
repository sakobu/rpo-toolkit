import { withBlur } from '@/utils/blur';

export function SegControl({
  active,
  onClick,
  children,
}: {
  active: boolean;
  onClick: () => void;
  children: React.ReactNode;
}) {
  return (
    <button
      type="button"
      onClick={withBlur(onClick)}
      className={`duration-fast cursor-pointer rounded-xs border px-2 py-0.5 font-mono tracking-wider uppercase transition-colors ease-out ${
        active
          ? 'border-accent bg-accent-dim text-accent'
          : 'border-border bg-transparent text-text-muted hover:text-text'
      }`}
    >
      {children}
    </button>
  );
}

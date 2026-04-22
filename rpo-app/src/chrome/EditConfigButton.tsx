import { useNavigate } from 'react-router';
import { TriangleAlert } from 'lucide-react';

import { withBlur } from '@/utils/blur';

type Variant = 'inline' | 'rail';

type EditConfigButtonProps = {
  variant?: Variant;
};

const TOOLTIP = 'Edit configuration — re-runs classification, resets downstream phases';

export function EditConfigButton({ variant = 'inline' }: EditConfigButtonProps) {
  const navigate = useNavigate();
  const onClick = withBlur(() => {
    void navigate('/');
  });

  if (variant === 'rail') {
    return (
      <button
        type="button"
        title={TOOLTIP}
        aria-label="edit configuration"
        onClick={onClick}
        className="flex h-5 w-6 cursor-pointer items-center justify-center rounded-xs border border-border-strong bg-surface-2 hover:bg-surface-3"
      >
        <TriangleAlert size={10} strokeWidth={1.75} className="text-signal-hold" />
      </button>
    );
  }

  return (
    <button
      type="button"
      title={TOOLTIP}
      aria-label="edit configuration"
      onClick={onClick}
      className="flex shrink-0 cursor-pointer items-center gap-1 rounded-xs border border-border-strong bg-surface-2 px-1.5 py-1 font-mono text-[9px] tracking-wider text-text-muted uppercase hover:text-text"
    >
      <TriangleAlert size={10} strokeWidth={1.75} className="text-signal-hold" />
      edit
    </button>
  );
}

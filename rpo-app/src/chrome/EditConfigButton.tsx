import { useNavigate } from 'react-router';
import { Pencil, TriangleAlert } from 'lucide-react';

import { usePlanner } from '@/stores/planner';
import { withBlur } from '@/utils/blur';

type Variant = 'inline' | 'rail';

type EditConfigButtonProps = {
  variant?: Variant;
};

const TOOLTIP_NEUTRAL = 'Edit configuration';
const TOOLTIP_WARN = 'Edit configuration — editing inputs will reset downstream phases';

export function EditConfigButton({ variant = 'inline' }: EditConfigButtonProps) {
  const navigate = useNavigate();
  const hasDownstreamToLose = usePlanner((s) => s.phases.xfr.state === 'done');
  const onClick = withBlur(() => {
    void navigate('/');
  });

  const tooltip = hasDownstreamToLose ? TOOLTIP_WARN : TOOLTIP_NEUTRAL;
  const Icon = hasDownstreamToLose ? TriangleAlert : Pencil;
  const iconClass = hasDownstreamToLose ? 'text-signal-hold' : 'text-text-dim';

  if (variant === 'rail') {
    return (
      <button
        type="button"
        title={tooltip}
        aria-label="edit configuration"
        onClick={onClick}
        className="flex h-5 w-6 cursor-pointer items-center justify-center rounded-xs border border-border-strong bg-surface-2 hover:bg-surface-3"
      >
        <Icon size={10} strokeWidth={1.75} className={iconClass} />
      </button>
    );
  }

  return (
    <button
      type="button"
      title={tooltip}
      aria-label="edit configuration"
      onClick={onClick}
      className="flex shrink-0 cursor-pointer items-center gap-1 rounded-xs border border-border-strong bg-surface-2 px-1.5 py-1 font-mono text-[9px] tracking-wider text-text-muted uppercase hover:text-text"
    >
      <Icon size={10} strokeWidth={1.75} className={iconClass} />
      edit
    </button>
  );
}

import type { ReactNode } from 'react';
import { Minus, X } from 'lucide-react';

import { IconButton } from '@/ui/IconButton';
import { withBlur } from '@/utils/blur';

type PanelHeaderProps = {
  leading?: ReactNode;
  label: ReactNode;
  meta?: ReactNode;
  onCollapse: () => void;
  onHide: () => void;
  collapseLabel: string;
  hideLabel: string;
  className?: string;
};

export function PanelHeader({
  leading,
  label,
  meta,
  onCollapse,
  onHide,
  collapseLabel,
  hideLabel,
  className,
}: PanelHeaderProps) {
  return (
    <header
      className={`flex items-center justify-between border-b border-border px-3 py-2 ${className ?? ''}`}
    >
      <div className="flex items-center gap-2 text-text-muted">
        {leading}
        {label}
        {meta}
      </div>
      <div className="flex items-center gap-1">
        <IconButton onClick={withBlur(onCollapse)} aria-label={collapseLabel}>
          <Minus size={12} strokeWidth={1.5} />
        </IconButton>
        <IconButton onClick={withBlur(onHide)} aria-label={hideLabel}>
          <X size={12} strokeWidth={1.5} />
        </IconButton>
      </div>
    </header>
  );
}

import { ChevronUp } from 'lucide-react';

import { DOCK_COLLAPSED_HEIGHT_CLASS } from '@/chrome/constants';
import { useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';
import { IconButton } from '@/ui/IconButton';
import { withBlur } from '@/utils/blur';

export function DockCollapsed() {
  const expand = useUI((s) => s.expand);

  return (
    <aside
      className={`pointer-events-auto fixed right-0 bottom-0 left-0 z-30 flex ${DOCK_COLLAPSED_HEIGHT_CLASS} items-center justify-between border-t border-border bg-surface-1/90 px-4 backdrop-blur-sm`}
    >
      <div className="flex items-center gap-3">
        <Caps color="text-text-muted" className="font-semibold">
          mission control
        </Caps>
        <span className="font-mono text-[10px] text-text-dim">T+ 00:00:00</span>
      </div>
      <IconButton onClick={withBlur(() => expand('dock'))} aria-label="expand mission dock">
        <ChevronUp size={12} strokeWidth={1.5} />
      </IconButton>
    </aside>
  );
}

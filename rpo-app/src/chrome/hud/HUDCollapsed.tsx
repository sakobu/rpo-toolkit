import { useLocation } from 'react-router';
import { ChevronDown, GripHorizontal } from 'lucide-react';

import { HUD_COLLAPSED_WIDTH_CLASS } from '@/chrome/constants';
import { ConnectionDot } from '@/chrome/hud/ConnectionIndicator';
import { useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';
import { Chip } from '@/ui/Chip';
import { IconButton } from '@/ui/IconButton';
import { withBlur } from '@/utils/blur';

export function HUDCollapsed() {
  const expand = useUI((s) => s.expand);
  const { pathname } = useLocation();
  const isFarField = pathname.startsWith('/far-field');
  const frame = isFarField ? 'ECI' : 'RIC';

  return (
    <aside
      className={`pointer-events-auto fixed top-4 right-4 z-30 ${HUD_COLLAPSED_WIDTH_CLASS} rounded-xs border border-border bg-surface-1/85 backdrop-blur-sm`}
    >
      <div className="flex items-center justify-between px-2.5 py-1.5">
        <div className="flex items-center gap-1.5 text-text-muted">
          <GripHorizontal size={10} strokeWidth={1.5} />
          <Caps>hud</Caps>
          <ConnectionDot />
          <Chip tone="muted">{frame}</Chip>
        </div>
        <IconButton onClick={withBlur(() => expand('hud'))} aria-label="expand hud">
          <ChevronDown size={12} strokeWidth={1.5} />
        </IconButton>
      </div>
    </aside>
  );
}

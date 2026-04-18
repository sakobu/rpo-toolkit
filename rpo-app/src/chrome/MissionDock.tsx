import { X } from 'lucide-react';

import { useHotkey } from '@/hooks/useHotkey';
import { useUI } from '@/stores/ui';
import { IconButton } from '@/ui/IconButton';
import { withBlur } from '@/utils/blur';

import ResizeHandle from './ResizeHandle';

const MIN_HEIGHT = 160;
const MAX_HEIGHT = 720;

export default function MissionDock() {
  const missionDockOpen = useUI((s) => s.missionDockOpen);
  const missionDockHeight = useUI((s) => s.missionDockHeight);
  const toggleMissionDock = useUI((s) => s.toggleMissionDock);
  const setMissionDockHeight = useUI((s) => s.setMissionDockHeight);

  useHotkey('t', toggleMissionDock);

  return (
    <aside
      style={{ height: missionDockHeight }}
      className={`duration-base pointer-events-auto fixed right-0 bottom-0 left-0 z-30 flex flex-col border-t border-border bg-surface-1/90 backdrop-blur-sm transition-transform ease-out ${
        missionDockOpen ? 'translate-y-0' : 'translate-y-full'
      }`}
    >
      <ResizeHandle
        initialHeight={missionDockHeight}
        minHeight={MIN_HEIGHT}
        maxHeight={MAX_HEIGHT}
        onResize={setMissionDockHeight}
      />
      <header className="flex items-center justify-between border-b border-border px-4 py-2">
        <span className="font-mono text-[10px] tracking-wider text-text-muted uppercase">
          mission control
        </span>
        <IconButton onClick={withBlur(toggleMissionDock)} aria-label="close mission dock">
          <X size={12} strokeWidth={1.5} />
        </IconButton>
      </header>
      <div className="flex-1 overflow-y-auto px-4 py-3 font-mono text-xs text-text-dim">
        timeline · playback — coming soon
      </div>
    </aside>
  );
}

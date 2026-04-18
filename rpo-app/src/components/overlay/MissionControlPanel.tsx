import { X } from 'lucide-react';

import { useHotkey } from '@/hooks/useHotkey';
import { useUI } from '@/stores/ui';
import { withBlur } from '@/utils/blur';

import ResizeHandle from './ResizeHandle';

const MIN_HEIGHT = 160;
const MAX_HEIGHT = 720;

export default function MissionControlPanel() {
  const missionPanelOpen = useUI((s) => s.missionPanelOpen);
  const missionPanelHeight = useUI((s) => s.missionPanelHeight);
  const toggleMissionPanel = useUI((s) => s.toggleMissionPanel);
  const setMissionPanelHeight = useUI((s) => s.setMissionPanelHeight);

  useHotkey('t', toggleMissionPanel);

  return (
    <aside
      style={{ height: missionPanelHeight }}
      className={`duration-base pointer-events-auto fixed right-0 bottom-0 left-0 z-30 flex flex-col border-t border-border bg-surface-1/90 backdrop-blur-sm transition-transform ease-out ${
        missionPanelOpen ? 'translate-y-0' : 'translate-y-full'
      }`}
    >
      <ResizeHandle
        initialHeight={missionPanelHeight}
        minHeight={MIN_HEIGHT}
        maxHeight={MAX_HEIGHT}
        onResize={setMissionPanelHeight}
      />
      <header className="flex items-center justify-between border-b border-border px-4 py-2">
        <span className="font-mono text-[10px] tracking-wider text-text-muted uppercase">
          mission control
        </span>
        <button
          type="button"
          onClick={withBlur(toggleMissionPanel)}
          className="flex h-5 w-5 cursor-pointer items-center justify-center rounded-xs text-text-dim hover:text-text"
          aria-label="close mission panel"
        >
          <X size={12} strokeWidth={1.5} />
        </button>
      </header>
      <div className="flex-1 overflow-y-auto px-4 py-3 font-mono text-xs text-text-dim">
        timeline · playback — coming soon
      </div>
    </aside>
  );
}

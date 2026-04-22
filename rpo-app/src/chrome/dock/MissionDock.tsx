import { DOCK_MAX_HEIGHT_PX, DOCK_MIN_HEIGHT_PX } from '@/chrome/constants';
import { DockCollapsed } from '@/chrome/dock/DockCollapsed';
import { ResizeHandle } from '@/chrome/dock/ResizeHandle';
import { PanelHeader } from '@/chrome/PanelHeader';
import { useHotkey } from '@/hooks/useHotkey';
import { useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';

export function MissionDock() {
  const dock = useUI((s) => s.dock);
  const cycleDock = useUI((s) => s.cycleDock);
  const hide = useUI((s) => s.hide);
  const dockHeight = useUI((s) => s.dockHeight);
  const setDockHeight = useUI((s) => s.setDockHeight);

  useHotkey('d', cycleDock);

  if (dock === 'hidden') return null;
  if (dock === 'collapsed') return <DockCollapsed />;

  return (
    <aside
      style={{ height: dockHeight }}
      className="pointer-events-auto fixed right-0 bottom-0 left-0 z-30 flex flex-col border-t border-border bg-surface-1/90 backdrop-blur-sm"
    >
      <ResizeHandle
        initialHeight={dockHeight}
        minHeight={DOCK_MIN_HEIGHT_PX}
        maxHeight={DOCK_MAX_HEIGHT_PX}
        onResize={setDockHeight}
      />
      <PanelHeader
        className="px-4"
        label={
          <Caps color="text-text-muted" className="font-semibold">
            mission control
          </Caps>
        }
        meta={<span className="ml-1 font-mono text-[10px] text-text-dim">T+ 00:00:00</span>}
        onCollapse={cycleDock}
        onHide={() => hide('dock')}
        collapseLabel="collapse mission dock"
        hideLabel="hide mission dock"
      />
      <div className="flex-1 overflow-y-auto px-4 py-3 font-mono text-xs text-text-dim">
        timeline · Gantt · FTRT playback — stubbed
      </div>
    </aside>
  );
}

import { useShallow } from 'zustand/react/shallow';

import { type ChromeRegion, type ChromeState, useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';
import { withBlur } from '@/utils/blur';

type Region = {
  key: 'S' | 'H' | 'D';
  label: ChromeRegion;
  state: ChromeState;
  onCycle: () => void;
};

const STATE_GLYPH: Record<ChromeState, string> = {
  expanded: '▣',
  collapsed: '▢',
  hidden: '◌',
};

const STATE_COLOR: Record<ChromeState, string> = {
  expanded: 'text-signal-go',
  collapsed: 'text-signal-hold',
  hidden: 'text-text-dim',
};

export function HotkeyLegend() {
  const { sidebar, hud, dock, cycleSidebar, cycleHud, cycleDock } = useUI(
    useShallow((s) => ({
      sidebar: s.sidebar,
      hud: s.hud,
      dock: s.dock,
      cycleSidebar: s.cycleSidebar,
      cycleHud: s.cycleHud,
      cycleDock: s.cycleDock,
    })),
  );

  const regions: readonly Region[] = [
    { key: 'S', label: 'sidebar', state: sidebar, onCycle: cycleSidebar },
    { key: 'H', label: 'hud', state: hud, onCycle: cycleHud },
    { key: 'D', label: 'dock', state: dock, onCycle: cycleDock },
  ];

  return (
    <div className="pointer-events-auto fixed right-4 bottom-4 z-40 flex items-center gap-3 rounded-xs border border-border bg-surface-1/85 px-2.5 py-1.5 backdrop-blur-sm">
      {regions.map((r) => (
        <button
          key={r.key}
          type="button"
          onClick={withBlur(r.onCycle)}
          className="flex cursor-pointer items-center gap-1.5 text-left"
          aria-label={`cycle ${r.label} · currently ${r.state}`}
        >
          <span className="rounded-xs border border-border-strong bg-surface-2 px-1.5 py-px font-mono text-[9px] text-text-muted">
            {r.key}
          </span>
          <Caps>{r.label}</Caps>
          <span className={`font-mono text-[10px] leading-none ${STATE_COLOR[r.state]}`}>
            {STATE_GLYPH[r.state]}
          </span>
        </button>
      ))}
    </div>
  );
}

import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import { DOCK_COLLAPSED_HEIGHT_PX, DOCK_DEFAULT_HEIGHT_PX } from '@/chrome/constants';

export type ChromeState = 'expanded' | 'collapsed' | 'hidden';
export type ChromeRegion = 'sidebar' | 'hud' | 'dock';
export type HudReadoutFrame = 'ric' | 'roe';

type ChromeSnapshot = {
  sidebar: ChromeState;
  hud: ChromeState;
  dock: ChromeState;
};

type UIState = {
  sidebar: ChromeState;
  hud: ChromeState;
  dock: ChromeState;
  previousChrome: ChromeSnapshot | null;
  dockHeight: number;
  hudReadoutFrame: HudReadoutFrame;
  cycleSidebar: () => void;
  cycleHud: () => void;
  cycleDock: () => void;
  expand: (region: ChromeRegion) => void;
  hide: (region: ChromeRegion) => void;
  toggleHideAll: () => void;
  setDockHeight: (height: number) => void;
  setHudReadoutFrame: (frame: HudReadoutFrame) => void;
};

const NEXT_STATE: Record<ChromeState, ChromeState> = {
  expanded: 'collapsed',
  collapsed: 'hidden',
  hidden: 'expanded',
};

// Pixel height the dock currently occupies from the viewport bottom. Used by
// elements that anchor to the bottom edge (HotkeyLegend, RICAxes gizmo) so
// they lift above the dock instead of overlapping it.
export const selectDockOffsetPx = (s: UIState): number => {
  if (s.dock === 'expanded') return s.dockHeight;
  if (s.dock === 'collapsed') return DOCK_COLLAPSED_HEIGHT_PX;
  return 0;
};

export const useUI = create<UIState>()(
  devtools(
    (set) => ({
      sidebar: 'expanded',
      hud: 'expanded',
      dock: 'hidden',
      previousChrome: null,
      dockHeight: DOCK_DEFAULT_HEIGHT_PX,
      hudReadoutFrame: 'ric',
      cycleSidebar: () => set((s) => ({ sidebar: NEXT_STATE[s.sidebar] }), false, 'cycleSidebar'),
      cycleHud: () => set((s) => ({ hud: NEXT_STATE[s.hud] }), false, 'cycleHud'),
      cycleDock: () => set((s) => ({ dock: NEXT_STATE[s.dock] }), false, 'cycleDock'),
      expand: (region) =>
        set(
          (s) => (s[region] === 'expanded' ? s : { [region]: 'expanded' }),
          false,
          `expand/${region}`,
        ),
      hide: (region) =>
        set((s) => (s[region] === 'hidden' ? s : { [region]: 'hidden' }), false, `hide/${region}`),
      toggleHideAll: () =>
        set(
          (s) => {
            const allHidden = s.sidebar === 'hidden' && s.hud === 'hidden' && s.dock === 'hidden';
            if (allHidden) {
              if (s.previousChrome === null) {
                // No snapshot (e.g., user manually hid each region one-by-one). Fall back to all expanded.
                return {
                  sidebar: 'expanded',
                  hud: 'expanded',
                  dock: 'expanded',
                  previousChrome: null,
                };
              }
              return { ...s.previousChrome, previousChrome: null };
            }
            return {
              sidebar: 'hidden',
              hud: 'hidden',
              dock: 'hidden',
              previousChrome: { sidebar: s.sidebar, hud: s.hud, dock: s.dock },
            };
          },
          false,
          'toggleHideAll',
        ),
      setDockHeight: (height) =>
        set((s) => (s.dockHeight === height ? s : { dockHeight: height }), false, 'setDockHeight'),
      setHudReadoutFrame: (frame) => set({ hudReadoutFrame: frame }, false, 'setHudReadoutFrame'),
    }),
    { name: 'ui', enabled: import.meta.env.DEV },
  ),
);

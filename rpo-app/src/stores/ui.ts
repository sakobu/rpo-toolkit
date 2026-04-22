import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import { DOCK_DEFAULT_HEIGHT_PX } from '@/chrome/constants';

export type ChromeState = 'expanded' | 'collapsed' | 'hidden';
export type ChromeRegion = 'sidebar' | 'hud' | 'dock';
export type HudReadoutFrame = 'ric' | 'roe';

type UIState = {
  sidebar: ChromeState;
  hud: ChromeState;
  dock: ChromeState;
  dockHeight: number;
  hudReadoutFrame: HudReadoutFrame;
  cycleSidebar: () => void;
  cycleHud: () => void;
  cycleDock: () => void;
  expand: (region: ChromeRegion) => void;
  hide: (region: ChromeRegion) => void;
  setDockHeight: (height: number) => void;
  setHudReadoutFrame: (frame: HudReadoutFrame) => void;
};

const NEXT_STATE: Record<ChromeState, ChromeState> = {
  expanded: 'collapsed',
  collapsed: 'hidden',
  hidden: 'expanded',
};

export const useUI = create<UIState>()(
  devtools(
    (set) => ({
      sidebar: 'expanded',
      hud: 'expanded',
      dock: 'hidden',
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
      setDockHeight: (height) =>
        set((s) => (s.dockHeight === height ? s : { dockHeight: height }), false, 'setDockHeight'),
      setHudReadoutFrame: (frame) => set({ hudReadoutFrame: frame }, false, 'setHudReadoutFrame'),
    }),
    { name: 'ui', enabled: import.meta.env.DEV },
  ),
);

import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

type UIState = {
  sidebarOpen: boolean;
  hudVisible: boolean;
  missionDockOpen: boolean;
  missionDockHeight: number;
  toggleSidebar: () => void;
  toggleHUD: () => void;
  toggleMissionDock: () => void;
  setMissionDockHeight: (height: number) => void;
};

const MISSION_DOCK_DEFAULT_HEIGHT = 300;

export const useUI = create<UIState>()(
  devtools(
    (set) => ({
      sidebarOpen: true,
      hudVisible: true,
      missionDockOpen: false,
      missionDockHeight: MISSION_DOCK_DEFAULT_HEIGHT,
      toggleSidebar: () => set((s) => ({ sidebarOpen: !s.sidebarOpen }), false, 'toggleSidebar'),
      toggleHUD: () => set((s) => ({ hudVisible: !s.hudVisible }), false, 'toggleHUD'),
      toggleMissionDock: () =>
        set((s) => ({ missionDockOpen: !s.missionDockOpen }), false, 'toggleMissionDock'),
      setMissionDockHeight: (height) =>
        set({ missionDockHeight: height }, false, 'setMissionDockHeight'),
    }),
    { name: 'ui', enabled: import.meta.env.DEV },
  ),
);

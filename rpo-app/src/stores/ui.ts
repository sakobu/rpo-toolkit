import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

type UIState = {
  sidebarOpen: boolean;
  hudVisible: boolean;
  missionPanelOpen: boolean;
  missionPanelHeight: number;
  toggleSidebar: () => void;
  toggleHUD: () => void;
  toggleMissionPanel: () => void;
  setMissionPanelHeight: (height: number) => void;
};

const MISSION_PANEL_DEFAULT_HEIGHT = 300;

export const useUI = create<UIState>()(
  devtools(
    (set) => ({
      sidebarOpen: true,
      hudVisible: true,
      missionPanelOpen: false,
      missionPanelHeight: MISSION_PANEL_DEFAULT_HEIGHT,
      toggleSidebar: () => set((s) => ({ sidebarOpen: !s.sidebarOpen }), false, 'toggleSidebar'),
      toggleHUD: () => set((s) => ({ hudVisible: !s.hudVisible }), false, 'toggleHUD'),
      toggleMissionPanel: () =>
        set((s) => ({ missionPanelOpen: !s.missionPanelOpen }), false, 'toggleMissionPanel'),
      setMissionPanelHeight: (height) =>
        set({ missionPanelHeight: height }, false, 'setMissionPanelHeight'),
    }),
    { name: 'ui', enabled: import.meta.env.DEV },
  ),
);

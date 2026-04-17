import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import type { SpacecraftConfig } from '../schemas/spacecraft';

type ScenarioState = {
  chief: SpacecraftConfig | null;
  deputy: SpacecraftConfig | null;
  setChief: (c: SpacecraftConfig | null) => void;
  setDeputy: (d: SpacecraftConfig | null) => void;
};

export const useScenario = create<ScenarioState>()(
  devtools(
    (set) => ({
      chief: null,
      deputy: null,
      setChief: (c) => set({ chief: c }, false, 'setChief'),
      setDeputy: (d) => set({ deputy: d }, false, 'setDeputy'),
    }),
    { name: 'scenario', enabled: import.meta.env.DEV },
  ),
);

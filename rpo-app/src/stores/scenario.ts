import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import type { SpacecraftConfig } from '../schemas/spacecraft';

export type VehicleState = {
  values: SpacecraftConfig;
  isValid: boolean;
};

type ScenarioState = {
  chief: VehicleState | null;
  deputy: VehicleState | null;
  setChief: (c: VehicleState | null) => void;
  setDeputy: (d: VehicleState | null) => void;
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

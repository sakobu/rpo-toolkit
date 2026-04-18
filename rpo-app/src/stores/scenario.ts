import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import type { SpacecraftConfig } from '../schemas/spacecraft';
import type { StateVectorInput } from '../schemas/stateVector';

export type VehicleState = {
  values: SpacecraftConfig;
  isValid: boolean;
};

export type VehicleStateSlot =
  | { status: 'empty' }
  | { status: 'error'; message: string }
  | { status: 'loaded'; fileName: string; vector: StateVectorInput };

export const EMPTY_SLOT: VehicleStateSlot = { status: 'empty' };

type ScenarioState = {
  chief: VehicleState | null;
  deputy: VehicleState | null;
  chiefState: VehicleStateSlot;
  deputyState: VehicleStateSlot;
  setChief: (c: VehicleState | null) => void;
  setDeputy: (d: VehicleState | null) => void;
  setChiefState: (s: VehicleStateSlot) => void;
  setDeputyState: (s: VehicleStateSlot) => void;
};

export const useScenario = create<ScenarioState>()(
  devtools(
    (set) => ({
      chief: null,
      deputy: null,
      chiefState: EMPTY_SLOT,
      deputyState: EMPTY_SLOT,
      setChief: (c) => set({ chief: c }, false, 'setChief'),
      setDeputy: (d) => set({ deputy: d }, false, 'setDeputy'),
      setChiefState: (s) => set({ chiefState: s }, false, 'setChiefState'),
      setDeputyState: (s) => set({ deputyState: s }, false, 'setDeputyState'),
    }),
    { name: 'scenario', enabled: import.meta.env.DEV },
  ),
);

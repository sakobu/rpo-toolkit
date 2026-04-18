import { create } from 'zustand';
import { devtools, subscribeWithSelector } from 'zustand/middleware';

import type { SpacecraftConfig } from '@/schemas/spacecraft';
import type { StateVectorInput } from '@/schemas/stateVector';

export type VehicleState = {
  values: SpacecraftConfig;
  isValid: boolean;
};

export type VehicleStateSlot =
  | { status: 'empty' }
  | { status: 'error'; message: string }
  | { status: 'loaded'; fileName: string; vector: StateVectorInput };

export const EMPTY_SLOT: VehicleStateSlot = { status: 'empty' };

type ConfigState = {
  chiefConfig: VehicleState | null;
  deputyConfig: VehicleState | null;
  chiefState: VehicleStateSlot;
  deputyState: VehicleStateSlot;
  setChiefConfig: (c: VehicleState | null) => void;
  setDeputyConfig: (d: VehicleState | null) => void;
  setChiefState: (s: VehicleStateSlot) => void;
  setDeputyState: (s: VehicleStateSlot) => void;
};

export const useConfig = create<ConfigState>()(
  subscribeWithSelector(
    devtools(
      (set) => ({
        chiefConfig: null,
        deputyConfig: null,
        chiefState: EMPTY_SLOT,
        deputyState: EMPTY_SLOT,
        setChiefConfig: (c) => set({ chiefConfig: c }, false, 'setChiefConfig'),
        setDeputyConfig: (d) => set({ deputyConfig: d }, false, 'setDeputyConfig'),
        setChiefState: (s) => set({ chiefState: s }, false, 'setChiefState'),
        setDeputyState: (s) => set({ deputyState: s }, false, 'setDeputyState'),
      }),
      { name: 'configuration', enabled: import.meta.env.DEV },
    ),
  ),
);

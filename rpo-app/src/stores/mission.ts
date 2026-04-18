import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import { match } from '@railway-ts/pipelines/result';

import type { MissionPhase, ProximityConfig, WasmError } from 'rpo-wasm';

import { ROE_THRESHOLD_DEFAULT } from '@/schemas/proximityConfig';
import { classify } from '@/wasm/classify';

import { useScenario, type VehicleStateSlot } from './scenario';

export type Classification =
  | { status: 'idle' }
  | { status: 'ok'; phase: MissionPhase }
  | { status: 'err'; error: WasmError };

export const IDLE_CLASSIFICATION: Classification = { status: 'idle' };

const DEFAULT_PROXIMITY_CONFIG: ProximityConfig = { roe_threshold: ROE_THRESHOLD_DEFAULT };

type MissionState = {
  classification: Classification;
  proximityConfig: ProximityConfig;
  setProximityConfig: (config: ProximityConfig) => void;
};

export const useMission = create<MissionState>()(
  devtools(
    (set) => ({
      classification: IDLE_CLASSIFICATION,
      proximityConfig: DEFAULT_PROXIMITY_CONFIG,
      setProximityConfig: (config) => set({ proximityConfig: config }, false, 'setProximityConfig'),
    }),
    { name: 'mission', enabled: import.meta.env.DEV },
  ),
);

function recomputeClassification() {
  const { chiefState, deputyState } = useScenario.getState();
  const { proximityConfig } = useMission.getState();
  useMission.setState(
    { classification: deriveClassification(chiefState, deputyState, proximityConfig) },
    false,
    'deriveClassification',
  );
}

useScenario.subscribe((state, prev) => {
  if (state.chiefState === prev.chiefState && state.deputyState === prev.deputyState) return;
  recomputeClassification();
});

useMission.subscribe((state, prev) => {
  if (state.proximityConfig === prev.proximityConfig) return;
  recomputeClassification();
});

function deriveClassification(
  chiefSlot: VehicleStateSlot,
  deputySlot: VehicleStateSlot,
  config: ProximityConfig,
): Classification {
  if (chiefSlot.status !== 'loaded' || deputySlot.status !== 'loaded') {
    return IDLE_CLASSIFICATION;
  }
  if (chiefSlot.vector.epoch !== deputySlot.vector.epoch) {
    return IDLE_CLASSIFICATION;
  }
  return match(classify(chiefSlot.vector, deputySlot.vector, config), {
    ok: (phase): Classification => ({ status: 'ok', phase }),
    err: (error): Classification => ({ status: 'err', error }),
  });
}

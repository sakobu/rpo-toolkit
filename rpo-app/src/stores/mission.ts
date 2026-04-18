import { create } from 'zustand';
import { devtools } from 'zustand/middleware';
import { match } from '@railway-ts/pipelines/result';
import type { MissionPhase, WasmError } from 'rpo-wasm';

import { classify } from '../wasm/classify';
import { useScenario, type VehicleStateSlot } from './scenario';

export type Classification =
  | { status: 'idle' }
  | { status: 'ok'; phase: MissionPhase }
  | { status: 'err'; error: WasmError };

export const IDLE_CLASSIFICATION: Classification = { status: 'idle' };

type MissionState = {
  classification: Classification;
};

export const useMission = create<MissionState>()(
  devtools(
    () => ({
      classification: IDLE_CLASSIFICATION,
    }),
    { name: 'mission', enabled: import.meta.env.DEV },
  ),
);

useScenario.subscribe((state, prev) => {
  if (state.chiefState === prev.chiefState && state.deputyState === prev.deputyState) return;
  useMission.setState(
    { classification: deriveClassification(state.chiefState, state.deputyState) },
    false,
    'deriveClassification',
  );
});

function deriveClassification(
  chiefSlot: VehicleStateSlot,
  deputySlot: VehicleStateSlot,
): Classification {
  if (chiefSlot.status !== 'loaded' || deputySlot.status !== 'loaded') {
    return IDLE_CLASSIFICATION;
  }
  if (chiefSlot.vector.epoch !== deputySlot.vector.epoch) {
    return IDLE_CLASSIFICATION;
  }
  return match(classify(chiefSlot.vector, deputySlot.vector), {
    ok: (phase): Classification => ({ status: 'ok', phase }),
    err: (error): Classification => ({ status: 'err', error }),
  });
}

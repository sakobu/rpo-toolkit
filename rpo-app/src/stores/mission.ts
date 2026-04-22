import { create } from 'zustand';
import { devtools, subscribeWithSelector } from 'zustand/middleware';
import { shallow } from 'zustand/shallow';

import { match } from '@railway-ts/pipelines/result';

import type { MissionPhase, ProximityConfig, SafetyRequirements, WasmError } from 'rpo-wasm';

import { ROE_THRESHOLD_DEFAULT } from '@/schemas/proximityConfig';
import { classify } from '@/wasm/classify';

import { useConfig, type VehicleStateSlot } from './configuration';

// ─── Types ─────────────────────────────────────────────────────────────────

export type Classification =
  | { status: 'idle' }
  | { status: 'ok'; phase: MissionPhase }
  | { status: 'err'; error: WasmError };

type MissionState = {
  classification: Classification;
  proximityConfig: ProximityConfig;
  safetyRequirements: SafetyRequirements | null;
  setProximityConfig: (config: ProximityConfig) => void;
  setSafetyRequirements: (r: SafetyRequirements | null) => void;
};

// ─── Defaults ──────────────────────────────────────────────────────────────

export const IDLE_CLASSIFICATION: Classification = { status: 'idle' };

const DEFAULT_PROXIMITY_CONFIG: ProximityConfig = { roe_threshold: ROE_THRESHOLD_DEFAULT };

// ─── Store ─────────────────────────────────────────────────────────────────

export const useMission = create<MissionState>()(
  subscribeWithSelector(
    devtools(
      (set) => ({
        classification: IDLE_CLASSIFICATION,
        proximityConfig: DEFAULT_PROXIMITY_CONFIG,
        safetyRequirements: null,
        setProximityConfig: (config) =>
          set({ proximityConfig: config }, false, 'setProximityConfig'),
        setSafetyRequirements: (r) =>
          set({ safetyRequirements: r }, false, 'setSafetyRequirements'),
      }),
      { name: 'mission', enabled: import.meta.env.DEV },
    ),
  ),
);

// ─── Derivation ────────────────────────────────────────────────────────────

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

// ─── Cross-store reactivity ────────────────────────────────────────────────
// Classification is derived, not owned. These subscriptions fire at module-import
// time and keep it in sync when either the config slots or the threshold change.
// `shallow` is required on the first subscribe because the selector returns a
// fresh tuple on every emit.

function recomputeClassification() {
  const { chiefState, deputyState } = useConfig.getState();
  const { proximityConfig } = useMission.getState();
  useMission.setState(
    { classification: deriveClassification(chiefState, deputyState, proximityConfig) },
    false,
    'deriveClassification',
  );
}

useConfig.subscribe((s) => [s.chiefState, s.deputyState] as const, recomputeClassification, {
  equalityFn: shallow,
});

useMission.subscribe((s) => s.proximityConfig.roe_threshold, recomputeClassification);

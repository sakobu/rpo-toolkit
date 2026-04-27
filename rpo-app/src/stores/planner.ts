import { create, type StateCreator } from 'zustand';
import { devtools, subscribeWithSelector } from 'zustand/middleware';
import { shallow } from 'zustand/shallow';

import { match } from '@railway-ts/pipelines/result';

import type {
  EnrichmentSuggestion,
  MissionPhase,
  ProximityConfig,
  SafetyRequirements,
  TransferFeasibility,
  TransferResult,
  WasmError,
} from 'rpo-wasm';

import { classify } from '@/wasm/classify';
import { getEngineConstants } from '@/wasm/constants';

import { useConfig, type VehicleStateSlot } from './configuration';

// ─── Types ─────────────────────────────────────────────────────────────────

export type Classification =
  | { status: 'idle' }
  | { status: 'ok'; phase: MissionPhase }
  | { status: 'err'; error: WasmError };

export type PhaseKey = 'xfr' | 'px' | 'val' | 'mc';
export type PhaseState = 'active' | 'done' | 'skipped' | 'idle' | 'locked';

export type PhaseRecord = {
  state: PhaseState;
  dimmedBy: PhaseKey[];
  resets?: string;
};

export const PHASE_KEYS: readonly PhaseKey[] = ['xfr', 'px', 'val', 'mc'] as const;

// TODO(phase2c-ui-followup): when `val` and `mc` views land, audit them for
// honest fallback labeling — same check applied to ProximityViewport in the
// 2026-04-23 phase2c UI follow-up. They must not display "safety applied"
// when `enrichment.perch.status === 'fallback'`.
export const PHASE_DEFINITIONS: Record<PhaseKey, { title: string; shortLabel: string }> = {
  xfr: { title: 'Transfer', shortLabel: 'XFR' },
  px: { title: 'Proximity ops', shortLabel: 'PX' },
  val: { title: 'Validate', shortLabel: 'VAL' },
  mc: { title: 'Monte Carlo', shortLabel: 'MC' },
};

type MissionSlice = {
  classification: Classification;
  // `null` until the user edits the threshold. Reads go through
  // `selectProximityConfig`, which substitutes the engine default so consumers
  // never have to null-check.
  proximityConfig: ProximityConfig | null;
  safetyRequirements: SafetyRequirements | null;
  transfer: TransferResult | null;
  enrichment: EnrichmentSuggestion | null;
  setProximityConfig: (config: ProximityConfig) => void;
  setSafetyRequirements: (r: SafetyRequirements | null) => void;
  setTransfer: (t: TransferResult, enrichment: EnrichmentSuggestion | null) => void;
  clearTransfer: () => void;
};

type PhaseSlice = {
  phases: Record<PhaseKey, PhaseRecord>;
  initFromClassification: (isProximity: boolean) => void;
  acceptTransfer: () => void;
  acceptProximity: () => void;
  editPhase: (key: PhaseKey) => void;
};

export type PlannerState = MissionSlice & PhaseSlice;

// ─── Defaults ──────────────────────────────────────────────────────────────

export const IDLE_CLASSIFICATION: Classification = { status: 'idle' };

const INITIAL_FAR_FIELD: Record<PhaseKey, PhaseRecord> = {
  xfr: { state: 'active', dimmedBy: [] },
  px: { state: 'locked', dimmedBy: ['xfr'] },
  val: { state: 'locked', dimmedBy: ['xfr', 'px'] },
  mc: { state: 'locked', dimmedBy: ['xfr', 'px'] },
};

const INITIAL_PROXIMITY: Record<PhaseKey, PhaseRecord> = {
  xfr: { state: 'skipped', dimmedBy: [] },
  px: { state: 'active', dimmedBy: [] },
  val: { state: 'idle', dimmedBy: ['px'] },
  mc: { state: 'idle', dimmedBy: ['px'] },
};

/// Derive the effective proximity config, substituting the engine-recommended
/// default when the user hasn't set one. Only safe to call post-WasmGate
/// (i.e. from React render/effects or user-triggered callbacks) because it
/// reaches into WASM via `getEngineConstants`.
export function selectProximityConfig(s: PlannerState): ProximityConfig {
  return s.proximityConfig ?? { roe_threshold: getEngineConstants().roe_threshold_default };
}

/// Returns the `SubSurface` feasibility variant when the active transfer's
/// conic dips below `MIN_PERIAPSIS_ALTITUDE_KM` (so consumers can both gate
/// on the predicate and read the periapsis altitude); otherwise `null`.
///
/// Returns `s.transfer.plan.transfer.feasibility` directly — a stable
/// reference into the store under zustand's `Object.is` selector equality.
/// Do not spread/clone, that would defeat memoization.
export function selectTransferSubSurface(s: PlannerState): TransferFeasibility | null {
  const feasibility = s.transfer?.plan.transfer?.feasibility;
  return feasibility?.kind === 'sub_surface' ? feasibility : null;
}

// ─── Slices ────────────────────────────────────────────────────────────────

const createMissionSlice: StateCreator<
  PlannerState,
  [['zustand/subscribeWithSelector', never], ['zustand/devtools', never]],
  [],
  MissionSlice
> = (set) => ({
  classification: IDLE_CLASSIFICATION,
  proximityConfig: null,
  safetyRequirements: null,
  transfer: null,
  enrichment: null,
  setProximityConfig: (config) => set({ proximityConfig: config }, false, 'setProximityConfig'),
  setSafetyRequirements: (r) => set({ safetyRequirements: r }, false, 'setSafetyRequirements'),
  setTransfer: (t, enrichment) => set({ transfer: t, enrichment }, false, 'setTransfer'),
  clearTransfer: () => set({ transfer: null, enrichment: null }, false, 'clearTransfer'),
});

const createPhaseSlice: StateCreator<
  PlannerState,
  [['zustand/subscribeWithSelector', never], ['zustand/devtools', never]],
  [],
  PhaseSlice
> = (set, get) => ({
  phases: INITIAL_FAR_FIELD,
  initFromClassification: (isProximity) => {
    // Reseeding phases is the moment a prior transfer becomes stale. Cascade
    // directly instead of relying on a separate subscription.
    const prevXfrState = get().phases.xfr.state;
    const next = isProximity ? INITIAL_PROXIMITY : INITIAL_FAR_FIELD;
    set({ phases: next }, false, 'initFromClassification');
    if (prevXfrState === 'done' && next.xfr.state === 'active') {
      get().clearTransfer();
    }
  },
  acceptTransfer: () =>
    set(
      (s) => {
        // Mirror rpo-core::pipeline::apply_perch_enrichment: when an
        // enriched suggestion is available, commit it into the transfer's
        // perch_roe so downstream (proximity view, waypoint planning) sees
        // the enriched values. Baseline / fallback statuses leave perch_roe
        // unchanged — same semantics as the Rust helper.
        const enrichedTransfer =
          s.transfer && s.enrichment?.perch.status === 'enriched'
            ? {
                ...s.transfer,
                plan: { ...s.transfer.plan, perch_roe: s.enrichment.perch.roe },
              }
            : s.transfer;
        return {
          transfer: enrichedTransfer,
          phases: {
            ...s.phases,
            xfr: {
              state: 'done',
              dimmedBy: [],
              resets: 'all waypoints, propagation',
            },
            px: { state: 'active', dimmedBy: ['xfr'] },
          },
        };
      },
      false,
      'acceptTransfer',
    ),
  acceptProximity: () =>
    set(
      (s) => ({
        phases: {
          ...s.phases,
          px: { ...s.phases.px, state: 'done' },
          val: { state: 'idle', dimmedBy: ['xfr', 'px'] },
          mc: { state: 'idle', dimmedBy: ['xfr', 'px'] },
        },
      }),
      false,
      'acceptProximity',
    ),
  editPhase: (key) => {
    const prevXfrState = get().phases.xfr.state;
    switch (key) {
      case 'xfr': {
        set({ phases: INITIAL_FAR_FIELD }, false, 'editPhase/xfr');
        if (prevXfrState === 'done') {
          get().clearTransfer();
        }
        return;
      }
      case 'px': {
        set(
          (s) => ({
            phases: {
              ...s.phases,
              px: { state: 'active', dimmedBy: ['xfr'] },
              val: { state: 'idle', dimmedBy: ['xfr', 'px'] },
              mc: { state: 'idle', dimmedBy: ['xfr', 'px'] },
            },
          }),
          false,
          'editPhase/px',
        );
        return;
      }
      case 'val':
      case 'mc':
        return;
      default: {
        const exhaustive: never = key;
        return exhaustive;
      }
    }
  },
});

// ─── Store ─────────────────────────────────────────────────────────────────

export const usePlanner = create<PlannerState>()(
  subscribeWithSelector(
    devtools(
      (...a) => ({
        ...createMissionSlice(...a),
        ...createPhaseSlice(...a),
      }),
      { name: 'planner', enabled: import.meta.env.DEV },
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
// Classification is derived, not owned. These subscriptions only fire on
// subsequent state changes, not at module load — so the `getEngineConstants`
// call inside `selectProximityConfig` runs post-WasmGate.
//
// `recomputeClassification` also folds in the phases reseed that used to live
// in the separate `bindPhasesToMission` module-level subscription — whenever
// classification resolves to `ok`, the phase sequence is the function of
// whether we're far-field or proximity, so it's set here.

function recomputeClassification() {
  const { chiefState, deputyState } = useConfig.getState();
  const config = selectProximityConfig(usePlanner.getState());
  const classification = deriveClassification(chiefState, deputyState, config);
  usePlanner.setState({ classification }, false, 'deriveClassification');
  if (classification.status === 'ok') {
    const isProximity = 'proximity' in classification.phase;
    usePlanner.getState().initFromClassification(isProximity);
  }
}

useConfig.subscribe((s) => [s.chiefState, s.deputyState] as const, recomputeClassification, {
  equalityFn: shallow,
});

usePlanner.subscribe((s) => s.proximityConfig?.roe_threshold, recomputeClassification);

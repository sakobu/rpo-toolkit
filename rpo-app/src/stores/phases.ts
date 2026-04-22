import { create } from 'zustand';
import { devtools } from 'zustand/middleware';

import { useMission } from './mission';

export type PhaseKey = 'xfr' | 'px' | 'val' | 'mc';
export type PhaseState = 'active' | 'done' | 'skipped' | 'idle' | 'locked';

export type PhaseRecord = {
  state: PhaseState;
  dimmedBy: PhaseKey[];
  resets?: string;
};

export const PHASE_KEYS: readonly PhaseKey[] = ['xfr', 'px', 'val', 'mc'] as const;

export const PHASE_DEFINITIONS: Record<PhaseKey, { title: string; shortLabel: string }> = {
  xfr: { title: 'Transfer', shortLabel: 'XFR' },
  px: { title: 'Proximity ops', shortLabel: 'PX' },
  val: { title: 'Validate', shortLabel: 'VAL' },
  mc: { title: 'Monte Carlo', shortLabel: 'MC' },
};

type PhasesState = {
  phases: Record<PhaseKey, PhaseRecord>;
  initFromClassification: (isProximity: boolean) => void;
  acceptTransfer: () => void;
  acceptProximity: () => void;
  editPhase: (key: PhaseKey) => void;
};

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

export const usePhases = create<PhasesState>()(
  devtools(
    (set) => ({
      phases: INITIAL_FAR_FIELD,
      initFromClassification: (isProximity) =>
        set(
          { phases: isProximity ? INITIAL_PROXIMITY : INITIAL_FAR_FIELD },
          false,
          'initFromClassification',
        ),
      acceptTransfer: () =>
        set(
          (s) => ({
            phases: {
              ...s.phases,
              xfr: {
                state: 'done',
                dimmedBy: [],
                resets: 'all waypoints, propagation',
              },
              px: { state: 'active', dimmedBy: ['xfr'] },
            },
          }),
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
      editPhase: (key) =>
        set(
          (s) => {
            switch (key) {
              case 'xfr':
                return { phases: INITIAL_FAR_FIELD };
              case 'px':
                return {
                  phases: {
                    ...s.phases,
                    px: { state: 'active', dimmedBy: ['xfr'] },
                    val: { state: 'idle', dimmedBy: ['xfr', 'px'] },
                    mc: { state: 'idle', dimmedBy: ['xfr', 'px'] },
                  },
                };
              case 'val':
              case 'mc':
                return s;
              default: {
                const exhaustive: never = key;
                return exhaustive;
              }
            }
          },
          false,
          'editPhase',
        ),
    }),
    { name: 'phases', enabled: import.meta.env.DEV },
  ),
);

// Reseed phase states whenever classification flips far-field ↔ proximity. Invoked
// once from the app entry point; do not call at module scope (breaks testing).
export function bindPhasesToMission(): () => void {
  return useMission.subscribe((state) => {
    const { classification } = state;
    if (classification.status === 'ok') {
      const isProximity = 'proximity' in classification.phase;
      usePhases.getState().initFromClassification(isProximity);
    }
  });
}

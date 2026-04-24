import { useMemo } from 'react';

import { match } from '@railway-ts/pipelines/result';

import { useConfig } from '@/stores/configuration';
import { getEngineConstants } from '@/wasm/constants';
import { stateToKeplerian } from '@/wasm/elements';

// V-bar/R-bar achievable separation cap at this orbit:
//   a · linearization_perturbation_bound / sqrt(2)
// Engine asymmetry: Custom perch path is the looser `a · BOUND`; the cap
// reported here is the tighter V-bar/R-bar cap because that's the default
// perch mode. Returns `null` while chief is unloaded or semi-major axis can't
// be recovered — callers render "cap unknown" rather than gating UI.
export function useAchievableCap(): number | null {
  const chiefState = useConfig((s) => s.chiefState);

  return useMemo<number | null>(() => {
    if (chiefState.status !== 'loaded') return null;
    const { linearization_perturbation_bound: bound } = getEngineConstants();
    return match(stateToKeplerian(chiefState.vector), {
      ok: (kep) =>
        Number.isFinite(kep.a_km) && kep.a_km > 0 ? (kep.a_km * bound) / Math.SQRT2 : null,
      err: () => null,
    });
  }, [chiefState]);
}

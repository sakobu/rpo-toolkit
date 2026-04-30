import { useMemo } from 'react';

import { match } from '@railway-ts/pipelines/result';

import { useConfig } from '@/stores/configuration';
import { getEngineConstants } from '@/wasm/constants';
import { stateToKeplerian } from '@/wasm/elements';

export function useAchievableCap(): number | null {
  const chiefState = useConfig((s) => s.chiefState);

  return useMemo<number | null>(() => {
    if (chiefState.status !== 'loaded') return null;
    const { linearization_perturbation_bound: bound, vbar_rbar_perturbation_ratio: ratio } =
      getEngineConstants();
    return match(stateToKeplerian(chiefState.vector), {
      ok: (kep) => (Number.isFinite(kep.a_km) && kep.a_km > 0 ? (kep.a_km * bound) / ratio : null),
      err: () => null,
    });
  }, [chiefState]);
}

import { useMemo } from 'react';

import { match } from '@railway-ts/pipelines/result';

import { useConfig } from '@/stores/configuration';
import { getEngineConstants } from '@/wasm/constants';
import { stateToKeplerian } from '@/wasm/elements';

// Achievable Min R/C separation cap at this orbit, used as the slider's
// hard upper bound:
//
//   cap_km = a · linearization_perturbation_bound / vbar_rbar_perturbation_ratio
//
// Both factors come from `getEngineConstants()` so this file holds no
// physics literals. The ratio is locked in by the engine's
// `vbar_rbar_parallel_perturbation_norm_equals_sqrt3_dmin` test (see
// `rpo-core/src/mission/formation/safety_envelope.rs`), which proves that
// `||Δ|| = sqrt(3) · d_min` for V-bar/R-bar parallel zero-baseline perches.
//
// Custom perches with non-zero baseline e/i have a geometry-dependent
// linearization bound. For `dex/dey/dix/diy` already pointing toward the
// safe direction the perturbation is smaller than `sqrt(3) · d_min`; for
// the worst opposing geometry it can be larger. Using the V-bar/R-bar
// ratio as the slider cap is therefore conservative for the common case
// and an approximation for Custom — the engine remains the authoritative
// bound and will return `FormationDesignError::SeparationUnachievable`
// for any geometry where this approximation is unsafe.
//
// Returns `null` while chief is unloaded or semi-major axis can't be
// recovered — the form disables the slider in that state.
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

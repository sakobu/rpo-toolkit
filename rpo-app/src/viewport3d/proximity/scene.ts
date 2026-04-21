import { match, type Result } from '@railway-ts/pipelines/result';

import type { WasmError } from 'rpo-wasm';

import type { StateVectorInput } from '@/schemas/stateVector';
import type { Vec3 } from '@/viewport3d/types';
import { moonPositionEciKm, sunPositionEciKm } from '@/wasm/ephemeris';
import { eciToRicDcm, eciToRicRelativeState } from '@/wasm/ric';

import { deputyDMaxM, fallbackScale, pickScale, type ProximityScale } from './scale';

type CelestialDirsRic = {
  earth: Vec3 | null;
  sun: Vec3 | null;
  moon: Vec3 | null;
};

const KM_TO_M = 1000;

export function computeDeputyPositionRicM(
  chief: StateVectorInput | null,
  deputy: StateVectorInput | null,
): Vec3 | null {
  if (!chief || !deputy) return null;
  return match(eciToRicRelativeState(chief, deputy), {
    ok: (s) => [
      s.position_ric_km[0] * KM_TO_M,
      s.position_ric_km[1] * KM_TO_M,
      s.position_ric_km[2] * KM_TO_M,
    ],
    err: () => null,
  });
}

export function computeScale(deputyRicM: Vec3 | null): ProximityScale {
  return deputyRicM ? pickScale(deputyDMaxM(deputyRicM)) : fallbackScale();
}

// Earth direction is `-r_chief_eci` rotated into RIC. Sun and Moon use
// their ECI positions directly: chief-ECI offset is ≤ 10⁻⁴ of heliocentric
// / lunar distance, negligible for a unit direction indicator.
export function computeCelestialDirsRic(chief: StateVectorInput | null): CelestialDirsRic {
  if (!chief) return { earth: null, sun: null, moon: null };
  return match(eciToRicDcm(chief), {
    ok: (dcm) => ({
      earth: applyDcmUnit(dcm, negate(chief.position_eci_km)),
      sun: directionFromWasm(dcm, sunPositionEciKm(chief.epoch)),
      moon: directionFromWasm(dcm, moonPositionEciKm(chief.epoch)),
    }),
    err: () => ({ earth: null, sun: null, moon: null }),
  });
}

function negate(v: Vec3): Vec3 {
  return [-v[0], -v[1], -v[2]];
}

// Apply row-major 3×3 DCM to a vector and normalize to unit length.
// Rows of `m` are R̂, Î, Ĉ expressed in ECI (per `eci_to_ric_dcm`).
function applyDcmUnit(m: [Vec3, Vec3, Vec3], v: Vec3): Vec3 {
  const r = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
  const i = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
  const c = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
  const n = Math.hypot(r, i, c);
  return n === 0 ? [0, 0, 0] : [r / n, i / n, c / n];
}

function directionFromWasm(dcm: [Vec3, Vec3, Vec3], r: Result<Vec3, WasmError>): Vec3 | null {
  return match(r, {
    ok: (eci) => applyDcmUnit(dcm, eci),
    err: () => null,
  });
}

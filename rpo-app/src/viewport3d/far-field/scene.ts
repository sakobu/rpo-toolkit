import { match } from '@railway-ts/pipelines/result';

import type { StateVectorInput } from '@/schemas/stateVector';
import type { Vec3 } from '@/viewport3d/types';
import { earthRotationAngleRad, sunPositionEciKm } from '@/wasm/earth';

import { EARTH_TEXTURE_LON_OFFSET_RAD } from './constants';
import { eciKmToScenePosition, scaleToSceneFar } from './coordinates';

export const FALLBACK_LIGHT_POS: Vec3 = scaleToSceneFar([10, 5, 10]);

// ~3× Earth radius, sized to fill ~2/3 of frame.
export const CAMERA_DISTANCE_SCENE = 6;
export const DEFAULT_CAMERA_POS: Vec3 = [0, 0, CAMERA_DISTANCE_SCENE];

export function computeEraRad(epoch: string): number {
  return match(earthRotationAngleRad(epoch), {
    ok: (v) => v + EARTH_TEXTURE_LON_OFFSET_RAD,
    err: () => 0,
  });
}

export function computeSunScenePos(epoch: string): Vec3 {
  return match(sunPositionEciKm(epoch), {
    ok: (eciKm) => scaleToSceneFar(eciKm),
    err: () => FALLBACK_LIGHT_POS,
  });
}

export function computeCameraPos(
  chief: StateVectorInput | null,
  deputy: StateVectorInput | null,
): Vec3 {
  const sources = [chief, deputy].filter((v): v is StateVectorInput => v !== null);
  if (sources.length === 0) return DEFAULT_CAMERA_POS;
  const mean: Vec3 = [0, 0, 0];
  for (const v of sources) {
    const scenePos = eciKmToScenePosition(v.position_eci_km);
    mean[0] += scenePos[0];
    mean[1] += scenePos[1];
    mean[2] += scenePos[2];
  }
  const norm = Math.hypot(mean[0], mean[1], mean[2]);
  if (norm === 0) return DEFAULT_CAMERA_POS;
  const s = CAMERA_DISTANCE_SCENE / norm;
  return [mean[0] * s, mean[1] * s, mean[2] * s];
}

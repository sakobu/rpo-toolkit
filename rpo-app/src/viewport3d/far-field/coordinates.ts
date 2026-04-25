import type { Vec3 } from '@/viewport3d/types';
import { getEngineConstants } from '@/wasm/constants';

import { EARTH_RADIUS } from './constants';

// Getter (not a module-load constant) because the scale factor depends on
// engine constants from WASM, which aren't available at module load.
let cachedSceneUnitsPerKm: number | null = null;
function sceneUnitsPerKm(): number {
  if (cachedSceneUnitsPerKm === null) {
    cachedSceneUnitsPerKm = EARTH_RADIUS / getEngineConstants().earth_radius_km;
  }
  return cachedSceneUnitsPerKm;
}

// ECI (right-handed, Z toward north pole) → Three (Y-up, right-handed).
// Map: x→X, z→Y, y→-Z. Preserves chirality and orients Earth's rotation axis
// along scene Y, matching how aerospace viz tools (STK, Cesium) present ECI.
export function eciDirToScene([x, y, z]: Vec3): Vec3 {
  return [x, z, -y];
}

export function eciKmToScenePosition(r_eci_km: Vec3): Vec3 {
  const [x, y, z] = eciDirToScene(r_eci_km);
  const s = sceneUnitsPerKm();
  return [x * s, y * s, z * s];
}

// Same axis swap as eciKmToScenePosition; caller wraps in ERA-rotated group.
export function ecefKmToScenePosition(r_ecef_km: Vec3): Vec3 {
  return eciKmToScenePosition(r_ecef_km);
}

// Normalize, apply scene axis swap, and place on a sphere of `distance` scene
// units — used as a parallel-light source point.
export function scaleToSceneFar(r_eci: Vec3, distance = 20): Vec3 {
  const norm = Math.hypot(r_eci[0], r_eci[1], r_eci[2]);
  if (norm === 0) return [distance, 0, 0];
  const [x, y, z] = eciDirToScene(r_eci);
  const s = distance / norm;
  return [x * s, y * s, z * s];
}

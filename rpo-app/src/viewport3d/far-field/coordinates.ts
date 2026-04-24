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
export function eciKmToScenePosition([x_km, y_km, z_km]: Vec3): Vec3 {
  const s = sceneUnitsPerKm();
  return [x_km * s, z_km * s, -y_km * s];
}

// Same axis swap as eciKmToScenePosition; caller wraps in ERA-rotated group.
export function ecefKmToScenePosition(r_ecef_km: Vec3): Vec3 {
  return eciKmToScenePosition(r_ecef_km);
}

// Normalize, apply scene axis swap, and place on a sphere of `distance` scene
// units — used as a parallel-light source point.
export function scaleToSceneFar([x, y, z]: Vec3, distance = 20): Vec3 {
  const norm = Math.hypot(x, y, z);
  if (norm === 0) return [distance, 0, 0];
  const s = distance / norm;
  return [x * s, z * s, -y * s];
}

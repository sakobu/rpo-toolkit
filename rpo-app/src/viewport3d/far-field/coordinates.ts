import type { Vec3 } from '@/viewport3d/types';

import { EARTH_RADIUS } from './constants';

// WGS84 equatorial radius. Must stay consistent with whatever physical radius
// is represented by EARTH_RADIUS scene units.
export const EARTH_RADIUS_KM = 6378.137;

export const SCENE_UNITS_PER_KM = EARTH_RADIUS / EARTH_RADIUS_KM;

// ECI (right-handed, Z toward north pole) → Three (Y-up, right-handed).
// Map: x→X, z→Y, y→-Z. Preserves chirality and orients Earth's rotation axis
// along scene Y, matching how aerospace viz tools (STK, Cesium) present ECI.
export function eciKmToScenePosition([x_km, y_km, z_km]: Vec3): Vec3 {
  return [x_km * SCENE_UNITS_PER_KM, z_km * SCENE_UNITS_PER_KM, -y_km * SCENE_UNITS_PER_KM];
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

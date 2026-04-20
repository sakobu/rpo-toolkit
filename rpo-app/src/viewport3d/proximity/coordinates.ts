import type { Vector3 } from 'three';

import type { Vec3 } from '@/viewport3d/types';

// RIC → Three.js scene position: R → Y (up), I → X (right), C → Z (depth).
export function ricToPosition([r, i, c]: Vec3): Vec3 {
  return [i, r, c];
}

export function threeToRicPosition(v: Vector3): Vec3 {
  return [v.y, v.x, v.z];
}

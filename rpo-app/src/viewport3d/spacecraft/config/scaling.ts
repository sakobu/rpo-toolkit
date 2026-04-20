import { MAX_BODY_SCALE, MIN_BODY_SCALE, REF_DRAG_AREA_M2, REF_SRP_AREA_M2 } from './constants';

const MIN_AREA_M2 = 1e-4;

export function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
}

// Body scale follows a cube-root relationship with drag area so a 10x drag area
// produces a ~2.15x linear dimension change (matches physical surface scaling).
export function bodyScaleFromDrag(dragAreaM2: number): number {
  const raw = Math.cbrt(Math.max(dragAreaM2, MIN_AREA_M2) / REF_DRAG_AREA_M2);
  return clamp(raw, MIN_BODY_SCALE, MAX_BODY_SCALE);
}

// Panel height scales as sqrt of SRP area (SRP area is a surface, height is linear).
export function panelScaleFromSrp(srpAreaM2: number): number {
  return Math.sqrt(Math.max(srpAreaM2, MIN_AREA_M2) / REF_SRP_AREA_M2);
}

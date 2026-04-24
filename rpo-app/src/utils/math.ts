export function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
}

export function vectorMag(v: readonly [number, number, number]): number {
  return Math.hypot(...v);
}

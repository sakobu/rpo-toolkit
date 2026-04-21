import type { Vec3 } from '@/viewport3d/types';

// Ruler-style scale ladder, 1-2-5 decade progression in meters.
// Covers LEO docking (1–10 m) through GEO station-keeping (200–500 km).
const SCALE_LADDER_M: readonly number[] = [
  1, 2, 5, 10, 20, 50, 100, 200, 500, 1_000, 2_000, 5_000, 10_000, 20_000, 50_000, 100_000, 200_000,
  500_000,
] as const;

// ~4 sections across the grid half-width (8 across the full grid). At this
// target, the deputy sits at ~50–70% of grid half when d_max is just under
// the chosen section — deputy well-contained with breathing room.
const TARGET_SECTIONS_PER_HALF = 4;

// Hysteresis factor: step the ladder up only when d_max exceeds the next
// threshold by this margin. Prevents flicker when float recomputes nudge
// d_max across a decade boundary.
const SCALE_STEP_UP_MARGIN = 1.15;

// Chief-only fallback section size. Matches the legacy static scale.
const DEFAULT_SECTION_M = 50;

// Spacecraft + label sizing baseline. At this grid extent, Spacecraft
// `scale = 1` and SpacecraftLabel default `fontSize = 5` look correct —
// the legacy demo we calibrate against. Both values scale linearly with
// `extentM / SPACECRAFT_REFERENCE_EXTENT_M` so the same visual ratio
// (~7.5% wingspan, ~1.25% label height) holds at every grid scale.
const SPACECRAFT_REFERENCE_EXTENT_M = 400;
const LABEL_FONT_NATIVE_M = 5;

export type ProximityScale = {
  sectionM: number; // one section of the grid, meters
  extentM: number; // full grid side length
  cameraDistanceM: number;
  farPlaneM: number;
  nearPlaneM: number;
  orbitMinM: number;
  orbitMaxM: number;
  spacecraftScale: number; // unitless multiplier for Spacecraft `scale` prop
  labelFontSizeM: number; // label font size in scene meters
};

// Axis-wise max of deputy RIC position (meters). The grid is square, so
// whichever axis component is largest is what frames the viewport.
export function deputyDMaxM(ricM: Vec3): number {
  return Math.max(Math.abs(ricM[0]), Math.abs(ricM[1]), Math.abs(ricM[2]));
}

export function pickScale(dMaxM: number): ProximityScale {
  const sectionM =
    SCALE_LADDER_M.find((s) => s * TARGET_SECTIONS_PER_HALF >= dMaxM * SCALE_STEP_UP_MARGIN) ??
    SCALE_LADDER_M[SCALE_LADDER_M.length - 1];

  const extentM = sectionM * TARGET_SECTIONS_PER_HALF * 2;
  const cameraDistanceM = extentM * 1.25;
  const spacecraftScale = extentM / SPACECRAFT_REFERENCE_EXTENT_M;
  return {
    sectionM,
    extentM,
    cameraDistanceM,
    farPlaneM: cameraDistanceM * 20,
    nearPlaneM: Math.max(sectionM / 50, 0.01),
    orbitMinM: sectionM * 0.4,
    orbitMaxM: cameraDistanceM * 4,
    spacecraftScale,
    labelFontSizeM: LABEL_FONT_NATIVE_M * spacecraftScale,
  };
}

export function fallbackScale(): ProximityScale {
  return pickScale(DEFAULT_SECTION_M * TARGET_SECTIONS_PER_HALF - 1);
}

// Scene units are meters.

export const GRID_COLORS = {
  cell: '#1a3d5c',
  section: '#2a5478',
} as const;

export const GRID_LABEL_COLOR = '#4a6a8a';

export const AXES = {
  R: { color: '#20df80', label: 'R' },
  I: { color: '#ff2060', label: 'I' },
  C: { color: '#2080ff', label: 'C' },
} as const;

// Corner axis gizmo (drei GizmoHelper). Values are in the gizmo's local viewport,
// not world meters.
export const AXES_GIZMO = {
  lineLength: 0.8,
  lineWidth: 3,
  sphereRadius: 0.15,
  sphereSegments: 16,
  labelFontSize: 0.15,
  labelColor: '#000000',
  scale: 55,
  margin: [80, 80],
} as const;

export const ORBIT_CONTROLS = {
  enableDamping: true,
} as const;

// Celestial leader-line tints. Hex mirrors @theme tokens in src/index.css:
//   EARTH → --color-viz-nominal
//   SUN   → --color-viz-numerical
//   MOON  → --color-text-muted
export const CELESTIAL_TINTS = {
  earth: '#4ade80',
  sun: '#fb923c',
  moon: '#9099ad',
} as const;

// Celestial leader-line styling. Length and opacity both fall off with
// log10(distance_km), so a chief in LEO renders Earth as a short bright line
// and the Sun as a long faint line — the order encodes proximity at a glance.
// Range bracketed to comfortably cover LEO chief altitude (~7e3 km) through
// the Sun (~1.5e8 km) with margin on both ends. Length ratios are fractions
// of the full grid side (extentM), so lengthRatioFar must stay ≲ 0.5 to keep
// the farthest tip inside the visible ±extentM/2 viewport.
export const CELESTIAL_INDICATOR = {
  lineWidth: 0.5,
  iconSizePx: 20,
  iconStrokeWidth: 1.5,
  labelOffsetRatio: 0,
  logKmMin: 3.5,
  logKmMax: 8.5,
  lengthRatioNear: 0.15,
  lengthRatioFar: 0.5,
  opacityNear: 0.85,
  opacityFar: 0.35,
  // Icon sits one notch brighter than its leader line so far bodies (Sun) don't
  // fade to invisible at the line's opacityFar.
  iconOpacityBias: 0.2,
} as const;

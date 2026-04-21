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

// Fraction of grid extent at which celestial indicators sit (unit direction
// scaled to this radius). Keeps tips inside the camera frame.
export const CELESTIAL_EDGE_RATIO = 0.45;

// Celestial leader-line styling. Thin + translucent so the line reads as a
// hint, not a primary visual element. Icon sits just past the line tip;
// with drei <Html center>, the icon's center is anchored at the offset point.
export const CELESTIAL_INDICATOR = {
  lineWidth: 0.5,
  lineOpacity: 0.6,
  iconSizePx: 20,
  iconStrokeWidth: 1.5,
  labelOffsetRatio: 0.02,
} as const;

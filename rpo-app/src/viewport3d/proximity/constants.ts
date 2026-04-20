import type { Vec3 } from '@/viewport3d/types';

// Scene units are meters.

export const GRID_SIZE_M = 400;
export const GRID_CELL_SIZE_M = 10;
export const GRID_SECTION_SIZE_M = 50;

export const GRID_COLORS = {
  cell: '#1a3d5c',
  section: '#2a5478',
} as const;

export const GRID_LABEL_COLOR = '#4a6a8a';
export const GRID_LABEL_FONT_SIZE_M = 4;
export const GRID_LABEL_OFFSET_M = 6;

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

// Camera sits on pure +C (+Z), looking at origin, Three's default +Y up.
// Result: R points up, I points right, C points out of the screen toward viewer.
export const CAMERA = {
  fov: 50,
  position: [0, 0, 500],
  near: 1,
  far: 10000,
} as const;

export const ORBIT_CONTROLS = {
  minDistance: 20,
  maxDistance: 2000,
  enableDamping: true,
};

export const DEMO_DEPUTY_OFFSET_RIC_M: Vec3 = [0, -200, 0];

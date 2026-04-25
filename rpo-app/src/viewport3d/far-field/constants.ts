import { resolveColor } from '@/utils/cssVars';

export const EARTH_RADIUS = 2;
export const EARTH_SEGMENTS = 64;

// Icon scale for spacecraft rendered in the far-field scene. Physical size is
// invisible at ~3189 km per scene unit, so this is a UX value. At 0.003 the
// total spacecraft extent (panels included) is ~0.09 scene units ≈ 290 km —
// visible but small next to Earth (radius 2 scene units).
export const FARFIELD_SPACECRAFT_SCALE = 0.003;

// Label text size and offset in scene units — independent of the spacecraft
// scale so the label stays readable even when the icon shrinks.
export const FARFIELD_LABEL_FONT_SIZE = 0.05;
export const FARFIELD_LABEL_OFFSET_Y = -0.04;

export const EARTH_MATERIAL = {
  normalScale: [0.25, 0.25] as const,
  shininess: 15,
  specular: 0x2d4ea0,
} as const;

export const ATMOSPHERE = {
  color: '#88a6ff',
  opacity: 0.2,
  scaleFactor: 1 + 100 / 6378.1,
} as const;

// Trajectory palette: chief/deputy reuse the spacecraft accent tokens so the
// 3D viz matches the HTML readouts; the Lambert arc uses the numerical token
// because it is produced by the nyx (numerical) engine, not WASM (analytical).
export const CHIEF_ORBIT_COLOR = resolveColor('--color-viz-free-drift', '#f43f5e');
export const DEPUTY_ORBIT_COLOR = resolveColor('--color-viz-analytical', '#60a5fa');
export const TRANSFER_ARC_COLOR = resolveColor('--color-viz-numerical', '#fb923c');

export const TEXTURE_PATHS = {
  color: '/textures/earth/earth_atmos.jpg',
  normal: '/textures/earth/earth_normal.jpg',
  specular: '/textures/earth/earth_specular.jpg',
} as const;

// Offset to absorb Three.js SphereGeometry UV seam shift if a future texture
// swap introduces one; keep at 0 otherwise.
export const EARTH_TEXTURE_LON_OFFSET_RAD = 0;

// Sample count for chief/deputy orbit polylines (~2.8°/segment, visually
// smooth). The server-side Lambert arc uses LAMBERT_ARC_SAMPLES (256) and is
// pre-densified — these client-side polylines need fewer points because the
// orbits are simple ellipses, not partial-arc transfers.
export const ORBIT_POLYLINE_SAMPLES = 128;

// Trajectory line widths (drei Line, pixels). Thin lines avoid masking
// spacecraft meshes at LEO altitudes where the orbit hugs Earth's limb.
export const ORBIT_LINE_WIDTH_PX = 1.0;
export const ARC_LINE_WIDTH_PX = 1.0;

// Endpoint dot: scene units. Earth radius = 2 scene units, so 0.025 ≈ 80 km
// physical — readable without overpowering the arc.
export const ARC_ENDPOINT_DOT_RADIUS = 0.025;
export const ARC_ENDPOINT_DOT_SEGMENTS = 32;

// Δv chevron: a fixed-length cone glyph at each burn point, pointing along
// Δv. Magnitude is reserved for the side-panel readout — the chevron only
// communicates direction.
export const DV_CHEVRON_LENGTH = 0.09;
export const DV_CHEVRON_RADIUS = 0.025;
export const DV_CHEVRON_RADIAL_SEGMENTS = 16;

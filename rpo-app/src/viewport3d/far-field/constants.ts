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

export const TEXTURE_PATHS = {
  color: '/textures/earth/earth_atmos.jpg',
  normal: '/textures/earth/earth_normal.jpg',
  specular: '/textures/earth/earth_specular.jpg',
} as const;

// Offset to absorb Three.js SphereGeometry UV seam shift if a future texture
// swap introduces one; keep at 0 otherwise.
export const EARTH_TEXTURE_LON_OFFSET_RAD = 0;

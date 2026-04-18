export const EARTH_RADIUS = 2;
export const EARTH_SEGMENTS = 64;

export const EARTH_MATERIAL = {
  normalScale: [0.25, 0.25] as const,
  shininess: 15,
  specular: 0x2d4ea0,
} as const;

export const ECI_ATMOSPHERE = {
  color: '#88a6ff',
  opacity: 0.2,
  scaleFactor: 1 + 100 / 6378.1,
} as const;

export const TEXTURE_PATHS = {
  color: '/textures/earth/earth_atmos.jpg',
  normal: '/textures/earth/earth_normal.jpg',
  specular: '/textures/earth/earth_specular.jpg',
} as const;

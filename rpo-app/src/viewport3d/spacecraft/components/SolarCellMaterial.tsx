import { DoubleSide, ShaderMaterial } from 'three';

const VERTEX_SHADER = /* glsl */ `
varying vec2 vUv;
void main() {
  vUv = uv;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`;

// Unlit on purpose: solar panels in space face direct sun, so we fake the
// dark-silicon + busbar look rather than rely on a scene light that may not
// hit the correct face. fwidth()-based AA keeps gridlines readable from
// thumbnail (~60 px) to proximity zoom.
const FRAGMENT_SHADER = /* glsl */ `
varying vec2 vUv;

const vec3 CELL_COLOR     = vec3(0.04, 0.10, 0.20);
const vec3 GRID_COLOR     = vec3(0.01, 0.01, 0.01);
const vec3 BUSBAR_COLOR   = vec3(0.68, 0.68, 0.72);
const vec3 SUBSTRATE_COLOR = vec3(0.42, 0.42, 0.44);
const vec3 OUTER_GRID_COLOR = vec3(0.05);
const vec2 CELLS          = vec2(6.0, 4.0);
const float GRID_WIDTH    = 0.04;
const float BUSBAR_WIDTH  = 0.018;
const float BUSBAR_MIX    = 0.70;
const float OUTER_GRID_WIDTH = 0.012;

void main() {
  if (!gl_FrontFacing) {
    gl_FragColor = vec4(SUBSTRATE_COLOR, 1.0);
    return;
  }

  vec2 uvCells = vUv * CELLS;
  vec2 f = fract(uvCells);
  vec2 eps = fwidth(uvCells);

  vec2 d = min(f, 1.0 - f);
  float gx = 1.0 - smoothstep(GRID_WIDTH - eps.x, GRID_WIDTH + eps.x, d.x);
  float gy = 1.0 - smoothstep(GRID_WIDTH - eps.y, GRID_WIDTH + eps.y, d.y);
  float grid = max(gx, gy);

  float busbarDist = abs(f.y - 0.5);
  float busbar = 1.0 - smoothstep(BUSBAR_WIDTH - eps.y, BUSBAR_WIDTH + eps.y, busbarDist);

  vec3 col = CELL_COLOR;
  col = mix(col, GRID_COLOR, grid);
  col = mix(col, BUSBAR_COLOR, busbar * BUSBAR_MIX);

  // Sub-panel grid drawn last so tile boundaries mask cells/busbar at the seam.
  float distX = abs(vUv.x - 0.5);
  float distY = abs(vUv.y - 0.5);
  vec2 outerEps = fwidth(vUv);
  float outerGx = 1.0 - smoothstep(OUTER_GRID_WIDTH - outerEps.x, OUTER_GRID_WIDTH + outerEps.x, distX);
  float outerGy = 1.0 - smoothstep(OUTER_GRID_WIDTH - outerEps.y, OUTER_GRID_WIDTH + outerEps.y, distY);
  float outerGrid = max(outerGx, outerGy);
  col = mix(col, OUTER_GRID_COLOR, outerGrid);

  gl_FragColor = vec4(col, 1.0);
}
`;

// Module-level singleton: no instance state, no props, shared across all
// panels. Avoids GLSL recompile and material diffing on every re-render.
const SOLAR_CELL_MATERIAL = new ShaderMaterial({
  vertexShader: VERTEX_SHADER,
  fragmentShader: FRAGMENT_SHADER,
  side: DoubleSide,
});

export function SolarCellMaterial() {
  return <primitive object={SOLAR_CELL_MATERIAL} attach="material" />;
}

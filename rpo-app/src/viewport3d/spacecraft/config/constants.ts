import type { Vehicle } from '@/domain/vehicle';
import { resolveColor } from '@/utils/cssVars';
import type { Vec3 } from '@/viewport3d/types';

export interface Tint {
  accent: string;
  marker: string;
}

// Accent paints the equator stripe and the screen-space tracking marker;
// the bus itself is always thermal white. Tokens reuse the viz palette so
// chief/deputy reads consistently across HTML panels and the 3D scene.
const CHIEF_ACCENT = resolveColor('--color-viz-free-drift', '#f43f5e');
const DEPUTY_ACCENT = resolveColor('--color-viz-analytical', '#60a5fa');

export const TINTS: Record<Vehicle, Tint> = {
  chief: { accent: CHIEF_ACCENT, marker: CHIEF_ACCENT },
  deputy: { accent: DEPUTY_ACCENT, marker: DEPUTY_ACCENT },
};

export const REF_DRAG_AREA_M2 = 1.0;
export const MIN_BODY_SCALE = 0.35;
export const MAX_BODY_SCALE = 1.15;
export const DEPLOYED_PANEL_THRESHOLD_M2 = 0.15;
export const REF_SRP_AREA_M2 = 1.0;

export const BASE_BODY = { width: 6, height: 6, depth: 8 } as const;

// Panel frame: width = X (outboard, long axis), depth = Z (cross-track span),
// height = Y (plate thickness). Plate lies flat with normal along +Y.
export const BASE_PANEL = { width: 10, height: 0.08, depth: 6 } as const;

export const ARM_LENGTH = 5.5;
export const ARM_OFFSET = BASE_BODY.width / 2 + ARM_LENGTH / 2;
export const ARM_END_X = BASE_BODY.width / 2 + ARM_LENGTH;

export const STOWED_PANEL_HEIGHT = 0.3;
export const STOWED_PANEL_SCALE = 0.85;

// Thermal white (matte Z93-style paint), shared by both vehicles.
export const BUS_COLOR = '#e8e8e6';

export const ACCENT_STRIPE_HEIGHT = 0.8;
export const ACCENT_STRIPE_INTENSITY = 0.35;

export const MARKER_OFFSET: Vec3 = [0, 0, 0];
export const MARKER_PIXEL_SIZE = 5;

// Docking collar radius is a fraction of the bus semi-axes so it tracks
// elliptical buses (width != height) correctly.
export const DOCKING_COLLAR_RADIUS_FRAC = 0.45;
export const DOCKING_COLLAR_LENGTH = 0.6;
export const DOCKING_COLLAR_COLOR = '#3a3a3e';

export const THRUSTER_COUNT = 4;
export const THRUSTER_PLACEMENT_RADIUS_FRAC = 0.55;
export const THRUSTER_NOZZLE_LENGTH = 0.9;
export const THRUSTER_NOZZLE_BASE_RADIUS = 0.32;
export const THRUSTER_NOZZLE_APEX_RADIUS = 0.12;
export const THRUSTER_COLOR = '#2a2a2e';

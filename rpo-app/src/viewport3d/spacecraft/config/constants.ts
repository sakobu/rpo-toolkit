import type { Vehicle } from '@/domain/vehicle';
import type { Vec3 } from '@/viewport3d/types';

export interface Tint {
  body: string;
  emissive: string;
  panel: string;
  nav: string;
}

// Red team / blue team convention: chief = red, deputy = blue.
// Red pulls from --color-viz-free-drift (#f43f5e); blue from --color-viz-analytical (#60a5fa).
export const TINTS: Record<Vehicle, Tint> = {
  chief: { body: '#8a2e3a', emissive: '#f43f5e', panel: '#efc7d0', nav: '#f43f5e' },
  deputy: { body: '#3a5a8a', emissive: '#60a5fa', panel: '#c7d6ef', nav: '#60a5fa' },
};

export const REF_DRAG_AREA_M2 = 1.0;
export const MIN_BODY_SCALE = 0.35;
export const MAX_BODY_SCALE = 1.15;
export const DEPLOYED_PANEL_THRESHOLD_M2 = 0.15;
export const REF_SRP_AREA_M2 = 1.0;

export const BASE_BODY = { width: 6, height: 6, depth: 8 } as const;
export const BASE_PANEL = { width: 1, height: 10, depth: 8 } as const;
export const ARM_OFFSET = 8.5;
export const ARM_LENGTH = 11;
export const PANEL_OFFSET = 14;

export const NAV_TOP: Vec3 = [0, 4, 5];
export const NAV_BOTTOM: Vec3 = [0, -4, 5];

export const NAV_INTENSITY = 1.0;
export const NAV_DISTANCE = 30;

export const BODY_EMISSIVE_INTENSITY = 0.35;
export const STOWED_PANEL_HEIGHT = 0.3;
export const STOWED_PANEL_SCALE = 0.85;

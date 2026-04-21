import type { ArmProps, MainBodyProps, SolarPanelProps, TrackingMarkerProps } from '../types';

import {
  ARM_END_X,
  ARM_LENGTH,
  ARM_OFFSET,
  BASE_BODY,
  BASE_PANEL,
  MARKER_OFFSET,
  STOWED_PANEL_HEIGHT,
  STOWED_PANEL_SCALE,
  type Tint,
} from './constants';

export const buildMainBody = (tint: Tint): MainBodyProps => ({
  ...BASE_BODY,
  accent: tint.accent,
});

export const buildDeployedArms = (): ArmProps[] => [
  { position: [-ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
  { position: [ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
];

// Inner edge stays anchored to the arm tip regardless of scale so the panel
// doesn't clip into the arm.
export const buildDeployedPanels = (panelScale: number): SolarPanelProps[] => {
  const length = BASE_PANEL.width * panelScale;
  const span = BASE_PANEL.depth * panelScale;
  const centerX = ARM_END_X + length / 2;
  return [
    { width: length, height: BASE_PANEL.height, depth: span, position: [-centerX, 0, 0] },
    { width: length, height: BASE_PANEL.height, depth: span, position: [centerX, 0, 0] },
  ];
};

export const buildStowedPanels = (): SolarPanelProps[] => [
  {
    width: BASE_BODY.width * STOWED_PANEL_SCALE,
    height: STOWED_PANEL_HEIGHT,
    depth: BASE_BODY.depth * STOWED_PANEL_SCALE,
    position: [0, BASE_BODY.height / 2 + 0.2, 0],
  },
];

export const buildTrackingMarkers = (tint: Tint): TrackingMarkerProps[] => [
  { position: MARKER_OFFSET, color: tint.marker },
];

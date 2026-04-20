import type { ArmProps, MainBodyProps, NavigationLightProps, SolarPanelProps } from '../types';

import {
  ARM_LENGTH,
  ARM_OFFSET,
  BASE_BODY,
  BASE_PANEL,
  BODY_EMISSIVE_INTENSITY,
  NAV_BOTTOM,
  NAV_DISTANCE,
  NAV_INTENSITY,
  NAV_TOP,
  PANEL_OFFSET,
  STOWED_PANEL_HEIGHT,
  STOWED_PANEL_SCALE,
  type Tint,
} from './constants';

export const buildMainBody = (tint: Tint): MainBodyProps => ({
  ...BASE_BODY,
  color: tint.body,
  emissive: tint.emissive,
  emissiveIntensity: BODY_EMISSIVE_INTENSITY,
});

export const buildDeployedArms = (): ArmProps[] => [
  { position: [-ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
  { position: [ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
];

export const buildDeployedPanels = (panelScale: number, tint: Tint): SolarPanelProps[] => {
  const height = BASE_PANEL.height * panelScale;
  return [
    { ...BASE_PANEL, height, position: [-PANEL_OFFSET, 0, 0], color: tint.panel },
    { ...BASE_PANEL, height, position: [PANEL_OFFSET, 0, 0], color: tint.panel },
  ];
};

export const buildStowedPanels = (tint: Tint): SolarPanelProps[] => [
  {
    width: BASE_BODY.width * STOWED_PANEL_SCALE,
    height: STOWED_PANEL_HEIGHT,
    depth: BASE_BODY.depth * STOWED_PANEL_SCALE,
    position: [0, BASE_BODY.height / 2 + 0.2, 0],
    color: tint.panel,
  },
];

export const buildNavLights = (tint: Tint): NavigationLightProps[] => [
  { position: NAV_TOP, color: tint.nav, intensity: NAV_INTENSITY, distance: NAV_DISTANCE },
  { position: NAV_BOTTOM, color: tint.nav, intensity: NAV_INTENSITY, distance: NAV_DISTANCE },
];

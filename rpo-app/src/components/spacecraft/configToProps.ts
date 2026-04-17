import type { MainBodyProps } from './MainBody';
import type { NavigationLightProps } from './NavigationLight';
import type { SolarPanelProps } from './SolarPanel';
import type { ArmProps, SpacecraftProps } from './Spacecraft';
import type { Vec3 } from './types';
import type { SpacecraftConfig } from '../../schemas/spacecraft';

export type Vehicle = 'chief' | 'deputy';

// Red team / blue team convention: chief = red, deputy = blue.
// Red pulls from --color-viz-free-drift (#f43f5e); blue from --color-viz-analytical (#60a5fa).
const TINT: Record<Vehicle, { body: string; emissive: string; panel: string; nav: string }> = {
  chief: { body: '#8a2e3a', emissive: '#f43f5e', panel: '#efc7d0', nav: '#f43f5e' },
  deputy: { body: '#3a5a8a', emissive: '#60a5fa', panel: '#c7d6ef', nav: '#60a5fa' },
};

const REF_DRAG_AREA = 1.0;
const MIN_SCALE = 0.35;
const MAX_SCALE = 1.15;
const DEPLOYED_PANEL_THRESHOLD = 0.15;

const BASE_BODY = { width: 6, height: 6, depth: 8 };
const BASE_PANEL = { width: 1, height: 10, depth: 8 };
const ARM_OFFSET = 8.5;
const ARM_LENGTH = 11;
const PANEL_OFFSET = 14;
const NAV_TOP: Vec3 = [0, 4, 5];
const NAV_BOTTOM: Vec3 = [0, -4, 5];

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
}

export function configToSpacecraftProps(
  config: SpacecraftConfig,
  vehicle: Vehicle,
): SpacecraftProps {
  const c = TINT[vehicle];
  const raw = Math.cbrt(Math.max(config.drag_area_m2, 1e-4) / REF_DRAG_AREA);
  const scale = clamp(raw, MIN_SCALE, MAX_SCALE);

  const mainBody: MainBodyProps = {
    ...BASE_BODY,
    color: c.body,
    emissive: c.emissive,
    emissiveIntensity: 0.35,
  };

  const deployed = config.srp_area_m2 > DEPLOYED_PANEL_THRESHOLD;
  let arms: ArmProps[];
  let solarPanels: SolarPanelProps[];

  if (deployed) {
    const panelScale = Math.sqrt(Math.max(config.srp_area_m2, 1e-4) / 1.0);
    const panelHeight = BASE_PANEL.height * panelScale;
    arms = [
      { position: [-ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
      { position: [ARM_OFFSET, 0, 0], length: ARM_LENGTH, direction: 'x' },
    ];
    solarPanels = [
      {
        ...BASE_PANEL,
        height: panelHeight,
        position: [-PANEL_OFFSET, 0, 0],
        color: c.panel,
      },
      {
        ...BASE_PANEL,
        height: panelHeight,
        position: [PANEL_OFFSET, 0, 0],
        color: c.panel,
      },
    ];
  } else {
    arms = [];
    solarPanels = [
      {
        width: BASE_BODY.width * 0.85,
        height: 0.3,
        depth: BASE_BODY.depth * 0.85,
        position: [0, BASE_BODY.height / 2 + 0.2, 0],
        color: c.panel,
      },
    ];
  }

  const navigationLights: NavigationLightProps[] = [
    { position: NAV_TOP, color: c.nav, intensity: 1.0, distance: 30 },
    { position: NAV_BOTTOM, color: c.nav, intensity: 1.0, distance: 30 },
  ];

  return { scale, mainBody, arms, solarPanels, navigationLights };
}

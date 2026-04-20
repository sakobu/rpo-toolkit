import type { Vec3 } from '@/viewport3d/types';

export interface MainBodyProps {
  width: number;
  height: number;
  depth: number;
  color: string;
  emissive?: string;
  emissiveIntensity?: number;
  metalness?: number;
  roughness?: number;
}

export interface SolarPanelProps {
  width: number;
  height: number;
  depth: number;
  position: Vec3;
  rotation?: Vec3;
  color?: string;
  emissive?: string;
  emissiveIntensity?: number;
}

export interface NavigationLightProps {
  position: Vec3;
  color: string;
  intensity?: number;
  distance?: number;
}

export interface ArmProps {
  position: Vec3;
  length: number;
  direction: 'x' | 'y' | 'z';
}

export interface SpacecraftLabelProps {
  label: string;
  color?: string;
  offsetY: number;
  fontSize?: number;
}

export interface SpacecraftProps {
  position?: Vec3;
  rotation?: Vec3;
  scale?: number;
  mainBody: MainBodyProps;
  solarPanels: SolarPanelProps[];
  arms?: ArmProps[];
  navigationLights?: NavigationLightProps[];
  label?: string;
  labelColor?: string;
  // Label text size in scene units. Independent of `scale` so far-field icons
  // can stay tiny while labels remain readable.
  labelFontSize?: number;
  // Vertical offset of the label from `position`, in scene units. Defaults to
  // -(mainBody.height + pad) * scale so label sits just below the scaled body.
  labelOffsetY?: number;
}

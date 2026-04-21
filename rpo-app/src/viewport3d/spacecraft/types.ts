import type { Vec3 } from '@/viewport3d/types';

export interface MainBodyProps {
  width: number;
  height: number;
  depth: number;
  accent: string;
}

export interface SolarPanelProps {
  width: number;
  height: number;
  depth: number;
  position: Vec3;
  rotation?: Vec3;
}

export interface TrackingMarkerProps {
  position: Vec3;
  color: string;
  pixelSize?: number;
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
  trackingMarkers?: TrackingMarkerProps[];
  label?: string;
  labelColor?: string;
  labelFontSize?: number;
  labelOffsetY?: number;
}

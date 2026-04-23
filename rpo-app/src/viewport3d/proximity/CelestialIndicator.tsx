import { useMemo } from 'react';
import { Html, Line } from '@react-three/drei';
import { Earth, Moon, Sun } from 'lucide-react';

import { clamp } from '@/utils/math';
import type { Vec3 } from '@/viewport3d/types';

import { CELESTIAL_INDICATOR } from './constants';
import { ricToPosition } from './coordinates';
import type { CelestialBodyDir } from './scene';

type CelestialBody = 'earth' | 'sun' | 'moon';

const ICON_BY_BODY = {
  earth: Earth,
  sun: Sun,
  moon: Moon,
} as const;

type Props = {
  dir: CelestialBodyDir;
  gridExtentM: number;
  body: CelestialBody;
  color: string;
};

export default function CelestialIndicator({ dir, gridExtentM, body, color }: Props) {
  const { tip, iconPos, opacity, iconOpacity } = useMemo(() => {
    const { unitRic, distanceKm } = dir;
    const log = Math.log10(Math.max(distanceKm, 1));
    const span = CELESTIAL_INDICATOR.logKmMax - CELESTIAL_INDICATOR.logKmMin;
    const t = clamp((log - CELESTIAL_INDICATOR.logKmMin) / span, 0, 1);

    const lengthRatio =
      CELESTIAL_INDICATOR.lengthRatioNear +
      t * (CELESTIAL_INDICATOR.lengthRatioFar - CELESTIAL_INDICATOR.lengthRatioNear);
    const lengthM = gridExtentM * lengthRatio;

    const opacity =
      CELESTIAL_INDICATOR.opacityNear +
      t * (CELESTIAL_INDICATOR.opacityFar - CELESTIAL_INDICATOR.opacityNear);
    const iconOpacity = Math.min(1, opacity + CELESTIAL_INDICATOR.iconOpacityBias);

    const tipRic: Vec3 = [unitRic[0] * lengthM, unitRic[1] * lengthM, unitRic[2] * lengthM];
    const offset = lengthM * CELESTIAL_INDICATOR.labelOffsetRatio;
    const iconRic: Vec3 = [
      unitRic[0] * (lengthM + offset),
      unitRic[1] * (lengthM + offset),
      unitRic[2] * (lengthM + offset),
    ];
    return {
      tip: ricToPosition(tipRic),
      iconPos: ricToPosition(iconRic),
      opacity,
      iconOpacity,
    };
  }, [dir, gridExtentM]);

  const Icon = ICON_BY_BODY[body];

  return (
    <group>
      <Line
        points={[[0, 0, 0], tip]}
        color={color}
        lineWidth={CELESTIAL_INDICATOR.lineWidth}
        opacity={opacity}
        transparent
        depthWrite={false}
      />
      <Html
        position={iconPos}
        center
        zIndexRange={[20, 0]}
        style={{ pointerEvents: 'none', color, opacity: iconOpacity }}
      >
        <Icon
          size={CELESTIAL_INDICATOR.iconSizePx}
          strokeWidth={CELESTIAL_INDICATOR.iconStrokeWidth}
        />
      </Html>
    </group>
  );
}

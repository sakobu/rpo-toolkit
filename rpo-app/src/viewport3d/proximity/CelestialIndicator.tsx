import { useMemo } from 'react';
import { Html, Line } from '@react-three/drei';
import { Earth, Moon, Sun } from 'lucide-react';

import type { Vec3 } from '@/viewport3d/types';

import { CELESTIAL_INDICATOR } from './constants';
import { ricToPosition } from './coordinates';

type CelestialBody = 'earth' | 'sun' | 'moon';

const ICON_BY_BODY = {
  earth: Earth,
  sun: Sun,
  moon: Moon,
} as const;

type Props = {
  unitRic: Vec3;
  edgeM: number;
  body: CelestialBody;
  color: string;
};

export default function CelestialIndicator({ unitRic, edgeM, body, color }: Props) {
  const { tip, iconPos } = useMemo(() => {
    const tipRic: Vec3 = [unitRic[0] * edgeM, unitRic[1] * edgeM, unitRic[2] * edgeM];
    const offset = edgeM * CELESTIAL_INDICATOR.labelOffsetRatio;
    const iconRic: Vec3 = [
      unitRic[0] * (edgeM + offset),
      unitRic[1] * (edgeM + offset),
      unitRic[2] * (edgeM + offset),
    ];
    return { tip: ricToPosition(tipRic), iconPos: ricToPosition(iconRic) };
  }, [unitRic, edgeM]);

  const Icon = ICON_BY_BODY[body];

  return (
    <group>
      <Line
        points={[[0, 0, 0], tip]}
        color={color}
        lineWidth={CELESTIAL_INDICATOR.lineWidth}
        opacity={CELESTIAL_INDICATOR.lineOpacity}
        transparent
        depthWrite={false}
      />
      <Html
        position={iconPos}
        center
        zIndexRange={[20, 0]}
        style={{ pointerEvents: 'none', color }}
      >
        <Icon
          size={CELESTIAL_INDICATOR.iconSizePx}
          strokeWidth={CELESTIAL_INDICATOR.iconStrokeWidth}
        />
      </Html>
    </group>
  );
}

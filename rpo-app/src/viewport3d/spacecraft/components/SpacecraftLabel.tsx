import { Billboard, Text } from '@react-three/drei';

import type { SpacecraftLabelProps } from '../types';

export function SpacecraftLabel({ label, color = '#ffffff', offsetY }: SpacecraftLabelProps) {
  return (
    <Billboard position={[0, offsetY, 0]}>
      <Text
        fontSize={5}
        color={color}
        anchorX="center"
        anchorY="top"
        outlineWidth={0.15}
        outlineColor="#000000"
      >
        {label}
      </Text>
    </Billboard>
  );
}

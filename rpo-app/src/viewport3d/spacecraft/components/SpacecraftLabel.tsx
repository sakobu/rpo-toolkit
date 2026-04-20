import { Billboard, Text } from '@react-three/drei';

import type { SpacecraftLabelProps } from '../types';

const DEFAULT_FONT_SIZE = 5;

export function SpacecraftLabel({
  label,
  color = '#ffffff',
  offsetY,
  fontSize = DEFAULT_FONT_SIZE,
}: SpacecraftLabelProps) {
  return (
    <Billboard position={[0, offsetY, 0]}>
      <Text
        fontSize={fontSize}
        color={color}
        anchorX="center"
        anchorY="top"
        outlineWidth={fontSize * 0.03}
        outlineColor="#000000"
      >
        {label}
      </Text>
    </Billboard>
  );
}

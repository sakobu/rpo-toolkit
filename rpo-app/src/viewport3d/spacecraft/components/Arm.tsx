import type { ArmProps } from '../types';

const THICKNESS = 0.8;

export function Arm({ position, length, direction }: ArmProps) {
  const size: [number, number, number] =
    direction === 'x'
      ? [length, THICKNESS, THICKNESS]
      : direction === 'y'
        ? [THICKNESS, length, THICKNESS]
        : [THICKNESS, THICKNESS, length];

  return (
    <mesh position={position}>
      <boxGeometry args={size} />
      <meshStandardMaterial
        color="#b0b0b0"
        metalness={0.7}
        roughness={0.3}
        transparent
        opacity={1}
        depthWrite={true}
      />
    </mesh>
  );
}

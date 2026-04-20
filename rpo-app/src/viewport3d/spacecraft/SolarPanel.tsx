import type { Vec3 } from '@/viewport3d/types';

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

export function SolarPanel({
  width,
  height,
  depth,
  position,
  rotation = [0, 0, 0],
  color = '#d0d0d0',
  emissive = '#444444',
  emissiveIntensity = 0.2,
}: SolarPanelProps) {
  return (
    <mesh position={position} rotation={rotation}>
      <boxGeometry args={[width, height, depth]} />
      <meshStandardMaterial
        color={color}
        metalness={0.8}
        roughness={0.2}
        emissive={emissive}
        emissiveIntensity={emissiveIntensity}
        transparent
        opacity={1}
        depthWrite={true}
      />
    </mesh>
  );
}

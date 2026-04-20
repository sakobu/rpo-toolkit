import type { NavigationLightProps } from '../types';

export function NavigationLight({
  position,
  color,
  intensity = 2,
  distance = 100,
}: NavigationLightProps) {
  return (
    <group position={position}>
      <mesh>
        <sphereGeometry args={[0.5, 8, 8]} />
        <meshBasicMaterial color={color} transparent opacity={1} depthWrite={true} />
      </mesh>
      <pointLight color={color} intensity={intensity} distance={distance} />
    </group>
  );
}

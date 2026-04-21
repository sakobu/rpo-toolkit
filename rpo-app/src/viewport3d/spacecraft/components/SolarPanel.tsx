import type { SolarPanelProps } from '../types';

import { SolarCellMaterial } from './SolarCellMaterial';

// The shader handles both plane faces via gl_FrontFacing: +Y = cell grid,
// -Y = honeycomb substrate.

export function SolarPanel({ width, depth, position, rotation = [0, 0, 0] }: SolarPanelProps) {
  return (
    <group position={position} rotation={rotation}>
      <mesh rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[width, depth]} />
        <SolarCellMaterial />
      </mesh>
    </group>
  );
}

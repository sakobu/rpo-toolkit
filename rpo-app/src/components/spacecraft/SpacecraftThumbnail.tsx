import { useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import type { Group } from 'three';

import { Spacecraft, type SpacecraftProps } from './Spacecraft';
import { configToSpacecraftProps, type Vehicle } from './configToProps';
import type { SpacecraftConfig } from '../../schemas/spacecraft';

type Props = {
  vehicle: Vehicle;
  config: SpacecraftConfig;
};

function Orbiter({ props }: { props: SpacecraftProps }) {
  const group = useRef<Group>(null);
  useFrame((_, dt) => {
    if (group.current) group.current.rotation.y += dt * 0.3;
  });
  return (
    <group ref={group}>
      <Spacecraft {...props} />
    </group>
  );
}

export function SpacecraftThumbnail({ vehicle, config }: Props) {
  const props = configToSpacecraftProps(config, vehicle);
  return (
    <div className="h-48 overflow-hidden rounded-sm border border-border/60 bg-surface-2">
      <Canvas camera={{ position: [22, 11, 22], fov: 38 }} dpr={[1, 2]}>
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.5]} />
        <directionalLight position={[10, 20, 15]} intensity={0.9} />
        <Orbiter props={props} />
      </Canvas>
    </div>
  );
}

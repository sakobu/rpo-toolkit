import { Canvas } from '@react-three/fiber';

import type { Vehicle } from '@/domain/vehicle';
import type { SpacecraftConfig } from '@/schemas/spacecraft';

import { configToSpacecraftProps } from '../config/configToProps';

import { Orbiter } from './Orbiter';

interface Props {
  vehicle: Vehicle;
  config: SpacecraftConfig;
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

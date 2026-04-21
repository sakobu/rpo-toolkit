import { useMemo } from 'react';
import { OrbitControls } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';

import { VEHICLE_LABELS } from '@/domain/vehicle';
import { PRESETS } from '@/schemas/spacecraft';
import { configToSpacecraftProps, Spacecraft, TINTS } from '@/viewport3d/spacecraft';

import { CAMERA, DEMO_DEPUTY_OFFSET_RIC_M, ORBIT_CONTROLS } from './constants';
import { ricToPosition } from './coordinates';
import Grid from './Grid';
import RICAxes from './RICAxes';

const DEFAULT_CHIEF = { preset: 'Servicer 500kg' as const, ...PRESETS['Servicer 500kg'] };
const DEFAULT_DEPUTY = { preset: 'Servicer 500kg' as const, ...PRESETS['Servicer 500kg'] };

export default function ProximityViewport() {
  const chiefProps = useMemo(() => configToSpacecraftProps(DEFAULT_CHIEF, 'chief'), []);
  const deputyProps = useMemo(() => configToSpacecraftProps(DEFAULT_DEPUTY, 'deputy'), []);
  const deputyPosition = useMemo(() => ricToPosition(DEMO_DEPUTY_OFFSET_RIC_M), []);

  return (
    <div className="h-full w-full">
      <Canvas
        camera={{
          position: CAMERA.position,
          fov: CAMERA.fov,
          near: CAMERA.near,
          far: CAMERA.far,
        }}
        dpr={[1, 2]}
      >
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight position={[10, 5, 10]} intensity={1.0} />
        <Grid />
        <RICAxes />
        <Spacecraft {...chiefProps} label={VEHICLE_LABELS.chief} labelColor={TINTS.chief.accent} />
        <Spacecraft
          {...deputyProps}
          position={deputyPosition}
          label={VEHICLE_LABELS.deputy}
          labelColor={TINTS.deputy.accent}
        />
        <OrbitControls
          enablePan
          enableDamping={ORBIT_CONTROLS.enableDamping}
          minDistance={ORBIT_CONTROLS.minDistance}
          maxDistance={ORBIT_CONTROLS.maxDistance}
        />
      </Canvas>
    </div>
  );
}

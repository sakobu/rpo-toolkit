import { useMemo } from 'react';
import { OrbitControls } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { useShallow } from 'zustand/react/shallow';

import { PRESETS, type SpacecraftConfig } from '@/schemas/spacecraft';
import { loadedVector, useConfig } from '@/stores/configuration';

import { CHIEF_ORBIT_COLOR, DEPUTY_ORBIT_COLOR } from './constants';
import Earth from './Earth';
import OrbitLine from './OrbitLine';
import { computeCameraPos, computeEraRad, computeSunScenePos, FALLBACK_LIGHT_POS } from './scene';
import TransferArc from './TransferArc';
import VehicleMesh from './VehicleMesh';

const FALLBACK_CONFIG: SpacecraftConfig = {
  preset: 'Servicer 500kg',
  ...PRESETS['Servicer 500kg'],
};

export default function FarFieldViewport() {
  const { chiefConfig, deputyConfig, chiefState, deputyState } = useConfig(
    useShallow((s) => ({
      chiefConfig: s.chiefConfig?.values ?? FALLBACK_CONFIG,
      deputyConfig: s.deputyConfig?.values ?? FALLBACK_CONFIG,
      chiefState: s.chiefState,
      deputyState: s.deputyState,
    })),
  );

  const chiefVector = loadedVector(chiefState);
  const deputyVector = loadedVector(deputyState);

  const epoch = chiefVector?.epoch ?? deputyVector?.epoch ?? null;
  const eraRad = useMemo(() => (epoch === null ? 0 : computeEraRad(epoch)), [epoch]);
  const sunScenePos = useMemo(
    () => (epoch === null ? FALLBACK_LIGHT_POS : computeSunScenePos(epoch)),
    [epoch],
  );
  // OrbitControls owns the camera after first render; remount to re-apply.
  const cameraPos = useMemo(
    () => computeCameraPos(chiefVector, deputyVector),
    [chiefVector, deputyVector],
  );

  return (
    <div className="h-full w-full">
      <Canvas camera={{ position: cameraPos, fov: 50, near: 0.1, far: 100 }} dpr={[1, 2]}>
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight position={sunScenePos} intensity={1.0} />
        <Earth rotationY={eraRad} />
        {chiefVector && <OrbitLine state={chiefVector} color={CHIEF_ORBIT_COLOR} />}
        {deputyVector && <OrbitLine state={deputyVector} color={DEPUTY_ORBIT_COLOR} />}
        <TransferArc />
        {chiefVector && <VehicleMesh vehicle="chief" config={chiefConfig} vector={chiefVector} />}
        {deputyVector && (
          <VehicleMesh vehicle="deputy" config={deputyConfig} vector={deputyVector} />
        )}
        <OrbitControls enablePan={false} minDistance={3.5} maxDistance={20} />
      </Canvas>
    </div>
  );
}

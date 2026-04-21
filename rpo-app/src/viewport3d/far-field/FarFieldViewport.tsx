import { OrbitControls } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { useShallow } from 'zustand/react/shallow';

import { PRESETS, type SpacecraftConfig } from '@/schemas/spacecraft';
import type { StateVectorInput } from '@/schemas/stateVector';
import { useConfig, type VehicleStateSlot } from '@/stores/configuration';

import Earth from './Earth';
import { computeCameraPos, computeEraRad, computeSunScenePos, FALLBACK_LIGHT_POS } from './scene';
import VehicleMesh from './VehicleMesh';

const FALLBACK_CONFIG: SpacecraftConfig = {
  preset: 'Servicer 500kg',
  ...PRESETS['Servicer 500kg'],
};

function loadedVector(slot: VehicleStateSlot): StateVectorInput | null {
  return slot.status === 'loaded' ? slot.vector : null;
}

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
  const eraRad = epoch === null ? 0 : computeEraRad(epoch);
  const sunScenePos = epoch === null ? FALLBACK_LIGHT_POS : computeSunScenePos(epoch);
  // OrbitControls owns the camera after first render; remount to re-apply.
  const cameraPos = computeCameraPos(chiefVector, deputyVector);

  return (
    <div className="h-full w-full">
      <Canvas camera={{ position: cameraPos, fov: 50, near: 0.1, far: 100 }} dpr={[1, 2]}>
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight position={sunScenePos} intensity={1.0} />
        <Earth rotationY={eraRad} />
        {chiefVector && <VehicleMesh vehicle="chief" config={chiefConfig} vector={chiefVector} />}
        {deputyVector && (
          <VehicleMesh vehicle="deputy" config={deputyConfig} vector={deputyVector} />
        )}
        <OrbitControls enablePan={false} minDistance={3.5} maxDistance={20} />
      </Canvas>
    </div>
  );
}

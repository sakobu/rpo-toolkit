import { OrbitControls } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { useShallow } from 'zustand/react/shallow';

import { type Vehicle, VEHICLE_LABELS } from '@/domain/vehicle';
import { PRESETS, type SpacecraftConfig } from '@/schemas/spacecraft';
import type { StateVectorInput } from '@/schemas/stateVector';
import { useConfig, type VehicleStateSlot } from '@/stores/configuration';
import { configToSpacecraftProps, Spacecraft, TINTS } from '@/viewport3d/spacecraft';

import {
  FARFIELD_LABEL_FONT_SIZE,
  FARFIELD_LABEL_OFFSET_Y,
  FARFIELD_SPACECRAFT_SCALE,
} from './constants';
import { eciKmToScenePosition } from './coordinates';
import Earth from './Earth';

const FALLBACK_CONFIG: SpacecraftConfig = {
  preset: 'Servicer 500kg',
  ...PRESETS['Servicer 500kg'],
};

function loadedVector(slot: VehicleStateSlot): StateVectorInput | null {
  return slot.status === 'loaded' ? slot.vector : null;
}

function VehicleMesh({
  vehicle,
  config,
  vector,
}: {
  vehicle: Vehicle;
  config: SpacecraftConfig;
  vector: StateVectorInput;
}) {
  const props = configToSpacecraftProps(config, vehicle);
  const position = eciKmToScenePosition(vector.position_eci_km);
  return (
    <Spacecraft
      {...props}
      position={position}
      scale={FARFIELD_SPACECRAFT_SCALE}
      label={VEHICLE_LABELS[vehicle]}
      labelColor={TINTS[vehicle].emissive}
      labelFontSize={FARFIELD_LABEL_FONT_SIZE}
      labelOffsetY={FARFIELD_LABEL_OFFSET_Y}
    />
  );
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

  return (
    <div className="h-full w-full">
      <Canvas camera={{ position: [0, 0, 6], fov: 50, near: 0.1, far: 100 }} dpr={[1, 2]}>
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight position={[10, 5, 10]} intensity={1.0} />
        <Earth />
        {chiefVector && <VehicleMesh vehicle="chief" config={chiefConfig} vector={chiefVector} />}
        {deputyVector && (
          <VehicleMesh vehicle="deputy" config={deputyConfig} vector={deputyVector} />
        )}
        <OrbitControls enablePan={false} minDistance={3.5} maxDistance={20} />
      </Canvas>
    </div>
  );
}

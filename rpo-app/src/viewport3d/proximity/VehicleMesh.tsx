import { useMemo } from 'react';

import { type Vehicle, VEHICLE_LABELS } from '@/domain/vehicle';
import type { SpacecraftConfig } from '@/schemas/spacecraft';
import { configToSpacecraftProps, Spacecraft, TINTS } from '@/viewport3d/spacecraft';
import type { Vec3 } from '@/viewport3d/types';

import { ricToPosition } from './coordinates';

type Props = {
  vehicle: Vehicle;
  config: SpacecraftConfig;
  positionRicM: Vec3;
  scale: number;
  labelFontSizeM: number;
};

export default function VehicleMesh({
  vehicle,
  config,
  positionRicM,
  scale,
  labelFontSizeM,
}: Props) {
  const props = useMemo(() => configToSpacecraftProps(config, vehicle), [config, vehicle]);
  return (
    <Spacecraft
      {...props}
      position={ricToPosition(positionRicM)}
      scale={scale}
      label={VEHICLE_LABELS[vehicle]}
      labelColor={TINTS[vehicle].accent}
      labelFontSize={labelFontSizeM}
    />
  );
}

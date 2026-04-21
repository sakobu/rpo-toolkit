import { useMemo } from 'react';

import { type Vehicle, VEHICLE_LABELS } from '@/domain/vehicle';
import type { SpacecraftConfig } from '@/schemas/spacecraft';
import type { StateVectorInput } from '@/schemas/stateVector';
import { configToSpacecraftProps, Spacecraft, TINTS } from '@/viewport3d/spacecraft';

import {
  FARFIELD_LABEL_FONT_SIZE,
  FARFIELD_LABEL_OFFSET_Y,
  FARFIELD_SPACECRAFT_SCALE,
} from './constants';
import { eciKmToScenePosition } from './coordinates';

type Props = {
  vehicle: Vehicle;
  config: SpacecraftConfig;
  vector: StateVectorInput;
};

export default function VehicleMesh({ vehicle, config, vector }: Props) {
  const props = useMemo(() => configToSpacecraftProps(config, vehicle), [config, vehicle]);
  const position = useMemo(
    () => eciKmToScenePosition(vector.position_eci_km),
    [vector.position_eci_km],
  );
  return (
    <Spacecraft
      {...props}
      position={position}
      scale={FARFIELD_SPACECRAFT_SCALE}
      label={VEHICLE_LABELS[vehicle]}
      labelColor={TINTS[vehicle].accent}
      labelFontSize={FARFIELD_LABEL_FONT_SIZE}
      labelOffsetY={FARFIELD_LABEL_OFFSET_Y}
    />
  );
}

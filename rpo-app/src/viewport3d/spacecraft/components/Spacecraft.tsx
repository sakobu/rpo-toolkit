import type { SpacecraftProps } from '../types';

import { Arm } from './Arm';
import { MainBody } from './MainBody';
import { NavigationLight } from './NavigationLight';
import { SolarPanel } from './SolarPanel';
import { SpacecraftLabel } from './SpacecraftLabel';

const LABEL_OFFSET_PAD = 8;

export function Spacecraft({
  position = [0, 0, 0],
  rotation = [0, 0, 0],
  scale = 1,
  mainBody,
  solarPanels,
  arms = [],
  navigationLights = [],
  label,
  labelColor,
  labelFontSize,
  labelOffsetY,
}: SpacecraftProps) {
  const defaultOffsetY = -(mainBody.height + LABEL_OFFSET_PAD) * scale;
  const resolvedOffsetY = labelOffsetY ?? defaultOffsetY;

  return (
    <group position={position} rotation={rotation}>
      <group scale={[scale, scale, scale]} renderOrder={10}>
        <MainBody {...mainBody} />

        {arms.map((arm, index) => (
          <Arm key={`arm-${index}`} {...arm} />
        ))}

        {solarPanels.map((panel, index) => (
          <SolarPanel key={`panel-${index}`} {...panel} />
        ))}

        {navigationLights.map((light, index) => (
          <NavigationLight key={`light-${index}`} {...light} />
        ))}
      </group>

      {label && (
        <SpacecraftLabel
          label={label}
          color={labelColor}
          offsetY={resolvedOffsetY}
          fontSize={labelFontSize}
        />
      )}
    </group>
  );
}

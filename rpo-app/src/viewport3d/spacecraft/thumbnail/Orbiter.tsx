import { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import type { Group } from 'three';

import { Spacecraft } from '../components/Spacecraft';
import type { SpacecraftProps } from '../types';

const ROTATION_RATE_RAD_S = 0.3;

export function Orbiter({ props }: { props: SpacecraftProps }) {
  const group = useRef<Group>(null);
  useFrame((_, dt) => {
    if (group.current) group.current.rotation.y += dt * ROTATION_RATE_RAD_S;
  });
  return (
    <group ref={group}>
      <Spacecraft {...props} />
    </group>
  );
}

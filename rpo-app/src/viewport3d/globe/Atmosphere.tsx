import { AdditiveBlending, DoubleSide } from 'three';

import { EARTH_RADIUS, EARTH_SEGMENTS, ECI_ATMOSPHERE } from './constants';

export default function Atmosphere() {
  const s = ECI_ATMOSPHERE.scaleFactor;

  return (
    <mesh scale={[s, s, s]}>
      <sphereGeometry args={[EARTH_RADIUS, EARTH_SEGMENTS, EARTH_SEGMENTS]} />
      <meshPhongMaterial
        color={ECI_ATMOSPHERE.color}
        transparent
        opacity={ECI_ATMOSPHERE.opacity}
        depthWrite={false}
        side={DoubleSide}
        blending={AdditiveBlending}
      />
    </mesh>
  );
}

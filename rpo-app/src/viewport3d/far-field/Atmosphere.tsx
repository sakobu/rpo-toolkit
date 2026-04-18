import { AdditiveBlending, DoubleSide } from 'three';

import { ATMOSPHERE, EARTH_RADIUS, EARTH_SEGMENTS } from './constants';

export default function Atmosphere() {
  const s = ATMOSPHERE.scaleFactor;

  return (
    <mesh scale={[s, s, s]}>
      <sphereGeometry args={[EARTH_RADIUS, EARTH_SEGMENTS, EARTH_SEGMENTS]} />
      <meshPhongMaterial
        color={ATMOSPHERE.color}
        transparent
        opacity={ATMOSPHERE.opacity}
        depthWrite={false}
        side={DoubleSide}
        blending={AdditiveBlending}
      />
    </mesh>
  );
}

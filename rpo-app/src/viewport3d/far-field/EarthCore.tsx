import type { Color, Texture, Vector2 } from 'three';

import { EARTH_RADIUS, EARTH_SEGMENTS } from './constants';

type Props = {
  colorMap: Texture;
  normalMap: Texture;
  specularMap: Texture;
  materialProps: {
    normalScale: Vector2;
    shininess: number;
    specular: Color;
  };
};

export default function EarthCore({ colorMap, normalMap, specularMap, materialProps }: Props) {
  return (
    <mesh>
      <sphereGeometry args={[EARTH_RADIUS, EARTH_SEGMENTS, EARTH_SEGMENTS]} />
      <meshPhongMaterial
        map={colorMap}
        normalMap={normalMap}
        normalScale={materialProps.normalScale}
        specularMap={specularMap}
        shininess={materialProps.shininess}
        specular={materialProps.specular}
      />
    </mesh>
  );
}

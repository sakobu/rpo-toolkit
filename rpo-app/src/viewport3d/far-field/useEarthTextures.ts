import { useMemo } from 'react';
import { useTexture } from '@react-three/drei';
import { Color, Vector2 } from 'three';

import { EARTH_MATERIAL, TEXTURE_PATHS } from './constants';

export function useEarthTextures() {
  const textures = useTexture({
    colorMap: TEXTURE_PATHS.color,
    normalMap: TEXTURE_PATHS.normal,
    specularMap: TEXTURE_PATHS.specular,
  });

  const materialProps = useMemo(
    () => ({
      normalScale: new Vector2(...EARTH_MATERIAL.normalScale),
      shininess: EARTH_MATERIAL.shininess,
      specular: new Color(EARTH_MATERIAL.specular),
    }),
    [],
  );

  return { textures, materialProps };
}

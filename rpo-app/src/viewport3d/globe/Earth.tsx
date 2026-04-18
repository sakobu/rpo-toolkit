import Atmosphere from './Atmosphere';
import EarthCore from './EarthCore';
import { useEarthTextures } from './useEarthTextures';

type Props = {
  rotationY?: number;
};

export default function Earth({ rotationY = 0 }: Props) {
  const { textures, materialProps } = useEarthTextures();

  return (
    <group rotation={[0, rotationY, 0]}>
      <EarthCore {...textures} materialProps={materialProps} />
      <Atmosphere />
    </group>
  );
}

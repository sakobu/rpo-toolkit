import { useMemo } from 'react';
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { useShallow } from 'zustand/react/shallow';

import { PRESETS, type SpacecraftConfig } from '@/schemas/spacecraft';
import { loadedVector, useConfig } from '@/stores/configuration';
import type { Vec3 } from '@/viewport3d/types';

import CelestialIndicator from './CelestialIndicator';
import { CELESTIAL_EDGE_RATIO, CELESTIAL_TINTS, ORBIT_CONTROLS } from './constants';
import Grid from './Grid';
import RICAxes from './RICAxes';
import { computeCelestialDirsRic, computeDeputyPositionRicM, computeScale } from './scene';
import VehicleMesh from './VehicleMesh';

const CHIEF_ORIGIN_RIC_M: Vec3 = [0, 0, 0];

const FALLBACK_CONFIG: SpacecraftConfig = {
  preset: 'Servicer 500kg',
  ...PRESETS['Servicer 500kg'],
};

export default function ProximityViewport() {
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

  const deputyRicM = useMemo(
    () => computeDeputyPositionRicM(chiefVector, deputyVector),
    [chiefVector, deputyVector],
  );
  const scale = useMemo(() => computeScale(deputyRicM), [deputyRicM]);
  const celestials = useMemo(() => computeCelestialDirsRic(chiefVector), [chiefVector]);
  const celestialEdgeM = scale.extentM * CELESTIAL_EDGE_RATIO;

  return (
    <div className="h-full w-full">
      <Canvas dpr={[1, 2]}>
        <PerspectiveCamera
          makeDefault
          fov={50}
          position={[0, 0, scale.cameraDistanceM]}
          near={scale.nearPlaneM}
          far={scale.farPlaneM}
        />

        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight
          position={[scale.extentM, scale.extentM / 2, scale.extentM]}
          intensity={1.0}
        />

        <Grid sizeM={scale.extentM} sectionSizeM={scale.sectionM} />
        <RICAxes />

        {chiefVector && (
          <VehicleMesh
            vehicle="chief"
            config={chiefConfig}
            positionRicM={CHIEF_ORIGIN_RIC_M}
            scale={scale.spacecraftScale}
            labelFontSizeM={scale.labelFontSizeM}
          />
        )}
        {chiefVector && deputyVector && deputyRicM && (
          <VehicleMesh
            vehicle="deputy"
            config={deputyConfig}
            positionRicM={deputyRicM}
            scale={scale.spacecraftScale}
            labelFontSizeM={scale.labelFontSizeM}
          />
        )}

        {celestials.earth && (
          <CelestialIndicator
            unitRic={celestials.earth}
            edgeM={celestialEdgeM}
            body="earth"
            color={CELESTIAL_TINTS.earth}
          />
        )}
        {celestials.sun && (
          <CelestialIndicator
            unitRic={celestials.sun}
            edgeM={celestialEdgeM}
            body="sun"
            color={CELESTIAL_TINTS.sun}
          />
        )}
        {celestials.moon && (
          <CelestialIndicator
            unitRic={celestials.moon}
            edgeM={celestialEdgeM}
            body="moon"
            color={CELESTIAL_TINTS.moon}
          />
        )}

        <OrbitControls
          enablePan
          enableDamping={ORBIT_CONTROLS.enableDamping}
          minDistance={scale.orbitMinM}
          maxDistance={scale.orbitMaxM}
        />
      </Canvas>
    </div>
  );
}

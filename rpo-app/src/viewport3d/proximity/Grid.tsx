import { useMemo } from 'react';
import { Grid as DreiGrid, Text } from '@react-three/drei';

import type { Vec3 } from '@/viewport3d/types';

import {
  GRID_CELL_SIZE_M,
  GRID_COLORS,
  GRID_LABEL_COLOR,
  GRID_LABEL_FONT_SIZE_M,
  GRID_LABEL_OFFSET_M,
  GRID_SECTION_SIZE_M,
  GRID_SIZE_M,
} from './constants';

function formatDistance(meters: number): string {
  const abs = Math.abs(meters);
  if (abs >= 1000) {
    const km = meters / 1000;
    return Number.isInteger(km) ? `${km}km` : `${km.toFixed(1)}km`;
  }
  return `${meters}m`;
}

type Marker = { position: Vec3; label: string };

export default function Grid() {
  const markers = useMemo<Marker[]>(() => {
    const half = GRID_SIZE_M / 2;
    const interval = GRID_SECTION_SIZE_M;
    const result: Marker[] = [];
    for (let d = interval; d <= half; d += interval) {
      // I-axis (X) labels — sit just off the R=0 line, in the grid plane
      result.push({ position: [d, -GRID_LABEL_OFFSET_M, 0], label: formatDistance(d) });
      result.push({ position: [-d, -GRID_LABEL_OFFSET_M, 0], label: formatDistance(-d) });
      // R-axis (Y) labels — sit just off the I=0 line, in the grid plane
      result.push({ position: [-GRID_LABEL_OFFSET_M, d, 0], label: formatDistance(d) });
      result.push({ position: [-GRID_LABEL_OFFSET_M, -d, 0], label: formatDistance(-d) });
    }
    return result;
  }, []);

  return (
    <group>
      <DreiGrid
        args={[GRID_SIZE_M, GRID_SIZE_M]}
        rotation={[Math.PI / 2, 0, 0]}
        cellSize={GRID_CELL_SIZE_M}
        cellThickness={0.4}
        cellColor={GRID_COLORS.cell}
        sectionSize={GRID_SECTION_SIZE_M}
        sectionThickness={0.8}
        sectionColor={GRID_COLORS.section}
        fadeDistance={GRID_SIZE_M}
        fadeStrength={1.5}
      />
      {markers.map((marker, i) => (
        <Text
          key={`marker-${i}`}
          position={marker.position}
          fontSize={GRID_LABEL_FONT_SIZE_M}
          color={GRID_LABEL_COLOR}
          anchorX="center"
          anchorY="middle"
        >
          {marker.label}
        </Text>
      ))}
    </group>
  );
}

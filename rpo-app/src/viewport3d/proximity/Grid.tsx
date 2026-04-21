import { useMemo } from 'react';
import { Grid as DreiGrid, Text } from '@react-three/drei';

import type { Vec3 } from '@/viewport3d/types';

import { GRID_COLORS, GRID_LABEL_COLOR } from './constants';

// Label font size and axis offset as fractions of the section size, so
// labels stay legible as the grid scales from 1m/section to 500km/section.
// Tuned so the legacy look (section=50m, font=4m, offset=6m) is preserved.
const GRID_LABEL_FONT_RATIO = 0.08;
const GRID_LABEL_OFFSET_RATIO = 0.12;

function formatDistance(meters: number): string {
  const abs = Math.abs(meters);
  if (abs >= 1000) {
    const km = meters / 1000;
    return Number.isInteger(km) ? `${km}km` : `${km.toFixed(1)}km`;
  }
  return Number.isInteger(meters) ? `${meters}m` : `${meters.toFixed(1)}m`;
}

type Marker = { position: Vec3; label: string };

type Props = {
  sizeM: number;
  sectionSizeM: number;
};

// DreiGrid draws 5 cells per section; keep the ratio internal so callers
// can't pass an inconsistent cellSize.
const CELLS_PER_SECTION = 5;

export default function Grid({ sizeM, sectionSizeM }: Props) {
  const cellSizeM = sectionSizeM / CELLS_PER_SECTION;
  const labelFontSizeM = sectionSizeM * GRID_LABEL_FONT_RATIO;

  const markers = useMemo<Marker[]>(() => {
    const half = sizeM / 2;
    const offset = sectionSizeM * GRID_LABEL_OFFSET_RATIO;
    const result: Marker[] = [];
    for (let d = sectionSizeM; d <= half; d += sectionSizeM) {
      // I-axis (X) labels — sit just off the R=0 line, in the grid plane.
      result.push({ position: [d, -offset, 0], label: formatDistance(d) });
      result.push({ position: [-d, -offset, 0], label: formatDistance(-d) });
      // R-axis (Y) labels — sit just off the I=0 line, in the grid plane.
      result.push({ position: [-offset, d, 0], label: formatDistance(d) });
      result.push({ position: [-offset, -d, 0], label: formatDistance(-d) });
    }
    return result;
  }, [sizeM, sectionSizeM]);

  return (
    <group>
      <DreiGrid
        args={[sizeM, sizeM]}
        rotation={[Math.PI / 2, 0, 0]}
        cellSize={cellSizeM}
        cellThickness={0.4}
        cellColor={GRID_COLORS.cell}
        sectionSize={sectionSizeM}
        sectionThickness={0.8}
        sectionColor={GRID_COLORS.section}
        fadeDistance={sizeM}
        fadeStrength={1.5}
      />
      {markers.map((marker, i) => (
        <Text
          key={`marker-${i}`}
          position={marker.position}
          fontSize={labelFontSizeM}
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

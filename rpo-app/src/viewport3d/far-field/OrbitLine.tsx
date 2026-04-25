import { useMemo } from 'react';
import { Line } from '@react-three/drei';

import { pipe } from '@railway-ts/pipelines/composition';
import { flatMapWith, mapWith, match } from '@railway-ts/pipelines/result';

import type { StateVectorInput } from '@/schemas/stateVector';
import type { Vec3 } from '@/viewport3d/types';
import { sampleOrbitEci, stateToKeplerian } from '@/wasm/elements';

import { ORBIT_LINE_WIDTH_PX, ORBIT_POLYLINE_SAMPLES } from './constants';
import { eciKmToScenePosition } from './coordinates';

type Props = {
  state: StateVectorInput;
  color: string;
};

/**
 * Renders one Keplerian orbit as a closed polyline in ECI scene coordinates.
 *
 * Two WASM calls chained via railway-ts: `state_to_keplerian` extracts the
 * orbital elements, then `sample_orbit_eci` produces one period's worth of
 * ECI positions. The polyline closes by appending the first scene point at
 * the end — drei's `<Line>` does not auto-close.
 *
 * Standard depth testing lets Earth occlude the back half — that's the 3D
 * cue. On WASM error, returns `null`.
 */
export default function OrbitLine({ state, color }: Props) {
  const points = useMemo<Vec3[] | null>(
    () =>
      match(
        pipe(
          stateToKeplerian(state),
          flatMapWith((elements) => sampleOrbitEci(elements, ORBIT_POLYLINE_SAMPLES)),
          mapWith((samples) => {
            const scenePts: Vec3[] = samples.map(eciKmToScenePosition);
            if (scenePts.length > 0) scenePts.push(scenePts[0]);
            return scenePts;
          }),
        ),
        { ok: (pts) => pts, err: () => null },
      ),
    [state],
  );

  if (points === null || points.length < 2) return null;

  return <Line points={points} color={color} lineWidth={ORBIT_LINE_WIDTH_PX} />;
}

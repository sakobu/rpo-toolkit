import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import { Quaternion, Vector3 } from 'three';

import { selectTransfer, selectTransferSubSurface, usePlanner } from '@/stores/planner';
import type { Vec3 } from '@/viewport3d/types';

import {
  ARC_LINE_WIDTH_PX,
  DV_ARROW_HEAD_LENGTH,
  DV_ARROW_HEAD_RADIUS,
  DV_ARROW_RADIAL_SEGMENTS,
  DV_ARROW_SHAFT_LENGTH,
  DV_ARROW_SHAFT_RADIUS,
  TRANSFER_ARC_COLOR,
  TRANSFER_ARC_INFEASIBLE_COLOR,
  TRANSFER_ARC_INFEASIBLE_DASH_SIZE,
  TRANSFER_ARC_INFEASIBLE_GAP_SIZE,
} from './constants';
import { eciDirToScene, eciKmToScenePosition } from './coordinates';

// Cylinder + cone geometries both point along +Y by default; one quaternion
// rotates that axis to the Δv unit vector and orients shaft and head together.
const ARROW_AXIS = new Vector3(0, 1, 0);
function quaternionFromYTo(dir: Vector3): Quaternion {
  return new Quaternion().setFromUnitVectors(ARROW_AXIS, dir);
}

// Both primitives are positioned by their centroid, so the centroidOffset
// from the burn point is half-shaft for the shaft and shaft+half-head for
// the head. That places the shaft base at the burn point and the head tip
// at burn point + total arrow length along Δv.
function offsetAlong(base: Vec3, dir: Vector3, centroidOffset: number): Vec3 {
  return [
    base[0] + dir.x * centroidOffset,
    base[1] + dir.y * centroidOffset,
    base[2] + dir.z * centroidOffset,
  ];
}

type Arrow = {
  shaftPos: Vec3;
  headPos: Vec3;
  quat: Quaternion;
};

function buildArrow(burnPos: Vec3, dvDir: Vector3): Arrow {
  return {
    shaftPos: offsetAlong(burnPos, dvDir, DV_ARROW_SHAFT_LENGTH * 0.5),
    headPos: offsetAlong(burnPos, dvDir, DV_ARROW_SHAFT_LENGTH + DV_ARROW_HEAD_LENGTH * 0.5),
    quat: quaternionFromYTo(dvDir),
  };
}

type Scene = {
  points: Vec3[];
  departureArrow: Arrow | null;
  arrivalArrow: Arrow | null;
};

/**
 * Lambert transfer ellipse + Δv burn-vector arrows.
 *
 * Each burn gets a shaft + arrowhead pointing along its Δv direction,
 * starting at the burn point itself (no anchor sphere — the spacecraft icon
 * already marks the position). Arrow length is fixed; magnitude lives in
 * the side-panel readout. Color is the numerical-token orange for feasible
 * conics; switches to signal-abort red + dashed when the perigee dips
 * below `MIN_PERIAPSIS_ALTITUDE_KM`, so the user can see *why* the inputs
 * are bad without confusing it for the chief's free-drift red. Standard
 * depth testing lets Earth occlude the back half. Returns `null` when no
 * transfer is active.
 */
export default function TransferArc() {
  const transfer = usePlanner(selectTransfer);
  const subSurface = usePlanner(selectTransferSubSurface);

  const scene = useMemo<Scene | null>(() => {
    if (transfer === null) return null;
    const arcSamples = transfer.arc_samples_eci_km;
    const lambert = transfer.plan.transfer;
    if (arcSamples === undefined || arcSamples.length < 2 || lambert === undefined) return null;

    const points: Vec3[] = arcSamples.map(eciKmToScenePosition);
    const departurePos = eciKmToScenePosition(lambert.departure_state.position_eci_km);
    const arrivalPos = eciKmToScenePosition(lambert.arrival_state.position_eci_km);

    const departureDv = new Vector3(...eciDirToScene(lambert.departure_dv_eci_km_s));
    const arrivalDv = new Vector3(...eciDirToScene(lambert.arrival_dv_eci_km_s));
    const departureMag = departureDv.length();
    const arrivalMag = arrivalDv.length();
    const departureDir = departureMag > 0 ? departureDv.divideScalar(departureMag) : null;
    const arrivalDir = arrivalMag > 0 ? arrivalDv.divideScalar(arrivalMag) : null;

    return {
      points,
      departureArrow: departureDir ? buildArrow(departurePos, departureDir) : null,
      arrivalArrow: arrivalDir ? buildArrow(arrivalPos, arrivalDir) : null,
    };
  }, [transfer]);

  if (scene === null) return null;

  const isInfeasible = subSurface !== null;
  const lineColor = isInfeasible ? TRANSFER_ARC_INFEASIBLE_COLOR : TRANSFER_ARC_COLOR;

  return (
    <group>
      <Line
        points={scene.points}
        color={lineColor}
        lineWidth={ARC_LINE_WIDTH_PX}
        dashed={isInfeasible}
        dashSize={TRANSFER_ARC_INFEASIBLE_DASH_SIZE}
        gapSize={TRANSFER_ARC_INFEASIBLE_GAP_SIZE}
      />
      {scene.departureArrow !== null && <DvArrow arrow={scene.departureArrow} color={lineColor} />}
      {scene.arrivalArrow !== null && <DvArrow arrow={scene.arrivalArrow} color={lineColor} />}
    </group>
  );
}

function DvArrow({ arrow, color }: { arrow: Arrow; color: string }) {
  return (
    <group>
      <mesh position={arrow.shaftPos} quaternion={arrow.quat}>
        <cylinderGeometry
          args={[
            DV_ARROW_SHAFT_RADIUS,
            DV_ARROW_SHAFT_RADIUS,
            DV_ARROW_SHAFT_LENGTH,
            DV_ARROW_RADIAL_SEGMENTS,
          ]}
        />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={arrow.headPos} quaternion={arrow.quat}>
        <coneGeometry
          args={[DV_ARROW_HEAD_RADIUS, DV_ARROW_HEAD_LENGTH, DV_ARROW_RADIAL_SEGMENTS]}
        />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
}

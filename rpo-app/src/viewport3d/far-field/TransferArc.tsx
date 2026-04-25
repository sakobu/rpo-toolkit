import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import { Quaternion, Vector3 } from 'three';

import { usePlanner } from '@/stores/planner';
import type { Vec3 } from '@/viewport3d/types';

import {
  ARC_ENDPOINT_DOT_RADIUS,
  ARC_ENDPOINT_DOT_SEGMENTS,
  ARC_LINE_WIDTH_PX,
  DV_CHEVRON_LENGTH,
  DV_CHEVRON_RADIAL_SEGMENTS,
  DV_CHEVRON_RADIUS,
  TRANSFER_ARC_COLOR,
} from './constants';
import { eciDirToScene, eciKmToScenePosition } from './coordinates';

type Props = {
  color?: string;
};

// Three's coneGeometry points along +Y by default; build a quaternion that
// rotates that axis to the supplied unit vector.
const CONE_AXIS = new Vector3(0, 1, 0);
function quaternionFromYTo(dir: Vector3): Quaternion {
  return new Quaternion().setFromUnitVectors(CONE_AXIS, dir);
}

// Chevron's coneGeometry origin is the centroid; offset by half the cone
// length along `dir` so the base sits on the burn point and the apex points
// outward along Δv.
function chevronPos(base: Vec3, dir: Vector3, halfLength: number): Vec3 {
  return [base[0] + dir.x * halfLength, base[1] + dir.y * halfLength, base[2] + dir.z * halfLength];
}

type Scene = {
  points: Vec3[];
  departurePos: Vec3;
  arrivalPos: Vec3;
  departureDir: Vector3 | null;
  arrivalDir: Vector3 | null;
  departureChevronPos: Vec3 | null;
  arrivalChevronPos: Vec3 | null;
  departureQuat: Quaternion | null;
  arrivalQuat: Quaternion | null;
};

/**
 * Lambert transfer ellipse + endpoint dots + Δv chevrons.
 *
 * Server returns the transfer orbit sampled once around its underlying ellipse
 * regardless of revolution count (single-rev → partial arc, multi-rev → closed
 * loop). Color (the "numerical" orange token) is the only signal distinguishing
 * the transfer from chief/deputy. `depthWrite={false}` prevents the line from
 * fighting overlapping overlays; standard depth testing still lets Earth
 * occlude the back half. Returns `null` when no transfer is active.
 */
export default function TransferArc({ color = TRANSFER_ARC_COLOR }: Props) {
  const transfer = usePlanner((s) => s.transfer);

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

    const halfLength = DV_CHEVRON_LENGTH * 0.5;
    return {
      points,
      departurePos,
      arrivalPos,
      departureDir,
      arrivalDir,
      departureChevronPos: departureDir ? chevronPos(departurePos, departureDir, halfLength) : null,
      arrivalChevronPos: arrivalDir ? chevronPos(arrivalPos, arrivalDir, halfLength) : null,
      departureQuat: departureDir ? quaternionFromYTo(departureDir) : null,
      arrivalQuat: arrivalDir ? quaternionFromYTo(arrivalDir) : null,
    };
  }, [transfer]);

  if (scene === null) return null;

  return (
    <group>
      <Line points={scene.points} color={color} lineWidth={ARC_LINE_WIDTH_PX} depthWrite={false} />
      <mesh position={scene.departurePos}>
        <sphereGeometry
          args={[ARC_ENDPOINT_DOT_RADIUS, ARC_ENDPOINT_DOT_SEGMENTS, ARC_ENDPOINT_DOT_SEGMENTS]}
        />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={scene.arrivalPos}>
        <sphereGeometry
          args={[ARC_ENDPOINT_DOT_RADIUS, ARC_ENDPOINT_DOT_SEGMENTS, ARC_ENDPOINT_DOT_SEGMENTS]}
        />
        <meshBasicMaterial color={color} />
      </mesh>
      {scene.departureChevronPos !== null && scene.departureQuat !== null && (
        <mesh position={scene.departureChevronPos} quaternion={scene.departureQuat}>
          <coneGeometry args={[DV_CHEVRON_RADIUS, DV_CHEVRON_LENGTH, DV_CHEVRON_RADIAL_SEGMENTS]} />
          <meshBasicMaterial color={color} />
        </mesh>
      )}
      {scene.arrivalChevronPos !== null && scene.arrivalQuat !== null && (
        <mesh position={scene.arrivalChevronPos} quaternion={scene.arrivalQuat}>
          <coneGeometry args={[DV_CHEVRON_RADIUS, DV_CHEVRON_LENGTH, DV_CHEVRON_RADIAL_SEGMENTS]} />
          <meshBasicMaterial color={color} />
        </mesh>
      )}
    </group>
  );
}

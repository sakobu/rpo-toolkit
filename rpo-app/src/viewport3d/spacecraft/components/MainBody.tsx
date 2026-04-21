import {
  ACCENT_STRIPE_HEIGHT,
  ACCENT_STRIPE_INTENSITY,
  BUS_COLOR,
  DOCKING_COLLAR_COLOR,
  DOCKING_COLLAR_LENGTH,
  DOCKING_COLLAR_RADIUS_FRAC,
  THRUSTER_COLOR,
  THRUSTER_COUNT,
  THRUSTER_NOZZLE_APEX_RADIUS,
  THRUSTER_NOZZLE_BASE_RADIUS,
  THRUSTER_NOZZLE_LENGTH,
  THRUSTER_PLACEMENT_RADIUS_FRAC,
} from '../config/constants';
import type { MainBodyProps } from '../types';

const RADIAL_SEGMENTS = 48;
const NOZZLE_SEGMENTS = 20;
const STRIPE_OVERHANG = 0.05;

// cylinderGeometry's native axis is +Y. The outer group rotates +π/2 around X
// so inside the group local +Y maps to world +Z; end-cap positions below use
// local Y for "along the tube axis."

export function MainBody({ width, height, depth, accent }: MainBodyProps) {
  const halfWidth = width / 2;
  const halfHeight = height / 2;

  const busScale = [halfWidth, 1, halfHeight] satisfies [number, number, number];
  const stripeScale = [(width + STRIPE_OVERHANG) / 2, 1, (height + STRIPE_OVERHANG) / 2] satisfies [
    number,
    number,
    number,
  ];

  const collarRadiusX = halfWidth * DOCKING_COLLAR_RADIUS_FRAC;
  const collarRadiusZ = halfHeight * DOCKING_COLLAR_RADIUS_FRAC;
  const collarY = depth / 2 + DOCKING_COLLAR_LENGTH / 2;

  const thrusterRadiusX = halfWidth * THRUSTER_PLACEMENT_RADIUS_FRAC;
  const thrusterRadiusZ = halfHeight * THRUSTER_PLACEMENT_RADIUS_FRAC;
  const thrusterY = -depth / 2 - THRUSTER_NOZZLE_LENGTH / 2;

  return (
    <group rotation={[Math.PI / 2, 0, 0]}>
      <mesh scale={busScale}>
        <cylinderGeometry args={[1, 1, depth, RADIAL_SEGMENTS]} />
        <meshStandardMaterial color={BUS_COLOR} metalness={0.05} roughness={0.75} />
      </mesh>

      <mesh scale={stripeScale}>
        <cylinderGeometry args={[1, 1, ACCENT_STRIPE_HEIGHT, RADIAL_SEGMENTS]} />
        <meshStandardMaterial
          color={accent}
          emissive={accent}
          emissiveIntensity={ACCENT_STRIPE_INTENSITY}
          metalness={0.1}
          roughness={0.6}
        />
      </mesh>

      <mesh position={[0, collarY, 0]} scale={[collarRadiusX, 1, collarRadiusZ]}>
        <cylinderGeometry args={[1, 1, DOCKING_COLLAR_LENGTH, RADIAL_SEGMENTS]} />
        <meshStandardMaterial color={DOCKING_COLLAR_COLOR} metalness={0.7} roughness={0.35} />
      </mesh>

      {Array.from({ length: THRUSTER_COUNT }, (_, i) => {
        const angle = (i / THRUSTER_COUNT) * Math.PI * 2;
        const x = thrusterRadiusX * Math.cos(angle);
        const z = thrusterRadiusZ * Math.sin(angle);
        return (
          <mesh key={`nozzle-${i}`} position={[x, thrusterY, z]}>
            <cylinderGeometry
              args={[
                THRUSTER_NOZZLE_APEX_RADIUS,
                THRUSTER_NOZZLE_BASE_RADIUS,
                THRUSTER_NOZZLE_LENGTH,
                NOZZLE_SEGMENTS,
              ]}
            />
            <meshStandardMaterial color={THRUSTER_COLOR} metalness={0.5} roughness={0.4} />
          </mesh>
        );
      })}
    </group>
  );
}

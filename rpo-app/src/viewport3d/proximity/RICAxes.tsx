import { Billboard, GizmoHelper, Line, Sphere, Text } from '@react-three/drei';

import type { Vec3 } from '@/viewport3d/types';

import { AXES, AXES_GIZMO } from './constants';

type AxisProps = {
  direction: Vec3;
  color: string;
  label: string;
};

function Axis({ direction, color, label }: AxisProps) {
  const end: Vec3 = [
    direction[0] * AXES_GIZMO.lineLength,
    direction[1] * AXES_GIZMO.lineLength,
    direction[2] * AXES_GIZMO.lineLength,
  ];
  return (
    <group>
      <Line points={[[0, 0, 0], end]} color={color} lineWidth={AXES_GIZMO.lineWidth} />
      <Sphere
        args={[AXES_GIZMO.sphereRadius, AXES_GIZMO.sphereSegments, AXES_GIZMO.sphereSegments]}
        position={end}
      >
        <meshBasicMaterial color={color} />
      </Sphere>
      <Billboard position={end}>
        <Text
          fontSize={AXES_GIZMO.labelFontSize}
          color={AXES_GIZMO.labelColor}
          anchorX="center"
          anchorY="middle"
          fontWeight="bold"
          renderOrder={1}
          material-depthTest={false}
        >
          {label}
        </Text>
      </Billboard>
    </group>
  );
}

// R = +Y (up), I = +X (right), C = +Z (depth) — matches ricToPosition mapping.
export default function RICAxes() {
  return (
    <GizmoHelper alignment="bottom-right" margin={[...AXES_GIZMO.margin]}>
      <group scale={AXES_GIZMO.scale}>
        <Axis direction={[0, 1, 0]} color={AXES.R.color} label={AXES.R.label} />
        <Axis direction={[1, 0, 0]} color={AXES.I.color} label={AXES.I.label} />
        <Axis direction={[0, 0, 1]} color={AXES.C.color} label={AXES.C.label} />
      </group>
    </GizmoHelper>
  );
}

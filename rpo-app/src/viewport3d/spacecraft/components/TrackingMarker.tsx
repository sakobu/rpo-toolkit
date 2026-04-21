import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { PerspectiveCamera, type Sprite, Vector3 } from 'three';

import { MARKER_PIXEL_SIZE } from '../config/constants';
import type { TrackingMarkerProps } from '../types';

// Screen-space tracking marker. The previous NavigationLight used a pointLight
// that lit the Earth in the far-field view; a sprite has zero scene-lighting
// contribution.

const SCALE_CHANGE_EPS = 1e-4;

export function TrackingMarker({
  position,
  color,
  pixelSize = MARKER_PIXEL_SIZE,
}: TrackingMarkerProps) {
  const ref = useRef<Sprite>(null);
  const worldPos = useMemo(() => new Vector3(), []);
  const prevScale = useRef(0);

  useFrame((state) => {
    const sprite = ref.current;
    if (!sprite) return;
    if (!(state.camera instanceof PerspectiveCamera)) return;
    sprite.getWorldPosition(worldPos);
    const distance = state.camera.position.distanceTo(worldPos);
    const fovRad = (state.camera.fov * Math.PI) / 180;
    const worldPerPixel = (2 * distance * Math.tan(fovRad / 2)) / state.size.height;
    const s = worldPerPixel * pixelSize;
    if (Math.abs(s - prevScale.current) < SCALE_CHANGE_EPS) return;
    prevScale.current = s;
    sprite.scale.set(s, s, 1);
  });

  return (
    <sprite ref={ref} position={position} renderOrder={999}>
      <spriteMaterial color={color} transparent depthTest={false} depthWrite={false} />
    </sprite>
  );
}

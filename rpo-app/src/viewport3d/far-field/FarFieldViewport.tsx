import { OrbitControls } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';

import Earth from './Earth';

export default function FarFieldViewport() {
  return (
    <div className="h-full w-full">
      <Canvas camera={{ position: [0, 0, 6], fov: 50, near: 0.1, far: 100 }} dpr={[1, 2]}>
        <ambientLight intensity={0.35} />
        <hemisphereLight args={['#8aa0c0', '#1a1f2c', 0.4]} />
        <directionalLight position={[10, 5, 10]} intensity={1.0} />
        <Earth />
        <OrbitControls enablePan={false} minDistance={3.5} maxDistance={20} />
      </Canvas>
    </div>
  );
}

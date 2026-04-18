import { Navigate, Outlet } from 'react-router';

import HUD from '@/chrome/HUD';
import MissionDock from '@/chrome/MissionDock';
import Sidebar from '@/chrome/Sidebar';
import { useMission } from '@/stores/mission';

export function ViewportShell() {
  const classification = useMission((s) => s.classification);

  if (classification.status !== 'ok') {
    return <Navigate to="/" replace />;
  }

  return (
    <div className="fixed inset-0 bg-bg">
      <Outlet />
      <Sidebar />
      <HUD />
      <MissionDock />
    </div>
  );
}

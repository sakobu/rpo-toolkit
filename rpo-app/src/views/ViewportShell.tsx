import { Navigate, Outlet } from 'react-router';

import HUD from '@/components/overlay/HUD';
import MissionControlPanel from '@/components/overlay/MissionControlPanel';
import Sidebar from '@/components/overlay/Sidebar';
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
      <MissionControlPanel />
    </div>
  );
}

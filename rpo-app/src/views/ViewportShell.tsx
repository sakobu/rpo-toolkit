import { Navigate, Outlet } from 'react-router';

import { MissionDock } from '@/chrome/dock/MissionDock';
import { HotkeyLegend } from '@/chrome/HotkeyLegend';
import { HUD } from '@/chrome/hud/HUD';
import { Sidebar } from '@/chrome/sidebar/Sidebar';
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
      <HotkeyLegend />
    </div>
  );
}

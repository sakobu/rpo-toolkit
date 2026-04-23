import { Navigate, Outlet } from 'react-router';

import { MissionDock } from '@/chrome/dock/MissionDock';
import { HotkeyLegend } from '@/chrome/HotkeyLegend';
import { HUD } from '@/chrome/hud/HUD';
import { Sidebar } from '@/chrome/sidebar/Sidebar';
import { useHotkey } from '@/hooks/useHotkey';
import { useMission } from '@/stores/mission';
import { useUI } from '@/stores/ui';

export function ViewportShell() {
  const classification = useMission((s) => s.classification);
  const toggleHideAll = useUI((s) => s.toggleHideAll);
  useHotkey('h', toggleHideAll, { shift: true });

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

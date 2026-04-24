import { useEffect } from 'react';
import { Navigate, Outlet } from 'react-router';

import { MissionDock } from '@/chrome/dock/MissionDock';
import { HotkeyLegend } from '@/chrome/HotkeyLegend';
import { HUD } from '@/chrome/hud/HUD';
import { Sidebar } from '@/chrome/sidebar/Sidebar';
import { useHotkey } from '@/hooks/useHotkey';
import { usePlanner } from '@/stores/planner';
import { useUI } from '@/stores/ui';
import { connect, disconnect } from '@/ws/client';

export function ViewportShell() {
  const classification = usePlanner((s) => s.classification);
  const toggleHideAll = useUI((s) => s.toggleHideAll);
  useHotkey('h', toggleHideAll, { shift: true });

  useEffect(() => {
    connect();
    return () => {
      disconnect();
    };
  }, []);

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

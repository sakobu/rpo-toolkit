import { Link, Navigate, Outlet } from 'react-router';

import { useMission } from '@/stores/mission';

export function ViewportShell() {
  const classification = useMission((s) => s.classification);

  if (classification.status !== 'ok') {
    return <Navigate to="/" replace />;
  }

  return (
    <div className="fixed inset-0 bg-bg">
      <Outlet />
      <div className="pointer-events-none absolute inset-0">
        <div className="pointer-events-auto absolute top-4 left-4">
          <Link
            to="/"
            className="rounded-sm border border-border/60 bg-surface-1/70 px-2 py-1 font-mono text-[10px] tracking-wider text-text-dim uppercase backdrop-blur-sm hover:text-text-muted"
          >
            ← setup
          </Link>
        </div>
      </div>
    </div>
  );
}

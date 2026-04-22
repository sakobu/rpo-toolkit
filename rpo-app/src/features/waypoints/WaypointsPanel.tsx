import { Caps } from '@/ui/Caps';

export function WaypointsPanel() {
  return (
    <div className="flex flex-col gap-2.5">
      <Caps>waypoints</Caps>
      <p className="font-mono text-[10px] leading-relaxed text-text-dim">
        waypoint editor + propagator picker — wired in Phase 4 (shift-click placement). Waypoints
        feed the mission-dock timeline in Phase 3.
      </p>
    </div>
  );
}

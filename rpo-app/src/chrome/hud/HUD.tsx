import { useLocation } from 'react-router';
import { GripHorizontal } from 'lucide-react';

import { HUD_WIDTH_CLASS } from '@/chrome/constants';
import { ConnectionIndicator } from '@/chrome/hud/ConnectionIndicator';
import { HUDCollapsed } from '@/chrome/hud/HUDCollapsed';
import { PanelHeader } from '@/chrome/PanelHeader';
import { useHotkey } from '@/hooks/useHotkey';
import { type HudReadoutFrame, useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';
import { Chip } from '@/ui/Chip';
import { KV } from '@/ui/KV';
import { SegControl } from '@/ui/SegControl';

export function HUD() {
  const hud = useUI((s) => s.hud);
  const cycleHud = useUI((s) => s.cycleHud);
  const hide = useUI((s) => s.hide);
  const hudReadoutFrame = useUI((s) => s.hudReadoutFrame);
  const setHudReadoutFrame = useUI((s) => s.setHudReadoutFrame);
  const { pathname } = useLocation();
  const isFarField = pathname.startsWith('/far-field');

  useHotkey('h', cycleHud);

  if (hud === 'hidden') return null;
  if (hud === 'collapsed') return <HUDCollapsed />;

  return (
    <aside
      className={`pointer-events-auto fixed top-4 right-4 z-30 ${HUD_WIDTH_CLASS} rounded-xs border border-border bg-surface-1/85 backdrop-blur-sm`}
    >
      <PanelHeader
        leading={<GripHorizontal size={14} strokeWidth={1.5} />}
        label={<Caps>hud</Caps>}
        meta={
          <span className="flex items-center gap-1">
            <ConnectionIndicator />
            <Chip tone="muted">{isFarField ? 'ECI' : 'RIC'}</Chip>
          </span>
        }
        onCollapse={cycleHud}
        onHide={() => hide('hud')}
        collapseLabel="collapse hud"
        hideLabel="hide hud"
      />
      <div className="flex flex-col gap-2 px-3 py-3">
        {isFarField ? (
          <FarFieldReadout />
        ) : (
          <ProximityReadout frame={hudReadoutFrame} onFrameChange={setHudReadoutFrame} />
        )}
      </div>
    </aside>
  );
}

function FarFieldReadout() {
  return (
    <div className="flex flex-col gap-2 font-mono text-[11px] text-text-dim">
      <KV k="range to chief" v="—" />
      <KV k="transfer elapsed" v="—" />
      <KV k="Δv spent / total" v="— / —" />
    </div>
  );
}

function ProximityReadout({
  frame,
  onFrameChange,
}: {
  frame: HudReadoutFrame;
  onFrameChange: (f: HudReadoutFrame) => void;
}) {
  return (
    <div className="flex flex-col gap-2 font-mono text-[11px] text-text-dim">
      <div className="flex items-center justify-between">
        <Caps>frame</Caps>
        <div className="flex gap-0.5 text-[9px]">
          <SegControl active={frame === 'ric'} onClick={() => onFrameChange('ric')}>
            RIC
          </SegControl>
          <SegControl active={frame === 'roe'} onClick={() => onFrameChange('roe')}>
            ROE
          </SegControl>
        </div>
      </div>
      <KV k="range" v="—" />
      <KV k="traveled" v="—" />
      <KV k="min d3d" v="—" />
    </div>
  );
}

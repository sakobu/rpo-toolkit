import { useLocation } from 'react-router';
import { GripHorizontal } from 'lucide-react';

import { HUD_WIDTH_CLASS } from '@/chrome/constants';
import { HUDCollapsed } from '@/chrome/hud/HUDCollapsed';
import { PanelHeader } from '@/chrome/PanelHeader';
import { useHotkey } from '@/hooks/useHotkey';
import { type HudReadoutFrame, useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';
import { Chip } from '@/ui/Chip';
import { KV } from '@/ui/KV';
import { withBlur } from '@/utils/blur';

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
        meta={<Chip tone="muted">{isFarField ? 'ECI' : 'RIC'}</Chip>}
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
        <div className="flex gap-0.5">
          <SegButton active={frame === 'ric'} onClick={() => onFrameChange('ric')}>
            RIC
          </SegButton>
          <SegButton active={frame === 'roe'} onClick={() => onFrameChange('roe')}>
            ROE
          </SegButton>
        </div>
      </div>
      <KV k="range" v="—" />
      <KV k="traveled" v="—" />
      <KV k="min d3d" v="—" />
    </div>
  );
}

function SegButton({
  active,
  onClick,
  children,
}: {
  active: boolean;
  onClick: () => void;
  children: React.ReactNode;
}) {
  return (
    <button
      type="button"
      onClick={withBlur(onClick)}
      className={`duration-fast cursor-pointer rounded-xs border px-2 py-0.5 font-mono text-[9px] tracking-wider uppercase transition-colors ease-out ${
        active
          ? 'border-accent bg-accent-dim text-accent'
          : 'border-border bg-transparent text-text-muted hover:text-text'
      }`}
    >
      {children}
    </button>
  );
}

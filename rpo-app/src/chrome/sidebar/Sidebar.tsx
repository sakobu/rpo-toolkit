import type { ReactNode } from 'react';
import { useNavigate } from 'react-router';
import { GripVertical } from 'lucide-react';
import { useShallow } from 'zustand/react/shallow';

import { SIDEBAR_WIDTH_CLASS } from '@/chrome/constants';
import { PanelHeader } from '@/chrome/PanelHeader';
import { MissionHeader } from '@/chrome/sidebar/MissionHeader';
import { PhaseCard } from '@/chrome/sidebar/PhaseCard';
import { SidebarRail } from '@/chrome/sidebar/SidebarRail';
import { TransferPanel } from '@/features/transfer/TransferPanel';
import { WaypointsPanel } from '@/features/waypoints/WaypointsPanel';
import { useHotkey } from '@/hooks/useHotkey';
import { PHASE_KEYS, type PhaseKey, type PhaseState, usePlanner } from '@/stores/planner';
import { useUI } from '@/stores/ui';
import { Caps } from '@/ui/Caps';

type PhaseRenderers = {
  renderSummary?: (state: PhaseState) => ReactNode;
  renderPanel?: () => ReactNode;
};

const PHASE_RENDERERS: Record<PhaseKey, PhaseRenderers> = {
  xfr: {
    renderSummary: (state) =>
      state === 'skipped' ? <Caps>already in proximity · no Lambert needed</Caps> : null,
    renderPanel: () => <TransferPanel />,
  },
  px: {
    renderPanel: () => <WaypointsPanel />,
  },
  val: {
    renderSummary: () => <Caps>full physics · on-demand</Caps>,
  },
  mc: {
    renderSummary: () => <Caps>ensemble · on-demand</Caps>,
  },
};

export function Sidebar() {
  const navigate = useNavigate();
  const sidebar = useUI((s) => s.sidebar);
  const cycleSidebar = useUI((s) => s.cycleSidebar);
  const hide = useUI((s) => s.hide);
  const phases = usePlanner(useShallow((s) => s.phases));
  const editPhase = usePlanner((s) => s.editPhase);

  // Editing a completed phase reverts the viewport to where that phase's
  // planning happens — xfr → far-field (Lambert is planned in ECI). Pair the
  // state reset with the route change at the call site, mirroring the
  // acceptTransfer + navigate('/proximity') pattern in TransferPanel.
  const handleEdit = (key: PhaseKey) => {
    editPhase(key);
    if (key === 'xfr') void navigate('/far-field');
  };

  useHotkey('s', cycleSidebar);

  if (sidebar === 'hidden') return null;
  if (sidebar === 'collapsed') return <SidebarRail />;

  return (
    <aside
      className={`pointer-events-auto fixed top-0 left-0 z-30 flex h-screen ${SIDEBAR_WIDTH_CLASS} flex-col border-r border-border bg-surface-1`}
    >
      <PanelHeader
        leading={<GripVertical size={14} strokeWidth={1.5} />}
        label={<Caps>sidebar</Caps>}
        onCollapse={cycleSidebar}
        onHide={() => hide('sidebar')}
        collapseLabel="collapse sidebar"
        hideLabel="hide sidebar"
      />
      <MissionHeader />
      <div className="border-b border-border px-4 py-2">
        <Caps>planning phases</Caps>
      </div>
      <div className="flex-1 overflow-y-auto">
        {PHASE_KEYS.map((key) => {
          const record = phases[key];
          const renderers = PHASE_RENDERERS[key];
          return (
            <PhaseCard
              key={key}
              phaseKey={key}
              state={record.state}
              dimmedBy={record.dimmedBy}
              resets={record.resets}
              onEdit={() => handleEdit(key)}
              summary={renderers.renderSummary?.(record.state)}
            >
              {record.state === 'active' && renderers.renderPanel?.()}
            </PhaseCard>
          );
        })}
      </div>
    </aside>
  );
}

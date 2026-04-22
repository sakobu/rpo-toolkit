import { useNavigate } from 'react-router';
import { ArrowRight } from 'lucide-react';

import { usePhases } from '@/stores/phases';
import { Caps } from '@/ui/Caps';
import { withBlur } from '@/utils/blur';

export function TransferPanel() {
  const navigate = useNavigate();

  const handleAccept = () => {
    usePhases.getState().acceptTransfer();
    void navigate('/proximity');
  };

  return (
    <div className="flex flex-col gap-2.5">
      <Caps>lambert transfer</Caps>
      <p className="font-mono text-[10px] leading-relaxed text-text-dim">
        perch geometry, offset, TOF, revs, Δv breakdown — wired in Phase 2 (WebSocket + Lambert).
      </p>
      <button
        type="button"
        onClick={withBlur(handleAccept)}
        className="mt-1 flex cursor-pointer items-center justify-center gap-1.5 rounded-xs border border-accent bg-accent-dim px-2 py-1.5 font-mono text-[10px] tracking-wider text-accent uppercase hover:bg-accent/10"
      >
        accept transfer
        <ArrowRight size={11} strokeWidth={1.75} />
      </button>
      <p className="font-mono text-[9px] tracking-wide text-text-dim">
        stub · accepts without Lambert until Phase 2
      </p>
    </div>
  );
}

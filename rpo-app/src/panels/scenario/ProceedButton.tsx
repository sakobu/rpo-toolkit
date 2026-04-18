import { useEffect, useRef } from 'react';
import { useNavigate } from 'react-router';

import { useMission } from '@/stores/mission';

type ProceedTarget =
  | { enabled: true; label: string; to: string }
  | { enabled: false; label: string };

export function ProceedButton() {
  const classification = useMission((s) => s.classification);
  const navigate = useNavigate();
  const ref = useRef<HTMLButtonElement>(null);

  const target = resolveTarget(classification);

  useEffect(() => {
    if (target.enabled) ref.current?.focus();
  }, [target.enabled]);

  const handleClick = () => {
    if (target.enabled) void navigate(target.to);
  };

  const base =
    'duration-fast rounded-sm border px-4 py-2 font-mono text-xs tracking-wider uppercase transition-colors';
  const variant = target.enabled
    ? 'cursor-pointer border-accent bg-accent/10 text-accent hover:bg-accent/20'
    : 'cursor-not-allowed border-border bg-surface-2 text-text-dim';

  return (
    <button
      ref={ref}
      type="button"
      onClick={handleClick}
      disabled={!target.enabled}
      className={`${base} ${variant}`}
    >
      {target.label}
    </button>
  );
}

function resolveTarget(
  classification: ReturnType<typeof useMission.getState>['classification'],
): ProceedTarget {
  if (classification.status === 'idle') {
    return { enabled: false, label: 'complete setup to proceed' };
  }
  if (classification.status === 'err') {
    return { enabled: false, label: 'fix classification error to proceed' };
  }
  const isProximity = 'proximity' in classification.phase;
  return isProximity
    ? { enabled: true, label: 'Proceed to RIC view →', to: '/mission' }
    : { enabled: true, label: 'Proceed to ECI globe →', to: '/transfer' };
}

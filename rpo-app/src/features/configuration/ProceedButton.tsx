import { useEffect, useRef } from 'react';
import { useNavigate } from 'react-router';

import { useMission } from '@/stores/mission';
import { Button } from '@/ui/Button';

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

  return (
    <Button ref={ref} onClick={handleClick} disabled={!target.enabled}>
      {target.label}
    </Button>
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
    ? { enabled: true, label: 'Proceed to proximity view →', to: '/proximity' }
    : { enabled: true, label: 'Proceed to far-field view →', to: '/far-field' };
}

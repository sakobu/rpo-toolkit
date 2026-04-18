import type { MissionPhase } from 'rpo-wasm';

import { ROE_THRESHOLD_SOFT_MAX } from '@/schemas/proximityConfig';
import { useMission } from '@/stores/mission';
import { Callout } from '@/ui/Callout';

const BORDERLINE_FRACTION = 0.1;

export function ThresholdAdvisories() {
  const threshold = useMission((s) => s.proximityConfig.roe_threshold);
  const classification = useMission((s) => s.classification);

  const softZone = threshold > ROE_THRESHOLD_SOFT_MAX;
  const deltaROverR =
    classification.status === 'ok' ? extractDeltaROverR(classification.phase) : null;
  const borderline =
    deltaROverR !== null && Math.abs(deltaROverR - threshold) / threshold < BORDERLINE_FRACTION;

  if (!softZone && !borderline) return null;

  return (
    <div className="flex flex-col gap-2">
      {softZone ? (
        <Callout tone="hold">
          threshold above D'Amico recommendation ({ROE_THRESHOLD_SOFT_MAX}) — ROE linearization
          errors may exceed J2 modeling residuals. Proximity results are advisory in this regime.
        </Callout>
      ) : null}
      {borderline && deltaROverR !== null ? (
        <Callout tone="hold">
          scenario δr/r ({deltaROverR.toExponential(2)}) is within{' '}
          {Math.round(BORDERLINE_FRACTION * 100)}% of threshold ({threshold}) — small parameter
          changes will flip classification.
        </Callout>
      ) : null}
    </div>
  );
}

function extractDeltaROverR(phase: MissionPhase): number {
  return 'proximity' in phase ? phase.proximity.delta_r_over_r : phase.far_field.delta_r_over_r;
}

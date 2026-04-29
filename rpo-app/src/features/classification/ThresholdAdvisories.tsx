import type { MissionPhase } from 'rpo-wasm';

import { selectProximityConfig, usePlanner } from '@/stores/planner';
import { Callout } from '@/ui/Callout';
import { getEngineConstants } from '@/wasm/constants';

const BORDERLINE_FRACTION = 0.1;

export function ThresholdAdvisories() {
  const threshold = usePlanner((s) => selectProximityConfig(s.proximityConfig).roe_threshold);
  const classification = usePlanner((s) => s.classification);

  const softMax = getEngineConstants().roe_threshold_default;
  const softZone = threshold > softMax;
  const deltaROverR =
    classification.status === 'ok' ? extractDeltaROverR(classification.phase) : null;
  const borderline =
    deltaROverR !== null && Math.abs(deltaROverR - threshold) / threshold < BORDERLINE_FRACTION;

  if (!softZone && !borderline) return null;

  return (
    <div className="flex flex-col gap-2">
      {softZone ? (
        <Callout tone="hold">
          threshold above D'Amico recommendation ({softMax}) — ROE linearization errors may exceed
          J2 modeling residuals. Proximity results are advisory in this regime.
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

import { useShallow } from 'zustand/react/shallow';

import { ProceedButton } from './ProceedButton';
import { ThresholdAdvisories } from './ThresholdAdvisories';
import { ThresholdInput } from './ThresholdInput';
import { VehicleStatePanel } from './VehicleStatePanel';
import { ClassificationResult } from '../mission/ClassificationResult';
import { Callout } from '../../components/primitives/Callout';
import { useScenario } from '../../stores/scenario';

export function ScenarioStep2() {
  const { chiefState, deputyState } = useScenario(
    useShallow((s) => ({ chiefState: s.chiefState, deputyState: s.deputyState })),
  );

  const chiefVector = chiefState.status === 'loaded' ? chiefState.vector : null;
  const deputyVector = deputyState.status === 'loaded' ? deputyState.vector : null;
  const epochMismatch =
    chiefVector !== null && deputyVector !== null && chiefVector.epoch !== deputyVector.epoch;

  return (
    <section className="flex flex-col gap-4">
      <div className="grid gap-6 md:grid-cols-2">
        <VehicleStatePanel vehicle="deputy" />
        <VehicleStatePanel vehicle="chief" />
      </div>
      {epochMismatch && chiefVector && deputyVector ? (
        <Callout tone="abort">
          epoch mismatch — chief {chiefVector.epoch} vs deputy {deputyVector.epoch}
        </Callout>
      ) : null}
      <ClassificationResult />
      <ThresholdInput />
      <ThresholdAdvisories />
      <div className="flex justify-end">
        <ProceedButton />
      </div>
    </section>
  );
}

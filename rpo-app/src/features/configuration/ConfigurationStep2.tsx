import { useShallow } from 'zustand/react/shallow';

import { ClassificationResult } from '@/features/classification/ClassificationResult';
import { ThresholdAdvisories } from '@/features/classification/ThresholdAdvisories';
import { loadedVector, useConfig } from '@/stores/configuration';
import { Callout } from '@/ui/Callout';

import { ProceedButton } from './ProceedButton';
import { ThresholdInput } from './ThresholdInput';
import { VehicleStatePanel } from './VehicleStatePanel';

export function ConfigurationStep2() {
  const { chiefState, deputyState } = useConfig(
    useShallow((s) => ({ chiefState: s.chiefState, deputyState: s.deputyState })),
  );

  const chiefVector = loadedVector(chiefState);
  const deputyVector = loadedVector(deputyState);
  const epochMismatch =
    chiefVector !== null && deputyVector !== null && chiefVector.epoch !== deputyVector.epoch;

  return (
    <section className="flex flex-col gap-4">
      <div className="grid gap-6 md:grid-cols-2">
        <VehicleStatePanel vehicle="chief" />
        <VehicleStatePanel vehicle="deputy" />
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

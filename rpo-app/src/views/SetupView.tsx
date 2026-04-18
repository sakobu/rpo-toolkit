import { ScenarioStep1 } from '@/panels/scenario/ScenarioStep1';
import { ScenarioStep2 } from '@/panels/scenario/ScenarioStep2';

export function SetupView() {
  return (
    <div className="flex flex-col gap-10">
      <ScenarioStep1 />
      <ScenarioStep2 />
    </div>
  );
}

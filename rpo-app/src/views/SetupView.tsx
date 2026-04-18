import { ConfigurationStep1 } from '@/features/configuration/ConfigurationStep1';
import { ConfigurationStep2 } from '@/features/configuration/ConfigurationStep2';

export function SetupView() {
  return (
    <div className="flex flex-col gap-10">
      <ConfigurationStep1 />
      <ConfigurationStep2 />
    </div>
  );
}

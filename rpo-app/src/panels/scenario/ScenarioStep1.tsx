import { SpacecraftPanel } from './SpacecraftPanel';

export function ScenarioStep1() {
  return (
    <section className="grid gap-6 md:grid-cols-2">
      <SpacecraftPanel vehicle="deputy" />
      <SpacecraftPanel vehicle="chief" />
    </section>
  );
}

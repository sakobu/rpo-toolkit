import { SpacecraftPanel } from './SpacecraftPanel';

export function ConfigurationStep1() {
  return (
    <section className="grid gap-6 md:grid-cols-2">
      <SpacecraftPanel vehicle="chief" />
      <SpacecraftPanel vehicle="deputy" />
    </section>
  );
}

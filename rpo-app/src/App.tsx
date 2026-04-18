import { WasmGate } from './wasm/WasmGate';
import { ScenarioStep1 } from './panels/scenario/ScenarioStep1';
import { ScenarioStep2 } from './panels/scenario/ScenarioStep2';

export default function App() {
  return (
    <WasmGate>
      <main className="mx-auto min-h-screen max-w-screen-2xl p-6">
        <header className="mb-6 flex items-baseline justify-between border-b border-border pb-3">
          <div>
            <h1 className="text-xl font-semibold tracking-tight text-text">RPO Toolkit</h1>
            <p className="text-sm text-text-muted">Steps 1-2 — configure + upload states</p>
          </div>
          <span className="font-mono text-xs tracking-wider text-text-dim uppercase">
            analytical · offline
          </span>
        </header>
        <div className="flex flex-col gap-10">
          <ScenarioStep1 />
          <ScenarioStep2 />
        </div>
      </main>
    </WasmGate>
  );
}

import { WasmGate } from './wasm/WasmGate';
import { ScenarioStep1 } from './panels/scenario/ScenarioStep1';

export default function App() {
  return (
    <WasmGate>
      <main className="mx-auto min-h-screen max-w-6xl p-6">
        <header className="mb-6 flex items-baseline justify-between border-b border-border pb-3">
          <div>
            <h1 className="text-xl font-semibold tracking-tight text-text">RPO Mission Planner</h1>
            <p className="text-sm text-text-muted">Step 1 — Configure spacecraft</p>
          </div>
          <span className="font-mono text-xs tracking-wider text-text-dim uppercase">
            analytical · offline
          </span>
        </header>
        <ScenarioStep1 />
      </main>
    </WasmGate>
  );
}

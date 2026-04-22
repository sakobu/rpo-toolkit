import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import { bindPhasesToMission } from './stores/phases.ts';
import { WasmGate } from './wasm/WasmGate.tsx';
import App from './App.tsx';

import './index.css';

bindPhasesToMission();

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <WasmGate>
      <App />
    </WasmGate>
  </StrictMode>,
);

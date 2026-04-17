import './index.css';
import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import App from './App.tsx';
import { WasmGate } from './wasm/WasmGate.tsx';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <WasmGate>
      <App />
    </WasmGate>
  </StrictMode>,
);

import { fileURLToPath, URL } from 'node:url';

import tailwindcss from '@tailwindcss/vite';
import react from '@vitejs/plugin-react-swc';
import { defineConfig } from 'vite';

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  optimizeDeps: { exclude: ['rpo-wasm'] },
  server: {
    fs: {
      allow: ['..'],
    },
    proxy: {
      '/ws': { target: 'ws://localhost:3001', ws: true },
    },
  },
});

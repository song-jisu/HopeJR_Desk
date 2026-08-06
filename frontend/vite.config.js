import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Dev server proxies /api and /ws to the FastAPI backend so the frontend can
// use same-origin relative URLs.
//
// Backend host is configurable via DESK_BACKEND (default http://localhost:8000).
// When the backend runs in WSL2 and the frontend on Windows, localhost
// forwarding can be flaky — set the WSL IP, e.g.:
//   DESK_BACKEND=http://172.28.197.154:8000 npm run dev      (bash)
//   $env:DESK_BACKEND="http://172.28.197.154:8000"; npm run dev   (PowerShell)
const backend = process.env.DESK_BACKEND || 'http://localhost:8000'
const wsBackend = backend.replace(/^http/, 'ws')

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      '/api': { target: backend, changeOrigin: true },
      '/assets': { target: backend, changeOrigin: true },
      '/ws': { target: wsBackend, ws: true },
    },
  },
})

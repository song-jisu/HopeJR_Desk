#!/usr/bin/env bash
# Start the HopeJR Desk frontend dev server (proxies /api and /ws to :8000).
# Run it, don't source it:  bash scripts/run_frontend.sh
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$here/../frontend"
[ -d node_modules ] || npm install
npm run dev -- --host

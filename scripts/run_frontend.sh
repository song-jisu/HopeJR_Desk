#!/usr/bin/env bash
# Start the HopeJR Desk frontend dev server (proxies /api and /ws to :8000).
# Run it, don't source it:  bash scripts/run_frontend.sh
# Guard: sourcing this would run `set -e` (and `exec`) in your login shell — a
# failure there closes the terminal. Refuse instead.
if [ "${BASH_SOURCE[0]:-$0}" != "$0" ]; then
  echo "ERROR: run this, don't source it:  bash scripts/run_frontend.sh" >&2
  return 1
fi
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$here/../frontend"
if ! command -v npm >/dev/null 2>&1; then
  echo "ERROR: npm not found — the frontend needs Node 18+." >&2
  echo "       e.g.  sudo apt install nodejs npm   (or install nvm/fnm)" >&2
  exit 1
fi
[ -d node_modules ] || npm install
npm run dev -- --host

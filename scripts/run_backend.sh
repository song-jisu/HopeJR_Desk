#!/usr/bin/env bash
# Start the HopeJR Desk backend (Robot Manager Server). Run it, don't source it.
#   bash scripts/run_backend.sh                        # mock backend (no robot needed)
#   HOPEJR_BACKEND=serial bash scripts/run_backend.sh  # real robot via lerobot serial (needs lerobot)
#   HOPEJR_BACKEND=ros  bash scripts/run_backend.sh    # real robot via ROS2 (source your ROS2 first)
#
# serial/ros need a real interpreter that can import lerobot / rclpy — NOT uv's
# ephemeral env. Point HOPEJR_PYTHON at it; defaults to the `lehome` conda env.
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$here/../backend"
export HOPEJR_BACKEND="${HOPEJR_BACKEND:-mock}"

# Interpreter that has lerobot (+ fastapi/uvicorn). Override with HOPEJR_PYTHON.
HOPEJR_PYTHON="${HOPEJR_PYTHON:-$HOME/miniconda3/envs/lehome/bin/python}"

if [ "$HOPEJR_BACKEND" = "mock" ] && command -v uv >/dev/null 2>&1; then
  # mock needs no robot libs — uv fetches deps into an ephemeral env
  exec uv run --with fastapi --with 'uvicorn[standard]' --with pydantic \
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
elif [ "$HOPEJR_BACKEND" = "ros" ]; then
  # ros mode must use the interpreter that can import rclpy (system python)
  exec uvicorn app.main:app --host 0.0.0.0 --port 8000
else
  # serial (default real-robot path) — use the lerobot interpreter
  if [ ! -x "$HOPEJR_PYTHON" ]; then
    echo "ERROR: HOPEJR_PYTHON not executable: $HOPEJR_PYTHON" >&2
    echo "       set it to a python that has lerobot, e.g. your conda env." >&2
    exit 1
  fi
  echo "backend=$HOPEJR_BACKEND  python=$HOPEJR_PYTHON"
  exec "$HOPEJR_PYTHON" -m uvicorn app.main:app --host 0.0.0.0 --port 8000
fi

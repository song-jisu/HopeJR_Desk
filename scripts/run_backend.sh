#!/usr/bin/env bash
# Start the HopeJR Desk backend (Robot Manager Server). Run it, don't source it.
#
#   bash scripts/run_backend.sh          # auto: real robot if it's plugged in, else mock
#   bash scripts/run_backend.sh serial   # force the real robot (lerobot serial)
#   bash scripts/run_backend.sh mock     # force the simulator (no robot needed)
#   bash scripts/run_backend.sh ros      # real robot via ROS2 (source your ROS2 first)
#
# HOPEJR_BACKEND=<mode> still works and is equivalent to the positional argument.
#
# serial/ros need a real interpreter that can import lerobot / rclpy — NOT uv's
# ephemeral env. Point HOPEJR_PYTHON at it; defaults to the repo's own .venv
# (see README "Python environment"), falling back to the `lehome` conda env.
# Guard: sourcing this would run `set -e` (and `exec`) in your login shell — a
# failure there closes the terminal. Refuse instead.
if [ "${BASH_SOURCE[0]:-$0}" != "$0" ]; then
  echo "ERROR: run this, don't source it:  bash scripts/run_backend.sh" >&2
  return 1
fi
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$here/../backend"

# --- interpreter ---------------------------------------------------------------
# Needs lerobot (+ fastapi/uvicorn). Override with HOPEJR_PYTHON.
# First existing wins: repo .venv, then the older `lehome` conda env.
if [ -z "${HOPEJR_PYTHON:-}" ]; then
  for _py in "$here/../.venv/bin/python" "$HOME/miniconda3/envs/lehome/bin/python"; do
    [ -x "$_py" ] && { HOPEJR_PYTHON="$_py"; break; }
  done
  # nothing found — keep the repo .venv path so the error below names it
  HOPEJR_PYTHON="${HOPEJR_PYTHON:-$here/../.venv/bin/python}"
fi

# --- mode ----------------------------------------------------------------------
# Explicit wins: positional arg, then HOPEJR_BACKEND, then auto-detect. Getting
# mock by accident is the classic time sink here — the robot never moves and the
# URDF ignores it — so when nothing is specified, look for the robot instead of
# silently assuming it is absent.
mode="${1:-${HOPEJR_BACKEND:-}}"
case "$mode" in
  mock|serial|ros) ;;
  "")
    arm="${HOPEJR_ARM_PORT:-/dev/hopejr_arm}"
    hand="${HOPEJR_HAND_PORT:-/dev/hopejr_hand}"
    # lerobot present? (cheap path probe — importing it costs seconds via torch)
    lerobot_ok=0
    for _sp in "$(dirname "$HOPEJR_PYTHON")"/../lib/python*/site-packages/lerobot; do
      [ -d "$_sp" ] && { lerobot_ok=1; break; }
    done
    if [ -e "$arm" ] && [ -e "$hand" ] && [ "$lerobot_ok" = 1 ]; then
      mode=serial
      echo "auto: found $arm + $hand and lerobot -> serial"
    else
      mode=mock
      [ -e "$arm" ] && [ -e "$hand" ] || echo "auto: $arm / $hand not both present -> mock"
      [ "$lerobot_ok" = 1 ] || echo "auto: lerobot not installed for $HOPEJR_PYTHON -> mock"
    fi ;;
  *)
    echo "ERROR: unknown mode '$mode' (expected: mock | serial | ros)" >&2
    exit 1 ;;
esac
export HOPEJR_BACKEND="$mode"

if [ "$HOPEJR_BACKEND" = "mock" ] && command -v uv >/dev/null 2>&1; then
  # Easy to land on by accident, so say it loudly.
  echo "=============================================================="
  echo " HOPEJR_BACKEND=mock — SIMULATED robot."
  echo " No serial port is opened; Servo Enable moves nothing real."
  echo " For the real robot:"
  echo "   bash scripts/run_backend.sh serial"
  echo "=============================================================="
  # mock needs no robot libs — uv fetches deps into an ephemeral env
  exec uv run --with fastapi --with 'uvicorn[standard]' --with pydantic \
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
elif [ "$HOPEJR_BACKEND" = "ros" ]; then
  # ros mode must use the interpreter that can import rclpy (system python)
  exec uvicorn app.main:app --host 0.0.0.0 --port 8000
else
  # serial (real-robot path) — use the lerobot interpreter
  if [ ! -x "$HOPEJR_PYTHON" ]; then
    echo "ERROR: HOPEJR_PYTHON not executable: $HOPEJR_PYTHON" >&2
    echo "       set it to a python that has lerobot, e.g. your conda env." >&2
    exit 1
  fi
  echo "backend=$HOPEJR_BACKEND  python=$HOPEJR_PYTHON"
  exec "$HOPEJR_PYTHON" -m uvicorn app.main:app --host 0.0.0.0 --port 8000
fi

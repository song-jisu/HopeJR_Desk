# Start the HopeJR Desk backend on Windows (mock mode; uses uv).
# Real-robot (ROS2) mode should run inside WSL/Ubuntu — see scripts/run_backend.sh.
$ErrorActionPreference = "Stop"
Set-Location "$PSScriptRoot\..\backend"
if (-not $env:HOPEJR_BACKEND) { $env:HOPEJR_BACKEND = "mock" }
uv run --with fastapi --with "uvicorn[standard]" --with pydantic `
  uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

"""Dashboard + Diagnostics endpoints (PLAN.md §3, §9)."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException

from ..manager import get_manager
from ..models import RobotStatus, Telemetry

router = APIRouter(prefix="/api", tags=["dashboard"])


@router.get("/status", response_model=Telemetry)
def status() -> Telemetry:
    snap = get_manager().latest
    if snap is None:
        raise HTTPException(503, "telemetry not ready")
    return snap


@router.get("/robot", response_model=RobotStatus)
def robot_status() -> RobotStatus:
    m = get_manager()
    st = m.backend.status()
    st.current_task = m.latest.status.current_task if m.latest else None
    return st


@router.get("/diagnostics")
def diagnostics() -> dict:
    snap = get_manager().latest
    if snap is None:
        raise HTTPException(503, "telemetry not ready")
    motors = snap.motors
    return {
        "ts": snap.ts,
        "comm_delay_ms": snap.status.comm_delay_ms,
        "packet_loss": snap.status.packet_loss,
        "max_temperature": max((m.temperature for m in motors), default=0.0),
        "max_current": max((m.current for m in motors), default=0.0),
        "errors": [m.name for m in motors if m.error],
        "offline": [m.name for m in motors if not m.online],
    }

"""Robot Configuration + Logs endpoints (PLAN.md §8, §11)."""
from __future__ import annotations

from fastapi import APIRouter

from ..manager import get_manager
from ..models import ActionResult, ConfigUpdate

router = APIRouter(prefix="/api", tags=["config"])


@router.get("/config")
def get_config():
    return {"motors": get_manager().config_snapshot()}


@router.post("/config", response_model=ActionResult)
def update_config(cfg: ConfigUpdate):
    return get_manager().update_config(cfg.name, cfg.cmd_min, cfg.cmd_max, cfg.home)


@router.get("/logs")
def get_logs(limit: int = 200):
    logs = list(get_manager().logs)[-limit:]
    return {"logs": logs}

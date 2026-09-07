"""Identification sweep endpoints — data collection for gravity/friction ID.

These endpoints only RECORD. Fitting link masses, the current->torque scale
k_tau, and the friction model is a separate offline step over the saved runs;
see identify.py for why a run looks the way it does and how runs combine.
"""
from __future__ import annotations

from fastapi import APIRouter
from fastapi.responses import FileResponse

from ..manager import get_manager
from ..models import ActionResult, SweepRequest

router = APIRouter(prefix="/api/identify", tags=["identify"])


@router.get("/status")
def status():
    return get_manager().sweep_status()


@router.post("/sweep", response_model=ActionResult)
async def sweep(req: SweepRequest):
    """Start one sweep: one joint, one payload condition. Returns immediately;
    poll /status for progress.

    MUST be async: begin_sweep schedules the run with asyncio.create_task, and a
    plain `def` handler is dispatched to a threadpool where there is no running
    event loop to schedule onto.
    """
    return get_manager().begin_sweep(req)


@router.post("/stop", response_model=ActionResult)
def stop():
    """Abort the running sweep. Whatever was recorded so far is still saved."""
    return get_manager().stop_sweep()


@router.get("/runs")
def runs():
    return {"runs": get_manager().ident.list_runs()}


@router.get("/runs/{name}.csv")
def run_csv(name: str):
    path = get_manager().ident.run_path(name, "csv")
    if path is None:
        return {"error": "not found"}
    return FileResponse(path, media_type="text/csv", filename=f"{name}.csv")


@router.get("/runs/{name}.json")
def run_meta(name: str):
    path = get_manager().ident.run_path(name, "json")
    if path is None:
        return {"error": "not found"}
    return FileResponse(path, media_type="application/json", filename=f"{name}.json")

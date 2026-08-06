"""Hand-guiding / Motion Teaching endpoints (PLAN.md §5).

Start → the arm goes compliant (torque on, goal follows present) so it can be
moved by hand while positions are recorded → Stop → Save/Replay/Delete motions.
"""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel

from ..manager import get_manager
from ..models import ActionResult

router = APIRouter(prefix="/api/teaching", tags=["teaching"])


class MotionName(BaseModel):
    name: str


@router.get("/status")
def status():
    return get_manager().teaching_status()


@router.post("/start", response_model=ActionResult)
def start():
    return get_manager().begin_teaching()


@router.post("/stop", response_model=ActionResult)
def stop():
    return get_manager().end_teaching()


@router.post("/save", response_model=ActionResult)
def save(cmd: MotionName):
    return get_manager().save_motion(cmd.name)


@router.post("/replay", response_model=ActionResult)
async def replay(cmd: MotionName):
    return await get_manager().replay_motion(cmd.name)


@router.post("/delete", response_model=ActionResult)
def delete(cmd: MotionName):
    return get_manager().delete_motion(cmd.name)

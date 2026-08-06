"""Calibration (range finder) endpoints — web version of lerobot's RangeFinderGUI.

Flow: start (torque off) → user backdrives every joint through its full range
while raw min/max are recorded → save (writes the lerobot calibration JSON and
reloads it live). Direction (drive_mode) can be flipped per motor and tested
immediately. Serial backend only.
"""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel, Field

from ..manager import get_manager
from ..models import ActionResult

router = APIRouter(prefix="/api/calibration", tags=["calibration"])


class DirectionCommand(BaseModel):
    name: str
    drive_mode: int = Field(..., ge=0, le=1)


class StartCommand(BaseModel):
    home: bool = False   # half-turn homing (only for wrap-crossing joints)


@router.get("/status")
def status():
    return get_manager().calibration_status()


@router.post("/start", response_model=ActionResult)
def start(cmd: StartCommand | None = None):
    return get_manager().begin_calibration(home=bool(cmd and cmd.home))


@router.post("/reset", response_model=ActionResult)
def reset():
    return get_manager().reset_calibration_ranges()


@router.post("/stop", response_model=ActionResult)
def stop():
    return get_manager().end_calibration()


@router.post("/save", response_model=ActionResult)
def save():
    return get_manager().save_calibration()


@router.post("/direction", response_model=ActionResult)
def set_direction(cmd: DirectionCommand):
    return get_manager().set_direction(cmd.name, cmd.drive_mode)

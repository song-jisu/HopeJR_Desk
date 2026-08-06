"""Link parameter endpoints (PLAN.md §8) — per-link mass + CoM for gravity calc."""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter
from pydantic import BaseModel

from .. import links as linklib
from ..models import ActionResult

router = APIRouter(prefix="/api/links", tags=["links"])


class LinkUpdate(BaseModel):
    name: str
    mass: Optional[float] = None
    com: Optional[list[float]] = None


@router.get("")
def get_links():
    return {"links": linklib.get_links()}


@router.post("", response_model=ActionResult)
def set_link(cmd: LinkUpdate):
    linklib.set_link(cmd.name, cmd.mass, cmd.com)
    return ActionResult(ok=True, message=f"updated {cmd.name}")


@router.post("/reset", response_model=ActionResult)
def reset():
    linklib.reset_links()
    return ActionResult(ok=True, message="reset to URDF defaults")

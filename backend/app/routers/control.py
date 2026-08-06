"""Robot Control endpoints (PLAN.md §4)."""
from __future__ import annotations

from fastapi import APIRouter

from ..manager import get_manager
from ..models import ActionResult, JointCommand, JointCommandBatch

router = APIRouter(prefix="/api/control", tags=["control"])


@router.post("/servo/enable", response_model=ActionResult)
def servo_enable():
    return get_manager().enable_servo()


@router.post("/servo/disable", response_model=ActionResult)
def servo_disable():
    return get_manager().disable_servo()


@router.post("/estop", response_model=ActionResult)
def estop():
    return get_manager().estop()


@router.post("/recover", response_model=ActionResult)
def recover():
    return get_manager().recover()


@router.post("/home", response_model=ActionResult)
def home():
    return get_manager().home()


@router.post("/reset", response_model=ActionResult)
def joint_reset():
    return get_manager().joint_reset()


@router.post("/joint", response_model=ActionResult)
def command_joint(cmd: JointCommand):
    return get_manager().command_joint(cmd.name, cmd.position)


@router.post("/joints", response_model=ActionResult)
def command_joints(batch: JointCommandBatch):
    m = get_manager()
    for c in batch.commands:
        m.command_joint(c.name, c.position)
    return ActionResult(ok=True, message=f"{len(batch.commands)} joints commanded")


@router.post("/servo/{name}/reboot", response_model=ActionResult)
def reboot_servo(name: str):
    return get_manager().reboot_servo(name)


@router.get("/scan")
def scan():
    return {"servos": get_manager().scan()}


@router.get("/raw")
def raw():
    """Raw encoder counts behind the normalized readings (serial backend only).

    Read-only. Use when a joint reads a saturated +-100: that is ambiguous
    between 'genuinely at the range edge' and 'encoder wrapped outside the
    range', and only the raw count distinguishes them."""
    fn = getattr(get_manager().backend, "raw_report", None)
    return {"motors": fn() if fn else [], "supported": bool(fn)}

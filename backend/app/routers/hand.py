"""Hand Control endpoints (PLAN.md §7)."""
from __future__ import annotations

import time
from typing import Literal

from fastapi import APIRouter

from pydantic import BaseModel, Field

from ..hardware import FINGERS
from ..manager import get_manager
from ..models import (ActionResult, ContactThreshold, FingerCommand, GripCommand)

router = APIRouter(prefix="/api/hand", tags=["hand"])


class HandMotorCommand(BaseModel):
    name: str
    value: float = Field(..., ge=0, le=100)


@router.get("/fingers")
def fingers():
    return {"fingers": [
        {"index": f.index, "name": f.name, "motors": f.motors, "aux": f.aux}
        for f in FINGERS
    ]}


@router.post("/finger", response_model=ActionResult)
def command_finger(cmd: FingerCommand):
    return get_manager().command_finger(cmd.index, cmd.aperture)


@router.post("/motor", response_model=ActionResult)
def command_motor(cmd: HandMotorCommand):
    """Command a single hand motor 0..100 (e.g. thumb_cmc rotation)."""
    return get_manager().command_joint(cmd.name, cmd.value)


@router.post("/grip", response_model=ActionResult)
def grip(cmd: GripCommand):
    return get_manager().command_grip(cmd.fingers, cmd.aperture)


@router.post("/contact-threshold", response_model=ActionResult)
def set_contact_threshold(cmd: ContactThreshold):
    return get_manager().set_contact_threshold(cmd.current_ma)


@router.get("/state")
def hand_state():
    snap = get_manager().latest
    return {"fingers": [f.model_dump() for f in snap.fingers]} if snap else {"fingers": []}


# --- tendon kinematics: motor value -> joint angle -----------------------------
class RawRefUpdate(BaseModel):
    raw_ref: dict[str, float]


@router.get("/kinematics")
def kinematics():
    """Live joint angles plus the extended-pose datum they were computed with."""
    mgr = get_manager()
    snap = mgr.latest
    out = dict(snap.hand_joints) if snap else {}
    out["reference_file"] = mgr.hand_kin.path
    return out


class ReferencePose(BaseModel):
    # {"index": {"mcp_flex": 20.0, "mcp_spread": 0.0, "pip": 15.0}, ...} in DEGREES
    pose: dict[str, dict[str, float]] = {}
    at: Literal["home", "current"] = "home"


@router.post("/kinematics/reference", response_model=ActionResult)
def set_reference(cmd: ReferencePose):
    """Declare the joint angles (degrees) the hand is at in the reference pose.

    `at="home"` (default) anchors it to the startup pose — every motor at its
    calibration end-stop — which is the pose you can always reproduce. Solved
    through joint -> motor to get each motor's expected theta; the gap against
    the theta that pose actually reads becomes the offset."""
    mgr = get_manager()
    pos = None
    if cmd.at == "current":
        snap = mgr.latest
        if not snap:
            return ActionResult(ok=False, message="no telemetry yet")
        pos = {m.name: m.position for m in snap.motors if m.unit == "hand"}
    mgr.hand_kin.set_reference(cmd.pose, pos)
    mgr.log("info", f"hand kinematics: reference pose set (anchored at {cmd.at})")
    return ActionResult(ok=True, message=f"reference set at {cmd.at} pose")


@router.post("/pose")
def command_pose(cmd: ReferencePose):
    """Drive the hand to a set of JOINT angles (degrees) — the camera-tracking
    path. Runs joint -> motor through the same model the display uses, so what
    you see on the 3D hand is what gets commanded."""
    mgr = get_manager()
    cmds = mgr.hand_kin.pose_to_commands(cmd.pose)
    applied, rejected = {}, None
    for name, value in cmds.items():
        r = mgr.command_joint(name, value)
        if r.ok:
            applied[name] = round(value, 2)
        elif rejected is None:
            rejected = r.message
    return {"ok": bool(applied), "commands": applied,
            "message": rejected or f"{len(applied)} motor(s) commanded"}


class TrackPayload(BaseModel):
    landmarks: list[list[float]]                 # 21 x 3, MediaPipe worldLandmarks
    handedness: str = "Right"
    w_apex: float = Field(1.0, ge=0.0, le=3.0)
    thumb_opp: float = Field(1.0, ge=0.0, le=1.0)   # strength of pinch->CMC bias


@router.post("/track")
def track(cmd: TrackPayload):
    """Camera hand -> robot, by RETARGETING placement rather than copying angles.

    The robot's fingers are rolling-contact linkages with 3 DOF against a human's
    4, so joint angles do not correspond. This matches each finger's fingertip
    and the peak of its arc instead — see hand_retarget.py."""
    from .. import hand_retarget
    mgr = get_manager()
    t0 = time.perf_counter()
    try:
        # Warm start from the previous frame. hand_retarget re-sweeps a finger
        # on its own if that solution comes out poor, so there is no periodic
        # sweep to stall a frame — and no single finger twitching on a timer
        # while the others sit still.
        seed = getattr(mgr, "_retarget_seed", None)
        out = hand_retarget.retarget(cmd.landmarks, seed=seed, w_apex=cmd.w_apex,
                                     handedness=cmd.handedness)
        pose = dict(out["pose"])
        pose["thumb"] = hand_retarget.thumb_pose(cmd.landmarks)
    except Exception as e:
        return {"ok": False, "message": f"retarget failed: {type(e).__name__}: {e}"}
    mgr._retarget_seed = out["pose"]

    cmds = mgr.hand_kin.pose_to_commands(pose)
    # Pinch closure: when the thumb closes on a fingertip, blend that thumb+finger
    # pair toward a stored 'tips touching' pose — independent retargeting never
    # makes them actually meet. Falls back to just driving the CMC toward max
    # (opposition) if no pinch pose has been captured for that finger.
    pinch_finger, pinch = hand_retarget.pinch_state(cmd.landmarks)
    pinch *= cmd.thumb_opp
    if pinch_finger and pinch > 0:
        if pinch_finger in mgr.hand_kin.pinch_pose:
            cmds = mgr.hand_kin.blend_to_pinch(cmds, pinch_finger, pinch)
        elif "thumb_cmc" in cmds:
            cmds["thumb_cmc"] += pinch * (100.0 - cmds["thumb_cmc"])
    for name, value in cmds.items():
        mgr.command_joint(name, value)
    # Where the time actually goes: solving is a few ms, but a command only
    # reaches a servo on the next pass of the serial loop, and that loop is
    # limited by the hand bus having no sync read (16 sequential transactions,
    # 64 on diagnostic passes).
    hz = getattr(mgr.backend, "loop_hz", None)
    return {"ok": bool(cmds), "message": f"{len(cmds)} motor(s) commanded",
            "pose": {f: {k: round(v, 1) for k, v in j.items()} for f, j in pose.items()},
            "tip_error_mm": out["tip_error_mm"], "apex": out["apex"],
            "pinch": round(pinch, 2), "pinch_finger": pinch_finger,
            "solve_ms": round((time.perf_counter() - t0) * 1000, 1),
            "serial_hz": round(hz(), 1) if hz else None}


@router.post("/kinematics/offsets", response_model=ActionResult)
def set_raw_ref(cmd: RawRefUpdate):
    """Nudge a motor's extended-pose datum (encoder counts) by hand."""
    get_manager().hand_kin.set_raw_ref(cmd.raw_ref)
    return ActionResult(ok=True, message=f"{len(cmd.raw_ref)} datum(s) updated")


class SlackUpdate(BaseModel):
    slack: dict[str, float]             # {motor: rad}; dead zone before flexion


@router.post("/kinematics/slack", response_model=ActionResult)
def set_slack(cmd: SlackUpdate):
    """Per-motor dead zone (rad of spool) before the joint starts to flex. Raise
    it if the 3D joint bends while the real one is still straight at low travel."""
    get_manager().hand_kin.set_slack(cmd.slack)
    return ActionResult(ok=True, message=f"{len(cmd.slack)} slack value(s) set")


class DeadzoneUpdate(BaseModel):
    pct: dict[str, float]              # {motor: percent of travel} assembly slack


@router.post("/kinematics/deadzone", response_model=ActionResult)
def set_deadzone(cmd: DeadzoneUpdate):
    """Assembly setup: percent of each motor's travel that is tendon slack (the
    joint stays put). Persisted; the model's rad value tracks the live range."""
    get_manager().hand_kin.set_deadzone_pct(cmd.pct)
    return ActionResult(ok=True, message=f"{len(cmd.pct)} dead zone(s) set")


class MomentArmUpdate(BaseModel):
    moment_arm: dict[str, float]        # {motor: mm}; bigger = less flexion/count


@router.post("/kinematics/moment-arm", response_model=ActionResult)
def set_moment_arm(cmd: MomentArmUpdate):
    """Tune per-motor tendon moment arm (mm) to match sim flexion to the real
    robot. Larger arm = the joint flexes less for the same motor travel."""
    get_manager().hand_kin.set_moment_arms(cmd.moment_arm)
    return ActionResult(ok=True, message=f"{len(cmd.moment_arm)} moment arm(s) set")


class McpGainUpdate(BaseModel):
    radial: dict[str, float] = {}    # {finger: moment arm} radial flexion tendon
    ulnar: dict[str, float] = {}     # {finger: moment arm} ulnar flexion tendon
    spread: dict[str, float] = {}    # {finger: gain} spread per angular imbalance


@router.post("/kinematics/mcp-gain", response_model=ActionResult)
def set_mcp_gain(cmd: McpGainUpdate):
    """Tune per-finger MCP against the real robot. radial/ulnar = each tendon's
    flexion moment arm (raise the side that over-flexes); spread may be negative
    to flip the tilt direction."""
    get_manager().hand_kin.set_mcp_gains(cmd.radial, cmd.ulnar, cmd.spread)
    return ActionResult(ok=True, message="mcp gains set")


class PinchCapture(BaseModel):
    finger: Literal["index", "middle", "ring", "pinky"]


@router.post("/kinematics/pinch/capture", response_model=ActionResult)
def capture_pinch(cmd: PinchCapture):
    """Store the CURRENT motor pose as the 'thumb tip touching this finger' pose.
    Drive the sliders until the tips actually meet on the robot, then call this;
    camera tracking blends toward it whenever it sees that pinch."""
    mgr = get_manager()
    snap = mgr.latest
    if not snap:
        return ActionResult(ok=False, message="no telemetry yet")
    pos = {m.name: m.position for m in snap.motors if m.unit == "hand"}
    mgr.hand_kin.capture_pinch(cmd.finger, pos)
    mgr.log("info", f"pinch pose captured for thumb+{cmd.finger}")
    return ActionResult(ok=True, message=f"pinch pose saved for {cmd.finger}")


@router.post("/kinematics/pinch/clear", response_model=ActionResult)
def clear_pinch(cmd: PinchCapture):
    mgr = get_manager()
    mgr.hand_kin.pinch_pose.pop(cmd.finger, None)
    mgr.hand_kin.save()
    return ActionResult(ok=True, message=f"pinch pose cleared for {cmd.finger}")


# --- contact-sample log (experimental: collect data for the MCP model) --------
@router.post("/contact/sample")
def contact_sample(cmd: PinchCapture):
    """Record that this finger is touching the thumb at the CURRENT motor state.
    Only the thumb + this finger's motors are stored; the rest are ignored.
    Take many samples across different contact poses."""
    import time
    mgr = get_manager()
    snap = mgr.latest
    if not snap:
        return {"ok": False, "message": "no telemetry yet", "count": 0}
    pos = {m.name: m.position for m in snap.motors if m.unit == "hand"}
    n = mgr.hand_kin.add_contact_sample(cmd.finger, pos, time.time())
    mgr.log("info", f"contact sample #{n} recorded (thumb+{cmd.finger})")
    return {"ok": True, "message": f"sample #{n} for thumb+{cmd.finger}", "count": n}


@router.get("/contact/samples")
def contact_samples():
    """All recorded contact samples, and a per-finger count."""
    samples = get_manager().hand_kin.load_samples()
    counts: dict[str, int] = {}
    for s in samples:
        counts[s["finger"]] = counts.get(s["finger"], 0) + 1
    return {"samples": samples, "counts": counts, "total": len(samples)}


@router.post("/contact/clear", response_model=ActionResult)
def contact_clear():
    get_manager().hand_kin.clear_samples()
    return ActionResult(ok=True, message="contact samples cleared")


@router.post("/kinematics/reset", response_model=ActionResult)
def reset_offsets():
    get_manager().hand_kin.reset()
    return ActionResult(ok=True, message="reference cleared")

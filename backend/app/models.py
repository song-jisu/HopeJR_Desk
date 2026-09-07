"""Pydantic API models shared by routers and backends."""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field


class MotorState(BaseModel):
    name: str
    servo_id: int
    unit: Literal["arm", "hand"]
    position: float            # present position (normalized)
    command: float             # last commanded position (normalized)
    velocity: float            # normalized units / s
    current: float             # magnitude, raw counts (scale varies by model)
    # SIGNED torque proxy, from a different register than `current` on the arm
    # (see serial_bus ARM_CURRENT_REG / ARM_LOAD_REG). Friction flips sign with
    # the direction of travel and gravity does not, so identification needs the
    # sign; the UI and contact detection use the magnitude above. Not the same
    # physical quantity as `current` — do not expect |current_signed| == current.
    current_signed: float = 0.0
    # The goal actually being sent to the servo right now. `command` is where the
    # UI wants the joint and jumps the instant a target is set; this is how far
    # the slew limiter has let it travel. They differ for the whole of a ramp,
    # so telling "still ramping" from "stalled against something" needs this one.
    goal: float = 0.0
    temperature: float         # deg C
    voltage: float             # V
    error: bool = False
    online: bool = True


class RobotStatus(BaseModel):
    connected: bool
    servo_enabled: bool
    estop: bool
    mode: str                                  # backend mode: "mock" | "ros"
    current_task: Optional[str] = None
    comm_ok: bool = True
    comm_delay_ms: float = 0.0
    packet_loss: float = 0.0                    # 0..1
    arm_online: int = 0
    hand_online: int = 0


class FingerState(BaseModel):
    index: int
    name: str
    state: Literal["idle", "closing", "contact", "holding", "release"]
    aperture: float            # 0 (open) .. 100 (closed), aggregate
    contact: bool = False
    max_current: float = 0.0   # mA, peak across finger motors


class Telemetry(BaseModel):
    """One snapshot streamed over the websocket / returned by /api/status."""
    ts: float
    status: RobotStatus
    motors: list[MotorState]
    fingers: list[FingerState]
    # Per-arm-joint gravity load torque (N*m). None means NOT KNOWN -- the joint
    # has no measured model and the URDF's placeholder masses are not a usable
    # substitute. Consumers must render/skip None rather than treat it as zero.
    gravity: dict[str, Optional[float]] = {}
    # tendon model output: {"fingers": {finger: {joint: deg}}, "theta": {...},
    # "offsets": {...}} — see hand_kinematics.py
    hand_joints: dict = {}


# --- Command payloads ---------------------------------------------------------
class JointCommand(BaseModel):
    name: str
    position: float = Field(..., description="normalized target position")


class JointCommandBatch(BaseModel):
    commands: list[JointCommand]


class FingerCommand(BaseModel):
    index: int = Field(..., ge=1, le=5)
    aperture: float = Field(..., ge=0, le=100, description="0 open .. 100 closed")


class GripCommand(BaseModel):
    fingers: str = Field(..., description='digits of fingers to grip, e.g. "15" = thumb+pinky')
    aperture: float = Field(100.0, ge=0, le=100)


class ContactThreshold(BaseModel):
    current_ma: float = Field(..., ge=0, description="current threshold for contact detection")


class SweepRequest(BaseModel):
    """One identification sweep: one arm joint, one payload condition.

    Leave start/end unset to sweep the joint's configured range inset by a
    safety margin — driving into a hard end-stop mid-sweep spikes the current
    for reasons that have nothing to do with gravity, which corrupts the run.
    """
    joint: str
    payload: str = Field("none", description='what is mounted, e.g. "none" or "500g@wrist"')
    start: Optional[float] = None
    end: Optional[float] = None
    rate: float = Field(8.0, gt=0, le=20,
                        description="sweep speed, normalized units/s; slow enough "
                                    "that inertia stays negligible beside gravity")
    cycles: int = Field(3, ge=1, le=20)
    settle: float = Field(1.0, ge=0, le=10, description="pause at each end, s")
    name: Optional[str] = Field(None, description="output basename; auto if unset")


class ConfigUpdate(BaseModel):
    """Per-motor configurable limits (Robot Configuration, PLAN.md §8)."""
    name: str
    cmd_min: Optional[float] = None
    cmd_max: Optional[float] = None
    home: Optional[float] = None


class ActionResult(BaseModel):
    ok: bool
    message: str = ""

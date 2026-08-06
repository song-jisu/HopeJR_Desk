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
    current: float             # mA
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
    gravity: dict[str, float] = {}   # per-arm-joint gravity load torque (N*m)
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


class ConfigUpdate(BaseModel):
    """Per-motor configurable limits (Robot Configuration, PLAN.md §8)."""
    name: str
    cmd_min: Optional[float] = None
    cmd_max: Optional[float] = None
    home: Optional[float] = None


class ActionResult(BaseModel):
    ok: bool
    message: str = ""

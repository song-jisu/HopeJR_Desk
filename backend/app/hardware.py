"""HopeJR hardware definitions.

Single source of truth for motor topology, IDs, and limits — derived from the
existing ROS2 stack (`teleop_server.py`, `hopejr_real_bridge/config.py`).

Arm  : 7 Feetech servos, normalized command range -100..100 (RANGE_M100_100)
Hand : 16 Feetech servos, normalized command range 0..100   (RANGE_0_100)

Fingers (per PLAN.md §7):  1=Thumb 2=Index 3=Middle 4=Ring 5=Pinky
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


@dataclass(frozen=True)
class MotorSpec:
    name: str
    servo_id: int
    model: str
    unit: Literal["arm", "hand"]      # which bus / normalization
    cmd_min: float
    cmd_max: float
    home: float = 0.0                  # normalized home position


# --- Arm: ids 1..7, normalized -100..100 -------------------------------------
ARM_MOTORS: list[MotorSpec] = [
    MotorSpec("shoulder_pitch", 1, "sm8512bl", "arm", -100.0, 100.0, 0.0),
    MotorSpec("shoulder_yaw",   2, "sts3250",  "arm", -100.0, 100.0, 0.0),
    MotorSpec("shoulder_roll",  3, "sts3250",  "arm", -100.0, 100.0, 0.0),
    MotorSpec("elbow_flex",     4, "sts3250",  "arm", -100.0, 100.0, 0.0),
    MotorSpec("wrist_roll",     5, "sts3250",  "arm", -100.0, 100.0, 0.0),
    MotorSpec("wrist_yaw",      6, "sts3250",  "arm", -100.0, 100.0, 0.0),
    MotorSpec("wrist_pitch",    7, "sts3250",  "arm", -100.0, 100.0, 0.0),
]

# --- Hand: ids 1..16, normalized 0..100 --------------------------------------
#
# The hand is a TENDON robot — it cannot be hand-backdriven for a range sweep
# like the arm, so its calibration is NOT taken from a file. Instead the user
# presets each servo's Min/Max Position Limit in EEPROM and the backend reads
# them off the motors at every startup (`_Bus(from_motors=True)`).
#
# Because lerobot normalizes RANGE_0_100 as range_min -> 0 and range_max -> 100
# (drive_mode is always 0 on the motor-read calibration), the home position is
# expressed directly as 0.0 or 100.0:
#
#   home = 0.0   (calibration MIN) : ids 1,2,3,5,7,8,10,11,13,14
#   home = 100.0 (calibration MAX) : ids 4,6,9,12,15,16
#
# The split is per-servo tendon routing, not per-finger — e.g. pinky_pip_dip
# (16) homes to MAX while the other fingers' pip_dip (7,10,13) home to MIN.
HAND_MOTORS: list[MotorSpec] = [
    MotorSpec("thumb_cmc",            1,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("thumb_mcp",            2,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("thumb_pip",            3,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("thumb_dip",            4,  "scs0009", "hand", 0.0, 100.0, 100.0),
    MotorSpec("index_radial_flexor",  5,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("index_ulnar_flexor",   6,  "scs0009", "hand", 0.0, 100.0, 100.0),
    MotorSpec("index_pip_dip",        7,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("middle_radial_flexor", 8,  "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("middle_ulnar_flexor",  9,  "scs0009", "hand", 0.0, 100.0, 100.0),
    MotorSpec("middle_pip_dip",       10, "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("ring_radial_flexor",   11, "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("ring_ulnar_flexor",    12, "scs0009", "hand", 0.0, 100.0, 100.0),
    MotorSpec("ring_pip_dip",         13, "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("pinky_radial_flexor",  14, "scs0009", "hand", 0.0, 100.0, 0.0),
    MotorSpec("pinky_ulnar_flexor",   15, "scs0009", "hand", 0.0, 100.0, 100.0),
    MotorSpec("pinky_pip_dip",        16, "scs0009", "hand", 0.0, 100.0, 100.0),
]

ALL_MOTORS: list[MotorSpec] = ARM_MOTORS + HAND_MOTORS
MOTOR_BY_NAME: dict[str, MotorSpec] = {m.name: m for m in ALL_MOTORS}


# --- Finger grouping (PLAN.md §7) --------------------------------------------
@dataclass(frozen=True)
class Finger:
    index: int                 # 1..5
    name: str
    motors: list[str]          # flexion motors — driven by aperture / grip
    aux: list[str] = field(default_factory=list)   # non-flexion motors (e.g. thumb rotation/opposition)


# thumb_cmc (carpometacarpal) is rotation/opposition, NOT flexion — kept out of
# the aperture mapping and controlled on its own axis.
FINGERS: list[Finger] = [
    Finger(1, "thumb",  ["thumb_mcp", "thumb_pip", "thumb_dip"], aux=["thumb_cmc"]),
    Finger(2, "index",  ["index_radial_flexor", "index_ulnar_flexor", "index_pip_dip"]),
    Finger(3, "middle", ["middle_radial_flexor", "middle_ulnar_flexor", "middle_pip_dip"]),
    Finger(4, "ring",   ["ring_radial_flexor", "ring_ulnar_flexor", "ring_pip_dip"]),
    Finger(5, "pinky",  ["pinky_radial_flexor", "pinky_ulnar_flexor", "pinky_pip_dip"]),
]
FINGER_BY_INDEX: dict[int, Finger] = {f.index: f for f in FINGERS}


def clamp(spec: MotorSpec, value: float) -> float:
    return max(spec.cmd_min, min(spec.cmd_max, float(value)))


# --- calibration remapping ---------------------------------------------------
#
# A normalized value only means something relative to the calibration it was
# produced under: it is "where in [range_min, range_max] the joint sits", as a
# percentage. Recorded motions store normalized values, so if the calibration
# changes afterwards (and it now can — both buses read range_min/range_max from
# the servos' Min/Max_Position_Limit at every startup) the same number points at
# a different physical pose.
#
# The fix is to go back through the raw encoder count, which IS calibration
# independent: norm --(old cal)--> raw --(new cal)--> norm'. These mirror
# lerobot's _normalize/_unnormalize, which use only range_min/range_max and
# drive_mode (homing_offset is applied by the servo itself, not in software).

def _flip(spec: MotorSpec, v: float) -> float:
    # lerobot inverts the two ranges differently: RANGE_M100_100 negates about 0,
    # RANGE_0_100 reflects about the midpoint (100 - v). Both are involutions, so
    # the same helper serves normalize and unnormalize.
    return (100.0 - v) if spec.unit == "hand" else -v


def norm_to_raw(spec: MotorSpec, value: float, cal: dict) -> float:
    lo, hi = cal["range_min"], cal["range_max"]
    v = _flip(spec, value) if cal.get("drive_mode") else value
    frac = (v / 100.0) if spec.unit == "hand" else ((v + 100.0) / 200.0)
    return frac * (hi - lo) + lo


def raw_to_norm(spec: MotorSpec, raw: float, cal: dict) -> float:
    lo, hi = cal["range_min"], cal["range_max"]
    if hi == lo:
        return 0.0
    frac = (raw - lo) / (hi - lo)
    v = frac * 100.0 if spec.unit == "hand" else frac * 200.0 - 100.0
    return _flip(spec, v) if cal.get("drive_mode") else v


def _usable(cal) -> bool:
    return bool(cal) and cal.get("range_max") != cal.get("range_min")


def remap_value(spec: MotorSpec, value: float, old_cal: dict, new_cal: dict) -> float:
    """Re-express `value` (normalized under old_cal) under new_cal. Returns it
    unchanged if either calibration is missing/degenerate — better a stale value
    than a garbage one."""
    if not _usable(old_cal) or not _usable(new_cal) or old_cal == new_cal:
        return value
    return clamp(spec, raw_to_norm(spec, norm_to_raw(spec, value, old_cal), new_cal))

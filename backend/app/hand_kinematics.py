"""Hand tendon kinematics — motor value -> finger joint angles.

Model source: "Hopejr Hand.pdf" (repo root), same equations as the ROS bridge's
`hopejr_real_bridge/calculate_jm.py`. Reimplemented here so the Desk backend has
no ROS dependency; keep the two in sync.

Chain, from what the Desk actually has to what we want to display:

    normalized 0..100  --(motor Min/Max_Position_Limit)-->  encoder count
                       --(COUNTS_PER_RAD, raw_ref)-------->  motor theta (rad)
                       --(tendon model)------------------->  joint angle (rad)

Only `raw_ref` — the count at which the finger is fully extended — is unknown,
and the Home Pose reference measures it. Everything else is hardware: the servo
's counts per turn and the Min/Max_Position_Limit the user set on it. That also
sidesteps the unresolved `FREE_LEN = 3.2` question in the PDF (at alpha=beta=0
the model wants theta = +-0.3498, not 0), since the datum is measured rather
than assumed.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from .hardware import HAND_MOTORS

# --- normalized 0..100 <-> motor theta (rad) ---------------------------------
#
# theta is spool rotation measured from the FULLY EXTENDED pose. Converting a
# motor reading to it needs exactly two things:
#
#   * counts per radian — a HARDWARE constant (scs0009: 1024 counts per turn),
#     identical for all sixteen servos
#   * the count at which that finger is fully extended — one number per motor,
#     which is what the Home Pose reference measures
#
# An earlier version instead stretched each motor's whole EEPROM range across
# its model theta range (the config.py limits below). That silently forced a
# different scale on every motor: back-solving counts-per-radian from the user's
# actual Min/Max_Position_Limit gave 122 to 773 for a quantity that must be
# constant. Hence "close but never quite right" tracking. The theta limits are
# now used ONLY for their direction, which is a wiring fact and still valid.
COUNTS_PER_RAD = 1024.0 / (2.0 * math.pi)      # ~163.0

THETA_LIMITS: dict[str, tuple[float, float]] = {
    "thumb_cmc": (-3.0, -0.043),   # SIGN +1 (motor↑ -> less opposition)
    "thumb_mcp": (0.0, 0.88),
    "thumb_pip": (0.0, 0.88),
    "thumb_dip": (0.88, 0.0),
    "index_radial_flexor": (0.0, 1.6562007649872527),
    "index_ulnar_flexor": (-1.8, 0.0),
    "index_pip_dip": (-0.02, 2.22),
    "middle_radial_flexor": (0.0, 1.6562007649872527),
    "middle_ulnar_flexor": (-1.8, 0.0),
    "middle_pip_dip": (-0.02, 2.22),
    "ring_radial_flexor": (0.0, 1.6562007649872527),
    "ring_ulnar_flexor": (-1.8, 0.0),
    "ring_pip_dip": (-0.02, 2.22),
    "pinky_radial_flexor": (0.0, 1.6562007649872527),
    "pinky_ulnar_flexor": (-1.8, 0.0),
    "pinky_pip_dip": (2.22, -0.02),
}
FINGERS4 = ("index", "middle", "ring", "pinky")

# Direction only: does a rising motor count wind the tendon in or pay it out?
# Taken from the ordering of the limits above (a reversed pair means the servo
# is mounted the other way round). Magnitudes are deliberately not used.
SIGN: dict[str, float] = {n: (1.0 if hi >= lo else -1.0)
                          for n, (lo, hi) in THETA_LIMITS.items()}
DEFAULT_RANGE = (0.0, 1024.0)     # used when no live calibration is available


# --- tendon -> joint flexion -------------------------------------------------
#
# MEASURED, and it replaces the PDF's chord construction for these joints.
#
# The PDF (p.1-2) models the tendon as a straight chord between two rolling
# circles, which makes flexion an arccos of the spooled length and puts a hard
# ceiling on it (70 deg per joint for the fingers). Photographing the middle
# finger at two slider positions says otherwise:
#
#     45%  ->  PIP 30 deg + DIP  60 deg = 90 deg
#     90%  ->  PIP 95 deg + DIP  85 deg = 180 deg
#
# The SUM is exactly proportional to motor travel and passes through zero, i.e.
# a constant moment arm — the tendon wraps the rolling surface instead of
# spanning it as a chord. So:
#
#     tendon pulled = MOMENT_ARM * (total joint flexion)
#
# Cross-check: fitting that to the measurement gives a moment arm of 7.74 mm
# against the PDF's rolling radius of 7.6, or equivalently a spool radius of
# 8.84 mm against the PDF's 9. Both agree to a couple of percent, which says the
# spool radius, the 1024-counts-per-turn and the rolling radius were all fine —
# only the chord model was wrong. Predicted totals land within 2% of measured.
MOMENT_ARM = 7.6        # mm, tendon moment arm about the joint
SPOOL_R = 9.0           # mm, motor spool

# How the two joints on one tendon share the travel. The tendon fixes only the
# SUM; on a closer look the two joints flex by about the SAME amount — any
# difference in the resting link angles is baked into the URDF geometry, not the
# split. (An earlier 1:2 reading came from measuring link angle against a wrong
# "straight" reference.) Under contact the split still redistributes, which no
# geometry predicts — that is what lets an underactuated finger conform.
PIP_SHARE = 0.5

# Per-motor moment arm (mm). MOMENT_ARM (7.6, measured on the middle finger) is
# the default. The thumb's three joints each have a different pulley radius, so
# they scale differently — measured symptom: at the same motor travel the model
# over-flexed thumb mcp/pip (needs a BIGGER arm) and under-flexed thumb dip
# (needs a SMALLER arm). Values below are placeholders until measured; a bigger
# arm means less flexion per motor count.
MOMENT_ARM_BY_MOTOR: dict[str, float] = {
    # Measured from two slider positions each (link angle by eye):
    #   mcp 40%->170deg 72%->150deg,  pip 41%->175deg 80%->95deg,
    #   dip (from 100%) -40%->175deg -80%->120deg. Each reproduces to <1 deg.
    "thumb_mcp": 21.26,
    "thumb_pip": 21.0,     # tuned live against the real thumb
    "thumb_dip": 9.5,
    # Four-finger PIP+DIP, tuned live. Nominally identical hardware, but the
    # effective arm lumps in each finger's own Min/Max_Position_Limit fraction
    # and string/spool winding, so it varies ~+-13% around 5.7.
    "index_pip_dip": 6.3,
    "middle_pip_dip": 5.0,
    "ring_pip_dip": 5.5,
    "pinky_pip_dip": 6.0,
}


def _arm(motor: str | None) -> float:
    return MOMENT_ARM_BY_MOTOR.get(motor, MOMENT_ARM) if motor else MOMENT_ARM


# --- dead zone (slack) + linear flexion --------------------------------------
#
# The first part of a motor's travel doesn't move the joint: the tendon takes up
# slack against the return spring first. Below that dead zone flexion is 0; above
# it, flexion is linear in the remaining travel (the moment-arm model). A pure
# linear model over-bends the start, so the sim looks curled while the real joint
# is still straight. Dead zones (as a FRACTION of each motor's travel), measured:
#   thumb_mcp 20%, thumb_pip 50%, thumb_dip 25%; the four fingers' PIP+DIP ~0.
# Stored in rad of spool (seeded from the fraction x range in set_calibration).
FLEX_SLACK: dict[str, float] = {}
DEADZONE_FRAC: dict[str, float] = {"thumb_mcp": 0.20, "thumb_pip": 0.50,
                                   "thumb_dip": 0.25}


def tendon_to_flexion(theta: float, motor: str | None = None) -> float:
    """Spool rotation (rad) -> total joint flexion (rad). Zero inside the dead
    zone, then linear."""
    te = theta - FLEX_SLACK.get(motor, 0.0)
    return SPOOL_R * te / _arm(motor) if te > 0.0 else 0.0


def flexion_to_tendon(flexion: float, motor: str | None = None) -> float:
    return flexion * _arm(motor) / SPOOL_R + FLEX_SLACK.get(motor, 0.0)


# --- 3p: MCP radial/ulnar pair (2 DOF) — DIFFERENTIAL model ------------------
# Two tendons attach either side of the finger base. Confirmed on the robot:
#   * pull BOTH equally      -> pure flexion toward the palm, no spread
#   * pull ONE more          -> flexes AND tilts to that side
# So flexion tracks the SUM of the pulls and spread tracks their DIFFERENCE:
#
#   alpha (flexion) = FLEX_GAIN   * (pull_radial + pull_ulnar) / 2
#   beta  (spread)  = SPREAD_GAIN * (pull_radial - pull_ulnar) / 2
#
# where pull = spool rotation from the extended pose (theta from home, made
# positive in the flexing direction). This replaces the PDF's page-3 geometry,
# which was unverified and folded (alpha ran BACKWARDS past ~75 deg, two poses
# mapping to the same tendon lengths). The differential form is monotone and
# exactly invertible with no solver.
#
# Each tendon's pull is turned into a FLEXION-ANGLE CONTRIBUTION by dividing by
# its own moment arm — this is what makes "both fully pulled" read as pure
# flexion (beta = 0) even though the two tendons have very different ranges. It
# is still string length (theta from counts), just scaled per tendon; it is NOT
# a motor-percent ratio.
#
#   a_radial = pull_radial / ARM_radial ,  a_ulnar = pull_ulnar / ARM_ulnar
#   alpha = (a_radial + a_ulnar) / 2
#   beta  = SPREAD_GAIN * (a_radial - a_ulnar) / 2
#
# Moment arms are seeded from each finger's range (so full pull ~= 90 deg per
# side, hence beta 0 when fully flexed) and then tuned live. SPREAD_GAIN may be
# negative to match the URDF's spread axis.
ALPHA_MIN, ALPHA_MAX = 0.0, math.pi / 2
BETA_MIN, BETA_MAX = -math.pi / 2, math.pi / 2

MCP_FLEX_ARM_RADIAL: dict[str, float] = {}       # seeded from range in set_calibration
MCP_FLEX_ARM_ULNAR: dict[str, float] = {}
# Positive after fitting the model to real thumb-contact samples (the fit put the
# spread on the +radial-minus-ulnar side). Per-finger values come from the
# reference file; this default is only for a fresh hand.
MCP_SPREAD_GAIN: dict[str, float] = {f: 0.6 for f in FINGERS4}


def _mcp_arms(finger: str) -> tuple[float, float]:
    return (MCP_FLEX_ARM_RADIAL.get(finger, 1.0),
            MCP_FLEX_ARM_ULNAR.get(finger, 1.0))


def _pulls(theta_radial: float, theta_ulnar: float) -> tuple[float, float]:
    """Spool rotations (from home) -> pull of each tendon, positive when flexing.
    Radial home is range_min (theta rises when pulled); ulnar home is range_max
    (theta falls when pulled), so its pull is -theta."""
    return theta_radial, -theta_ulnar


def alphabeta_to_thetas(alpha: float, beta: float,
                        finger: str = "index") -> tuple[float, float]:
    """(alpha, beta) rad -> (theta_radial, theta_ulnar). Exact, no iteration."""
    Rr, Ru = _mcp_arms(finger)
    kb = MCP_SPREAD_GAIN.get(finger, -0.6)
    a_radial = alpha + (beta / kb if kb else 0.0)     # = a_radial from sum/diff
    a_ulnar = alpha - (beta / kb if kb else 0.0)
    pull_radial = a_radial * Rr
    pull_ulnar = a_ulnar * Ru
    return pull_radial, -pull_ulnar                    # undo ulnar sign convention


def thetas_to_alphabeta(theta_radial: float, theta_ulnar: float,
                        finger: str = "index", **_ignore) -> tuple[float, float]:
    """(theta_radial, theta_ulnar) -> (alpha, beta) rad. Exact inverse."""
    Rr, Ru = _mcp_arms(finger)
    kb = MCP_SPREAD_GAIN.get(finger, -0.6)
    pr, pu = _pulls(theta_radial, theta_ulnar)
    a_radial = pr / Rr                                 # flexion angle each tendon
    a_ulnar = pu / Ru
    alpha = (a_radial + a_ulnar) / 2.0
    beta = kb * (a_radial - a_ulnar) / 2.0
    return alpha, beta


MOTOR_HOME: dict[str, float] = {m.name: m.home for m in HAND_MOTORS}


# --- reference pose ---------------------------------------------------------
# The model's zero is "fingers fully extended", which is NOT where the hand
# rests. So the reference is declared in JOINT space: the user reads the real
# joint angles off the hand as it sits, and we run joint -> motor to find the
# theta each motor *should* have in that pose. The difference against the theta
# actually measured is the offset:
#
#     offset[m] = theta_expected(pose)[m] - theta_measured[m]
#
# Every joint below is one physical DOF. The URDF splits each into two joints
# (a driven one and a mimic), so a single angle here drives both.

# joint key -> (label, unit range hint) per finger group
JOINT_KEYS = {
    "thumb": ("cmc", "mcp", "pip", "dip"),
    "other": ("mcp_flex", "mcp_spread", "pip"),
}
ZERO_POSE: dict[str, dict[str, float]] = {
    "thumb": {k: 0.0 for k in JOINT_KEYS["thumb"]},
    **{f: {k: 0.0 for k in JOINT_KEYS["other"]} for f in FINGERS4},
}


def pose_to_thetas(pose: dict) -> dict[str, float]:
    """Joint angles (DEGREES) -> the theta each motor must be at. Inverse of
    `HandKinematics.joints`, and the reason the reference can be given in the
    units the user can actually measure."""
    rad = math.radians
    out: dict[str, float] = {}

    t = pose.get("thumb") or {}
    if "cmc" in t:
        out["thumb_cmc"] = rad(t["cmc"])          # direct linkage, no tendon
    for joint, motor in (("mcp", "thumb_mcp"), ("pip", "thumb_pip"), ("dip", "thumb_dip")):
        if joint in t:
            # one motor per thumb joint, so the whole tendon serves it alone
            out[motor] = flexion_to_tendon(rad(t[joint]), motor)

    for f in FINGERS4:
        p = pose.get(f) or {}
        if "mcp_flex" in p or "mcp_spread" in p:
            t1, t2 = alphabeta_to_thetas(rad(p.get("mcp_flex", 0.0)),
                                         rad(p.get("mcp_spread", 0.0)), f)
            out[f"{f}_radial_flexor"] = t1
            out[f"{f}_ulnar_flexor"] = t2
        if "pip" in p:
            # the tendon fixes PIP+DIP, and PIP takes PIP_SHARE of it
            pd = f"{f}_pip_dip"
            out[pd] = flexion_to_tendon(rad(p["pip"]) / PIP_SHARE, pd)
    return out


class HandKinematics:
    """Motor readings -> joint angles, with a user-settable reference pose."""

    def __init__(self, path: str | None = None) -> None:
        self.path = path or os.environ.get(
            "HOPEJR_HAND_REF",
            os.path.join(os.path.dirname(os.path.dirname(__file__)), "hand_reference.json"))
        # Encoder count at which each finger is fully extended (theta = 0).
        # None = not measured yet, in which case the home pose is assumed to be
        # the extended one.
        self.raw_ref: dict[str, float | None] = {m.name: None for m in HAND_MOTORS}
        self.pose: dict = json.loads(json.dumps(ZERO_POSE))   # last declared pose
        # {finger: {motor: pct}} — the motor pose that makes the thumb tip and
        # that finger's tip actually touch on the robot. Captured from the live
        # pose; camera tracking blends toward it when a pinch is detected, since
        # independent per-finger retargeting never makes the tips meet.
        self.pinch_pose: dict[str, dict[str, float]] = {}
        # {motor: (range_min, range_max)} read off the servos, pushed in by the
        # manager — these are what a normalized 0..100 actually means.
        self.calibration: dict[str, tuple[float, float]] = {}
        self.load()

    # -- live calibration from the motors
    def set_calibration(self, cal: dict) -> None:
        self.calibration = {
            n: (float(c["range_min"]), float(c["range_max"]))
            for n, c in (cal or {}).items()
            if n in self.raw_ref and c.get("range_max") != c.get("range_min")
        }
        # Seed each motor's dead zone from its fraction of travel (thumb 20/50/25
        # %, four fingers 0). Stored in rad of spool. Skip any tuned/loaded live.
        for m, frac in DEADZONE_FRAC.items():
            if m not in FLEX_SLACK:
                rng = self.calibration.get(m)
                if rng:
                    FLEX_SLACK[m] = frac * abs(rng[1] - rng[0]) / COUNTS_PER_RAD
        # Seed each MCP tendon's moment arm so a FULL pull reads ~90 deg of
        # flexion, which makes "both tendons fully pulled" come out as beta=0
        # (pure flexion) despite their different ranges. Skip any already loaded
        # from the reference file or tuned live.
        for f in FINGERS4:
            for arm_map, tendon in ((MCP_FLEX_ARM_RADIAL, "radial_flexor"),
                                    (MCP_FLEX_ARM_ULNAR, "ulnar_flexor")):
                if f in arm_map:
                    continue
                rng = self.calibration.get(f"{f}_{tendon}")
                if rng:
                    full_pull = abs(rng[1] - rng[0]) / COUNTS_PER_RAD   # rad
                    arm_map[f] = full_pull / (math.pi / 2) or 1.0

    def _range(self, name: str) -> tuple[float, float]:
        return self.calibration.get(name, DEFAULT_RANGE)

    def norm_to_raw(self, name: str, value: float) -> float:
        lo, hi = self._range(name)
        return lo + (value / 100.0) * (hi - lo)

    def raw_to_norm(self, name: str, raw: float) -> float:
        lo, hi = self._range(name)
        return 0.0 if hi == lo else (raw - lo) / (hi - lo) * 100.0

    def ref_raw(self, name: str) -> float:
        """Count of the extended pose; defaults to wherever home sits."""
        r = self.raw_ref.get(name)
        return self.norm_to_raw(name, MOTOR_HOME[name]) if r is None else r

    # -- persistence
    def load(self) -> bool:
        try:
            with open(self.path) as f:
                data = json.load(f)
        except Exception:
            return False
        for k, v in (data.get("raw_ref") or {}).items():
            if k in self.raw_ref:
                self.raw_ref[k] = None if v is None else float(v)
        for k, v in (data.get("moment_arm") or {}).items():
            if k in THETA_LIMITS and v and v > 0:
                MOMENT_ARM_BY_MOTOR[k] = float(v)
        for k, v in (data.get("mcp_flex_arm_radial") or {}).items():
            if k in FINGERS4 and v:
                MCP_FLEX_ARM_RADIAL[k] = float(v)
        for k, v in (data.get("mcp_flex_arm_ulnar") or {}).items():
            if k in FINGERS4 and v:
                MCP_FLEX_ARM_ULNAR[k] = float(v)
        for k, v in (data.get("mcp_spread_gain") or {}).items():
            if k in MCP_SPREAD_GAIN:
                MCP_SPREAD_GAIN[k] = float(v)
        for m, v in (data.get("deadzone_frac") or {}).items():
            if m in THETA_LIMITS and v is not None:
                DEADZONE_FRAC[m] = float(v)
        for m, v in (data.get("flex_slack") or {}).items():
            if m in THETA_LIMITS and v is not None:
                FLEX_SLACK[m] = float(v)
        for f, mv in (data.get("pinch_pose") or {}).items():
            if f in FINGERS4:
                self.pinch_pose[f] = {m: float(x) for m, x in mv.items()}
        for f, joints in (data.get("pose") or {}).items():
            if f in self.pose:
                self.pose[f].update({k: float(v) for k, v in joints.items()
                                     if k in self.pose[f]})
        return True

    def save(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w") as f:
            json.dump({"pose": self.pose, "raw_ref": self.raw_ref,
                       "moment_arm": MOMENT_ARM_BY_MOTOR,
                       "mcp_flex_arm_radial": MCP_FLEX_ARM_RADIAL,
                       "mcp_flex_arm_ulnar": MCP_FLEX_ARM_ULNAR,
                       "mcp_spread_gain": MCP_SPREAD_GAIN,
                       "deadzone_frac": DEADZONE_FRAC,
                       "flex_slack": FLEX_SLACK,
                       "pinch_pose": self.pinch_pose}, f, indent=2)

    # -- pinch closure poses
    @staticmethod
    def _pinch_motors(finger: str) -> list[str]:
        """The digits that must meet: all four thumb motors + that finger's."""
        thumb = ["thumb_cmc", "thumb_mcp", "thumb_pip", "thumb_dip"]
        return thumb + [f"{finger}_radial_flexor", f"{finger}_ulnar_flexor",
                        f"{finger}_pip_dip"]

    # -- contact-sample log (experimental data collection) --------------------
    # Records "finger F is touching the thumb at this motor state" — only the
    # thumb + that finger's motors, many samples across different contact poses.
    # Kept in its own file so it can be cleared/downloaded without touching the
    # calibration. Purpose: fit the four-finger MCP model to real contact data.
    @property
    def samples_path(self) -> str:
        return os.path.join(os.path.dirname(self.path), "contact_samples.json")

    def load_samples(self) -> list:
        try:
            with open(self.samples_path) as f:
                return json.load(f)
        except Exception:
            return []

    def add_contact_sample(self, finger: str, positions: dict[str, float],
                           t: float) -> int:
        if finger not in FINGERS4:
            return len(self.load_samples())
        samples = self.load_samples()
        samples.append({
            "finger": finger, "t": round(t, 1),
            "motors": {m: round(float(positions[m]), 2)
                       for m in self._pinch_motors(finger) if m in positions},
        })
        with open(self.samples_path, "w") as f:
            json.dump(samples, f, indent=2)
        return len(samples)

    def clear_samples(self) -> None:
        try:
            os.remove(self.samples_path)
        except Exception:
            pass

    def capture_pinch(self, finger: str, positions: dict[str, float]) -> dict:
        """Store the current motor pose as the 'tips touching' target for this
        finger. `positions` = live {motor: pct}."""
        if finger not in FINGERS4:
            return self.pinch_pose
        self.pinch_pose[finger] = {m: float(positions[m])
                                   for m in self._pinch_motors(finger)
                                   if m in positions}
        self.save()
        return self.pinch_pose

    def blend_to_pinch(self, cmds: dict[str, float], finger: str,
                       strength: float) -> dict[str, float]:
        """Move the thumb+finger motors a fraction `strength` toward the stored
        pinch pose, leaving the rest of the hand on its normal command."""
        target = self.pinch_pose.get(finger)
        if not target or strength <= 0:
            return cmds
        s = min(1.0, max(0.0, strength))
        for m, tv in target.items():
            if m in cmds:
                cmds[m] += s * (tv - cmds[m])
        return cmds

    def set_mcp_gains(self, radial: dict[str, float] | None = None,
                      ulnar: dict[str, float] | None = None,
                      spread: dict[str, float] | None = None) -> dict:
        """Tune per-finger MCP params live. radial/ulnar = each tendon's flexion
        moment arm (bigger = less flexion per pull; raise the side that over-
        flexes). spread = spread per unit angular imbalance (negative flips the
        tilt direction)."""
        for f, v in (radial or {}).items():
            if f in FINGERS4 and v:
                MCP_FLEX_ARM_RADIAL[f] = float(v)
        for f, v in (ulnar or {}).items():
            if f in FINGERS4 and v:
                MCP_FLEX_ARM_ULNAR[f] = float(v)
        for f, v in (spread or {}).items():
            if f in MCP_SPREAD_GAIN:
                MCP_SPREAD_GAIN[f] = float(v)
        self.save()
        return {"radial": MCP_FLEX_ARM_RADIAL, "ulnar": MCP_FLEX_ARM_ULNAR,
                "spread": MCP_SPREAD_GAIN}

    def set_slack(self, slack: dict[str, float]) -> dict:
        """Per-motor dead zone (rad of spool) before flexion starts. Raise it if
        the sim bends while the real joint is still straight."""
        for m, v in (slack or {}).items():
            if m in THETA_LIMITS and v is not None and v >= 0:
                FLEX_SLACK[m] = float(v)
        self.save()
        return dict(FLEX_SLACK)

    def _reseed_slack_from_frac(self) -> None:
        """Recompute the rad dead zones from the assembly percentages, given the
        live ranges."""
        for m, frac in DEADZONE_FRAC.items():
            rng = self.calibration.get(m)
            if rng:
                FLEX_SLACK[m] = frac * abs(rng[1] - rng[0]) / COUNTS_PER_RAD

    def set_deadzone_pct(self, pct: dict[str, float]) -> dict:
        """Assembly setup: the first N% of each motor's travel that doesn't move
        the joint (tendon slack). Stored as the source of truth; the rad value
        used in the model is recomputed from the live range."""
        for m, v in (pct or {}).items():
            if m in THETA_LIMITS and v is not None and 0 <= v < 100:
                DEADZONE_FRAC[m] = float(v) / 100.0
        self._reseed_slack_from_frac()
        self.save()
        return {m: round(f * 100, 1) for m, f in DEADZONE_FRAC.items()}

    def set_moment_arms(self, arms: dict[str, float]) -> dict:
        """Tune per-motor moment arm (mm) live. Bigger arm = less flexion per
        motor count. Persisted alongside the reference."""
        for k, v in (arms or {}).items():
            if k in THETA_LIMITS and v and v > 0:
                MOMENT_ARM_BY_MOTOR[k] = float(v)
        self.save()
        return dict(MOMENT_ARM_BY_MOTOR)

    # -- reference
    @staticmethod
    def home_positions() -> dict[str, float]:
        """The startup pose: every hand motor sits at a calibration end-stop
        (MotorSpec.home is 0 = range_min or 100 = range_max). This — not
        wherever the hand happens to be — is what the reference is anchored to,
        so the setting survives the hand being moved."""
        return {m.name: m.home for m in HAND_MOTORS}

    def set_reference(self, pose: dict, positions: dict[str, float] | None = None) -> dict:
        """Declare what joint angles (DEGREES) the hand is at in the reference
        pose — by default the HOME pose (motors at their min/max end-stop).

        Solves joint -> motor for the theta each motor should be at there, then
        back-projects to the encoder count that would be theta = 0:

            raw_ref = raw(reference pose) - sign * theta * COUNTS_PER_RAD

        One number per motor, which is all the scale-from-hardware formula needs.
        """
        if positions is None:
            positions = self.home_positions()
        for f, joints in (pose or {}).items():
            if f in self.pose:
                self.pose[f].update({k: float(v) for k, v in joints.items()
                                     if k in self.pose[f]})
        for name, theta in pose_to_thetas(self.pose).items():
            if name in self.raw_ref and name in positions:
                raw_here = self.norm_to_raw(name, positions[name])
                self.raw_ref[name] = raw_here - SIGN[name] * theta * COUNTS_PER_RAD
        self.save()
        return {"pose": self.pose, "raw_ref": self.raw_ref}

    def reset(self) -> dict:
        self.raw_ref = {m.name: None for m in HAND_MOTORS}
        self.pose = json.loads(json.dumps(ZERO_POSE))
        self.save()
        return {"pose": self.pose, "raw_ref": self.raw_ref}

    def set_raw_ref(self, refs: dict[str, float]) -> dict:
        for k, v in refs.items():
            if k in self.raw_ref:
                self.raw_ref[k] = None if v is None else float(v)
        self.save()
        return dict(self.raw_ref)

    def theta(self, name: str, value: float) -> float:
        """Motor reading -> spool rotation from the extended pose."""
        return SIGN[name] * (self.norm_to_raw(name, value) - self.ref_raw(name)) \
            / COUNTS_PER_RAD

    def pose_to_commands(self, pose: dict) -> dict[str, float]:
        """Joint angles (DEGREES) -> normalized 0..100 motor commands.

        The exact inverse of `theta()`. Out-of-reach angles clamp rather than
        fail — a tracked hand routinely asks for poses beyond the tendon range."""
        out: dict[str, float] = {}
        for name, theta in pose_to_thetas(pose).items():
            if name not in SIGN:
                continue
            raw = self.ref_raw(name) + SIGN[name] * theta * COUNTS_PER_RAD
            out[name] = max(0.0, min(100.0, self.raw_to_norm(name, raw)))
        return out

    # -- the actual conversion
    def joints(self, positions: dict[str, float]) -> dict:
        """{motor: normalized 0..100} -> joint angles in DEGREES, grouped by
        finger, plus the corrected theta per motor for debugging."""
        th = {n: self.theta(n, v) for n, v in positions.items() if n in THETA_LIMITS}
        deg = math.degrees
        out: dict[str, dict] = {}

        # thumb: cmc is a direct linkage (no tendon), the rest are tendon joints
        thumb: dict[str, float] = {}
        if "thumb_cmc" in th:
            thumb["cmc"] = deg(th["thumb_cmc"])
        for joint, motor in (("mcp", "thumb_mcp"), ("pip", "thumb_pip"), ("dip", "thumb_dip")):
            if motor in th:
                thumb[joint] = deg(tendon_to_flexion(th[motor], motor))
        if thumb:
            out["thumb"] = thumb

        for f in FINGERS4:
            j: dict[str, float] = {}
            rad, uln = f"{f}_radial_flexor", f"{f}_ulnar_flexor"
            if rad in th and uln in th:
                a, b = thetas_to_alphabeta(th[rad], th[uln], f)
                j["mcp_flex"] = deg(a)      # alpha
                j["mcp_spread"] = deg(b)    # beta
            pd = f"{f}_pip_dip"
            if pd in th:
                # one motor drives PIP and DIP: the tendon sets their SUM, the
                # springs set the split. Free-motion ratio only — on contact the
                # load redistributes it and no geometry predicts that.
                total = deg(tendon_to_flexion(th[pd], pd))
                j["pip"] = total * PIP_SHARE
                j["dip"] = total * (1.0 - PIP_SHARE)
            if j:
                out[f] = j

        return {"fingers": out,
                "theta": {n: round(v, 5) for n, v in th.items()},
                "moment_arm": {n: round(_arm(n), 2) for n in th},
                "flex_slack": {n: round(FLEX_SLACK.get(n, 0.0), 3) for n in th},
                "deadzone_pct": {m: round(f * 100, 1) for m, f in DEADZONE_FRAC.items()},
                "mcp_flex_arm_radial": {f: round(v, 3) for f, v in MCP_FLEX_ARM_RADIAL.items()},
                "mcp_flex_arm_ulnar": {f: round(v, 3) for f, v in MCP_FLEX_ARM_ULNAR.items()},
                "mcp_spread_gain": dict(MCP_SPREAD_GAIN),
                "pinch_captured": sorted(self.pinch_pose.keys()),
                "raw_ref": {n: (None if v is None else round(v, 1))
                            for n, v in self.raw_ref.items()},
                "counts_per_rad": round(COUNTS_PER_RAD, 2),
                "reference_pose": self.pose,
                "reference_at": self.home_positions(),
                "configured": os.path.exists(self.path)}

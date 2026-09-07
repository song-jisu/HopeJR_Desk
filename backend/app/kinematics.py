"""Forward kinematics + per-joint gravity torque for the HopeJR arm.

This is the Python port of the gravity-torque math that used to live only in the
3D viewer (frontend/src/RobotViewer.jsx). Moving it here makes gravity
compensation independent of the browser: the telemetry loop computes, for each
arm joint, the gravity load torque

    tau_g(joint) = sum over downstream links of  (r x m*g) . axis

where r = (link CoM in world) - (joint origin in world), evaluated at the live
pose. Later this feeds admittance-based (gravity-compensated) hand-guiding.

Geometry (joint origins/axes/limits, parent/child tree) is parsed once from the
URDF. Link masses + CoM come from links.get_links() so live UI edits apply.

NOTE: the URDF world is Z-up, so gravity is (0, 0, -9.81) in the root frame.
(The frontend used Y-up because urdf-loader rotates the scene; do not copy its
Y-up gravity vector here.)
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from . import links as linklib

_URDF = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "ros2_ws", "src",
    "hopejr_right_arm_description", "urdf", "hopejr_right_arm.urdf"))

# Arm motors (telemetry names) in kinematic order. "wrist_pitch" maps to the
# URDF joint "hand_wrist_pitch" (see _joint_for).
ARM_MOTORS = ["shoulder_pitch", "shoulder_yaw", "shoulder_roll", "elbow_flex",
              "wrist_roll", "wrist_yaw", "wrist_pitch"]

# Mounting correction: URDF joint zero/axis vs the physical robot. MUST match the
# frontend viewer (RobotViewer.jsx BASE_OFFSET_DEG / BASE_INVERT) so the pose the
# backend reconstructs equals the real arm's pose. Keep the two in sync.
BASE_OFFSET_DEG = {"shoulder_pitch": 0.0}
# elbow_flex is here because the URDF turns it the wrong way round. Measured
# against the encoder reference the joint was set up to (raw 2047 = 90 deg
# between upper arm and forearm, 4096 counts/rev), the real elbow runs at
# +0.659 deg per normalized unit -- folding as the value rises -- while the
# unpatched URDF gave -0.502, straightening. Normalized -88 is a nearly
# straight arm (13 deg), not the folded one the URDF reported.
#
# This only fixes the DIRECTION. The magnitudes still differ, because the URDF
# joint range (136.7 deg) and the servo's calibrated span (131.8 deg) are not
# the same number; that residual is small and has not been corrected here.
BASE_INVERT = {"shoulder_pitch", "shoulder_yaw", "shoulder_roll", "elbow_flex"}

G = np.array([0.0, 0.0, -9.81])   # world gravity, Z-up root frame

# Arm servo encoder resolution. The normalized -100..100 span maps onto the
# servo's calibrated [Min,Max]_Position_Limit window, so the ANGLE that span
# covers is (max-min)/COUNTS_PER_REV of a turn -- a number the URDF does not
# know. Measured, the two disagree: the elbow's calibrated window is 131.8 deg
# while its URDF limits span 136.7, and other joints are further off. Feeding
# the URDF span into motor_to_angle stretches every pose.
COUNTS_PER_REV = 4096


def _rpy_to_R(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw -> R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_R(axis: np.ndarray, q: float) -> np.ndarray:
    """Rodrigues rotation of angle q about a unit axis."""
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c, s, C = math.cos(q), math.sin(q), 1 - math.cos(q)
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


class _Joint:
    __slots__ = ("name", "parent", "child", "origin", "axis", "type", "lower", "upper")

    def __init__(self, el: ET.Element):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        o = el.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        self.origin = _T(_rpy_to_R(rpy), np.array(xyz))
        a = el.find("axis")
        self.axis = np.array([float(v) for v in a.get("xyz").split()]) if a is not None else np.array([1.0, 0, 0])
        lim = el.find("limit")
        self.lower = float(lim.get("lower")) if lim is not None and lim.get("lower") else None
        self.upper = float(lim.get("upper")) if lim is not None and lim.get("upper") else None


class ArmKinematics:
    def __init__(self) -> None:
        root = ET.parse(_URDF).getroot()
        self.joints: dict[str, _Joint] = {}
        self.children: dict[str, list[_Joint]] = {}   # parent link -> joints
        child_links = set()
        for el in root.findall("joint"):
            j = _Joint(el)
            self.joints[j.name] = j
            self.children.setdefault(j.parent, []).append(j)
            child_links.add(j.child)
        # root link = the one that is never a child
        all_links = {el.get("name") for el in root.findall("link")}
        roots = list(all_links - child_links)
        self.root = roots[0] if roots else "base_footprint"
        # precompute the set of descendant links for each arm joint's subtree
        self._subtree: dict[str, list[str]] = {}
        for m in ARM_MOTORS:
            jn = self._joint_for(m)
            self._subtree[m] = self._descendant_links(self.joints[jn].child) if jn else []

    def _joint_for(self, motor: str) -> str | None:
        if motor in self.joints:
            return motor
        alt = "hand_" + motor
        return alt if alt in self.joints else None

    def _descendant_links(self, link: str) -> list[str]:
        out, stack = [], [link]
        while stack:
            lk = stack.pop()
            out.append(lk)
            for j in self.children.get(lk, []):
                stack.append(j.child)
        return out

    def set_calibration(self, cal: dict) -> None:
        """Adopt the servos' own encoder windows as the angular scale.

        `cal` is {motor: {range_min, range_max, ...}} as the backend reports it.
        Without this the normalized span is stretched onto the URDF's joint
        limits, which are not the same angle -- see COUNTS_PER_REV. Joints not
        present, or with a degenerate window, keep the URDF behaviour.
        """
        spans: dict[str, float] = {}
        for m in ARM_MOTORS:
            c = cal.get(m) or {}
            lo, hi = c.get("range_min"), c.get("range_max")
            if lo is None or hi is None or hi <= lo:
                continue
            spans[m] = (hi - lo) / COUNTS_PER_REV * 2.0 * math.pi
        self._span_rad = spans

    def motor_to_angle(self, motor: str, pos_norm: float) -> float | None:
        """Normalized motor position (-100..100) -> URDF joint angle (rad),
        mirroring the frontend mapping (invert, scaled, +base offset).

        The scale comes from the servo's encoder window when set_calibration has
        supplied one, and from the URDF joint limits otherwise. The URDF limits
        still fix WHERE the span sits; only its width is corrected.
        """
        jn = self._joint_for(motor)
        if jn is None:
            return None
        j = self.joints[jn]
        if j.lower is None or j.upper is None or j.upper <= j.lower:
            return None
        p = -pos_norm if motor in BASE_INVERT else pos_norm
        p = max(-100.0, min(100.0, p))
        span = getattr(self, "_span_rad", {}).get(motor)
        if span is None:
            a = j.lower + ((p + 100.0) / 200.0) * (j.upper - j.lower)
        else:
            a = 0.5 * (j.lower + j.upper) + (p / 100.0) * (span / 2.0)
        return a + math.radians(BASE_OFFSET_DEG.get(motor, 0.0))

    def _fk(self, angles: dict[str, float]):
        """Return (link_world[4x4], joint_world) for the current pose.
        joint_world[name] = (origin_xyz(3), axis_unit(3)) in the root frame."""
        world = {self.root: np.eye(4)}
        joint_world: dict[str, tuple] = {}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            Tp = world[parent]
            for j in self.children.get(parent, []):
                Tj = Tp @ j.origin                      # joint frame (before its rotation)
                q = angles.get(j.name, 0.0) if j.type in ("revolute", "continuous") else 0.0
                world[j.child] = Tj @ _T(_axis_R(j.axis, q), np.zeros(3))
                origin = Tj[:3, 3]
                axis_w = Tj[:3, :3] @ j.axis
                nrm = np.linalg.norm(axis_w)
                joint_world[j.name] = (origin, axis_w / nrm if nrm > 1e-12 else axis_w)
                stack.append(j.child)
        return world, joint_world

    def gravity_torque(self, positions: dict[str, float], links: dict) -> dict[str, float]:
        """Per-arm-motor gravity load torque (N*m) at the pose given by the
        normalized motor `positions`. `links` = {link: {mass, com}}."""
        angles = {}
        for m in ARM_MOTORS:
            if m in positions and positions[m] is not None:
                jn = self._joint_for(m)
                a = self.motor_to_angle(m, positions[m])
                if jn is not None and a is not None:
                    angles[jn] = a
        world, joint_world = self._fk(angles)

        out: dict[str, float] = {}
        for m in ARM_MOTORS:
            jw = joint_world.get(self._joint_for(m) or "")
            if jw is None:
                continue
            origin, axis = jw
            tau = 0.0
            for lk in self._subtree.get(m, []):
                meta = links.get(lk)
                if not meta:
                    continue
                mass = float(meta.get("mass", 0.0) or 0.0)
                if mass <= 0.0 or lk not in world:
                    continue
                com_local = np.array((meta.get("com") or [0, 0, 0])[:3] + [0, 0, 0])[:3]
                com_world = (world[lk] @ np.array([*com_local, 1.0]))[:3]
                r = com_world - origin
                tau += float(np.dot(np.cross(r, mass * G), axis))
            out[m] = round(tau, 4)
        return out


_KIN: ArmKinematics | None = None


def get_kinematics() -> ArmKinematics | None:
    global _KIN
    if _KIN is None:
        try:
            _KIN = ArmKinematics()
        except Exception:
            return None
    return _KIN


def gravity_torque(positions: dict[str, float]) -> dict[str, float]:
    """Convenience: gravity torque using the live link masses from links.py."""
    kin = get_kinematics()
    if kin is None:
        return {}
    return kin.gravity_torque(positions, linklib.get_links())

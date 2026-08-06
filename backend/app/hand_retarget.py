"""Camera hand -> HopeJR finger pose by matching PLACEMENT, not joint angles.

The robot's fingers are not built like a human's, so copying joint angles is
wrong in principle (see the URDF):

  * every DOF is a ROLLING CONTACT pair — `X_support` plus a mimic of it one
    diameter away, each turning half the total, so the joint centre translates
    by r*theta instead of staying put. Radii: 7.60 mm for MCP flex / PIP / DIP,
    21.75 mm for MCP spread (the same numbers as "Hopejr Hand.pdf").
  * MCP flexion and MCP spread are SERIAL and not co-located, where a human's
    share one centre.
  * DIP is not free: it mimics PIP at 0.8.

So a robot finger has 3 DOF against a human's 4, with different link geometry.
What can be matched is where the finger physically sits. This module matches two
points per finger:

    tip   — the fingertip
    apex  — the peak of the arc, i.e. the point standing furthest off the palm

The apex is what makes a curl look like the human's: the fingertip alone leaves
the curl shape free (many joint triples reach the same tip), and the apex pins
it. Six residuals against three unknowns, solved as least squares — which also
degrades gracefully when the human pose is simply out of reach.

The apex's position ALONG the finger is taken from the human hand each frame and
then held fixed while solving, so the robot's matched point is a smooth function
of the joint angles (picking the robot's own max every iteration would make the
objective jump as the argmax switches).
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

FINGERS4 = ("index", "middle", "ring", "pinky")

# Fingertip beyond the last joint frame. The URDF ends at `*_dip_2` with no tip
# link, so this is the distal phalanx length — same 15.2 mm as the other rolling
# segments. Only the ratio against the human finger matters (targets are scaled),
# so a small error here is harmless.
TIP_LEN = 0.0152

_URDF = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "ros2_ws", "src", "hopejr_hand_description",
    "urdf", "hopejr_hand.urdf")

# Which DOF drives each joint, and by how much. The 0.5 everywhere is the
# support/mimic split (each half of a rolling pair takes half the rotation).
#
# PIP and DIP flex about equally (see hand_kinematics.PIP_SHARE = 0.5), so the
# distal support gets the same rotation as the proximal one.
_DIP_PER_PIP = 1.0
_DOF = {
    "mcp_1": (None, 0.0), "mcp_2_support": ("alpha", 0.5), "mcp_2": ("alpha", 0.5),
    "pip_support": ("beta", 0.5), "pip": ("beta", 0.5),
    "dip_1_support": ("pip", 0.5), "dip_1": ("pip", 0.5),
    "dip_2_support": ("pip", 0.5 * _DIP_PER_PIP), "dip_2": ("pip", 0.5 * _DIP_PER_PIP),
}
# Robot frames that correspond to the human's MCP / PIP / DIP landmarks. The
# fingertip is appended separately.
_POLY = ("mcp_2_support", "dip_1_support", "dip_2_support")


def _rot(axis, t):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < 1e-12 or abs(t) < 1e-15:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def _rpy(r, p, y):
    return _rot([0, 0, 1], y) @ _rot([0, 1, 0], p) @ _rot([1, 0, 0], r)


class _Chain:
    """One finger's joint list, read straight out of the URDF."""

    def __init__(self, joints):
        self.joints = joints          # [(suffix, xyz, R_fixed, axis, dof, gain)]

    def frames(self, alpha, beta, pip):
        """Origins of every joint frame, in the `hand` link frame (metres)."""
        q = {"alpha": alpha, "beta": beta, "pip": pip}
        M = np.eye(4)
        out = {}
        for suffix, xyz, Rf, axis, dof, gain in self.joints:
            R = Rf if dof is None else Rf @ _rot(axis, q[dof] * gain)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = xyz
            M = M @ T
            out[suffix] = M[:3, 3].copy()
        out["_tip"] = (M @ np.array([TIP_LEN, 0, 0, 1.0]))[:3]
        return out

    def polyline(self, alpha, beta, pip):
        f = self.frames(alpha, beta, pip)
        return np.array([f[s] for s in _POLY] + [f["_tip"]])


def _load_chains(path=_URDF):
    root = ET.parse(path).getroot()
    by_name = {j.get("name"): j for j in root.findall("joint")}
    chains = {}
    for f in FINGERS4:
        joints = []
        for suffix, (dof, gain) in _DOF.items():
            j = by_name.get(f"{f}_{suffix}")
            if j is None:
                raise KeyError(f"{f}_{suffix} missing from {path}")
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0")).split()]) if o is not None \
                else np.zeros(3)
            r, p, y = [float(v) for v in (o.get("rpy", "0 0 0")).split()] if o is not None \
                else (0, 0, 0)
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1]
            joints.append((suffix, xyz, _rpy(r, p, y), axis, dof, gain))
        chains[f] = _Chain(joints)
    return chains


CHAINS = _load_chains()

# Search bounds (degrees) — deliberately generous. What a finger can actually
# reach depends on its own Min/Max_Position_Limit and reference datum, and
# hand_kinematics.pose_to_commands clamps there; bounding tighter here would
# instead quietly distort the solve. PIP now runs to ~84 deg on the pinky (it
# has the longest travel), where the old chord model capped everything at 68.
BOUNDS = {"alpha": (0.0, 90.0), "beta": (-45.0, 45.0), "pip": (0.0, 90.0)}


# --- generic polyline helpers -------------------------------------------------
def _arc_fractions(pts):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    return (cum / total if total > 1e-9 else cum), total


def _point_at(pts, frac):
    """Sample a polyline at a normalized arc-length fraction."""
    f, _ = _arc_fractions(pts)
    i = int(np.searchsorted(f, frac, side="right")) - 1
    i = max(0, min(len(pts) - 2, i))
    span = f[i + 1] - f[i]
    t = 0.0 if span < 1e-9 else (frac - f[i]) / span
    return pts[i] + t * (pts[i + 1] - pts[i])


def _apex(pts):
    """The peak of the arc the finger bends into: the polyline point furthest
    from the straight MCP->tip chord. Returns (position, arc-length fraction,
    bulge height).

    Measuring the bulge off the chord rather than off the palm plane is what
    makes this well defined. Height above the palm grows monotonically along a
    finger, so its maximum is always the fingertip and carries no shape
    information; distance from the chord peaks in the middle, which is the
    knuckle that stands out when the hand closes. It also needs no palm normal,
    so there is no left/right sign to get wrong."""
    chord = pts[-1] - pts[0]
    n = np.linalg.norm(chord)
    if n < 1e-9:
        d = np.linalg.norm(pts - pts[0], axis=1)
    else:
        u = chord / n
        rel = pts - pts[0]
        d = np.linalg.norm(rel - np.outer(rel @ u, u), axis=1)
    i = int(np.argmax(d))
    f, _ = _arc_fractions(pts)
    return pts[i].copy(), float(f[i]), float(d[i])


# --- palm frames --------------------------------------------------------------
def _frame(x_dir, in_plane):
    """Right-handed frame: x along the fingers, z off the palm."""
    x = x_dir / (np.linalg.norm(x_dir) or 1)
    z = np.cross(x, in_plane)
    z /= (np.linalg.norm(z) or 1)
    y = np.cross(z, x)
    return np.stack([x, y, z])          # rows = axes


def _robot_palm_frame():
    """From the robot's own extended pose: x toward the fingertips, y across the
    knuckles, z off the palm."""
    mcp = {f: CHAINS[f].frames(0, 0, 0)["mcp_2_support"] for f in FINGERS4}
    tip = {f: CHAINS[f].frames(0, 0, 0)["_tip"] for f in FINGERS4}
    across = mcp["pinky"] - mcp["index"]
    along = np.mean([tip[f] - mcp[f] for f in FINGERS4], axis=0)
    return _frame(along, across), mcp


ROBOT_R, ROBOT_MCP = _robot_palm_frame()

# MediaPipe landmark indices: MCP, PIP, DIP, TIP
_LM = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12),
       "ring": (13, 14, 15, 16), "pinky": (17, 18, 19, 20)}


def _human_palm_frame(lm):
    across = lm[17] - lm[5]                       # index knuckle -> pinky knuckle
    along = lm[9] - lm[0]                         # wrist -> middle knuckle
    return _frame(along, across)


def targets_from_landmarks(landmarks, handedness: str = "Right") -> dict:
    """Per finger: where the robot's tip and arc apex should sit, in the hand
    frame (metres), plus the apex's arc-length fraction and bulge height.

    Each finger is anchored at its OWN MCP and scaled by its own length ratio, so
    a big or small hand retargets without the palm sizes having to agree.

    The robot is a RIGHT hand. Both palm frames are built by the same rule, so a
    right human hand maps across directly; a left one is a mirror image and its
    across-the-palm axis is negated, otherwise the fingers would come out in
    reverse order."""
    lm = np.asarray(landmarks, float)
    if lm.shape != (21, 3):
        raise ValueError(f"expected 21x3 landmarks, got {lm.shape}")
    Rh = _human_palm_frame(lm)
    # A mirror negates ONE axis — the across-the-palm one, so a left hand's
    # fingers stop coming out in reverse order. (Negating two axes is a rotation,
    # not a reflection, and saturates every joint.)
    mirror = np.array([1.0, -1.0, 1.0]) if str(handedness).lower().startswith("l") \
        else np.ones(3)
    out = {}
    for f in FINGERS4:
        pts = lm[list(_LM[f])]
        _, human_len = _arc_fractions(pts)
        if human_len < 1e-6:
            continue
        _, robot_len = _arc_fractions(CHAINS[f].polyline(0, 0, 0))
        scale = robot_len / human_len

        apex_h, frac, bulge = _apex(pts)

        # express relative to this finger's own MCP, in the palm frame, rescaled
        def to_robot(p, _pts=pts, _f=f):
            local = (Rh @ (p - _pts[0])) * scale * mirror
            return ROBOT_MCP[_f] + ROBOT_R.T @ local

        out[f] = {"tip": to_robot(pts[3]), "apex": to_robot(apex_h),
                  "frac": frac, "bulge": bulge * scale}
    return out


# --- inverse kinematics -------------------------------------------------------
# Gauss-Newton on this objective has local minima — the apex term is sampled off
# a polyline, so its Jacobian jumps at segment boundaries and a single descent
# can stall on a bound (measured: 28 deg off on spread-heavy poses). Restart from
# a coarse spread of curls and keep the best.
_SEEDS = np.array([[10.0, 0.0, 10.0], [45.0, 0.0, 30.0], [75.0, 0.0, 55.0],
                   [30.0, -25.0, 30.0], [30.0, 25.0, 30.0]])


def _descend(chain, x0, lo, hi, residual, iters, tol):
    x = np.clip(x0, lo, hi)
    r = residual(x)
    for _ in range(iters):
        c = r @ r
        if c < tol:
            break
        h = 1e-4
        J = np.column_stack([(residual(np.clip(x + h * e, lo, hi)) - r) / h
                             for e in np.eye(3)])
        try:
            step = np.linalg.lstsq(J, -r, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        for scale in (1.0, 0.5, 0.25, 0.1):
            cand = np.clip(x + scale * step, lo, hi)
            rn = residual(cand)
            if rn @ rn < c:
                x, r = cand, rn
                break
        else:
            break
    return x, r


def solve_finger(finger, tip, apex, frac, seed=None, w_apex=1.0,
                 iters=25, tol=1e-10, full=False):
    """Least-squares fit of (alpha, beta, pip) to the tip and apex targets.

    3 unknowns against 6 residuals. Note the tip alone would be exactly
    determined (3 = 3); the apex makes it OVERdetermined on purpose, trading a
    little tip accuracy for a curl that matches the human's. `w_apex` sets that
    trade — 0 reproduces pure fingertip retargeting."""
    chain = CHAINS[finger]
    lo = np.array([BOUNDS["alpha"][0], BOUNDS["beta"][0], BOUNDS["pip"][0]])
    hi = np.array([BOUNDS["alpha"][1], BOUNDS["beta"][1], BOUNDS["pip"][1]])

    def residual(v):
        pts = chain.polyline(*np.radians(v))
        return np.concatenate([pts[-1] - tip, w_apex * (_point_at(pts, frac) - apex)])

    # Streaming: the previous frame's answer is a good basin, so one descent is
    # enough and the whole hand stays well under a camera frame. The seed sweep
    # is for the first frame, after a tracking dropout, or a periodic re-check
    # (`full=True`) in case continuity walked us into a bad local minimum.
    if seed is not None and not full:
        x, r = _descend(chain, np.asarray(seed, float), lo, hi, residual, iters, tol)
        return x, float(np.linalg.norm(r[:3]))

    # The sweep only has to identify the right basin — the next frame's warm
    # descent refines it — so run it short. Full-length descents from five seeds
    # cost ~16 ms per finger; clipped, it is a few.
    starts = list(_SEEDS)
    if seed is not None:
        starts.insert(0, np.asarray(seed, float))
    sweep_iters = max(4, iters // 4)
    best_x, best_r = None, None
    for s in starts:
        x, r = _descend(chain, s, lo, hi, residual, sweep_iters, tol)
        if best_r is None or r @ r < best_r @ best_r:
            best_x, best_r = x, r
            if best_r @ best_r < tol:
                break
    return best_x, float(np.linalg.norm(best_r[:3]))   # (angles, tip error in m)


# A nearly straight finger has no arc, so its "apex" is an arbitrary point on a
# line and constraining it only fights the fingertip. Fade the term in with the
# bulge, full weight by the time the finger stands ~15 mm off its own chord.
BULGE_FULL = 0.015

# Tip error (mm) above which a warm-started finger is re-solved from the seed
# sweep. Well clear of the few mm a genuinely unreachable human pose leaves, so
# it only fires when the solver is actually lost.
RESWEEP_MM = 15.0


# --- thumb --------------------------------------------------------------------
# Not retargeted. The robot's CMC (`thumb_mcp_2`) is ONE revolute about a skewed
# axis over 169 deg, where a human's is a two-DOF saddle joint plus axial roll —
# there is no pose correspondence to solve for. And the robot's thumb continues
# with three flexion segments while MediaPipe only sees two bends. So the thumb
# stays on a direct angle mapping, with the CMC driven by how far the thumb is
# abducted from the index metacarpal.
CMC_SPAN = (10.0, 60.0)          # measured degrees mapped across the joint range
CMC_RANGE = (-172.0, -3.0)


def _bend(a, b, c):
    u, v = b - a, c - b
    n = np.linalg.norm(u) * np.linalg.norm(v)
    if n < 1e-12:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(u @ v) / n))))


def _angle_between(u, v):
    n = np.linalg.norm(u) * np.linalg.norm(v)
    if n < 1e-12:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(u @ v) / n))))


_PINCH_TIP = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}


def pinch_state(landmarks) -> tuple[str | None, float]:
    """Which finger the thumb is pinching, and how strongly (0..1).

    MediaPipe barely sees the thumb's opposition rotation, so instead of trusting
    per-digit angles we detect the pinch geometrically: the closest fingertip to
    the thumb tip, scaled by palm width. Above `far` there is no pinch; below
    `near` it is full. The caller then drives that thumb+finger pair to a stored
    'tips together' pose, which is what actually makes them meet on the robot."""
    lm = np.asarray(landmarks, float)
    scale = np.linalg.norm(lm[17] - lm[5]) or 1.0        # index->pinky knuckles
    dists = {f: np.linalg.norm(lm[4] - lm[t]) / scale for f, t in _PINCH_TIP.items()}
    f = min(dists, key=dists.get)
    near, far = 0.35, 0.9
    s = float(min(1.0, max(0.0, (far - dists[f]) / (far - near))))
    return (f if s > 0 else None), s


def thumb_opposition(landmarks) -> float:
    """Back-compat: pinch strength against the nearest finger."""
    return pinch_state(landmarks)[1]


def thumb_pose(landmarks) -> dict:
    lm = np.asarray(landmarks, float)
    abd = _angle_between(lm[2] - lm[1], lm[5] - lm[0])   # thumb vs index metacarpal
    t = min(1.0, max(0.0, (abd - CMC_SPAN[0]) / (CMC_SPAN[1] - CMC_SPAN[0])))
    distal = min(56.0, _bend(lm[2], lm[3], lm[4]))
    return {"cmc": CMC_RANGE[0] + t * (CMC_RANGE[1] - CMC_RANGE[0]),
            "mcp": min(56.0, _bend(lm[1], lm[2], lm[3])),
            "pip": distal, "dip": distal}


def retarget(landmarks, seed=None, w_apex=1.0, handedness="Right", full=False) -> dict:
    """MediaPipe landmarks -> {finger: {mcp_flex, mcp_spread, pip}} in degrees,
    plus per-finger tip error. `seed` is the previous solution (warm start)."""
    targets = targets_from_landmarks(landmarks, handedness)
    pose, err, info = {}, {}, {}
    for f, t in targets.items():
        s = None
        if seed and f in seed:
            s = [seed[f].get("mcp_flex", 0.0), seed[f].get("mcp_spread", 0.0),
                 seed[f].get("pip", 0.0)]
        w = w_apex * min(1.0, t["bulge"] / BULGE_FULL)
        # Warm start first, and only fall back to the seed sweep if that solution
        # is actually poor. Sweeping on a timer instead cost ~15 ms whenever it
        # fired AND was visible: one finger would twitch to a new solution while
        # the others sat still. Demand-driven, it costs nothing in steady state
        # and still rescues a finger that gets stuck in a bad local minimum.
        (a, b, p), e = solve_finger(f, t["tip"], t["apex"], t["frac"], s, w, full=full)
        if not full and s is not None and e > RESWEEP_MM / 1000.0:
            (a2, b2, p2), e2 = solve_finger(f, t["tip"], t["apex"], t["frac"], s, w, full=True)
            if e2 < e:
                (a, b, p), e = (a2, b2, p2), e2
        pose[f] = {"mcp_flex": float(a), "mcp_spread": float(b), "pip": float(p)}
        err[f] = round(e * 1000, 2)                 # mm
        info[f] = {"apex_at": round(t["frac"], 2), "bulge_mm": round(t["bulge"] * 1000, 1),
                   "w_apex": round(w, 2)}
    return {"pose": pose, "tip_error_mm": err, "apex": info}

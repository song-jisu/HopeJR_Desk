#!/usr/bin/env python3
"""Offline solve over identification sweeps. Reads only; touches no hardware.

    python identification/analyze.py <baseline.csv> [loaded.csv] [--mass 0.5] [--lever 0.25]

Stage A (needs both CSVs) — the current->torque scale k_tau
    Two runs at the SAME endpoints, rate and background pose, one with a known
    mass, differenced. Everything that does not depend on the payload cancels,
    so what is left is a known torque against a measured load reading. Without
    this every identified mass is off by an unknown factor, because the load
    register is drive duty and its counts->N*m scale is not on any datasheet.

Stage B (baseline alone) — gravity and friction at the joint
    Each pose bin averages the two sweep directions to cancel Coulomb friction
    and differences them to recover it:
        (fwd + rev)/2 -> gravity      (fwd - rev)/2 -> friction
    The gravity curve is then fitted as tau = A cos(th) + B sin(th), and the fit
    is reported WITH its conditioning: over a narrow arc cos and sin are nearly
    collinear, and a confident-looking amplitude can be an artifact of that.
    Check `correlation` before believing A and B separately.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from app.kinematics import ARM_MOTORS, G, get_kinematics  # noqa: E402
from app import links as linklib  # noqa: E402

# Fraction of the run's own sweep rate below which a sample is treated as still
# accelerating through a turnaround. It CANNOT be an absolute speed: fixed at
# 5.0 units/s it kept 48% of a rate-5 run but only 6% of a rate-4 one, and the
# 6% that survived were the samples whose noisy velocity estimate happened to
# overshoot -- a biased subset, not a cleaner one. The rate is recorded in each
# run's metadata, so scale to it.
QD_FRAC = 0.6
QD_FLOOR = 0.5
BIN = 5.0        # pose bin width, normalized units
MIN_PER_LEG = 3


def run_rate(path: str) -> float:
    """The sweep rate this run was recorded at, from its metadata sidecar."""
    try:
        with open(os.path.splitext(path)[0] + ".json") as f:
            return float(json.load(f)["rate"])
    except (OSError, KeyError, ValueError, TypeError):
        # Deliberately NOT a bare `except Exception`: it swallowed a missing
        # `import json` as "no metadata" and silently fell back to the floor,
        # so every run analysed with an unfiltered turnaround for a while.
        return 0.0


def bands(path: str, joint: str):
    """-> ({q: (gravity_counts, friction_counts)}, {motor: mean background pose})"""
    rows = list(csv.DictReader(open(path)))
    qd_min = max(QD_FLOOR, QD_FRAC * run_rate(path))
    b, bg = {}, {}
    for r in rows:
        if r["leg"] not in ("fwd", "rev") or abs(float(r[f"{joint}_qd"])) < qd_min:
            continue
        q = round(float(r[f"{joint}_q"]) / BIN) * BIN
        b.setdefault(q, {"fwd": [], "rev": []})[r["leg"]].append(float(r[f"{joint}_i"]))
        for m in ARM_MOTORS:
            bg.setdefault(m, []).append(float(r[f"{m}_q"]))
    out = {}
    for q, d in b.items():
        if len(d["fwd"]) >= MIN_PER_LEG and len(d["rev"]) >= MIN_PER_LEG:
            fm = sum(d["fwd"]) / len(d["fwd"])
            rm = sum(d["rev"]) / len(d["rev"])
            out[q] = ((fm + rm) / 2, (fm - rm) / 2)
    return out, {m: sum(v) / len(v) for m, v in bg.items()}


def payload_torque(kin, joint, q, bg, mass, lever):
    """Gravity torque a point mass at `lever` metres along the link adds at `joint`.

    The direction comes from FK (toward the palm); only the distance is taken
    from the tape measure, since that is the number actually measured.
    """
    angles = {}
    for m in ARM_MOTORS:
        a = kin.motor_to_angle(m, q if m == joint else bg[m])
        jn = kin._joint_for(m)
        if a is not None and jn:
            angles[jn] = a
    world, jw = kin._fk(angles)
    origin, axis = jw[kin._joint_for(joint)]
    tip = (world["hand_palm"] @ np.array([0, 0, 0, 1.0]))[:3]
    u = (tip - origin) / np.linalg.norm(tip - origin)
    return float(np.dot(np.cross(lever * u, mass * G), axis)), angles


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("baseline")
    ap.add_argument("loaded", nargs="?")
    ap.add_argument("--joint", default="elbow_flex")
    ap.add_argument("--mass", type=float, default=0.5, help="payload, kg")
    ap.add_argument("--lever", type=float, default=0.25, help="payload distance from the axis, m")
    ap.add_argument("--k-tau", type=float, default=None,
                    help="counts->N*m; computed from the two runs when omitted")
    a = ap.parse_args()

    kin = get_kinematics()
    if kin is None:
        print("no kinematics (URDF not found)")
        return 1
    none, bg = bands(a.baseline, a.joint)
    print(f"rate {run_rate(a.baseline):.1f}/s -> qd filter "
          f"{max(QD_FLOOR, QD_FRAC * run_rate(a.baseline)):.2f} units/s")
    print(f"background pose: "
          f"{ {m: round(v, 1) for m, v in bg.items() if m != a.joint} }")

    # --- Stage A ------------------------------------------------------------
    k_tau = a.k_tau
    if a.loaded:
        w, _ = bands(a.loaded, a.joint)
        ks = []
        print(f"\n{'q':>7} {'tau_payload':>12} {'dLoad':>8} {'k_tau':>10}")
        for q in sorted(set(none) & set(w)):
            tau, _ = payload_torque(kin, a.joint, q, bg, a.mass, a.lever)
            d = w[q][0] - none[q][0]
            if not d:
                continue
            ks.append(tau / d)
            print(f"{q:>7.0f} {tau:>12.4f} {d:>8.1f} {tau / d:>10.5f}")
        if ks:
            k_tau = float(np.mean(ks))
            sd = float(np.std(ks, ddof=1))
            print(f"\nk_tau = {k_tau:.5f} N*m/count  (sd {sd:.5f}, "
                  f"{100 * sd / abs(k_tau):.1f}%)")
            print("  scatter with no trend across poses = the linear model holds;"
                  " a monotonic drift would mean it does not")
    if k_tau is None:
        print("\nno k_tau: pass a loaded run or --k-tau. Stopping.")
        return 0

    # --- Stage B ------------------------------------------------------------
    th, tau, fric = [], [], []
    for q in sorted(none):
        _, angles = payload_torque(kin, a.joint, q, bg, 0.0, 0.0)
        th.append(angles[kin._joint_for(a.joint)])
        tau.append(k_tau * none[q][0])
        fric.append(abs(k_tau * none[q][1]))
    th, tau = np.array(th), np.array(tau)

    M = np.column_stack([np.cos(th), np.sin(th)])
    p, *_ = np.linalg.lstsq(M, tau, rcond=None)
    resid = tau - M @ p
    sig = np.sqrt((resid ** 2).sum() / max(1, len(tau) - 2))
    cov = sig ** 2 * np.linalg.inv(M.T @ M)
    corr = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])
    C = float(np.hypot(*p))

    print(f"\nswept arc {np.degrees(th.min()):.1f}..{np.degrees(th.max()):.1f} deg "
          f"({np.degrees(np.ptp(th)):.0f} wide)")
    print(f"  tau = {p[0]:.3f}(+-{np.sqrt(cov[0,0]):.3f}) cos(th) "
          f"+ {p[1]:.3f}(+-{np.sqrt(cov[1,1]):.3f}) sin(th)")
    print(f"  amplitude {C:.3f} N*m at phase {np.degrees(np.arctan2(p[1], p[0])):.1f} deg")
    print(f"  residual rms {np.sqrt((resid ** 2).mean()):.4f} N*m over a "
          f"{np.ptp(tau):.3f} span")
    print(f"  cond {np.linalg.cond(M):.1f}, correlation(A,B) {corr:.3f}"
          f"{'   <-- TOO NARROW, widen the sweep' if abs(corr) > 0.8 else ''}")
    print(f"\n  implied m*L = {C / 9.81:.4f} kg*m")
    print(f"  Coulomb friction = {np.mean(fric):.3f} N*m (sd {np.std(fric, ddof=1):.3f})")

    links = linklib.get_links()
    sub = kin._subtree[a.joint]
    um = sum(float(links.get(l, {}).get("mass", 0) or 0) for l in sub)
    print(f"\n  URDF subtree mass ({len(sub)} links): {um:.3f} kg")
    print(f"{'q':>7} {'tau_meas':>10} {'tau_urdf':>10}")
    for q in sorted(none):
        pos = {m: (q if m == a.joint else bg[m]) for m in ARM_MOTORS}
        print(f"{q:>7.0f} {k_tau * none[q][0]:>10.4f} "
              f"{kin.gravity_torque(pos, links).get(a.joint, 0.0):>10.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

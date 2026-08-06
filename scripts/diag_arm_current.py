#!/usr/bin/env python3
"""Diagnose whether the arm's Present_Current reflects gravity — the RIGHT way.

Gravity-compensated hand-guiding via admittance needs the STEADY HOLDING current
(servo fighting gravity at a FIXED goal) to change with pose. Measuring in
compliant mode (Goal=Present) is useless: the servo's position error is ~0, so it
outputs ~0 torque/current and friction holds the arm. We must instead pin the
goal and let the servo bear the load.

This script alternates two phases so you can sample several poses:
  POSE phase  (~3 s, COMPLIANT/soft): move the arm by hand to a new pose.
  HOLD phase  (~2.5 s, STIFF fixed goal): let go; the servo holds against
              gravity. The last 0.5 s of holding current is recorded with tau_g.

USAGE (WSL, arm powered + connected):
    /home/jisu22/miniconda3/envs/lehome/bin/python scripts/diag_arm_current.py

Do ~6-10 cycles spanning shoulder-up / horizontal / down, then Ctrl-C. The
summary prints, per joint, holding current vs tau_g across samples and a rough
current-per-N*m slope. A clear monotonic relation -> admittance is viable. Flat
or all-zero holding current -> current sensing is dead; use model feedforward.

SAFETY: torque is ON in HOLD phase (raw-hold, no jerk). Keep a hand near the arm.
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

from app.backends.serial_bus import SerialBackend   # noqa: E402
from app import kinematics                            # noqa: E402

ARM = kinematics.ARM_MOTORS


def _avg_current(be, secs: float) -> tuple[dict, dict]:
    """Average holding current + pose over `secs` (the settled tail of HOLD)."""
    acc: dict[str, list] = {n: [] for n in ARM}
    pos_acc: dict[str, list] = {n: [] for n in ARM}
    t0 = time.time()
    while time.time() - t0 < secs:
        for m in be.read_motors():
            if m.unit == "arm":
                acc[m.name].append(m.current)
                pos_acc[m.name].append(m.position)
        time.sleep(0.05)
    cur = {n: (sum(v) / len(v)) if v else 0.0 for n, v in acc.items()}
    pos = {n: (sum(v) / len(v)) if v else 0.0 for n, v in pos_acc.items()}
    return cur, pos


def main() -> None:
    be = SerialBackend()
    be.start()
    print("waiting for the arm bus to come up...")
    time.sleep(1.5)
    be.set_servo_enabled(True)     # raw-hold: energize at present pose, no jerk
    time.sleep(0.5)
    print("Cycle: MOVE (soft) -> HOLD (stiff, measuring). Ctrl-C to finish.\n")

    samples: list[tuple[dict, dict]] = []   # (holding_current, tau_g) per pose
    try:
        while True:
            # POSE phase: soft, user repositions the arm
            be.set_compliant(True)
            print(">>> MOVE the arm to a new pose (soft)...        ", end="\r", flush=True)
            time.sleep(3.0)
            # HOLD phase: stiff fixed goal at wherever they left it
            be.set_compliant(False)     # syncs goal=present, servo now holds it
            print(">>> HOLD — let go, servo fighting gravity...     ", end="\r", flush=True)
            time.sleep(2.0)             # settle
            cur, pos = _avg_current(be, 0.5)
            tau = kinematics.gravity_torque(pos)
            samples.append((cur, tau))
            print("sample %2d:  " % len(samples)
                  + "  ".join(f"{n[:9]}: I{cur[n]:5.0f} tg{tau.get(n,0):+6.2f}"
                              for n in ("shoulder_pitch", "shoulder_yaw", "elbow_flex")))
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n--- holding current vs tau_g across %d poses ---" % len(samples))
        for n in ARM:
            xs = [t.get(n, 0.0) for _, t in samples]   # tau_g
            ys = [c[n] for c, _ in samples]            # holding current
            if not xs:
                continue
            spread = max(ys) - min(ys)
            # rough slope (counts per N*m) via least squares through the samples
            slope = _slope(xs, ys)
            print(f"{n:>14}  I[min={min(ys):5.0f} max={max(ys):5.0f} spread={spread:5.0f}]"
                  f"  tg[{min(xs):+6.2f}..{max(xs):+6.2f}]  slope~{slope:7.1f} cnt/Nm")
        print("\nMonotonic, big spread that tracks tau_g -> admittance viable.")
        print("Flat / near-zero holding current -> use model feedforward instead.")
        be.set_compliant(False)
        be.set_servo_enabled(False)
        be.stop()


def _slope(xs, ys) -> float:
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den > 1e-9 else 0.0


if __name__ == "__main__":
    main()

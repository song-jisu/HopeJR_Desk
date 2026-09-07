"""Gravity / friction identification sweeps for the arm.

WHY A SWEEP AND NOT A STATIC HOLD
---------------------------------
Because friction on these geared servos is the same size as the signal.

Measured on the real arm (elbow_flex, -80..-30, 8 units/s, no payload), the
friction term runs 50-66 counts of PWM duty while the gravity term over that
whole range spans 0 to 113. A single-direction measurement therefore carries a
~55-count systematic error through the middle of a ~110-count signal: not a
correction, a corruption.

(A static hold does respond to load — holding the raised end drew a steady
36..58 while the lowered end drew a flat 0, so the earlier expectation that gear
friction would swallow the gravity signal entirely was too pessimistic. It is
still not a usable measurement: the holding value mixes gravity with the P-gain
response to a steady-state position error, and nothing in it separates the two.)

Sweeping the joint slowly in BOTH directions does separate them, because friction
opposes motion (so it flips sign with the direction of travel) while gravity does
not:

    tau_up(q)   = tau_g(q) + tau_f
    tau_down(q) = tau_g(q) - tau_f

    (up + down) / 2  ->  tau_g(q)   friction cancels
    (up - down) / 2  ->  tau_f      and the friction model falls out free

"Slowly" means slow enough that the inertial and Coriolis terms stay negligible
next to gravity and friction, so that the two-direction average really is a
gravity estimate. That is why the sweep temporarily overrides the arm's slew
rate: the jog default (60 units/s) is far too fast for this.

WHAT A RUN IS, AND HOW THEY COMBINE
-----------------------------------
One run = one joint, one payload condition. Runs are combined offline:

  * Two runs at the SAME endpoints and rate, one with payload "none" and one
    with a known mass, differenced, give the current->torque scale k_tau for
    that joint. The unknown-scale problem is otherwise fatal — without k_tau
    every identified mass is off by an unknown factor. The arm mixes an
    sm8512bl (shoulder_pitch) with six sts3250, so k_tau is per joint anyway.

  * Runs across several BACKGROUND POSES (the other joints parked differently)
    are what make the mass regression well posed. A single background pose does
    not excite enough of the parameter space.

Nothing here solves anything: a run only produces data. The regression is a
separate, offline step.

OUTPUT (HOPEJR_IDENT_DIR, default backend/identification/)
    <name>.csv    one row per telemetry sample, all seven arm joints
    <name>.json   metadata: endpoints, rate, payload, calibration, link params

Every row is tagged with a `leg`:
    approach    moving to the start point; not part of the measurement
    fwd / rev   the two measured traverses — these are what gets averaged
    hold_*      stationary at an endpoint, kept deliberately: it is what the
                static measurement would have given, recorded alongside the
                sweep so the two can be compared rather than argued about
"""
from __future__ import annotations

import asyncio
import csv
import json
import os
import time
from typing import Optional

from .hardware import ARM_MOTORS, MOTOR_BY_NAME
from .models import ActionResult, SweepRequest

ARM_ORDER = [m.name for m in ARM_MOTORS]

IDENT_DIR = os.environ.get(
    "HOPEJR_IDENT_DIR",
    os.path.join(os.path.dirname(os.path.dirname(__file__)), "identification"))

# Per-joint columns.
#   i    SIGNED torque proxy (Present_Load) — the whole point: the sign is what
#        separates gravity from friction.
#   cur  measured current MAGNITUDE (Present_Current). A different register and
#        a different quantity, kept because it is a real measurement where `i`
#        is only the drive duty — and because it reads a flat 0 on models that
#        do not implement it, which is worth seeing in the data rather than
#        discovering later.
#   qd   signed, so the offline pass can select samples by direction of travel
#        rather than trusting the leg tag alone.
#   out  the slew-limited goal actually sent to the servo. `cmd` jumps to the
#        endpoint the instant a leg starts, so (cmd - q) is dominated by the ramp
#        and says nothing; (out - q) is the real tracking error, which is how a
#        joint pressed against an end-stop is told from one still ramping.
_PER_JOINT = ("q", "cmd", "out", "i", "cur", "qd", "temp")

# Endpoints are inset from the configured limits by this much. Pushing into a
# hard end-stop spikes the current for reasons unrelated to gravity — measured
# at 5.0 the elbow still reached its stop and held there at load 283, six times
# what gravity was giving at the same pose, so the inset is now 10.
EDGE_MARGIN = 10.0


def _columns() -> list[str]:
    cols = ["t", "cycle", "leg", "target", "volt"]
    for n in ARM_ORDER:
        cols += [f"{n}_{f}" for f in _PER_JOINT]
    return cols


class SweepRunner:
    """Owns the one sweep that may be in flight. Held by the RobotManager."""

    def __init__(self, mgr) -> None:
        self.mgr = mgr
        self._task: Optional[asyncio.Task] = None
        self._abort = False
        self._rows: list[dict] = []
        self._t0 = 0.0
        self._last_ts = 0.0
        self._state: dict = {"active": False}
        os.makedirs(IDENT_DIR, exist_ok=True)

    # --- capability / status -------------------------------------------------
    def supported(self) -> bool:
        """Needs a backend whose slew rate can be overridden — i.e. the real
        serial bus. There is nothing to identify on a simulated arm."""
        return callable(getattr(self.mgr.backend, "set_slew_rate", None))

    def status(self) -> dict:
        st = dict(self._state)
        st["supported"] = self.supported()
        st["runs"] = self.list_runs()
        return st

    def list_runs(self) -> list[dict]:
        out = []
        try:
            names = sorted(f for f in os.listdir(IDENT_DIR) if f.endswith(".json"))
        except OSError:
            return out
        for f in names:
            try:
                with open(os.path.join(IDENT_DIR, f)) as fh:
                    meta = json.load(fh)
                out.append({k: meta.get(k) for k in
                            ("name", "joint", "payload", "rate", "cycles",
                             "start", "end", "samples", "ts", "aborted")})
            except Exception:
                continue
        return out

    def run_path(self, name: str, ext: str) -> Optional[str]:
        """Resolve a run file, refusing anything that escapes IDENT_DIR."""
        p = os.path.normpath(os.path.join(IDENT_DIR, f"{name}.{ext}"))
        if os.path.dirname(p) != os.path.normpath(IDENT_DIR):
            return None
        return p if os.path.exists(p) else None

    # --- start / stop --------------------------------------------------------
    def start(self, req: SweepRequest) -> ActionResult:
        """Validate synchronously so a bad request fails in the HTTP response
        rather than vanishing into a background task."""
        if not self.supported():
            return ActionResult(ok=False, message="identification needs the serial backend")
        if self._task and not self._task.done():
            return ActionResult(ok=False, message="a sweep is already running")

        # The RUNTIME spec, not the static one: a limit narrowed through
        # /api/config lives there, and command_joint clamps to it. Reading the
        # static table instead would plan a sweep to endpoints the manager then
        # silently clamps, so the run would press on the software limit for the
        # whole of every hold.
        spec = self.mgr.spec(req.joint) if req.joint in MOTOR_BY_NAME else None
        if spec is None:
            return ActionResult(ok=False, message=f"unknown joint {req.joint}")
        if spec.unit != "arm":
            # The hand is tendon-driven and its Present_Load is a PWM duty
            # proxy, not a torque with a clean lever arm. Nothing here applies.
            return ActionResult(ok=False, message="identification is arm-only")

        be = self.mgr.backend
        st = be.status()
        if st.estop:
            return ActionResult(ok=False, message="estop engaged")
        if not st.servo_enabled:
            return ActionResult(ok=False, message="enable the servos first")
        if getattr(be, "drive_arm", True) is False:
            return ActionResult(ok=False, message="arm driving is disabled (HOPEJR_DRIVE_ARM=0)")
        if self.mgr._teaching:
            return ActionResult(ok=False, message="stop teaching first")
        if self.mgr._replay_task and not self.mgr._replay_task.done():
            return ActionResult(ok=False, message="a replay is running")
        if getattr(be, "_calib_active", False):
            return ActionResult(ok=False, message="calibration is active")

        lo = spec.cmd_min + EDGE_MARGIN
        hi = spec.cmd_max - EDGE_MARGIN
        if hi <= lo:
            return ActionResult(ok=False, message=f"{req.joint} range too narrow to sweep")
        a = lo if req.start is None else max(lo, min(hi, req.start))
        b = hi if req.end is None else max(lo, min(hi, req.end))
        if abs(b - a) < 1.0:
            return ActionResult(ok=False, message="start and end are the same point")

        name = req.name or f"{req.joint}_{req.payload or 'none'}_{time.strftime('%Y%m%d_%H%M%S')}"
        name = "".join(c if (c.isalnum() or c in "-_.@") else "_" for c in name)

        plan = {"joint": req.joint, "payload": req.payload, "start": a, "end": b,
                "rate": req.rate, "cycles": req.cycles, "settle": req.settle,
                "name": name}
        self._abort = False
        self._rows = []
        self._last_ts = 0.0
        self._task = asyncio.create_task(self._run(plan))
        return ActionResult(ok=True, message=f"sweeping {req.joint} -> {name}")

    def stop(self) -> ActionResult:
        if not (self._task and not self._task.done()):
            return ActionResult(ok=False, message="no sweep running")
        self._abort = True
        return ActionResult(ok=True, message="stopping; partial data will be saved")

    # --- the run -------------------------------------------------------------
    async def _run(self, plan: dict) -> None:
        mgr, be = self.mgr, self.mgr.backend
        joint, rate = plan["joint"], plan["rate"]
        a, b, settle = plan["start"], plan["end"], plan["settle"]
        leg_s = abs(b - a) / rate           # the slew limiter takes exactly this long
        here = (mgr.latest and next((m.command for m in mgr.latest.motors
                                     if m.name == joint), a)) or a
        approach_s = abs(here - a) / rate

        prev_rate = be.get_slew_rate("arm")
        self._t0 = time.time()
        self._state = {
            "active": True, "joint": joint, "payload": plan["payload"],
            "name": plan["name"], "cycle": 0, "cycles": plan["cycles"],
            "leg": "approach", "samples": 0, "elapsed": 0.0,
            "eta": round(approach_s + settle + plan["cycles"] * 2 * (leg_s + settle), 1),
            "aborted": False, "error": None,
        }
        mgr._current_task = f"identify {joint}"
        mgr.log("info", f"sweep '{plan['name']}': {joint} {a:.1f}<->{b:.1f} @ {rate}/s "
                        f"x{plan['cycles']}, payload={plan['payload']}")
        aborted = False
        try:
            # Slow the arm down for the duration. try/finally restores it, and
            # set_slew_rate(None) would restore it even if that failed.
            be.set_slew_rate("arm", rate)

            # Approach the start point. Recorded, but tagged so the offline
            # pass can drop it: it starts from wherever the arm happened to be.
            mgr.command_joint(joint, a)
            ok = await self._collect(approach_s + settle, 0, "approach", a)

            for c in range(1, plan["cycles"] + 1):
                if not ok:
                    break
                mgr.command_joint(joint, b)
                ok = await self._collect(leg_s, c, "fwd", b)
                if ok:
                    ok = await self._collect(settle, c, "hold_end", b)
                if ok:
                    mgr.command_joint(joint, a)
                    ok = await self._collect(leg_s, c, "rev", a)
                if ok:
                    ok = await self._collect(settle, c, "hold_start", a)
            aborted = not ok
        except asyncio.CancelledError:
            aborted = True
            raise
        except Exception as e:
            aborted = True
            self._state["error"] = f"{type(e).__name__}: {e}"
            mgr.log("error", f"sweep '{plan['name']}': {e}")
        finally:
            be.set_slew_rate("arm", None)
            if be.get_slew_rate("arm") != prev_rate:
                # Somebody changed the default underneath us; say so rather
                # than silently leaving the arm at a rate nobody chose.
                mgr.log("warn", f"arm slew rate restored to default "
                                f"{be.get_slew_rate('arm')}, was {prev_rate} before the sweep")
            mgr._current_task = None
            path = self._save(plan, aborted)
            self._state.update(active=False, aborted=aborted, file=path,
                               samples=len(self._rows))
            mgr.log("info" if not aborted else "warn",
                    f"sweep '{plan['name']}' {'aborted' if aborted else 'done'} — "
                    f"{len(self._rows)} samples -> {os.path.basename(path or '?')}")

    async def _collect(self, seconds: float, cycle: int, leg: str, target: float) -> bool:
        """Record telemetry for `seconds`. False if the run must stop.

        Deliberately time-driven rather than waiting for the joint to arrive: a
        stalled or blocked joint would hang a position-triggered wait forever,
        and the slew limiter makes the traverse time exactly predictable anyway.
        The measured pose is logged regardless, and the offline pass uses that,
        not the command.
        """
        mgr = self.mgr
        # Oversample the telemetry rate and de-duplicate on snapshot ts, so no
        # sample is missed to a scheduling hiccup and none is counted twice.
        period = 0.5 / max(1.0, getattr(mgr, "telemetry_hz", 20.0))
        deadline = time.monotonic() + max(0.0, seconds)
        self._state["leg"] = leg
        self._state["cycle"] = cycle
        while time.monotonic() < deadline:
            if self._abort:
                return False
            snap = mgr.latest
            if snap is not None:
                if snap.status.estop:
                    self._state["error"] = "estop during sweep"
                    return False
                if snap.ts != self._last_ts:
                    self._last_ts = snap.ts
                    self._rows.append(self._row(snap, cycle, leg, target))
                    self._state["samples"] = len(self._rows)
                    self._state["elapsed"] = round(snap.ts - self._t0, 1)
            await asyncio.sleep(period)
        return True

    def _row(self, snap, cycle: int, leg: str, target: float) -> dict:
        by = {m.name: m for m in snap.motors}
        swept = by.get(self._state.get("joint"))
        row = {"t": round(snap.ts - self._t0, 4), "cycle": cycle, "leg": leg,
               "target": round(target, 3),
               "volt": round(swept.voltage, 2) if swept else 0.0}
        for n in ARM_ORDER:
            m = by.get(n)
            if m is None:
                continue
            row[f"{n}_q"] = m.position
            row[f"{n}_cmd"] = m.command
            row[f"{n}_out"] = m.goal
            row[f"{n}_i"] = m.current_signed
            row[f"{n}_cur"] = m.current
            row[f"{n}_qd"] = m.velocity
            row[f"{n}_temp"] = m.temperature
        return row

    def _save(self, plan: dict, aborted: bool) -> Optional[str]:
        if not self._rows:
            return None
        base = os.path.join(IDENT_DIR, plan["name"])
        cols = _columns()
        try:
            with open(base + ".csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
                w.writeheader()
                w.writerows(self._rows)
            meta = dict(plan)
            meta.update({
                "ts": self._t0,
                "samples": len(self._rows),
                "aborted": aborted,
                "csv": os.path.basename(base + ".csv"),
                "columns": cols,
                "telemetry_hz": getattr(self.mgr, "telemetry_hz", None),
                "loop_hz": round(getattr(self.mgr.backend, "loop_hz", lambda: 0.0)(), 2),
                "models": {n: MOTOR_BY_NAME[n].model for n in ARM_ORDER},
                # Normalized positions mean nothing without the calibration they
                # were recorded under, and link params fix what the regression
                # is solving for. Both belong with the data, not alongside it.
                "calibration": self.mgr._calibration_snapshot(),
                "links": self._links(),
                "note": "i = SIGNED raw current/load counts; k_tau (counts->N*m) is "
                        "not applied and must come from a known-mass differential run",
            })
            with open(base + ".json", "w") as f:
                json.dump(meta, f, indent=2)
        except Exception as e:
            self.mgr.log("error", f"saving sweep '{plan['name']}': {e}")
            return None
        return base + ".csv"

    def _links(self) -> dict:
        try:
            from . import links as linklib
            return linklib.get_links()
        except Exception:
            return {}

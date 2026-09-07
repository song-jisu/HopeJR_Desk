"""RobotManager — the 'Robot Manager Server' from PLAN.md §2.

Owns the active backend, runs the telemetry loop, tracks per-finger state
machines + contact detection, holds runtime config overrides and an in-memory
log buffer, and broadcasts telemetry to WebSocket subscribers.

Routers talk only to this manager, never to a backend directly.
"""
from __future__ import annotations

import asyncio
import collections
import os
import time
from typing import Optional

from .backends.base import RobotBackend
from .backends.mock import MockBackend
from .hand_kinematics import HandKinematics
from .identify import SweepRunner
from .hardware import (ALL_MOTORS, FINGER_BY_INDEX, FINGERS, HAND_MOTORS,
                       MOTOR_BY_NAME, MotorSpec, clamp, remap_value)
from .models import (ActionResult, FingerState, MotorState, RobotStatus,
                     Telemetry)

TELEMETRY_HZ = float(os.environ.get("HOPEJR_TELEMETRY_HZ", "20"))
LOG_CAPACITY = 500


class RobotManager:
    def __init__(self, mode: Optional[str] = None) -> None:
        mode = mode or os.environ.get("HOPEJR_BACKEND", "mock")
        self.backend: RobotBackend = self._make_backend(mode)
        self._subscribers: set[asyncio.Queue] = set()
        self._latest: Optional[Telemetry] = None
        self._task: Optional[asyncio.Task] = None
        self._last_tick = time.time()
        self._current_task: Optional[str] = None

        # finger state machines
        self._finger_target: dict[int, float] = {f.index: 0.0 for f in FINGERS}
        self._finger_state: dict[int, str] = {f.index: "idle" for f in FINGERS}
        self._finger_contact: dict[int, bool] = {f.index: False for f in FINGERS}
        self.contact_threshold_ma = float(os.environ.get("HOPEJR_CONTACT_MA", "150"))

        # runtime config overrides (limits/home) keyed by motor name
        self._limit_override: dict[str, MotorSpec] = {}

        # teaching — record positions while the robot is backdriven (torque off)
        self._teaching = False
        self._teach_t0 = 0.0
        self._trajectory: list[dict] = []
        self._replay_task: Optional[asyncio.Task] = None
        self._motions_dir = os.environ.get(
            "HOPEJR_MOTIONS_DIR",
            os.path.join(os.path.dirname(os.path.dirname(__file__)), "motions"))
        os.makedirs(self._motions_dir, exist_ok=True)

        # gravity torque (backend FK) — cache link masses, refresh ~1 Hz
        self._links_cache: dict = {}
        self._links_cache_t = 0.0

        # hand tendon kinematics (motor value -> finger joint angles)
        self.hand_kin = HandKinematics()

        # identification sweeps (gravity/friction parameter ID). Sampled from
        # this loop's snapshots, so it needs the rate they arrive at.
        self.telemetry_hz = TELEMETRY_HZ
        self.ident = SweepRunner(self)

        self.logs: collections.deque = collections.deque(maxlen=LOG_CAPACITY)
        self.log("info", f"RobotManager created (backend={mode})")

    def home_targets(self) -> dict:
        """Per-motor home command. Uses the encoder midpoint (raw ~2048),
        recomputed from the current calibration, when the backend supports it;
        otherwise the spec default."""
        fn = getattr(self.backend, "home_center_targets", None)
        return fn() if fn else {}

    # --- backend factory -----------------------------------------------------
    def _make_backend(self, mode: str) -> RobotBackend:
        if mode == "ros":
            from .backends.ros import RosBackend
            return RosBackend()
        if mode == "serial":
            from .backends.serial_bus import SerialBackend
            return SerialBackend()
        return MockBackend()

    def spec(self, name: str) -> MotorSpec:
        return self._limit_override.get(name) or MOTOR_BY_NAME[name]

    # --- lifecycle -----------------------------------------------------------
    async def start(self) -> None:
        try:
            self.backend.start()
            self.log("info", "backend started")
        except Exception as e:
            # keep serving so the UI loads and can show the failure
            self.log("error", f"backend start failed: {e}")
        self._task = asyncio.create_task(self._loop())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
        self.backend.stop()

    async def _loop(self) -> None:
        period = 1.0 / TELEMETRY_HZ
        while True:
            now = time.time()
            dt = now - self._last_tick
            self._last_tick = now
            try:
                self.backend.step(dt)
                motors = self.backend.read_motors()
                fingers = self._update_fingers(motors)
                status = self.backend.status()
                status.current_task = self._current_task
                gravity = self._gravity_torque(motors, now)
                snap = Telemetry(ts=now, status=status, motors=motors,
                                 fingers=fingers, gravity=gravity,
                                 hand_joints=self._hand_joints(motors))
                self._latest = snap
                self._broadcast(snap)
                if self._teaching:
                    self._trajectory.append({
                        "t": round(now - self._teach_t0, 3),
                        "pos": {m.name: m.position for m in motors},
                    })
            except Exception as e:  # keep the loop alive
                self.log("error", f"telemetry loop: {e}")
            await asyncio.sleep(period)

    def _hand_joints(self, motors: list[MotorState]) -> dict:
        """Finger joint angles (deg) from the tendon model, for live display.
        ~0.25 ms for the whole hand, so it rides along in every telemetry tick."""
        try:
            # A normalized 0..100 only means an encoder count once you know the
            # servo's Min/Max_Position_Limit, and those are re-read from the
            # motors at every startup — so keep the kinematics in step with them.
            if not self.hand_kin.calibration:
                self.hand_kin.set_calibration(self._calibration_snapshot())
            pos = {m.name: m.position for m in motors if m.unit == "hand"}
            return self.hand_kin.joints(pos)
        except Exception as e:
            self.log("error", f"hand kinematics: {e}")
            return {}

    # --- finger state machine + contact detection (PLAN §7) ------------------
    def _update_fingers(self, motors: list[MotorState]) -> list[FingerState]:
        by_name = {m.name: m for m in motors}
        out: list[FingerState] = []
        for f in FINGERS:
            fmotors = [by_name[n] for n in f.motors if n in by_name]
            aperture = (sum(m.position for m in fmotors) / len(fmotors)) if fmotors else 0.0
            max_cur = max((m.current for m in fmotors), default=0.0)
            max_vel = max((abs(m.velocity) for m in fmotors), default=0.0)
            target = self._finger_target[f.index]
            still_closing = target > aperture + 2.0

            # Contact = commanded to keep closing, but stalled (not moving) while
            # drawing high current — the true signature of hitting an object.
            # (Current alone false-triggers during fast free motion.)
            new_contact = still_closing and max_vel < 2.0 and max_cur >= self.contact_threshold_ma
            holding = self._finger_contact[f.index] or new_contact

            if target <= 1.0 and aperture <= 2.0:
                state = "idle"
                self._finger_contact[f.index] = False
            elif holding and still_closing:
                state = "holding"
                self._finger_contact[f.index] = True
                # freeze commands at present position (stop closing)
                for n in f.motors:
                    if n in MOTOR_BY_NAME:
                        self.backend.set_command(n, by_name[n].position)
            elif target < aperture - 1:
                state = "release"
                self._finger_contact[f.index] = False
            elif still_closing:
                state = "closing"
                self._finger_contact[f.index] = False
            else:
                state = "idle"
                self._finger_contact[f.index] = False

            self._finger_state[f.index] = state
            out.append(FingerState(
                index=f.index, name=f.name, state=state,
                aperture=round(aperture, 2), contact=self._finger_contact[f.index],
                max_current=round(max_cur, 1),
            ))
        return out

    # --- gravity torque (backend FK, PLAN §8) --------------------------------
    def _arm_kin_calibrated(self):
        """ArmKinematics with the servos' encoder windows installed as the
        angular scale (see kinematics.set_calibration). Refreshed on the same
        ~1 Hz tick as the link cache, since a re-calibration changes it."""
        from .kinematics import get_kinematics
        from .kinematics import set_gravity_calibration
        cal = self._calibration_snapshot()
        kin = get_kinematics()
        if kin is not None:
            kin.set_calibration(cal)
        # the measured gravity model is anchored to raw counts, so it needs the
        # same live ranges to put a normalized position back into raw
        set_gravity_calibration(cal)
        return kin

    def _gravity_torque(self, motors: list[MotorState], now: float) -> dict:
        """Per-arm-joint gravity load torque at the live pose, or None per joint
        where it is not known.

        Goes through kinematics.gravity_torque(), the module-level function --
        NOT ArmKinematics.gravity_torque(), which is the URDF-only half. Calling
        the method directly is what silently kept the measured model off this
        path while every direct test of it passed.
        """
        try:
            from . import kinematics
            if now - self._links_cache_t > 1.0 or not self._links_cache:
                from . import links as linklib
                self._links_cache = linklib.get_links()
                self._links_cache_t = now
                self._arm_kin_calibrated()   # refresh the angular scale too
            positions = {m.name: m.position for m in motors if m.unit == "arm"}
            return kinematics.gravity_torque(positions)
        except Exception as e:
            self.log("error", f"gravity torque: {e}")
            return {}

    # --- pub/sub -------------------------------------------------------------
    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=5)
        self._subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subscribers.discard(q)

    def _broadcast(self, snap: Telemetry) -> None:
        for q in list(self._subscribers):
            if q.full():
                try:
                    q.get_nowait()
                except Exception:
                    pass
            try:
                q.put_nowait(snap)
            except Exception:
                pass

    @property
    def latest(self) -> Optional[Telemetry]:
        return self._latest

    # --- high-level control (PLAN §4) ----------------------------------------
    def enable_servo(self) -> ActionResult:
        self.backend.set_servo_enabled(True)
        self.log("info", "servo enabled")
        return ActionResult(ok=True, message="servo enabled")

    def disable_servo(self) -> ActionResult:
        self.backend.set_servo_enabled(False)
        self.log("info", "servo disabled")
        return ActionResult(ok=True, message="servo disabled")

    def estop(self) -> ActionResult:
        self.backend.set_estop(True)
        self._current_task = None
        self.log("warn", "EMERGENCY STOP engaged")
        return ActionResult(ok=True, message="estop engaged")

    def recover(self) -> ActionResult:
        self.backend.set_estop(False)
        self.log("info", "recovered from estop")
        return ActionResult(ok=True, message="recovered")

    def home(self) -> ActionResult:
        # Arm: encoder centre, supplied by the backend. Hand: the per-servo
        # MotorSpec.home (0 = calibration min, 100 = calibration max — the tendon
        # hand's rest pose), since the backend returns arm entries only.
        centers = self.home_targets()
        targets = {
            m.name: clamp(self.spec(m.name), centers.get(m.name, self.spec(m.name).home))
            for m in ALL_MOTORS
        }
        for name, target in targets.items():
            self.backend.set_command(name, target)
        # keep finger-state display consistent with the home pose
        for f in FINGERS:
            vals = [targets[n] for n in f.motors if n in targets]
            self._finger_target[f.index] = (sum(vals) / len(vals)) if vals else 0.0
        self._current_task = "home"
        self.log("info", "moving to home (arm: encoder centre, hand: calibration end-stop)")
        return ActionResult(ok=True, message="homing")

    def joint_reset(self) -> ActionResult:
        return self.home()

    def reboot_servo(self, name: str) -> ActionResult:
        if name not in MOTOR_BY_NAME:
            return ActionResult(ok=False, message=f"unknown servo {name}")
        self.backend.reboot_servo(name)
        self.log("info", f"reboot servo {name}")
        return ActionResult(ok=True, message=f"rebooted {name}")

    def scan(self) -> list[dict]:
        """Servo scan — report id/model/online per PLAN §4."""
        latest = {m.name: m for m in (self._latest.motors if self._latest else [])}
        return [{
            "name": m.name, "servo_id": m.servo_id, "unit": m.unit,
            "model": m.model, "online": latest.get(m.name).online if m.name in latest else False,
        } for m in ALL_MOTORS]

    # --- motion commands -----------------------------------------------------
    def command_joint(self, name: str, position: float) -> ActionResult:
        if name not in MOTOR_BY_NAME:
            return ActionResult(ok=False, message=f"unknown joint {name}")
        self.backend.set_command(name, clamp(self.spec(name), position))
        return ActionResult(ok=True)

    def command_finger(self, index: int, aperture: float) -> ActionResult:
        f = FINGER_BY_INDEX.get(index)
        if f is None:
            return ActionResult(ok=False, message=f"unknown finger {index}")
        aperture = max(0.0, min(100.0, aperture))
        self._finger_target[index] = aperture
        if aperture <= 1.0:
            self._finger_contact[index] = False
        for n in f.motors:
            spec = self.spec(n)
            # map 0..100 aperture onto the motor's normalized command range
            cmd = spec.cmd_min + (spec.cmd_max - spec.cmd_min) * (aperture / 100.0)
            self.backend.set_command(n, cmd)
        return ActionResult(ok=True)

    def command_grip(self, fingers: str, aperture: float) -> ActionResult:
        touched = []
        for ch in fingers:
            if ch.isdigit() and int(ch) in FINGER_BY_INDEX:
                self.command_finger(int(ch), aperture)
                touched.append(int(ch))
        self._current_task = f"grip {fingers}"
        self.log("info", f"grip fingers={touched} aperture={aperture}")
        return ActionResult(ok=True, message=f"grip {touched}")

    def set_contact_threshold(self, ma: float) -> ActionResult:
        self.contact_threshold_ma = max(0.0, ma)
        self.log("info", f"contact threshold = {self.contact_threshold_ma} mA")
        return ActionResult(ok=True)

    # --- config (PLAN §8) ----------------------------------------------------
    def update_config(self, name: str, cmd_min=None, cmd_max=None, home=None) -> ActionResult:
        base = self.spec(name)
        if name not in MOTOR_BY_NAME:
            return ActionResult(ok=False, message=f"unknown motor {name}")
        from dataclasses import replace
        new = replace(
            base,
            cmd_min=base.cmd_min if cmd_min is None else float(cmd_min),
            cmd_max=base.cmd_max if cmd_max is None else float(cmd_max),
            home=base.home if home is None else float(home),
        )
        self._limit_override[name] = new
        self.log("info", f"config {name}: [{new.cmd_min},{new.cmd_max}] home={new.home}")
        return ActionResult(ok=True)

    def config_snapshot(self) -> list[dict]:
        return [{
            "name": m.name, "servo_id": m.servo_id, "unit": m.unit, "model": m.model,
            "cmd_min": self.spec(m.name).cmd_min, "cmd_max": self.spec(m.name).cmd_max,
            "home": self.spec(m.name).home,
        } for m in ALL_MOTORS]

    # --- calibration (range finder) ------------------------------------------
    def calibration_supported(self) -> bool:
        return getattr(self.backend, "supports_calibration", lambda: False)()

    def begin_calibration(self, home: bool = False) -> ActionResult:
        if not self.calibration_supported():
            return ActionResult(ok=False, message="calibration only available in serial mode")
        self.backend.begin_calibration(home=home)
        self._current_task = "calibration"
        self.log("info", f"calibration started — torque OFF, backdrive each joint (home={home})")
        return ActionResult(ok=True, message="calibration started")

    def reset_calibration_ranges(self) -> ActionResult:
        if not self.calibration_supported():
            return ActionResult(ok=False, message="only in serial mode")
        self.backend.reset_calibration_ranges()
        self.log("info", "calibration ranges reset")
        return ActionResult(ok=True)

    def end_calibration(self) -> ActionResult:
        if self.calibration_supported():
            self.backend.end_calibration()
        self._current_task = None
        self.log("info", "calibration stopped")
        return ActionResult(ok=True)

    def calibration_status(self) -> dict:
        if not self.calibration_supported():
            return {"active": False, "supported": False, "motors": []}
        st = self.backend.calibration_status()
        st["supported"] = True
        return st

    def save_calibration(self) -> ActionResult:
        if not self.calibration_supported():
            return ActionResult(ok=False, message="calibration only available in serial mode")
        res = self.backend.save_calibration()
        self.log("info", f"calibration saved: {res}")
        return ActionResult(ok=True, message=f"saved ranges: {res}")

    def set_direction(self, name: str, drive_mode: int) -> ActionResult:
        if not self.calibration_supported():
            return ActionResult(ok=False, message="only in serial mode")
        if name not in MOTOR_BY_NAME:
            return ActionResult(ok=False, message=f"unknown motor {name}")
        self.backend.set_direction(name, drive_mode)
        self.log("info", f"{name} drive_mode -> {drive_mode}")
        return ActionResult(ok=True)

    # --- hand-guiding / teaching (PLAN §5) -----------------------------------
    def teaching_supported(self) -> bool:
        return getattr(self.backend, "supports_teaching", lambda: False)()

    def begin_teaching(self) -> ActionResult:
        if not self.teaching_supported():
            return ActionResult(ok=False, message="teaching not supported by this backend")
        self.backend.set_estop(False)
        # Teaching is split by unit, because the two halves cannot be taught the
        # same way:
        #   ARM  — torque OFF, moved by hand, positions recorded (backdrive).
        #   HAND — a tendon mechanism; it cannot be backdriven at all, so it
        #          stays POWERED and is posed from the camera (or the sliders).
        #          Its commanded positions get recorded alongside the arm's.
        self.backend.set_servo_enabled(False, unit="arm")
        self.backend.set_servo_enabled(True, unit="hand")
        self._trajectory = []
        self._teach_t0 = time.time()
        self._teaching = True
        self._current_task = "teaching"
        self.log("info", "teaching started — arm torque OFF (move by hand), "
                         "hand powered (pose it from the camera); recording")
        return ActionResult(ok=True, message="teaching started")

    def end_teaching(self) -> ActionResult:
        self._teaching = False
        self._current_task = None
        # Leave the arm free (torque still off) so it can be repositioned; the
        # hand keeps holding its last pose rather than collapsing.
        self.log("info", f"teaching stopped — {len(self._trajectory)} points recorded")
        return ActionResult(ok=True, message=f"{len(self._trajectory)} points recorded")

    def teaching_status(self) -> dict:
        return {
            "supported": self.teaching_supported(),
            "active": self._teaching,
            "points": len(self._trajectory),
            "duration": round(self._trajectory[-1]["t"], 1) if self._trajectory else 0.0,
            "motions": self.list_motions(),
        }

    # --- calibration drift ---------------------------------------------------
    def _calibration_snapshot(self) -> dict:
        """{motor: {range_min, range_max, drive_mode}} from the backend, or {}
        for backends that have no notion of calibration (mock, ros)."""
        fn = getattr(self.backend, "calibration_snapshot", None)
        try:
            return fn() if fn else {}
        except Exception:
            return {}

    def _remap_points(self, pts: list, recorded_cal: dict) -> tuple[list, int]:
        """Re-express recorded normalized values under the CURRENT calibration.

        Both are percentages of [range_min, range_max], so a changed range makes
        the stored number point somewhere else physically. Going through the raw
        encoder count restores the original pose. Returns (points, n_remapped);
        untouched (and n=0) when there's no snapshot or nothing drifted."""
        now = self._calibration_snapshot()
        if not recorded_cal or not now:
            return pts, 0
        drifted = {n for n, c in recorded_cal.items()
                   if n in now and n in MOTOR_BY_NAME and now[n] != c}
        if not drifted:
            return pts, 0
        out = []
        for p in pts:
            pos = dict(p["pos"])
            for n in drifted & pos.keys():
                pos[n] = remap_value(MOTOR_BY_NAME[n], pos[n], recorded_cal[n], now[n])
            out.append({"t": p["t"], "pos": pos})
        return out, len(drifted)

    def save_motion(self, name: str) -> ActionResult:
        import json, re
        if not self._trajectory:
            return ActionResult(ok=False, message="nothing recorded")
        safe = re.sub(r"[^A-Za-z0-9_-]", "_", name).strip("_") or "motion"
        path = os.path.join(self._motions_dir, f"{safe}.json")
        # Stamp the calibration the points were recorded under. Normalized values
        # are only meaningful relative to it, so replay remaps them if the ranges
        # have changed since (see hardware.remap_value).
        with open(path, "w") as f:
            json.dump({"name": safe, "calibration": self._calibration_snapshot(),
                       "points": self._trajectory}, f)
        self.log("info", f"saved motion '{safe}' ({len(self._trajectory)} pts)")
        return ActionResult(ok=True, message=f"saved {safe}")

    def list_motions(self) -> list:
        import json
        now = self._calibration_snapshot()
        out = []
        for fn in sorted(os.listdir(self._motions_dir)):
            if fn.endswith(".json"):
                try:
                    with open(os.path.join(self._motions_dir, fn)) as f:
                        d = json.load(f)
                    pts = d.get("points", [])
                    cal = d.get("calibration") or {}
                    # drifted: replay will remap it. unknown: recorded before
                    # calibration stamping existed, so it can't be remapped.
                    drift = sum(1 for n, c in cal.items() if n in now and now[n] != c)
                    out.append({"name": d.get("name", fn[:-5]), "points": len(pts),
                                "duration": round(pts[-1]["t"], 1) if pts else 0.0,
                                "calib_drift": drift,
                                "calib_unknown": not cal})
                except Exception:
                    pass
        return out

    def delete_motion(self, name: str) -> ActionResult:
        path = os.path.join(self._motions_dir, f"{name}.json")
        if os.path.exists(path):
            os.remove(path)
            return ActionResult(ok=True, message=f"deleted {name}")
        return ActionResult(ok=False, message="not found")

    async def replay_motion(self, name: str) -> ActionResult:
        import json
        path = os.path.join(self._motions_dir, f"{name}.json")
        if not os.path.exists(path):
            return ActionResult(ok=False, message="not found")
        if self._teaching:
            return ActionResult(ok=False, message="stop teaching first")
        with open(path) as f:
            doc = json.load(f)
        pts = doc.get("points", [])
        if not pts:
            return ActionResult(ok=False, message="empty motion")
        pts, remapped = self._remap_points(pts, doc.get("calibration") or {})
        if remapped:
            self.log("warn", f"'{name}' was recorded under a different calibration — "
                             f"remapped {remapped} joint(s) via raw encoder counts")
        self.backend.set_servo_enabled(True)   # torque ON to drive the recorded path

        async def _run():
            self._current_task = f"replay {name}"
            self.log("info", f"replaying '{name}' ({len(pts)} pts)")
            t0 = time.time()
            for p in pts:
                while (time.time() - t0) < p["t"]:
                    await asyncio.sleep(0.005)
                for mname, val in p["pos"].items():
                    self.command_joint(mname, val)
            self._current_task = None
            self.log("info", f"replay '{name}' done")

        self._replay_task = asyncio.create_task(_run())
        return ActionResult(ok=True, message=f"replaying {name}")

    # --- identification (gravity / friction parameter ID) --------------------
    def identify_supported(self) -> bool:
        return self.ident.supported()

    def begin_sweep(self, req) -> ActionResult:
        return self.ident.start(req)

    def stop_sweep(self) -> ActionResult:
        return self.ident.stop()

    def sweep_status(self) -> dict:
        return self.ident.status()

    # --- logs (PLAN §11) -----------------------------------------------------
    def log(self, level: str, message: str) -> None:
        self.logs.append({"ts": time.time(), "level": level, "message": message})


_manager: Optional[RobotManager] = None


def get_manager() -> RobotManager:
    assert _manager is not None, "manager not initialized"
    return _manager


def init_manager(mode: Optional[str] = None) -> RobotManager:
    global _manager
    _manager = RobotManager(mode)
    return _manager

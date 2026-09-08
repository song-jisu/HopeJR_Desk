"""Direct-serial backend — drives the real HopeJR over lerobot, no ROS2.

This is the simplest real-robot path: Desk opens the Feetech buses itself
(exactly like `teleop_server.py`) and talks straight to the servos.

    Desk backend ──(lerobot FeetechMotorsBus)──> /dev/hopejr_arm  (7 servos, proto 0)
                                                  /dev/hopejr_hand (16 servos, proto 1)

Unlike the ROS backend it also *reads* feedback (present position / current /
temperature), so the Dashboard and Diagnostics are fully live and contact
detection works on hardware.

Requires the environment where lerobot is installed (the hopejr_right_arm
`.venv`), and the serial ports readable. `lerobot` is imported lazily so this
module is importable anywhere.

Env: HOPEJR_ARM_PORT (default /dev/hopejr_arm), HOPEJR_HAND_PORT (/dev/hopejr_hand).

Motion smoothing (SerialBackend.__init__ explains what each layer defends):
    HOPEJR_ARM_RATE / HOPEJR_HAND_RATE     slew limit, normalized units/s (0 = off)
    HOPEJR_ARM_ACCEL / HOPEJR_HAND_ACCEL   servo Acceleration reg, x100 steps/s^2
    HOPEJR_ARM_SPEED / HOPEJR_HAND_SPEED   servo Goal_Velocity reg, steps/s
"""
from __future__ import annotations

import os
import threading
import time

from ..hardware import ALL_MOTORS, ARM_MOTORS, HAND_MOTORS, MOTOR_BY_NAME, clamp
from ..models import MotorState, RobotStatus

ARM_ORDER = [m.name for m in ARM_MOTORS]
HAND_ORDER = [m.name for m in HAND_MOTORS]

# Serialises EVERY serial transaction on EVERY bus in this process.
#
# Module-global, not per-bus, because the thing it protects is module-global: the
# Feetech SDK keeps endianness in scservo_def.SCS_END, which _select() re-asserts
# before each operation. Two buses of different protocols (arm proto 0, hand
# proto 1) therefore fight over one variable, so a per-bus lock leaves the arm
# and hand free to flip it under each other -- a concurrency test caught exactly
# that after the first attempt at this fix.
#
# Two threads contend: the worker loop, and any HTTP handler reading diagnostics
# (raw_report, calibration_status). The symptom without it is a read that comes
# back 0 or None for every motor while the bus is otherwise healthy, which is
# indistinguishable from a real reading unless you already suspect it -- and it
# is the reading someone would use to set a calibration offset.
#
# RLock: read_profile() calls read_all().
_BUS_IO_LOCK = threading.RLock()

# Two different readings stand in for joint torque, and they are not the same
# quantity — both are recorded because each answers something the other cannot.
#
#   Present_Current  magnitude only. Measured on hardware, so it is the honest
#                    "how hard is this motor working" number, but it carries no
#                    direction: an sts3250 reports 0..N whether it is lifting or
#                    lowering. Not implemented on every model (sm8512bl reads a
#                    flat 0), and absent entirely on the scs0009 hand.
#   Present_Load     SIGNED PWM duty, +-1000. Lower fidelity — it is the drive
#                    command, not a measurement — but the sign is what separates
#                    gravity from friction (friction flips with the direction of
#                    travel, gravity does not), so identification needs it.
HAND_CURRENT_REG = "Present_Load"
ARM_CURRENT_REG = "Present_Current"     # magnitude, arm only
ARM_LOAD_REG = "Present_Load"           # signed torque proxy, arm

# Both registers are SIGN-MAGNITUDE, and lerobot does not decode either of them:
# its per-model encoding table omits "Present_Current" for the STS/SMS series,
# and the scs_series table is empty altogether. So both arrive raw, where a
# reading in the negative direction looks like a large positive one (direction
# bit set beside the magnitude) — abs() on that yields a garbage number, not a
# magnitude.
#
# The sign is not cosmetic here. Friction opposes motion, so it flips sign with
# the direction of travel while gravity does not; separating the two (see
# identify.py) needs the direction, not just how hard the servo is pulling.
#
#     arm   Present_Current  bit 15   (Feetech SDK reads it as SCS_TOHOST(v, 15))
#     hand  Present_Load     bit 10   (SCS load is magnitude 0..1000 + direction)
#
# The ARM's Present_Load is the exception: it IS in lerobot's STS/SMS table, so
# lerobot has already decoded it by the time we see it. Decoding again would
# corrupt it — hence sign_bit None below, meaning "already signed, leave it".
ARM_CURRENT_SIGN_BIT = 15
ARM_LOAD_SIGN_BIT = None      # lerobot decodes STS Present_Load for us
HAND_CURRENT_SIGN_BIT = 10


def _signed(value, sign_bit: int | None) -> float:
    """Decode one sign-magnitude word. Negative = the servo is loaded/driving
    toward decreasing position. `sign_bit` None means the value already carries
    its sign (lerobot decoded it) and must be passed through untouched."""
    v = int(value)
    if sign_bit is None:
        return float(v)
    mag = v & ((1 << sign_bit) - 1)
    return float(-mag if (v >> sign_bit) & 1 else mag)



def _build_lerobot_motors():
    """Motor dicts identical to teleop_server.py (ids, models, norm modes)."""
    from lerobot.motors import Motor, MotorNormMode
    arm = {
        "shoulder_pitch": Motor(1, "sm8512bl", MotorNormMode.RANGE_M100_100),
        "shoulder_yaw":   Motor(2, "sts3250",  MotorNormMode.RANGE_M100_100),
        "shoulder_roll":  Motor(3, "sts3250",  MotorNormMode.RANGE_M100_100),
        "elbow_flex":     Motor(4, "sts3250",  MotorNormMode.RANGE_M100_100),
        "wrist_roll":     Motor(5, "sts3250",  MotorNormMode.RANGE_M100_100),
        "wrist_yaw":      Motor(6, "sts3250",  MotorNormMode.RANGE_M100_100),
        "wrist_pitch":    Motor(7, "sts3250",  MotorNormMode.RANGE_M100_100),
    }
    hand = {m.name: Motor(m.servo_id, "scs0009", MotorNormMode.RANGE_0_100) for m in HAND_MOTORS}
    return arm, hand


def _load_calibration(path):
    """Load a lerobot calibration JSON into {motor: MotorCalibration}, or None."""
    if not path or not os.path.exists(path):
        return None
    import json
    from lerobot.motors import MotorCalibration
    with open(path) as f:
        raw = json.load(f)
    return {name: MotorCalibration(**vals) for name, vals in raw.items()}


class _Bus:
    """Thin wrapper around a lerobot FeetechMotorsBus with teleop_server's
    firmware-mismatch-tolerant connect."""

    def __init__(self, port, motors, proto, name, calib_path=None, from_motors=False,
                 drive_modes=None):
        from lerobot.motors.feetech import FeetechMotorsBus
        self.name = name
        self.proto = proto
        self.order = list(motors.keys())
        self.from_motors = from_motors
        # BOTH buses now take their range from the MOTORS (Min/Max_Position_Limit),
        # re-read at every startup — the user sets those limits on the hardware and
        # the backend simply adopts them.
        #   HAND: it's a tendon robot and can't be backdrive-swept, so the limits
        #         are preset by hand in EEPROM.
        #   ARM : the limits come from a normal backdrive sweep, but they now live
        #         in the servos instead of a JSON file.
        # `drive_modes` is the one piece of calibration a Feetech servo can't
        # store (read_calibration() always returns drive_mode=0), so a per-motor
        # direction flip is overlaid from the calibration file if one exists.
        self.drive_modes = dict(drive_modes or {})
        calibration = None if from_motors else _load_calibration(calib_path)
        self.calibrated_from_file = calibration is not None
        self._io = _BUS_IO_LOCK
        self.bus = FeetechMotorsBus(port=port, motors=motors,
                                    protocol_version=proto, calibration=calibration)
        try:
            self.bus.connect()
        except Exception as e:
            # Tolerate a firmware-version mismatch AND a partial bus (some servos
            # not responding, e.g. a loose daisy-chain connector): open the port
            # manually and work with whatever motors are present. Only give up if
            # the port itself cannot be opened.
            msg = str(e).lower()
            if "firmware" in msg or "found motor" in msg or "motor" in msg:
                self.bus.port_handler.openPort()
            else:
                raise
        if calibration is None:
            # Read the range straight off the motors (Min/Max_Position_Limit).
            self.read_calibration_from_motors()
        # NOTE: do NOT call write_calibration() here. The motors already hold the
        # homing offsets from the original calibration; the file calibration is
        # applied in software via the constructor. Re-writing it corrupts the
        # hardware offset and saturates the arm to ±100. (lerobot's own robot
        # classes likewise skip re-writing when already calibrated.)

    def _select(self):
        # CRITICAL: the Feetech SDK keeps a *module-global* endianness
        # (scservo_def.SCS_END), set whenever a PacketHandler is created. With an
        # arm (protocol 0, little-endian) and a hand (protocol 1, big-endian) bus
        # in one process, the last-created bus wins and every read/write on the
        # other bus is parsed with the wrong endianness → garbage (the arm
        # saturates to ±100). Re-assert THIS bus's protocol before every serial
        # operation. teleop_server sidesteps this by using one process per bus.
        try:
            from scservo_sdk import scservo_def
            scservo_def.SCS_SETEND(self.proto)
        except Exception:
            pass

    def read_calibration_from_motors(self) -> bool:
        """Pull range_min/range_max from each servo's Min/Max_Position_Limit
        EEPROM registers and install it as the live calibration.

        lerobot's read_calibration() issues one read per motor, so the bus's
        endianness must be asserted first (see _select). It always returns
        drive_mode=0, so any direction flip is re-applied from self.drive_modes."""
        import dataclasses
        with self._io:
            self._select()
            try:
                cal = self.bus.read_calibration()
            except Exception:
                return False
            for n, dm in self.drive_modes.items():
                if n in cal and dm:
                    cal[n] = dataclasses.replace(cal[n], drive_mode=int(dm))
            self.bus.calibration = cal
            return True

    def write_ranges_to_motors(self, ranges: dict) -> int:
        """Persist {motor: (min, max)} into the servos' Min/Max_Position_Limit.

        Deliberately does NOT touch Homing_Offset (lerobot's write_calibration()
        does, and re-writing it corrupts the hardware offset and saturates the
        arm). Torque must be off so the EEPROM lock is released."""
        with self._io:
            self._select()
            n = 0
            for name, (mn, mx) in ranges.items():
                if name not in self.order or mx <= mn:
                    continue
                self.bus.write("Min_Position_Limit", name, int(mn), normalize=False)
                self.bus.write("Max_Position_Limit", name, int(mx), normalize=False)
                n += 1
            return n

    def enable(self):
        with self._io:
            self._select()
            self.bus.enable_torque()

    def disable(self):
        with self._io:
            self._select()
            try:
                self.bus.disable_torque()
            except Exception:
                pass

    # Onboard motion profile. Both control tables carry these at the same
    # addresses — STS/SMS for the arm, SCS for the hand:
    #     Acceleration  (41, 1 byte)   units of 100 steps/s^2
    #     Goal_Velocity (46, 2 bytes)  steps/s
    # Left at the factory default of 0 a servo reads both as "no limit" and
    # answers a Goal_Position jump at full speed and full torque. Writing them
    # makes the servo generate its own trapezoidal profile.
    PROFILE_REGS = ("Acceleration", "Goal_Velocity")

    def read_profile(self) -> dict:
        """Current {reg: {motor: value}}. Logged before we overwrite, so what
        the servos were actually set to is visible rather than assumed."""
        with self._io:
            return {r: self.read_all(r, normalize=False) for r in self.PROFILE_REGS}

    def write_profile(self, accel: int, speed: int) -> dict:
        """Install the onboard profile on every motor of this bus.

        Both are SRAM registers, so they reset when a servo loses power — this
        runs on each torque enable rather than once at startup. A value <= 0
        leaves that register alone. Returns {reg: motors_written}."""
        with self._io:
            return self._write_profile(accel, speed)

    def _write_profile(self, accel: int, speed: int) -> dict:
        self._select()
        wrote: dict[str, int] = {}
        for reg, val in (("Acceleration", accel), ("Goal_Velocity", speed)):
            if not val or val <= 0:
                continue
            values = {n: int(val) for n in self.order}
            try:
                self.bus.sync_write(reg, values, normalize=False)
                wrote[reg] = len(values)
            except Exception:
                # One deaf servo must not cost the whole bus its profile.
                ok = 0
                for n in self.order:
                    try:
                        self.bus.write(reg, n, int(val), normalize=False)
                        ok += 1
                    except Exception:
                        pass
                wrote[reg] = ok
        return wrote

    def write_goals(self, goals: dict):
        # sync_write works on both protocols (proto 1 hand included).
        with self._io:
            self._select()
            self.bus.sync_write("Goal_Position", goals)

    def write_raw(self, data_name: str, values: dict):
        with self._io:
            self._select()
            self.bus.sync_write(data_name, values, normalize=False)

    def read_some(self, data_name, names, normalize) -> dict:
        """Read a SUBSET of this bus's motors. Protocol 1 has no sync read, so
        splitting a pass across cycles is the only way to keep the loop fast."""
        with self._io:
            self._select()
            out = {}
            for name in names:
                try:
                    out[name] = self.bus.read(data_name, name, normalize=normalize)
                except Exception:
                    pass
            return out

    def read_all(self, data_name, normalize) -> dict:
        # Protocol 1 (scs0009 hand) has no Sync Read — read each motor
        # sequentially. Protocol 0 (arm) uses the fast sync_read.
        with self._io:
            self._select()
            if self.proto == 0:
                return self.bus.sync_read(data_name, normalize=normalize)
            out = {}
            for name in self.order:
                try:
                    out[name] = self.bus.read(data_name, name, normalize=normalize)
                except Exception:
                    pass
            return out


class SerialBackend:
    mode = "serial"

    def __init__(self) -> None:
        self._cmd = {m.name: m.home for m in ARM_MOTORS + HAND_MOTORS}
        # _cmd is where the UI wants the joint; _out is how far the slew limiter
        # has let the goal actually travel (see _worker). They differ only while
        # a commanded move is still ramping.
        self._out = dict(self._cmd)
        self._pos = dict(self._cmd)
        self._cur = {k: 0.0 for k in self._cmd}    # magnitude (see ARM_CURRENT_REG)
        self._load = {k: 0.0 for k in self._cmd}   # SIGNED torque proxy
        self._temp = {k: 0.0 for k in self._cmd}
        self._vel = {k: 0.0 for k in self._cmd}
        # When each motor's position was last read. Velocity needs a per-motor
        # dt, not one global one: the hand is read half per cycle (protocol 1
        # has no sync read), so its samples are two cycles apart while the
        # arm's are one, and a shared dt would report the hand at double speed.
        self._pos_t: dict[str, float] = {}
        self._volt = {k: 0.0 for k in self._cmd}
        self._servo_enabled = False
        self._estop = False
        self._connected = False
        # desired/actual torque state, per bus (applied in the worker thread)
        self._want_arm = False
        self._want_hand = False
        self._arm_on = False
        self._hand_on = False
        self._arm: _Bus | None = None
        self._hand: _Bus | None = None
        self._lock = threading.Lock()
        # calibration (range-finder) state
        self._calib_active = False
        self._calib_pending_home = False
        self._calib_min: dict[str, int] = {}
        self._calib_max: dict[str, int] = {}
        self._calib_cur: dict[str, int] = {}
        self._calib_homing: dict[str, int] = {}
        self._run = False
        self._thread: threading.Thread | None = None
        self._last_read = 0.0
        self._loop_dt = 0.0          # EMA of the serial loop period (see loop_hz)
        self._error: str | None = None
        self.arm_port = os.environ.get("HOPEJR_ARM_PORT", "/dev/hopejr_arm")
        self.hand_port = os.environ.get("HOPEJR_HAND_PORT", "/dev/hopejr_hand")
        _cal = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots")
        # The arm JSON is no longer the range source of truth — the servos'
        # Min/Max_Position_Limit registers are, re-read at every startup. The file
        # survives only as (a) a human-readable record written by save_calibration
        # and (b) the store for drive_mode, which the servo cannot hold.
        self.arm_calib = os.environ.get(
            "HOPEJR_ARM_CALIB", os.path.join(_cal, "hope_jr_arm", "hopejr_arm.json"))
        # NOTE: there is deliberately no hand calibration file at all.
        # Arm follows the UI sliders. This is safe now that (a) the endianness
        # bug is fixed and the arm calibration reads correctly (no more slamming
        # to a limit), (b) enabling torque holds the current pose (raw-hold, no
        # jerk), and (c) homing is a separate explicit action. Set
        # HOPEJR_DRIVE_ARM=0 to hold the arm in place without re-commanding it.
        self.drive_arm = os.environ.get("HOPEJR_DRIVE_ARM", "1") == "1"
        # How often to read current/temperature/voltage, in loop cycles.
        self.diag_every = max(1, int(os.environ.get("HOPEJR_DIAG_EVERY", "20")))
        # Do not re-command a hand servo unless it moved this many normalized
        # percent — kills the hunting buzz from jittery camera commands.
        self.hand_deadband = float(os.environ.get("HOPEJR_HAND_DEADBAND", "1.5"))
        self._sent: dict[str, float] = {}

        # --- motion smoothing ------------------------------------------------
        # Two layers, both defending against the same thing: a slider drag moves
        # the goal instantly, and a position servo answers an instant jump with
        # full speed and full torque (current spike, gear shock, overload trip).
        #
        #   1. onboard profile — the servo ramps its own moves. Hardware
        #      backstop: it still applies to Home, to estop recovery, and to
        #      anything that bypasses the limiter (rate 0).
        #   2. slew limiter — the worker walks the commanded value toward the UI
        #      target at a bounded rate, so the goal a servo ever sees is
        #      already reachable. Normally this is the layer that governs; the
        #      profile only bites when a goal moves faster than the limiter.
        #
        # Profile units are servo counts (accel x100 steps/s^2, speed steps/s);
        # the arm is 4096 counts/rev, the hand 1024 counts over 300 degrees.
        self.arm_accel = int(os.environ.get("HOPEJR_ARM_ACCEL", "10"))
        self.arm_speed = int(os.environ.get("HOPEJR_ARM_SPEED", "1500"))
        self.hand_accel = int(os.environ.get("HOPEJR_HAND_ACCEL", "10"))
        self.hand_speed = int(os.environ.get("HOPEJR_HAND_SPEED", "500"))
        # Slew rates are normalized units per second (arm spans -100..100, hand
        # 0..100). 60 = a full arm sweep in ~3.3 s. The hand follows camera
        # tracking so it may be quicker: 200 = fully open to closed in 0.5 s.
        # 0 disables the limiter for that bus and leaves only the profile.
        self.arm_rate = float(os.environ.get("HOPEJR_ARM_RATE", "60"))
        self.hand_rate = float(os.environ.get("HOPEJR_HAND_RATE", "200"))
        # Kept so a temporary override (the identification sweep runs the arm
        # far slower than jogging) always has a known value to restore to,
        # even if the run that set it died without cleaning up.
        self._rate_default = {"arm": self.arm_rate, "hand": self.hand_rate}

    def _load_drive_modes(self) -> dict:
        """Per-motor direction flips from the arm calibration file. A Feetech
        servo has no register for this, so it's the one field the file still
        owns; missing file → every motor un-flipped."""
        cal = _load_calibration(self.arm_calib)
        return {n: c.drive_mode for n, c in cal.items()} if cal else {}

    @staticmethod
    def _log_bus(label, bus, order) -> None:
        """Dump the calibration each bus actually read off its servos, so a
        mis-set (or unset) Min/Max_Position_Limit is obvious at startup."""
        if bus is None:
            print(f"[serial] {label} bus=FAIL", flush=True)
            return
        cal = bus.bus.calibration or {}
        print(f"[serial] {label} bus=ok calib_from_motors={len(cal)}/{len(order)} "
              "(Min/Max_Position_Limit)", flush=True)
        for n_ in order:
            c = cal.get(n_)
            if c is None:
                print(f"[serial]   {n_:22s} MISSING", flush=True)
                continue
            # A servo left at its factory default reads as the full encoder span
            # (or an empty range) — normalization would then be meaningless.
            span = c.range_max - c.range_min
            warn = "  <-- CHECK (unset/degenerate limits)" if span <= 0 or span >= 4095 else ""
            print(f"[serial]   {n_:22s} range=[{c.range_min},{c.range_max}] "
                  f"drive_mode={c.drive_mode} home={MOTOR_BY_NAME[n_].home:.0f}{warn}",
                  flush=True)

    # --- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        # Buses are opened independently — if one (e.g. a faulted/unplugged arm)
        # fails, the other still works, so the hand stays usable while the arm
        # is down.
        arm_motors, hand_motors = _build_lerobot_motors()
        try:
            # Arm: ranges from the servos' Min/Max_Position_Limit, direction
            # (drive_mode) overlaid from the calibration file.
            self._arm = _Bus(self.arm_port, arm_motors, 0, "ARM", from_motors=True,
                             drive_modes=self._load_drive_modes())
        except Exception as e:
            self._arm = None
            self._error = f"ARM open failed: {e}"
        try:
            # Tendon hand: calibration comes from the servos' own Min/Max
            # Position Limit registers, re-read on every startup.
            self._hand = _Bus(self.hand_port, hand_motors, 1, "HAND", from_motors=True)
        except Exception as e:
            self._hand = None
            self._error = f"HAND open failed: {e}"
        self._connected = (self._arm is not None) or (self._hand is not None)
        self._log_bus("ARM", self._arm, ARM_ORDER)
        self._log_bus("HAND", self._hand, HAND_ORDER)
        if not self._connected:
            raise RuntimeError(self._error or "no serial buses could be opened")
        # Start in a known-safe state: torque OFF. If a previous backend was
        # killed while torque was on, the servos would otherwise stay stiff.
        for b in (self._arm, self._hand):
            if b:
                b.disable()
        self._arm_on = self._hand_on = False
        self._want_arm = self._want_hand = False
        self._run = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._run = False
        if self._thread:
            self._thread.join(timeout=1.0)
        for b in (self._arm, self._hand):
            if b:
                b.disable()
        self._connected = False

    def loop_hz(self) -> float:
        """Actual rate of the serial IO loop. Protocol 1 has no sync read, so the
        hand costs 16 sequential transactions per cycle (48 more on diagnostic
        cycles) — this is usually what limits how fast a command reaches a
        servo, not anything above it."""
        return 0.0 if self._loop_dt <= 0 else 1.0 / self._loop_dt

    def _worker(self) -> None:
        """Serial IO loop: apply torque state, write goals, read feedback."""
        n = 0
        t_prev = time.time()
        while self._run:
            now_ = time.time()
            dt_ = now_ - t_prev
            t_prev = now_
            if 0 < dt_ < 2.0:
                self._loop_dt = dt_ if self._loop_dt <= 0 else 0.9 * self._loop_dt + 0.1 * dt_
            try:
                # apply torque enable/disable transitions, per bus
                for name in ("arm", "hand"):
                    bus = self._arm if name == "arm" else self._hand
                    want = self._want_arm if name == "arm" else self._want_hand
                    on = self._arm_on if name == "arm" else self._hand_on
                    if bus is None or want == on:
                        continue
                    if want:
                        self._enable_holding_pose(bus)
                    else:
                        bus.disable()
                    if name == "arm":
                        self._arm_on = want
                    else:
                        self._hand_on = want

                # SLEW LIMIT: advance _out toward the UI target _cmd by at most
                # rate*dt. Sending _cmd straight through means a slider drag
                # hands the servo an instant jump, which it chases at full speed
                # and full torque; ramping the goal instead keeps every target
                # the servo sees within reach of the previous one.
                #
                # dt is clamped: a stalled serial cycle (a timeout blocks the
                # whole loop) would otherwise budget one huge step and defeat
                # the limit exactly when things are already going wrong.
                if not self._estop and not self._calib_active:
                    dts = min(max(dt_, 1e-3), 0.2)
                    with self._lock:
                        for order_, rate in ((ARM_ORDER, self.arm_rate),
                                             (HAND_ORDER, self.hand_rate)):
                            if rate <= 0:
                                for n_ in order_:
                                    self._out[n_] = self._cmd[n_]
                                continue
                            step = rate * dts
                            for n_ in order_:
                                d = self._cmd[n_] - self._out[n_]
                                self._out[n_] += max(-step, min(step, d))

                # write goals to whichever bus has torque (arm only if opted in).
                # A limp bus is skipped entirely — e.g. during teaching the arm is
                # backdriven and only read, while the hand keeps tracking goals.
                #
                # DEADBAND: a position servo holds its last goal, so re-sending a
                # goal that barely moved just makes it hunt — the buzzing and the
                # jitter under camera tracking. Only write motors whose command
                # actually changed by more than HOPEJR_HAND_DEADBAND percent, and
                # keep holding the rest. Also cuts bus traffic, helping the rate.
                if not self._estop and not self._calib_active:
                    if self._hand and self._hand_on:
                        with self._lock:
                            hand_goals = {}
                            for n_ in HAND_ORDER:
                                o = self._out[n_]
                                sent = self._sent.get(n_)
                                if sent is None or abs(o - sent) >= self.hand_deadband:
                                    hand_goals[n_] = o
                                elif o != sent and o == self._cmd[n_]:
                                    # The ramp has arrived but its last steps were
                                    # each under the deadband. Flush the remainder
                                    # once so a slewed move doesn't stop short of
                                    # the target it was given.
                                    hand_goals[n_] = o
                        if hand_goals:
                            self._hand.write_goals(hand_goals)
                            self._sent.update(hand_goals)
                    if self._arm and self._arm_on and self.drive_arm:
                        with self._lock:
                            arm_goals = {n_: self._out[n_] for n_ in ARM_ORDER}
                        self._arm.write_goals(arm_goals)

                # read present position every cycle (serial IO outside the lock).
                # Skipped during calibration: half-turn homing clears the software
                # calibration, so normalize=True would raise — we read raw below.
                if not self._calib_active:
                    # The arm is one sync_read. The hand is protocol 1, which has
                    # none — 16 sequential transactions, and that alone was
                    # holding the whole loop near 13 Hz. Read half the hand per
                    # pass instead: every motor still refreshes within two
                    # cycles, but goal writes (a single sync_write) go out at
                    # double the rate, which is what actually moves the robot.
                    ap = self._arm.read_all("Present_Position", normalize=True) if self._arm else {}
                    hp = {}
                    if self._hand:
                        half = HAND_ORDER[n % 2::2]
                        hp = self._hand.read_some("Present_Position", half, normalize=True)
                    with self._lock:
                        t_read = time.time()
                        # Differentiate position for velocity. The reading is
                        # quantized to encoder counts, so a bare difference at
                        # 50 Hz is mostly quantization noise — smooth it. Sign
                        # is what the identification sweep needs (which way the
                        # joint is travelling), so this stays signed.
                        for k, v in {**ap, **hp}.items():
                            t_was = self._pos_t.get(k)
                            if t_was is not None:
                                d = t_read - t_was
                                if 0 < d < 1.0:
                                    inst = (v - self._pos.get(k, v)) / d
                                    self._vel[k] = 0.7 * self._vel.get(k, 0.0) + 0.3 * inst
                            self._pos_t[k] = t_read
                        self._pos.update(ap)
                        self._pos.update(hp)
                        self._last_read = t_read

                # calibration: on start, center the arm pose (half-turn homing) so
                # ranges don't cross the encoder wrap — done here in the worker
                # thread to keep all serial access serialized.
                if self._calib_active and self._calib_pending_home:
                    self._do_half_turn_homing()
                    self._calib_pending_home = False

                # calibration: track RAW min/max as the user backdrives each joint
                if self._calib_active:
                    ar = self._arm.read_all("Present_Position", normalize=False) if self._arm else {}
                    hr = self._hand.read_all("Present_Position", normalize=False) if self._hand else {}
                    with self._lock:
                        for k, v in {**ar, **hr}.items():
                            v = int(v)
                            self._calib_cur[k] = v
                            if k not in self._calib_min or v < self._calib_min[k]:
                                self._calib_min[k] = v
                            if k not in self._calib_max or v > self._calib_max[k]:
                                self._calib_max[k] = v

                # Heavier reads (current/temp/voltage). On the hand that is three
                # more sequential passes over 16 servos, so doing it every 4th
                # cycle cost more than the position read itself. They only feed
                # the dashboard and contact detection, which do not need to be
                # fast — HOPEJR_DIAG_EVERY tunes it.
                if n % self.diag_every == 0 and not self._calib_active:
                    self._read_diagnostics()
                n += 1
            except Exception as e:
                self._error = f"{type(e).__name__}: {e}"
            time.sleep(0.02)   # ~50 Hz

    def _enable_holding_pose(self, bus) -> None:
        """Energize one bus without any jerk.

        Sets each servo's Goal_Position to its *present* position in RAW encoder
        units (normalize=False) BEFORE enabling torque, so the hold target equals
        the true physical pose regardless of whether the normalized calibration
        is correct. This is what prevents the arm from slamming to a limit.
        """
        try:
            raw = bus.read_all("Present_Position", normalize=False)
            if raw:
                bus.write_raw("Goal_Position", raw)
        except Exception as e:
            self._error = f"hold-pose {bus.name}: {e}"
        # start UI commands from the current pose too, for this bus's motors.
        # _out must follow _cmd here or the slew limiter would ramp away from a
        # stale value — the pose the bus was last commanded to, not the pose it
        # is actually in — which is the jerk the raw hold above just prevented.
        order = ARM_ORDER if bus is self._arm else HAND_ORDER
        with self._lock:
            for k in order:
                if k in self._pos:
                    self._cmd[k] = self._pos[k]
                self._out[k] = self._cmd[k]
        # Install the onboard trapezoidal profile before torque comes on. These
        # are SRAM registers that a servo loses with power, so this belongs on
        # every enable, not once at startup.
        arm = bus is self._arm
        accel = self.arm_accel if arm else self.hand_accel
        speed = self.arm_speed if arm else self.hand_speed
        try:
            before = bus.read_profile()
            wrote = bus.write_profile(accel, speed)
            def _span(reg):
                v = [x for x in before.get(reg, {}).values() if x is not None]
                return f"{min(v)}..{max(v)}" if v else "?"
            print(f"[serial] {bus.name} profile accel={_span('Acceleration')}->{accel} "
                  f"speed={_span('Goal_Velocity')}->{speed} wrote={wrote}", flush=True)
        except Exception as e:
            # A servo without these registers is not a reason to refuse torque;
            # the software slew limiter still applies.
            self._error = f"profile {bus.name}: {e}"
        try:
            bus.enable()
        except Exception as e:
            self._error = f"enable {bus.name}: {e}"
        self._sent.clear()   # force a fresh goal write after (re-)enabling

    def _read_diagnostics(self) -> None:
        def safe(bus, reg):
            try:
                return bus.read_all(reg, normalize=False)
            except Exception:
                return {}
        ac = safe(self._arm, ARM_CURRENT_REG)
        al = safe(self._arm, ARM_LOAD_REG)
        hc = safe(self._hand, HAND_CURRENT_REG)
        at = safe(self._arm, "Present_Temperature")
        ht = safe(self._hand, "Present_Temperature")
        av = safe(self._arm, "Present_Voltage")
        hv = safe(self._hand, "Present_Voltage")
        with self._lock:
            # Magnitude feeds the UI and contact detection (unchanged); the
            # signed load feeds identification. The arm has two distinct
            # registers for these, the hand only one — so on the hand the two
            # are the same reading, once as magnitude and once with its sign.
            for k, v in ac.items():
                self._cur[k] = abs(_signed(v, ARM_CURRENT_SIGN_BIT))
            for k, v in al.items():
                self._load[k] = _signed(v, ARM_LOAD_SIGN_BIT)
            for k, v in hc.items():
                sv = _signed(v, HAND_CURRENT_SIGN_BIT)
                self._cur[k] = abs(sv)
                self._load[k] = sv
            for k, v in {**at, **ht}.items():
                self._temp[k] = float(v)             # already deg C
            for k, v in {**av, **hv}.items():
                self._volt[k] = float(v) / 10.0      # Feetech Present_Voltage is 0.1V units

    # --- reads ---------------------------------------------------------------
    def step(self, dt: float) -> None:
        pass  # worker thread does the IO

    def read_motors(self) -> list[MotorState]:
        out: list[MotorState] = []
        with self._lock:
            for name, spec in MOTOR_BY_NAME.items():
                out.append(MotorState(
                    name=name, servo_id=spec.servo_id, unit=spec.unit,
                    position=round(self._pos.get(name, 0.0), 3),
                    command=round(self._cmd.get(name, 0.0), 3),
                    velocity=round(self._vel.get(name, 0.0), 3),
                    current=round(self._cur.get(name, 0.0), 1),
                    current_signed=round(self._load.get(name, 0.0), 1),
                    goal=round(self._out.get(name, 0.0), 3),
                    temperature=round(self._temp.get(name, 0.0), 1),
                    voltage=round(self._volt.get(name, 0.0), 1),
                    error=False,
                    online=self._connected,
                ))
        return out

    def status(self) -> RobotStatus:
        fresh = (time.time() - self._last_read) < 1.0
        return RobotStatus(
            connected=self._connected,
            servo_enabled=(self._arm_on or self._hand_on),
            estop=self._estop,
            mode=self.mode,
            comm_ok=self._connected and fresh,
            comm_delay_ms=0.0,
            packet_loss=0.0,
            arm_online=len(ARM_ORDER) if (self._arm and fresh) else 0,
            hand_online=len(HAND_ORDER) if (self._hand and fresh) else 0,
        )

    # --- commands ------------------------------------------------------------
    def set_command(self, name: str, position: float) -> None:
        spec = MOTOR_BY_NAME.get(name)
        if spec is None:
            raise KeyError(name)
        with self._lock:
            self._cmd[name] = clamp(spec, position)

    def get_slew_rate(self, unit: str) -> float:
        return self.arm_rate if unit == "arm" else self.hand_rate

    def set_slew_rate(self, unit: str, rate: float | None) -> float:
        """Override one bus's slew limit; `rate` None restores the configured
        default. Returns the rate now in force.

        The identification sweep uses this: it needs a known, deliberately slow
        constant velocity, which is the opposite of what jogging wants.
        """
        r = self._rate_default[unit] if rate is None else max(0.0, float(rate))
        if unit == "arm":
            self.arm_rate = r
        else:
            self.hand_rate = r
        return r

    def set_servo_enabled(self, enabled: bool, unit: str | None = None) -> None:
        """`unit` None = both buses. Per-unit exists for teaching: the arm goes
        limp so it can be backdriven, while the hand — a tendon mechanism that
        cannot be hand-guided at all — stays powered and follows the camera."""
        want = enabled and not self._estop
        if unit in (None, "arm"):
            self._want_arm = want
        if unit in (None, "hand"):
            self._want_hand = want
        self._servo_enabled = self._want_arm or self._want_hand

    def set_estop(self, engaged: bool) -> None:
        self._estop = engaged
        if engaged:
            self._servo_enabled = False
            self._want_arm = self._want_hand = False

    def reboot_servo(self, name: str) -> None:
        # lerobot has no simple per-servo reboot; integration hook.
        if name not in self._cmd:
            raise KeyError(name)

    # --- hand-guiding (teaching) ---------------------------------------------
    def supports_teaching(self) -> bool:
        return True

    def home_center_targets(self) -> dict:
        """ARM only: normalized command that drives each servo toward the ENCODER
        MIDPOINT (raw Present_Position = resolution/2, i.e. ~2048 on a 12-bit
        encoder) — a fixed raw reference whose normalized value is recomputed
        from the current calibration (so it tracks re-calibration). This is NOT
        the range midpoint (which is always normalized 0); it's the servo's
        physical centre, which lands inside the range when the joint is homed
        to centre.

        The HAND is deliberately excluded: its home is a calibration range
        END-STOP (min or max) per tendon routing, declared as MotorSpec.home in
        hardware.py, not the encoder centre."""
        out = {}
        for m in ARM_MOTORS:
            bus = self._arm
            if bus is None or not bus.bus.calibration or m.name not in bus.bus.calibration:
                continue
            c = bus.bus.calibration[m.name]
            if c.range_max == c.range_min:
                continue
            try:
                model = bus.bus.motors[m.name].model
                center = bus.bus.model_resolution_table[model] // 2
            except Exception:
                center = 2048
            frac = (center - c.range_min) / (c.range_max - c.range_min)  # where centre sits in [min,max]
            norm = frac * 200 - 100                   # arm is RANGE_M100_100
            if c.drive_mode:
                norm = -norm
            out[m.name] = norm
        return out

    # --- calibration (range finder) ------------------------------------------
    def supports_calibration(self) -> bool:
        return True

    def begin_calibration(self, home: bool = False) -> None:
        # Torque OFF so the joints can be backdriven by hand. Recorded min/max
        # start EMPTY and are filled from live hand-movement — on save they
        # OVERWRITE the existing range (not intersect it).
        #
        # `home=True` first centers the arm pose at the encoder midpoint
        # (half-turn homing, writes Homing_Offset) — only needed if a joint's
        # physical range crosses the 0/4095 wrap. Off by default: for
        # limited-range joints a plain min/max sweep is enough and less invasive.
        self._estop = False
        self._servo_enabled = False
        self._want_arm = self._want_hand = False
        with self._lock:
            self._calib_min.clear()
            self._calib_max.clear()
            self._calib_cur.clear()
            self._calib_homing.clear()
        self._calib_pending_home = bool(home)
        self._calib_active = True

    def reset_calibration_ranges(self) -> None:
        """Clear the recorded min/max (keep homing / stay active) so a glitched
        sweep can be redone without re-homing."""
        with self._lock:
            self._calib_min.clear()
            self._calib_max.clear()
            self._calib_cur.clear()

    def _do_half_turn_homing(self) -> None:
        """Center the ARM's current pose at the encoder midpoint (writes
        Homing_Offset). Runs in the worker thread. Hand is skipped: its ranges
        are small (no wrap) and proto-1 has no sync_read for this helper."""
        b = self._arm
        if b is None:
            return
        try:
            b._select()
            b.bus.disable_torque()
            offs = b.bus.set_half_turn_homings()   # writes Homing_Offset; clears bus.calibration
            with self._lock:
                for k, v in offs.items():
                    name = k if isinstance(k, str) else self._name_for_id(b, k)
                    if name:
                        self._calib_homing[name] = int(v)
        except Exception as e:
            self._error = f"half-turn homing: {e}"

    @staticmethod
    def _name_for_id(bus, motor_id):
        for name, spec in MOTOR_BY_NAME.items():
            if spec.servo_id == motor_id and name in bus.order:
                return name
        return None

    def end_calibration(self) -> None:
        self._calib_active = False
        # Restore software calibration so normalized reads resume (half-turn
        # homing cleared it). BOTH buses come back from the servos'
        # Min/Max_Position_Limit — if the user saved first those registers already
        # hold the new ranges, otherwise the previous ones return.
        for label, bus in (("arm", self._arm), ("hand", self._hand)):
            if bus is not None and not bus.read_calibration_from_motors():
                self._error = f"{label}: re-read Min/Max_Position_Limit failed"

    def calibration_status(self) -> dict:
        out = {"active": self._calib_active, "motors": []}
        with self._lock:
            for m in ALL_MOTORS:
                cal = self._current_calib(m.name)
                mn = self._calib_min.get(m.name)
                mx = self._calib_max.get(m.name)
                out["motors"].append({
                    "name": m.name, "unit": m.unit, "servo_id": m.servo_id,
                    "raw": self._calib_cur.get(m.name),
                    "rec_min": mn, "rec_max": mx,
                    "rec_span": (mx - mn) if (mn is not None and mx is not None) else 0,
                    "cur_min": cal.get("range_min"), "cur_max": cal.get("range_max"),
                    "drive_mode": cal.get("drive_mode", 0),
                    "online": (m.unit == "arm" and self._arm is not None) or
                              (m.unit == "hand" and self._hand is not None),
                })
        return out

    def raw_report(self) -> list[dict]:
        """Read-only diagnostic: the numbers behind a normalized position.

        Needed because a normalized reading is lossy — it clamps to the
        calibrated range, so a joint reading -100 could genuinely be at the
        range edge, or be an encoder value that has wrapped past 0/4095 and now
        lands outside the range entirely. Only the raw count tells them apart.
        """
        out = []
        for unit, bus, order in (("arm", self._arm, ARM_ORDER),
                                 ("hand", self._hand, HAND_ORDER)):
            if bus is None:
                continue
            raw = {}
            homing = {}
            try:
                raw = bus.read_all("Present_Position", normalize=False)
                homing = bus.read_all("Homing_Offset", normalize=False) if bus.proto == 0 else {}
            except Exception as e:
                self._error = f"raw_report {unit}: {e}"
            cal = bus.bus.calibration or {}
            for name in order:
                c = cal.get(name)
                r = raw.get(name)
                res = 4096 if unit == "arm" else 1024
                entry = {
                    "name": name, "unit": unit, "raw": r,
                    "homing_offset": homing.get(name),
                    "range_min": getattr(c, "range_min", None),
                    "range_max": getattr(c, "range_max", None),
                    "resolution": res,
                    "normalized": round(self._pos.get(name, 0.0), 2),
                }
                # where the raw value sits relative to the range, and what the
                # nearest wrapped equivalent would be
                if r is not None and c is not None:
                    mid = (c.range_min + c.range_max) / 2
                    k = round((mid - r) / res)
                    entry["unwrapped"] = int(r + k * res)
                    entry["wraps"] = int(k)
                    entry["in_range"] = bool(c.range_min <= r <= c.range_max)
                out.append(entry)
        return out

    def calibration_snapshot(self) -> dict:
        """The live calibration of every motor, for stamping into a recorded
        motion so it can be remapped if the ranges later change."""
        out = {}
        for m in ALL_MOTORS:
            c = self._current_calib(m.name)
            if c:
                out[m.name] = {"range_min": c["range_min"], "range_max": c["range_max"],
                               "drive_mode": c["drive_mode"]}
        return out

    def _current_calib(self, name: str) -> dict:
        """The live MotorCalibration for a motor as a plain dict (or {})."""
        bus = self._arm if MOTOR_BY_NAME[name].unit == "arm" else self._hand
        if bus is None or not bus.bus.calibration or name not in bus.bus.calibration:
            return {}
        c = bus.bus.calibration[name]
        return {"drive_mode": c.drive_mode, "homing_offset": c.homing_offset,
                "range_min": c.range_min, "range_max": c.range_max, "id": c.id}

    def set_direction(self, name: str, drive_mode: int) -> None:
        """Flip a motor's drive_mode live (and persist) so direction can be
        tested immediately during calibration.

        ARM ONLY, and it is the sole calibration field still kept in the file:
        Feetech servos have no drive_mode register, so it cannot round-trip
        through the motor like range_min/range_max now do."""
        import dataclasses
        bus = self._arm
        if MOTOR_BY_NAME[name].unit == "hand" or bus is None:
            # The hand's direction is expressed by the order of its preset
            # Min/Max_Position_Limit — swap those on the servo instead.
            self._error = "hand direction is set on the motor, not here"
            return
        dm = int(drive_mode)
        bus.drive_modes[name] = dm
        cal = bus.bus.calibration or {}
        if name in cal:
            cal[name] = dataclasses.replace(cal[name], drive_mode=dm)
            bus.bus.calibration = cal
        self._mirror_arm_calib_to_file()

    def save_calibration(self) -> dict:
        """Persist the recorded raw min/max as the servos' own Min/Max_Position_Limit.

        ARM ONLY. Since startup reads the range off the motors, saving to a file
        would be silently discarded — so the sweep goes into EEPROM instead (and
        is mirrored to the JSON purely as a record). The HAND is excluded: its
        limits are preset by the user and pushing a sweep in would clobber them.

        Only safe with torque off, which is the case during calibration."""
        bus = self._arm
        if bus is None:
            return {}
        with self._lock:
            ranges = {
                name: (self._calib_min[name], self._calib_max[name])
                for name in ARM_ORDER
                if self._calib_min.get(name) is not None
                and self._calib_max.get(name) is not None
                and self._calib_max[name] > self._calib_min[name]
            }
        try:
            changed = bus.write_ranges_to_motors(ranges)
        except Exception as e:
            self._error = f"save calib to motors: {e}"
            return {"error": str(e)}
        # adopt what the motors now actually report, not what we think we wrote
        bus.read_calibration_from_motors()
        self._mirror_arm_calib_to_file()
        return {"arm_motors": changed}

    def _mirror_arm_calib_to_file(self) -> None:
        """Write the live arm calibration to the JSON file. The file is not read
        back for ranges at startup — it exists as a human-readable record and as
        the store for drive_mode, which no servo register can hold."""
        bus = self._arm
        if bus is None or not bus.bus.calibration:
            return
        cal = {
            n: {"id": c.id, "drive_mode": c.drive_mode,
                "homing_offset": c.homing_offset,
                "range_min": c.range_min, "range_max": c.range_max}
            for n, c in bus.bus.calibration.items()
        }
        try:
            os.makedirs(os.path.dirname(self.arm_calib), exist_ok=True)
            self._backup_and_write(self.arm_calib, cal)
        except Exception as e:
            self._error = f"mirror calib file: {e}"

    @staticmethod
    def _backup_and_write(path: str, cal: dict) -> None:
        import json
        bak = path + ".bak"
        if not os.path.exists(bak):
            try:
                with open(path) as s, open(bak, "w") as d:
                    d.write(s.read())
            except Exception:
                pass
        with open(path, "w") as f:
            json.dump(cal, f, indent=4)

"""Mock backend — a lightweight servo simulator.

Lets the whole Desk (Dashboard, Robot Control, Hand Control) run and be demoed
with no robot attached. Positions ease toward their command, current spikes
while moving, temperature drifts with load. Good enough to exercise the UI and
the telemetry pipeline end-to-end.
"""
from __future__ import annotations

from ..hardware import ALL_MOTORS, MOTOR_BY_NAME, clamp
from ..models import MotorState, RobotStatus


class MockBackend:
    mode = "mock"

    def __init__(self) -> None:
        self._pos: dict[str, float] = {m.name: m.home for m in ALL_MOTORS}
        self._cmd: dict[str, float] = {m.name: m.home for m in ALL_MOTORS}
        self._vel: dict[str, float] = {m.name: 0.0 for m in ALL_MOTORS}
        self._temp: dict[str, float] = {m.name: 32.0 for m in ALL_MOTORS}
        self._servo_enabled = False
        self._estop = False
        self._connected = True
        self._compliant = False

    # --- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        self._connected = True

    def stop(self) -> None:
        self._connected = False

    def step(self, dt: float) -> None:
        if self._estop:
            for k in self._vel:
                self._vel[k] = 0.0
            self._cool(dt)
            return
        for m in ALL_MOTORS:
            target = self._cmd[m.name] if self._servo_enabled else self._pos[m.name]
            err = target - self._pos[m.name]
            # first-order tracking; ~6 units/s per unit error, capped
            step = max(-120.0 * dt, min(120.0 * dt, err * 6.0 * dt))
            self._pos[m.name] += step
            self._vel[m.name] = step / dt if dt > 0 else 0.0
            # temperature: rises with |velocity|, relaxes to ambient
            load = abs(self._vel[m.name])
            self._temp[m.name] += (0.0008 * load - 0.05 * (self._temp[m.name] - 32.0)) * dt

    def _cool(self, dt: float) -> None:
        for k in self._temp:
            self._temp[k] += -0.05 * (self._temp[k] - 32.0) * dt

    # --- reads ---------------------------------------------------------------
    def read_motors(self) -> list[MotorState]:
        out: list[MotorState] = []
        for m in ALL_MOTORS:
            v = self._vel[m.name]
            # Current models motor torque, which tracks position error (the PD
            # controller's demand). Large moves draw more current — realistic —
            # and a servo blocked against a load (large error, ~0 velocity)
            # reads high, which is exactly the contact signature.
            current = 0.0
            if self._servo_enabled and not self._estop:
                err = abs(self._cmd[m.name] - self._pos[m.name])
                current = 40.0 + 4.0 * err
            out.append(MotorState(
                name=m.name, servo_id=m.servo_id, unit=m.unit,
                position=round(self._pos[m.name], 3),
                command=round(self._cmd[m.name], 3),
                velocity=round(v, 3),
                current=round(current, 1),
                temperature=round(self._temp[m.name], 2),
                voltage=11.9 if m.unit == "arm" else 6.9,
                error=self._temp[m.name] > 70.0,
                online=self._connected,
            ))
        return out

    def status(self) -> RobotStatus:
        n_arm = sum(1 for m in ALL_MOTORS if m.unit == "arm") if self._connected else 0
        n_hand = sum(1 for m in ALL_MOTORS if m.unit == "hand") if self._connected else 0
        return RobotStatus(
            connected=self._connected,
            servo_enabled=self._servo_enabled,
            estop=self._estop,
            mode=self.mode,
            comm_ok=self._connected,
            comm_delay_ms=1.5,
            packet_loss=0.0,
            arm_online=n_arm,
            hand_online=n_hand,
        )

    # --- commands ------------------------------------------------------------
    def set_command(self, name: str, position: float) -> None:
        spec = MOTOR_BY_NAME.get(name)
        if spec is None:
            raise KeyError(name)
        self._cmd[name] = clamp(spec, position)

    def set_servo_enabled(self, enabled: bool, unit: str | None = None) -> None:
        # mock has no separate buses; a per-unit request drives the whole model
        self._servo_enabled = enabled and not self._estop

    def set_estop(self, engaged: bool) -> None:
        self._estop = engaged
        if engaged:
            self._servo_enabled = False

    def reboot_servo(self, name: str) -> None:
        if name not in self._pos:
            raise KeyError(name)
        self._temp[name] = 32.0

    def supports_teaching(self) -> bool:
        return True

    def set_compliant(self, on: bool) -> None:
        # No external force in sim; teaching just records the (static) pose.
        self._compliant = bool(on)

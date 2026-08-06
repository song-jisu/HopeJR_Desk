"""Abstract robot backend.

A backend owns the low-level link to the robot (real servos via ROS2, or a
simulator). The RobotManager drives it and never touches hardware directly, so
the same REST/WebSocket API works whether HopeJR is plugged in or not.
"""
from __future__ import annotations

import abc

from ..models import MotorState, RobotStatus


class RobotBackend(abc.ABC):
    mode: str = "base"

    @abc.abstractmethod
    def start(self) -> None:
        ...

    @abc.abstractmethod
    def stop(self) -> None:
        ...

    @abc.abstractmethod
    def step(self, dt: float) -> None:
        """Advance internal state / poll feedback. Called each telemetry tick."""

    @abc.abstractmethod
    def read_motors(self) -> list[MotorState]:
        ...

    @abc.abstractmethod
    def status(self) -> RobotStatus:
        ...

    # --- commands ------------------------------------------------------------
    @abc.abstractmethod
    def set_command(self, name: str, position: float) -> None:
        ...

    @abc.abstractmethod
    def set_servo_enabled(self, enabled: bool, unit: str | None = None) -> None:
        """`unit` None = both buses; "arm" or "hand" targets one (teaching)."""
        ...

    @abc.abstractmethod
    def set_estop(self, engaged: bool) -> None:
        ...

    @abc.abstractmethod
    def reboot_servo(self, name: str) -> None:
        ...

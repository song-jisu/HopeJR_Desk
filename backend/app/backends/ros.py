"""ROS2 backend — drives the real HopeJR over the existing ROS2 runtime.

Command path reuses the exact topics `teleop_server.py` already listens on:

    Desk backend ──/arm_goals (Float64MultiArray, 7)──┐
                 ──/hand_goals (Float64MultiArray,16)──┴─> teleop_server.py ─> Feetech bus

Feedback path: subscribes to `/joint_states` when a publisher (e.g.
`hopejr_state_publisher`) is running. Present current / temperature are not
published by the current stack, so those fields stay at 0 until a servo
feedback publisher is added (see README "Feedback TODO").

`rclpy` is imported lazily: on Windows / no-ROS environments importing this
module is fine — only start() requires a working ROS2 install.
"""
from __future__ import annotations

import threading
import time

from ..hardware import ARM_MOTORS, HAND_MOTORS, MOTOR_BY_NAME, clamp
from ..models import MotorState, RobotStatus

ARM_ORDER = [m.name for m in ARM_MOTORS]
HAND_ORDER = [m.name for m in HAND_MOTORS]


class RosBackend:
    mode = "ros"

    def __init__(self) -> None:
        self._cmd = {m.name: m.home for m in ARM_MOTORS + HAND_MOTORS}
        self._pos = dict(self._cmd)
        self._last_js_stamp = 0.0
        self._servo_enabled = False
        self._estop = False
        self._connected = False
        self._node = None
        self._executor = None
        self._spin_thread: threading.Thread | None = None
        self._arm_pub = None
        self._hand_pub = None
        self._lock = threading.Lock()

    # --- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("hopejr_desk_backend")
        self._arm_pub = self._node.create_publisher(Float64MultiArray, "/arm_goals", 10)
        self._hand_pub = self._node.create_publisher(Float64MultiArray, "/hand_goals", 10)
        self._node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._connected = True

    def stop(self) -> None:
        self._connected = False
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

    def _on_joint_states(self, msg) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                # joint_states may carry the 'hand_' prefix; match on suffix too
                key = name
                if key not in self._pos and name.startswith("hand_"):
                    key = name[len("hand_"):]
                if key in self._pos:
                    self._pos[key] = float(pos)
            self._last_js_stamp = time.time()

    # --- publish -------------------------------------------------------------
    def step(self, dt: float) -> None:
        if self._node is None or self._estop or not self._servo_enabled:
            return
        from std_msgs.msg import Float64MultiArray
        arm = Float64MultiArray()
        arm.data = [float(self._cmd[n]) for n in ARM_ORDER]
        self._arm_pub.publish(arm)
        hand = Float64MultiArray()
        hand.data = [float(self._cmd[n]) for n in HAND_ORDER]
        self._hand_pub.publish(hand)

    # --- reads ---------------------------------------------------------------
    def read_motors(self) -> list[MotorState]:
        out: list[MotorState] = []
        with self._lock:
            for name, spec in MOTOR_BY_NAME.items():
                out.append(MotorState(
                    name=name, servo_id=spec.servo_id, unit=spec.unit,
                    position=round(self._pos.get(name, 0.0), 3),
                    command=round(self._cmd.get(name, 0.0), 3),
                    velocity=0.0,
                    current=0.0,          # not published by current stack
                    temperature=0.0,      # not published by current stack
                    voltage=11.9 if spec.unit == "arm" else 6.9,
                    error=False,
                    online=self._connected,
                ))
        return out

    def status(self) -> RobotStatus:
        fresh = (time.time() - self._last_js_stamp) < 1.0
        return RobotStatus(
            connected=self._connected,
            servo_enabled=self._servo_enabled,
            estop=self._estop,
            mode=self.mode,
            comm_ok=self._connected,
            comm_delay_ms=0.0,
            packet_loss=0.0,
            arm_online=len(ARM_ORDER) if self._connected else 0,
            hand_online=len(HAND_ORDER) if (self._connected and fresh) else 0,
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
        if name not in self._cmd:
            raise KeyError(name)
        # A real reboot would call the Feetech bus; left as an integration hook.

#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# 모터 설정 (기존과 동일)
ARM_MOTORS = {
    "shoulder_pitch":       Motor(1, "sm8512bl", MotorNormMode.RANGE_M100_100),
    "shoulder_yaw":         Motor(2, "sts3250", MotorNormMode.RANGE_M100_100),
    "shoulder_roll":        Motor(3, "sts3250", MotorNormMode.RANGE_M100_100),
    "elbow_flex":           Motor(4, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_roll":           Motor(5, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_yaw":            Motor(6, "sts3250", MotorNormMode.RANGE_M100_100),
    "wrist_pitch":          Motor(7, "sts3250", MotorNormMode.RANGE_M100_100),
}

HAND_MOTORS = {
    "thumb_cmc":            Motor(1, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_mcp":            Motor(2, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_pip":            Motor(3, "scs0009", MotorNormMode.RANGE_0_100),
    "thumb_dip":            Motor(4, "scs0009", MotorNormMode.RANGE_0_100),
    "index_radial_flexor":  Motor(5, "scs0009", MotorNormMode.RANGE_0_100),
    "index_ulnar_flexor":   Motor(6, "scs0009", MotorNormMode.RANGE_0_100),
    "index_pip_dip":        Motor(7, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_radial_flexor": Motor(8, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_ulnar_flexor":  Motor(9, "scs0009", MotorNormMode.RANGE_0_100),
    "middle_pip_dip":       Motor(10, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_radial_flexor":   Motor(11, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_ulnar_flexor":    Motor(12, "scs0009", MotorNormMode.RANGE_0_100),
    "ring_pip_dip":         Motor(13, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_radial_flexor":  Motor(14, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_ulnar_flexor":   Motor(15, "scs0009", MotorNormMode.RANGE_0_100),
    "pinky_pip_dip":        Motor(16, "scs0009", MotorNormMode.RANGE_0_100),
}

class SafeBus:
    def __init__(self, port, motors, proto, name):
        self.bus = FeetechMotorsBus(port=port, motors=motors, protocol_version=proto)
        self.name = name
        try:
            self.bus.connect()
        except RuntimeError as e:
            if "firmware versions" in str(e):
                print(f"{name}: Firmware mismatch OK - opening manually")
                self.bus.port_handler.openPort()
            else: raise
        
        self.bus.calibration = self.bus.read_calibration()
        self.bus.enable_torque()
        print(f"✓ {name} Ready")

    def write(self, goals):
        try: self.bus.sync_write("Goal_Position", goals)
        except Exception as e: print(f"{self.name} Error: {e}")

class HopeJrSingleTeleop(Node):
    def __init__(self, mode, serial_port):
        super().__init__(f'hopejr_{mode}_teleop')
        self.mode = mode
        
        if mode == "arm":
            self.bus_wrapper = SafeBus(serial_port, ARM_MOTORS, 0, "ARM")
            self.order = list(ARM_MOTORS.keys())
            self.sub = self.create_subscription(Float64MultiArray, '/arm_goals', self.cb, 10)
            self.limits = (-100.0, 100.0)
        else: # hand
            self.bus_wrapper = SafeBus(serial_port, HAND_MOTORS, 1, "HAND")
            self.order = list(HAND_MOTORS.keys())
            self.sub = self.create_subscription(Float64MultiArray, '/hand_goals', self.cb, 10)
            self.limits = (0.0, 100.0)
            
        self.get_logger().info(f'Running in [{mode}] mode on {serial_port}')

    def cb(self, msg):
        goals = {}
        for i, val in enumerate(msg.data):
            if i >= len(self.order): break
            # 안전 범위 클리핑
            goals[self.order[i]] = max(self.limits[0], min(self.limits[1], float(val)))
        self.bus_wrapper.write(goals)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", choices=["arm", "hand"], required=True, help="arm or hand")
    parser.add_argument("--serial", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    args = parser.parse_args()
    
    rclpy.init()
    node = HopeJrSingleTeleop(args.type, args.serial)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\nShutting down {args.type}...")
    finally:
        if hasattr(node, 'bus_wrapper'):
            node.bus_wrapper.bus.disable_torque()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
teleop_key.py  –  ROS 2 keyboard teleoperation for 5-DOF robotic arm
──────────────────────────────────────────────────────────────────────
Key bindings (forward kinematics – each key increments/decrements by STEP°):

  Servo 1  MG996R  Base rotation     →  Q / A
  Servo 2  MG996R  Shoulder          →  W / S
  Servo 3  MG996R  Elbow             →  E / D
  Servo 4  MG90S   Wrist pitch       →  R / F
  Servo 5  MG90S   Gripper open/close→  T / G

  H  →  Home all servos (90, 90, 90, 90, 45)
  P  →  Print current angles
  +  →  Increase step size
  -  →  Decrease step size
  Ctrl-C  →  Exit

Publishes:
  /servo_angles   (std_msgs/Int32MultiArray)  –  [s1, s2, s3, s4, s5]

This topic is consumed by serial_bridge_node.py which forwards commands
over UART to the ESP32-S3.
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# ── Servo limits (must mirror ESP32 firmware) ──────────────────────────────────
ANGLE_MIN = [  0,   0,   0,   0,   0]
ANGLE_MAX = [180, 150, 150, 180,  90]
HOME      = [ 90,  90,  90,  90,  45]

# ── Key map: key → (servo_index, direction) ───────────────────────────────────
KEY_MAP = {
    'q': (0, +1),  # Base CCW
    'a': (0, -1),  # Base CW
    'w': (1, +1),  # Shoulder up
    's': (1, -1),  # Shoulder down
    'e': (2, +1),  # Elbow up
    'd': (2, -1),  # Elbow down
    'r': (3, +1),  # Wrist up
    'f': (3, -1),  # Wrist down
    't': (4, +1),  # Gripper open
    'g': (4, -1),  # Gripper close
}

SERVO_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Gripper"]


def get_key(settings):
    """Read a single keypress without blocking the terminal."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_banner(angles, step):
    print("\033[2J\033[H", end="")          # clear terminal
    print("╔══════════════════════════════════════╗")
    print("║   5-DOF Arm  Teleop  (ROS 2 + FK)   ║")
    print("╠══════════════════════════════════════╣")
    print(f"║  Step size: {step:>3}°                      ║")
    print("╠════════════╦══════╦═════╦═════╦═════╣")
    print("║  Servo     ║ Key+ ║ Key-║ Cur ║ Max ║")
    print("╠════════════╬══════╬═════╬═════╬═════╣")
    rows = [
        ("1 Base",    "Q", "A"),
        ("2 Shoulder","W", "S"),
        ("3 Elbow",   "E", "D"),
        ("4 Wrist",   "R", "F"),
        ("5 Gripper", "T", "G"),
    ]
    for i, (name, kp, km) in enumerate(rows):
        print(f"║ {name:<10} ║  {kp}   ║  {km}  ║ {angles[i]:>3} ║ {ANGLE_MAX[i]:>3} ║")
    print("╠════════════╩══════╩═════╩═════╩═════╣")
    print("║  H=Home  P=Print  +/-=Step  Ctrl-C  ║")
    print("╚══════════════════════════════════════╝")


class TeleopArmNode(Node):
    def __init__(self):
        super().__init__('teleop_arm_node')
        self.pub = self.create_publisher(Int32MultiArray, '/servo_angles', 10)
        self.angles = list(HOME)
        self.step = 5   # degrees per keypress

    def publish(self):
        msg = Int32MultiArray()
        msg.data = self.angles
        self.pub.publish(msg)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print_banner(self.angles, self.step)
        self.publish()          # Send home position on startup

        try:
            while rclpy.ok():
                key = get_key(settings)

                if key == '\x03':           # Ctrl-C
                    break

                elif key in KEY_MAP:
                    idx, direction = KEY_MAP[key]
                    new_angle = self.angles[idx] + direction * self.step
                    new_angle = max(ANGLE_MIN[idx], min(ANGLE_MAX[idx], new_angle))
                    self.angles[idx] = new_angle
                    self.publish()

                elif key.lower() == 'h':
                    self.angles = list(HOME)
                    self.publish()

                elif key.lower() == 'p':
                    self.get_logger().info(
                        "Angles: " + ", ".join(
                            f"{SERVO_NAMES[i]}={self.angles[i]}°"
                            for i in range(5)
                        )
                    )

                elif key == '+':
                    self.step = min(self.step + 1, 20)

                elif key == '-':
                    self.step = max(self.step - 1, 1)

                print_banner(self.angles, self.step)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.get_logger().info("Teleop node shut down.")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArmNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
serial_bridge_node.py  –  ROS 2 → UART bridge for ESP32-S3 servo controller
─────────────────────────────────────────────────────────────────────────────
Subscribes to:
  /servo_angles  (std_msgs/Int32MultiArray)  [s1, s2, s3, s4, s5]

Sends over serial to ESP32-S3:
  "SA:<a1>,<a2>,<a3>,<a4>,<a5>\n"

Usage:
  ros2 run <your_package> serial_bridge_node --ros-args \
      -p port:=/dev/ttyUSB0 \
      -p baud:=115200

Find your port:
  Linux:  ls /dev/ttyUSB*  or  ls /dev/ttyACM*
  macOS:  ls /dev/cu.usbserial*
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_all', True)     # True = "SA:..." bulk cmd
                                                      # False = individual "S<n>:..." per changed servo

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.send_all = self.get_parameter('send_all').get_parameter_value().bool_value

        # ── Serial connection ─────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)   # ESP32 resets on DTR; wait for boot
            self.get_logger().info(f"Serial connected: {port} @ {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise SystemExit(1)

        self.prev_angles = [None] * 5

        # ── ROS subscriber ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Int32MultiArray,
            '/servo_angles',
            self.angles_callback,
            10
        )

        # Optional: read ESP32 responses in a timer
        self.create_timer(0.1, self.read_serial_feedback)

        self.get_logger().info("Serial bridge node ready. Listening on /servo_angles")

    # ── Callback ──────────────────────────────────────────────────────────────
    def angles_callback(self, msg: Int32MultiArray):
        angles = list(msg.data)
        if len(angles) != 5:
            self.get_logger().warn(f"Expected 5 angles, got {len(angles)}")
            return

        if self.send_all:
            cmd = "SA:" + ",".join(str(a) for a in angles) + "\n"
            self._send(cmd)
        else:
            # Only send servos whose angle changed (saves serial bandwidth)
            for i, angle in enumerate(angles):
                if angle != self.prev_angles[i]:
                    cmd = f"S{i+1}:{angle}\n"
                    self._send(cmd)

        self.prev_angles = angles

    def _send(self, cmd: str):
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().debug(f"TX → {cmd.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial_feedback(self):
        """Read and log any responses from ESP32 (status / errors)."""
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.get_logger().info(f"ESP32 ← {line}")
        except serial.SerialException:
            pass

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

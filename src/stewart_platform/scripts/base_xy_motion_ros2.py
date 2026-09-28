#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll-pitch-yaw (rad) to a quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class StewartBaseXYMotionNode(Node):
    """Drive the full Stewart model base in XY via /stewart/base_pose_cmd."""

    def __init__(self) -> None:
        super().__init__("stewart_base_xy_motion")

        self.pose_pub = self.create_publisher(Pose, "/stewart/base_pose_cmd", 10)

        self.declare_parameter("enabled", True)
        self.declare_parameter("command_rate", 50.0)
        self.declare_parameter("x_center", 3.0)
        self.declare_parameter("y_center", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("x_amplitude", 0.5)
        self.declare_parameter("y_amplitude", 0.5)
        self.declare_parameter("frequency", 0.1)
        self.declare_parameter("phase_y", math.pi / 2.0)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)

        command_rate = max(1e-3, float(self.get_parameter("command_rate").value))
        self.command_period = 1.0 / command_rate
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.command_period, self._timer_callback)

        self.get_logger().info(
            "Base XY motion ready: topic=/stewart/base_pose_cmd, "
            f"rate={command_rate:.2f}Hz"
        )

    def _timer_callback(self) -> None:
        enabled = bool(self.get_parameter("enabled").value)
        if not enabled:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        x_center = float(self.get_parameter("x_center").value)
        y_center = float(self.get_parameter("y_center").value)
        z = float(self.get_parameter("z").value)
        x_amplitude = float(self.get_parameter("x_amplitude").value)
        y_amplitude = float(self.get_parameter("y_amplitude").value)
        frequency = float(self.get_parameter("frequency").value)
        phase_y = float(self.get_parameter("phase_y").value)
        roll = float(self.get_parameter("roll").value)
        pitch = float(self.get_parameter("pitch").value)
        yaw = float(self.get_parameter("yaw").value)

        omega_t = 2.0 * math.pi * frequency * elapsed
        x = x_center + x_amplitude * math.sin(omega_t)
        y = y_center + y_amplitude * math.sin(omega_t + phase_y)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = StewartBaseXYMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

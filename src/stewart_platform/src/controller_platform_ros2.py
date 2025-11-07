#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass
class CommandState:
    roll: float = 0.0
    pitch: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0
    x: float = 0.0
    y: float = 0.0


class StewartCommandNode(Node):
    """ROS 2 controller that publishes smooth sinusoidal pose commands."""

    def __init__(self) -> None:
        super().__init__("stewart_command_sinusoidal")

        self.publisher_ = self.create_publisher(
            Twist,
            "/stewart/platform_pose",
            10,
        )

        # Controller parameters (configurable via ROS parameters)
        self.declare_parameter("kp", 0.1)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("roll_amplitude", 0.0)
        self.declare_parameter("pitch_amplitude", 0.0)
        self.declare_parameter("heave_amplitude", 0.2)
        self.declare_parameter("frequency", 0.5)
        self.declare_parameter("filter_alpha", 0.5)
        self.declare_parameter("command_rate", 10.0)

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.prev_error = CommandState()
        self.last_command = CommandState()

        command_period = 1.0 / self.get_parameter("command_rate").value
        self.timer = self.create_timer(command_period, self._timer_callback)

        self.get_logger().info(
    f"Stewart command node ready (rate={command_period:.1f} Hz)"
)

    def _timer_callback(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        freq = self.get_parameter("frequency").value
        # desired = CommandState(
        #     roll=self.get_parameter("roll_amplitude").value * math.sin(freq * elapsed),
        #     pitch=self.get_parameter("pitch_amplitude").value * math.sin(
        #         freq * elapsed + math.pi / 2.0
        #     ),
        #     heave=self.get_parameter("heave_amplitude").value * math.sin(freq * elapsed),
        #     yaw=0.0,
        #     x=0.0,
        #     y=0.0,
        # )
        desired = CommandState(
            roll=0,
            pitch=-0.0,
            heave=self.get_parameter("heave_amplitude").value * math.sin(freq * elapsed),
            yaw=0.0,
            x=0.0,
            y=0.0,
        )

        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value

        errors = CommandState(
            roll=desired.roll - self.last_command.roll,
            pitch=desired.pitch - self.last_command.pitch,
            heave=desired.heave - self.last_command.heave,
            yaw=desired.yaw - self.last_command.yaw,
            x=desired.x - self.last_command.x,
            y=desired.y - self.last_command.y,
        )

        command = CommandState(
            roll=self._pd(desired.roll, errors.roll, self.prev_error.roll, kp, kd),
            pitch=self._pd(desired.pitch, errors.pitch, self.prev_error.pitch, kp, kd),
            heave=self._pd(desired.heave, errors.heave, self.prev_error.heave, kp, kd),
            yaw=self._pd(desired.yaw, errors.yaw, self.prev_error.yaw, kp, kd),
            x=self._pd(desired.x, errors.x, self.prev_error.x, kp, kd),
            y=self._pd(desired.y, errors.y, self.prev_error.y, kp, kd),
        )

        self.prev_error = errors

        alpha = self.get_parameter("filter_alpha").value
        smoothed = CommandState(
            roll=self._smooth(command.roll, self.last_command.roll, alpha),
            pitch=self._smooth(command.pitch, self.last_command.pitch, alpha),
            heave=self._smooth(command.heave, self.last_command.heave, alpha),
            yaw=self._smooth(command.yaw, self.last_command.yaw, alpha),
            x=self._smooth(command.x, self.last_command.x, alpha),
            y=self._smooth(command.y, self.last_command.y, alpha),
        )
        self.last_command = smoothed

        twist = Twist()
        twist.angular.x = smoothed.roll
        twist.angular.y = smoothed.pitch
        twist.angular.z = smoothed.yaw
        twist.linear.x = smoothed.x
        twist.linear.y = smoothed.y
        twist.linear.z = smoothed.heave

        self.publisher_.publish(twist)
        self.get_logger().debug(
    f"cmd roll={smoothed.roll:.3f} pitch={smoothed.pitch:.3f} heave={smoothed.heave:.3f}"
)

    @staticmethod
    def _pd(desired: float, error: float, prev_error: float, kp: float, kd: float) -> float:
        return desired + kp * error + kd * (error - prev_error)

    @staticmethod
    def _smooth(value: float, previous: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return alpha * value + (1.0 - alpha) * previous


def main() -> None:
    rclpy.init()
    node = StewartCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

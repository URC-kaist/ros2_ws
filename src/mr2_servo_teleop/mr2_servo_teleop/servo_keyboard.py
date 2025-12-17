import os
import select
import sys
import termios
import tty
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


@dataclass
class AxisConfig:
    idx: int
    step: float


class TerminalMode:
    """Context manager that switches a TTY to raw mode and restores on exit."""

    def __init__(self, stream):
        self.stream = stream
        self.fd = stream.fileno()

    def __enter__(self):
        self.old_attrs = termios.tcgetattr(self.stream)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(self.stream, termios.TCSADRAIN, self.old_attrs)


class ServoKeyboard(Node):
    def __init__(self):
        super().__init__("servo_keyboard")
        self.declare_parameter("cartesian_command_topic", "/moveit_servo/delta_twist_cmds")
        self.declare_parameter("frame_id", "base_link")
        # Default to snappier velocity steps for simulation; override via launch if needed.
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("stop_timeout", 0.5)

        topic = self.get_parameter("cartesian_command_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        linear_step = self.get_parameter("linear_step").get_parameter_value().double_value
        angular_step = self.get_parameter("angular_step").get_parameter_value().double_value
        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.stop_timeout = self.get_parameter("stop_timeout").get_parameter_value().double_value

        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        self.publish_period = 1.0 / rate if rate > 0.0 else 0.02

        # Twist layout: [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
        self.twist_components = [0.0] * 6
        self.last_input_time = time.time()

        self.linear_bindings: Dict[str, AxisConfig] = {
            "w": AxisConfig(0, linear_step),
            "s": AxisConfig(0, -linear_step),
            "a": AxisConfig(1, linear_step),
            "d": AxisConfig(1, -linear_step),
            "r": AxisConfig(2, linear_step),
            "f": AxisConfig(2, -linear_step),
        }
        self.angular_bindings: Dict[str, AxisConfig] = {
            "y": AxisConfig(3, angular_step),   # roll +
            "h": AxisConfig(3, -angular_step),  # roll -
            "t": AxisConfig(4, angular_step),   # pitch +
            "g": AxisConfig(4, -angular_step),  # pitch -
            "q": AxisConfig(5, angular_step),   # yaw +
            "e": AxisConfig(5, -angular_step),  # yaw -
        }

        self.timer = self.create_timer(self.publish_period, self._publish)

        # Pick an interactive TTY for keyboard input
        self.input_stream = self._select_tty()
        if self.input_stream is None:
            self.get_logger().fatal("No TTY available for keyboard control. Run from a terminal.")
            raise SystemExit(1)

    def _select_tty(self) -> Optional[object]:
        """Return a stream connected to a real TTY, or None if not available."""
        if sys.stdin.isatty():
            return sys.stdin
        # Try opening /dev/tty (works when ros2 launch pipes stdin)
        try:
            return open("/dev/tty")
        except OSError as exc:
            self.get_logger().error(f"Failed to open /dev/tty: {exc}")
            return None

    def handle_key(self, key: str):
        if key == " ":
            self.twist_components = [0.0] * 6
        elif key in self.linear_bindings:
            axis = self.linear_bindings[key]
            self.twist_components[axis.idx] = axis.step
        elif key in self.angular_bindings:
            axis = self.angular_bindings[key]
            self.twist_components[axis.idx] = axis.step
        elif key == "x":
            # emergency stop, zero everything
            self.twist_components = [0.0] * 6
        else:
            return
        self.last_input_time = time.time()

    def _publish(self):
        now = time.time()
        if now - self.last_input_time > self.stop_timeout:
            self.twist_components = [0.0] * 6

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = self.twist_components[0]
        msg.twist.linear.y = self.twist_components[1]
        msg.twist.linear.z = self.twist_components[2]
        msg.twist.angular.x = self.twist_components[3]
        msg.twist.angular.y = self.twist_components[4]
        msg.twist.angular.z = self.twist_components[5]
        self.publisher.publish(msg)


INSTRUCTIONS = """
MoveIt Servo keyboard teleop
---------------------------
Linear:  w/s x, a/d y, r/f z
Angular: y/h roll, t/g pitch, q/e yaw
space: zero command    x: e-stop (zero)    Ctrl-C: quit
"""


def main():
    rclpy.init()
    node = ServoKeyboard()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    print(INSTRUCTIONS)
    stream = node.input_stream
    with TerminalMode(stream):
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.0)
                if select.select([stream], [], [], 0.01)[0]:
                    key = stream.read(1)
                    if key == "\x03":  # Ctrl-C
                        break
                    node.handle_key(key)
        finally:
            if stream is not sys.stdin:
                stream.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

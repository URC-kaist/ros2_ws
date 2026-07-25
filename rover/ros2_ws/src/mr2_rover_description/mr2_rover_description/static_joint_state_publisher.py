import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("static_joint_state_publisher")
        self._joint_names = list(
            self.declare_parameter("joint_names", ["left_rocker_joint"]).value
        )
        self._positions = list(self.declare_parameter("positions", [0.0]).value)
        publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)

        if not self._joint_names:
            raise ValueError("joint_names must contain at least one joint")
        if len(self._positions) != len(self._joint_names):
            raise ValueError("positions length must match joint_names length")
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)
        self._publish()

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = self._positions
        self._publisher.publish(msg)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = StaticJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
